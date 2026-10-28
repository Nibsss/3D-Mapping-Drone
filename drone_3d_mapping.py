"""
3D mapping drone - CoppeliaSim Edu (ZMQ Remote API)
Target: Grade A (path + scan 3D + voxel + color mapping tinggi)

Scene yang dibutuhkan (dari drone_plannar.ttt):
  /Quadcopter  (model drone, mengikuti dummy /target)
  /target      (dummy/sphere yang digerakkan script ini)
Tambahkan 1 vision sensor (depth) bernama 'depthSensor' sebagai CHILD dari
/Quadcopter. Posisi/orientasi sensor di-set oleh script ini (pitch turun + yaw berputar).
Matikan/abaikan child script lama yang menggerakkan target bila ada.
"""
import time
import numpy as np
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# ================== PARAMETER ==================
ALT        = 1.5          # ketinggian terbang (m)
TILT_DEG   = 50.0         # sudut sensor menunduk dari horizontal (0=lurus ke depan, 90=ke bawah)
SENSOR_YAW_RATE = 2.0     # rad per detik putaran sensor terhadap badan drone
VOXEL      = 0.08         # ukuran voxel (m)
X_RANGE    = (-1.6, 1.6)  # area sapuan waypoint
Y_RANGE    = (-1.6, 1.6)
LANE_STEP  = 1.0
WP_TOL     = 0.15         # toleransi sampai waypoint (m)
SPEED      = 0.3          # kecepatan gerak target/drone (m/s), makin kecil makin pelan
SCAN_EVERY = 3            # ambil scan tiap N step simulasi
MAX_RANGE  = 4.0          # buang titik lebih jauh dari ini (m)
# ===============================================

client = RemoteAPIClient()
sim = client.require('sim')
sim.setStepping(True)

drone  = sim.getObject('/Quadcopter')
target = sim.getObject('/Quadcopter/target')  # sesuai hierarki scene (di-detach otomatis saat simulasi start)
cam    = sim.getObject('/Quadcopter/depthSensor')

# Intrinsik kamera
near = sim.getObjectFloatParam(cam, sim.visionfloatparam_near_clipping)
far  = sim.getObjectFloatParam(cam, sim.visionfloatparam_far_clipping)
fov  = sim.getObjectFloatParam(cam, sim.visionfloatparam_perspective_angle)
W, H = sim.getVisionSensorRes(cam)
f = (max(W, H) / 2.0) / np.tan(fov / 2.0)

# Grid piksel -> arah ray (dihitung sekali)
us, vs = np.meshgrid(np.arange(W), np.arange(H))
# Image Coppelia: baris 0 di bawah. Sumbu x sensor menghadap kiri-gambar,
# jika hasil peta ter-mirror, hapus tanda minus di X_RAY.
X_RAY = -(us - (W - 1) / 2.0) / f
Y_RAY = (vs - (H - 1) / 2.0) / f


def mat12_to_4x4(m):
    M = np.eye(4)
    M[:3, :] = np.array(m).reshape(3, 4)
    return M


def set_sensor_pose(yaw):
    """Sensor menghadap arah (yaw, tilt) relatif badan drone."""
    t = np.deg2rad(TILT_DEG)
    z = np.array([np.cos(t) * np.cos(yaw), np.cos(t) * np.sin(yaw), -np.sin(t)])
    up = np.array([0.0, 0.0, 1.0])
    x = np.cross(up, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    m = [x[0], y[0], z[0], 0.0,
         x[1], y[1], z[1], 0.0,
         x[2], y[2], z[2], -0.05]
    sim.setObjectMatrix(cam, drone, m)


def grab_points():
    buf, res = sim.getVisionSensorDepth(cam, 1)  # buffer float32 ternormalisasi 0..1
    d = np.frombuffer(buf, dtype=np.float32).reshape(H, W).astype(np.float64)
    if d.max() > 1.0001:          # depth sudah dalam meter
        z = d
    else:                         # depth ternormalisasi 0..1
        z = near + d * (far - near)
    if not hasattr(grab_points, 'printed'):
        print(f'[debug] depth min={d.min():.3f} max={d.max():.3f} | near={near} far={far} fov={np.rad2deg(fov):.1f} res={W}x{H}')
        grab_points.printed = True
    valid = (z < min(MAX_RANGE, far - 0.02)) & (z > near + 0.02)
    P = np.stack([X_RAY * z, Y_RAY * z, z], axis=-1)[valid]
    if len(P) == 0:
        return np.empty((0, 3))
    T = mat12_to_4x4(sim.getObjectMatrix(cam, -1))
    Pw = (T[:3, :3] @ P.T).T + T[:3, 3]
    return Pw


def make_waypoints():
    wps = []
    ys = np.arange(Y_RANGE[0], Y_RANGE[1] + 1e-6, LANE_STEP)
    for i, y in enumerate(ys):
        xs = X_RANGE if i % 2 == 0 else X_RANGE[::-1]
        wps.append((xs[0], y, ALT))
        wps.append((xs[1], y, ALT))
    return wps


def voxel_downsample(P, size):
    idx = np.floor(P / size).astype(np.int64)
    _, uniq = np.unique(idx, axis=0, return_index=True)
    centers = (idx[uniq] + 0.5) * size
    return centers


# ================== MAIN LOOP ==================
cloud_chunks, path = [], []
waypoints = make_waypoints()
wp_i = 0

sim.startSimulation()
dt = sim.getSimulationTimeStep()
step = 0
t0 = time.time()
tgt_pos = np.array(sim.getObjectPosition(target, -1))
try:
    while wp_i < len(waypoints):
        wp = waypoints[wp_i]
        # geser target bertahap menuju waypoint dengan kecepatan SPEED
        diff = np.array(wp) - tgt_pos
        dist = np.linalg.norm(diff)
        if dist > 1e-6:
            tgt_pos = tgt_pos + diff / dist * min(SPEED * dt, dist)
        sim.setObjectPosition(target, -1, tgt_pos.tolist())

        yaw = (sim.getSimulationTime() * SENSOR_YAW_RATE) % (2 * np.pi)
        set_sensor_pose(yaw)

        pos = sim.getObjectPosition(drone, -1)
        path.append(pos)

        if step % SCAN_EVERY == 0:
            cloud_chunks.append(voxel_downsample(grab_points(), VOXEL / 2))

        if np.linalg.norm(np.array(pos) - np.array(wp)) < WP_TOL:
            wp_i += 1

        sim.step()
        step += 1
        if time.time() - t0 > 600:  # pengaman 10 menit
            break
finally:
    sim.stopSimulation()

# ================== POST PROCESS ==================
raw = np.vstack(cloud_chunks)
vox = voxel_downsample(raw, VOXEL)
path = np.array(path)
print(f'Raw: {len(raw)} titik -> Voxel: {len(vox)} titik')
print(f'[debug] Z peta: min={vox[:,2].min():.2f} p5={np.percentile(vox[:,2],5):.2f} '
      f'median={np.median(vox[:,2]):.2f} p95={np.percentile(vox[:,2],95):.2f} max={vox[:,2].max():.2f}')

np.save('map_voxel.npy', vox)
np.save('drone_path.npy', path)
np.save('map_raw.npy', raw)

# --- Scan 2D: irisan horizontal tampak atas (bukti tahap BC) ---
Z_SLICE = (0.15, 0.25)
sl = vox[(vox[:, 2] > Z_SLICE[0]) & (vox[:, 2] < Z_SLICE[1])]
fig2, ax2 = plt.subplots(figsize=(6, 6))
ax2.scatter(sl[:, 0], sl[:, 1], s=6, c='k', label='Hasil scan 2D')
ax2.plot(path[:, 0], path[:, 1], 'b-', lw=1.5, label='Jalur drone')
ax2.set_aspect('equal'); ax2.legend(); ax2.set_xlabel('X'); ax2.set_ylabel('Y')
ax2.set_title(f'Scan 2D (irisan z = {Z_SLICE[0]}-{Z_SLICE[1]} m)')
fig2.savefig('2d_scan.png', dpi=200)

fig = plt.figure(figsize=(9, 7))
ax = fig.add_subplot(111, projection='3d', computed_zorder=False)  # zorder dihormati
sc = ax.scatter(vox[:, 0], vox[:, 1], vox[:, 2], c=vox[:, 2], cmap='RdYlGn_r',
                s=4, alpha=0.6, depthshade=False, zorder=1)
ax.plot(path[:, 0], path[:, 1], path[:, 2], color='blue', lw=3,
        label='Jalur drone', zorder=10)
ax.scatter(*path[0], color='black', s=60, marker='o', label='Start', zorder=11)
ax.scatter(*path[-1], color='magenta', s=60, marker='X', label='Finish', zorder=11)
ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
ax.set_title('3D Map')
zmax = max(vox[:, 2].max(), path[:, 2].max())
ax.set_zlim(0, zmax * 1.05)
ax.set_box_aspect((np.ptp(vox[:, 0]), np.ptp(vox[:, 1]), zmax * 1.5))
ax.view_init(elev=35, azim=-60)   # sudut miring supaya jalur di atas peta terlihat
fig.colorbar(sc, label='Ketinggian (m)', shrink=0.6)
ax.legend()
plt.savefig('3d_map.png', dpi=200)
plt.show()