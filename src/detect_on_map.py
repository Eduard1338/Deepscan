#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from scipy.ndimage import gaussian_filter, binary_fill_holes, label, find_objects
from scipy.interpolate import griddata
import os, glob, trimesh, asyncio, asyncpg
from datetime import datetime

def load_xyz(path):
    with open(path, 'r') as f:
        first = f.readline().strip()
    skip = 1 if first.startswith('X') else 0
    data = np.loadtxt(path, skiprows=skip)
    return data[:, 0], data[:, 1], data[:, 2]

def build_raster(x, y, z, resolution=0.5):
    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()
    grid_x, grid_y = np.meshgrid(
        np.arange(x_min, x_max, resolution),
        np.arange(y_min, y_max, resolution)
    )
    grid_z = griddata((x, y), z, (grid_x, grid_y), method='linear', fill_value=np.nan)
    mask = np.isnan(grid_z)
    if mask.any():
        grid_z[mask] = np.nanmedian(grid_z)
    return grid_x, grid_y, grid_z, x_min, y_min

def detect_by_gradient(Z, x_min, y_min, resolution,
                       grad_threshold=0.15, min_area=4.0, max_area=200.0,
                       smooth_sigma=1.0):
    Z_smooth = gaussian_filter(Z, sigma=smooth_sigma)
    gy, gx = np.gradient(Z_smooth)
    grad_mag = np.sqrt(gx**2 + gy**2)
    edge_mask = grad_mag > grad_threshold
    filled = binary_fill_holes(edge_mask)
    labeled, num_features = label(filled)
    slices = find_objects(labeled)
    boxes = []
    for i, sl in enumerate(slices):
        if sl is None:
            continue
        obj_mask = (labeled[sl] == (i + 1))
        area_pixels = np.sum(obj_mask)
        area_m2 = area_pixels * (resolution ** 2)
        if area_m2 < min_area or area_m2 > max_area:
            continue
        y_slice, x_slice = sl
        x_center = (x_slice.start + x_slice.stop - 1) / 2.0 * resolution + x_min
        y_center = (y_slice.start + y_slice.stop - 1) / 2.0 * resolution + y_min
        width = (x_slice.stop - x_slice.start) * resolution
        height = (y_slice.stop - y_slice.start) * resolution
        boxes.append((x_center, y_center, width, height))
    return boxes, edge_mask, filled

def export_glb(grid_x, grid_y, grid_z, boxes, output_path, resolution=0.5):
    # Берём все точки (плотность 1px)
    pts = np.column_stack([grid_x.ravel(), grid_y.ravel(), grid_z.ravel()])
    z_min, z_max = pts[:, 2].min(), pts[:, 2].max()
    if z_max > z_min:
        norm = (pts[:, 2] - z_min) / (z_max - z_min)
    else:
        norm = np.zeros(len(pts))
    colors = np.zeros((len(pts), 3))
    colors[:, 0] = norm       # красный
    colors[:, 2] = 1.0 - norm # синий

    # Добавляем объекты как красные сферы
    if boxes:
        vertices_list = []
        colors_list = []
        for (cx, cy, w, h) in boxes:
            radius = max(w, h) / 2
            sphere = trimesh.creation.icosphere(subdivisions=2, radius=radius)
            # Приблизительная высота в центре объекта
            z_center = grid_z[int(cy / resolution), int(cx / resolution)]
            sphere.vertices += [cx, cy, z_center]
            vertices_list.append(sphere.vertices)
            colors_list.append(np.tile([1.0, 0.0, 0.0], (len(sphere.vertices), 1)))
        if vertices_list:
            all_spheres = np.vstack(vertices_list)
            all_colors = np.vstack(colors_list)
            pts = np.vstack([pts, all_spheres])
            colors = np.vstack([colors, all_colors])

    pcd = trimesh.PointCloud(vertices=pts, colors=np.clip(colors, 0, 1) * 255)
    pcd.export(output_path)

async def save_to_db(boxes, map_file, glb_file):
    try:
        conn = await asyncpg.connect(user='deepscan', password='deepscan',
                                     database='deepscan_db', host='localhost')
        mission_id = await conn.fetchval(
            "INSERT INTO missions (map_file) VALUES ($1) RETURNING id", map_file
        )
        for (cx, cy, width, height) in boxes:
            await conn.execute(
                "INSERT INTO detections (mission_id, world_x, world_y, world_z, radius, width, length, height, point_count) "
                "VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9)",
                mission_id, cx, cy, -10.0, max(width, height)/2, width, height, 0.5, 100
            )
        await conn.close()
        print(f"✅ Данные записаны в БД (миссия {mission_id})")
    except Exception as e:
        print(f"❌ Ошибка записи в БД: {e}")
# ---------- Главная часть ----------
data_dir = os.path.expanduser('~/deepscan_sonar_data')
map_dir = os.path.join(data_dir, 'maps')
os.makedirs(map_dir, exist_ok=True)

xyz_files = sorted(glob.glob(os.path.join(data_dir, 'sonar_accurate_smart_*.xyz')))
if not xyz_files:
    xyz_files = sorted(glob.glob(os.path.join(data_dir, 'sonar_accurate_*.xyz')))
if not xyz_files:
    xyz_files = sorted(glob.glob(os.path.join(data_dir, 'sonar_smart_*.xyz')))
if not xyz_files:
    print("Нет файлов дна. Запустите gzcollect.")
    exit(1)

xyz_path = xyz_files[-1]
print(f"Анализируем: {xyz_path}")

RESOLUTION = 0.5
GRAD_THRESHOLD = 0.15
MIN_AREA_M2 = 3.0
MAX_AREA_M2 = 150.0
SMOOTH_SIGMA = 1.0

x, y, z = load_xyz(xyz_path)
grid_x, grid_y, grid_z, x_min, y_min = build_raster(x, y, z, RESOLUTION)

boxes, edge_mask, filled_mask = detect_by_gradient(
    grid_z, x_min, y_min, RESOLUTION,
    grad_threshold=GRAD_THRESHOLD,
    min_area=MIN_AREA_M2,
    max_area=MAX_AREA_M2,
    smooth_sigma=SMOOTH_SIGMA
)
print(f"Найдено объектов: {len(boxes)}")

# ---------- Визуализация ----------
fig, axes = plt.subplots(1, 3, figsize=(24, 8))
ax = axes[0]
im = ax.pcolormesh(grid_x, grid_y, grid_z, cmap='viridis_r', shading='auto')
plt.colorbar(im, ax=ax, label='Глубина (м)')
ax.set_aspect('equal')
ax.set_xlabel('X, м')
ax.set_ylabel('Y, м')
ax.set_title('Карта дна с обнаруженными объектами')
for (cx, cy, w, h) in boxes:
    rect = Rectangle((cx - w/2, cy - h/2), w, h,
                     linewidth=2.0, edgecolor='red', facecolor='none', alpha=0.8)
    ax.add_patch(rect)
    ax.text(cx, cy + h/2 + 1.0, f'{w:.1f}x{h:.1f} м',
            fontsize=8, color='red', ha='center',
            bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.6))
ax.grid(True, alpha=0.3)

ax2 = axes[1]
ax2.imshow(edge_mask, cmap='gray', origin='lower', extent=[x_min, x_min + grid_x.shape[1]*RESOLUTION,
                                                           y_min, y_min + grid_y.shape[0]*RESOLUTION])
ax2.set_title('Маска градиентов')
ax2.set_aspect('equal')
ax2.set_xlabel('X, м')
ax2.set_ylabel('Y, м')

ax3 = axes[2]
ax3.imshow(filled_mask, cmap='gray', origin='lower', extent=[x_min, x_min + grid_x.shape[1]*RESOLUTION,
                                                             y_min, y_min + grid_y.shape[0]*RESOLUTION])
ax3.set_title('Заполненные объекты')
ax3.set_aspect('equal')
ax3.set_xlabel('X, м')
ax3.set_ylabel('Y, м')

plt.tight_layout()

timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
png_name = f'seabed_2d_ai_{timestamp}.png'
glb_name = f'seabed_3d_{timestamp}.glb'
png_path = os.path.join(map_dir, png_name)
glb_path = os.path.join(map_dir, glb_name)

plt.savefig(png_path, dpi=150)
export_glb(grid_x, grid_y, grid_z, boxes, glb_path)
print(f"Файлы сохранены: {png_path}, {glb_path}")

# Запись в БД
asyncio.run(save_to_db(boxes, png_name, glb_name))

# Ротация старых файлов
for ext in ['png', 'glb']:
    files = sorted(glob.glob(os.path.join(map_dir, f'seabed_*_{ext}')))
    while len(files) > 10:
        os.remove(files.pop(0))



plt.show()
