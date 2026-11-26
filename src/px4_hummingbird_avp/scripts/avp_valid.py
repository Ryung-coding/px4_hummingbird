#!/usr/bin/env python3
"""Compare surface coverage in two recorded outdoor RGB-D flights."""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path

import cv2
import numpy as np

from avp_log_replay import FlightData, MOUNT, ROOTS, locate, quat_wxyz_to_R


VOXEL_M = 0.05
CELL_M = 0.10
DEPTH_MIN_M, DEPTH_MAX_M = 0.2, 4.0
SAMPLE_STEP = 4
ROI_RADIUS_M = 5.0
ROBUST_CELL_FRAMES = 3


def camera_transform(mode):
    optical = quat_wxyz_to_R((-0.5, 0.5, -0.5, 0.5))
    if mode == 'sim':
        return optical, np.array([0.20, 0.0, 0.05])
    pos, rpy = MOUNT['position_m'], MOUNT['rpy_rad']
    roll, pitch, yaw = (float(rpy[axis]) for axis in ('roll', 'pitch', 'yaw'))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx @ optical, np.array([pos['x'], pos['y'], pos['z']], dtype=float)


def measure(folder, reference=None):
    data = FlightData(folder, None)
    try:
        if not data.depth or not data.odom or data.camera is None or not data.sweep_center:
            raise ValueError(f'{folder}: depth, odometry, camera_info and sweep_center are required')
        camera_r, camera_p = camera_transform(data.mode)
        center = np.asarray(data.sweep_center[0][1][:3], dtype=float)
        start = np.asarray(data.odom[0][1][:3], dtype=float)
        heading = center[:2] - start[:2]
        heading /= max(np.linalg.norm(heading), 1e-9)
        if reference is None:
            reference = (center, np.array([-heading[1], heading[0]]), float(start[2]))
        grid_center, side, lower_top = reference
        fx, fy, cx, cy = (float(data.camera[i]) for i in (0, 4, 2, 5))
        if fx <= 0 or fy <= 0:
            raise ValueError(f'{folder}: invalid camera intrinsics')
        odom_times = np.asarray([row[0] for row in data.odom])
        voxels, visits, cell_hits = Counter(), Counter(), Counter()
        frames = samples = skipped = 0
        for stamp, db_file, frame_id, scale in data.depth:
            index = np.searchsorted(odom_times, stamp)
            index = min((max(index - 1, 0), min(index, len(odom_times) - 1)),
                        key=lambda i: abs(odom_times[i] - stamp))
            if abs(odom_times[index] - stamp) > 0.2:
                skipped += 1
                continue
            image = (data.read_frame(db_file, frame_id, cv2.IMREAD_UNCHANGED)
                     if db_file else np.load(folder / frame_id))
            if image is None or image.ndim != 2:
                skipped += 1
                continue
            z = image[::SAMPLE_STEP, ::SAMPLE_STEP].astype(np.float32) * scale
            v, u = np.mgrid[0:image.shape[0]:SAMPLE_STEP, 0:image.shape[1]:SAMPLE_STEP]
            valid = np.isfinite(z) & (z > DEPTH_MIN_M) & (z < DEPTH_MAX_M)
            valid &= np.abs((v - cy) * z / fy) <= z * math.tan(math.radians(15.0))
            if not np.any(valid):
                skipped += 1
                continue
            xyz = np.column_stack(((u[valid] - cx) * z[valid] / fx,
                                   (v[valid] - cy) * z[valid] / fy, z[valid]))
            pose = data.odom[index][1]
            body_r = quat_wxyz_to_R(pose[3:7])
            if body_r is None:
                skipped += 1
                continue
            world = (xyz @ camera_r.T + camera_p) @ body_r.T + np.asarray(pose[:3])
            roi = np.linalg.norm(world[:, :2] - grid_center[:2], axis=1) <= ROI_RADIUS_M
            roi &= (world[:, 2] >= grid_center[2] - 1.0) & (world[:, 2] <= grid_center[2] + 5.0)
            world = world[roi]
            if not len(world):
                continue
            frame_cells = set()
            keys, counts = np.unique(np.floor(world / VOXEL_M).astype(np.int32),
                                     axis=0, return_counts=True)
            for key, count in zip(keys, counts):
                voxel = tuple(int(value) for value in key)
                voxels[voxel] += int(count)
                visits[voxel] += 1
                point = (key + 0.5) * VOXEL_M
                lateral = np.dot(point[:2] - grid_center[:2], side)
                frame_cells.add((math.floor(lateral / CELL_M), math.floor(point[2] / CELL_M)))
            cell_hits.update(frame_cells)
            frames += 1
            samples += len(world)
        if not frames:
            raise ValueError(f'{folder}: no depth frames could be projected into the scan ROI')
        duration = max(data.depth[-1][0] - data.depth[0][0], 1e-6)
        lower_hits = [hits for (_, height), hits in cell_hits.items()
                      if (height + 0.5) * CELL_M <= lower_top]
        stats = {
            'folder': str(folder), 'mode': data.mode, 'depth_frames': frames,
            'skipped_frames': skipped, 'duration_s': round(duration, 2),
            'sampled_points': samples, 'occupied_voxels_5cm': len(voxels),
            'vehicle_height_range_m': [round(min(row[1][2] for row in data.odom), 2),
                                       round(max(row[1][2] for row in data.odom), 2)],
            'projected_coverage_m2': round(len(cell_hits) * CELL_M * CELL_M, 3),
            'lower_coverage_m2': round(len(lower_hits) * CELL_M * CELL_M, 3),
            'lower_robust_coverage_m2': round(sum(hits >= ROBUST_CELL_FRAMES for hits in lower_hits) * CELL_M * CELL_M, 3),
            'lower_median_frame_hits': float(np.median(lower_hits)) if lower_hits else 0.0,
            'samples_per_voxel': round(samples / len(voxels), 2),
            'reobserved_voxels_pct': round(100 * sum(n > 1 for n in visits.values()) / len(visits), 2),
            'voxels_per_second': round(len(voxels) / duration, 2),
        }
        coarse = {tuple(value // 2 for value in voxel) for voxel in voxels}
        return stats, coarse, cell_hits, reference
    finally:
        data.close()


def render_comparison(on_hits, off_hits, lower_top, on, off, output, show):
    import matplotlib.pyplot as plt
    from matplotlib.colors import BoundaryNorm, ListedColormap, LogNorm
    from matplotlib.patches import Patch

    cells = set(on_hits) | set(off_hits)
    xmin, xmax = min(x for x, _ in cells), max(x for x, _ in cells)
    ymin, ymax = min(y for _, y in cells), max(y for _, y in cells)
    shape = (ymax - ymin + 1, xmax - xmin + 1)
    on_map, off_map, difference = (np.zeros(shape, dtype=float) for _ in range(3))
    for cell in cells:
        row, col = cell[1] - ymin, cell[0] - xmin
        on_map[row, col], off_map[row, col] = on_hits[cell], off_hits[cell]
        difference[row, col] = 2 if on_hits[cell] and off_hits[cell] else (1 if on_hits[cell] else 3)
    lower_cells = [cell for cell in cells if (cell[1] + 0.5) * CELL_M <= lower_top]
    lower_xmin = min((x for x, _ in lower_cells), default=xmin)
    lower_xmax = max((x for x, _ in lower_cells), default=xmax)
    lower_ymin = min((y for _, y in lower_cells), default=ymin)
    extent = [xmin * CELL_M, (xmax + 1) * CELL_M, ymin * CELL_M, (ymax + 1) * CELL_M]
    counts = np.asarray(list(on_hits.values()) + list(off_hits.values()))
    density_cmap = plt.colormaps['viridis'].copy()
    density_cmap.set_bad('#edf0f2')
    density_norm = LogNorm(vmin=1, vmax=max(2, float(np.percentile(counts, 98))))
    difference_cmap = ListedColormap(['#159895', '#9aa4ac', '#e66a4e'])
    difference_norm = BoundaryNorm([0.5, 1.5, 2.5, 3.5], difference_cmap.N)

    fig, axes = plt.subplots(2, 3, figsize=(17, 10), constrained_layout=True)
    for row in range(2):
        for col, values in enumerate((on_map, off_map, difference)):
            ax = axes[row, col]
            image = np.ma.masked_where(values == 0, values)
            cmap = density_cmap if col < 2 else difference_cmap
            norm = density_norm if col < 2 else difference_norm
            ax.imshow(image, origin='lower', extent=extent, cmap=cmap, norm=norm,
                      aspect='equal' if row == 0 else 'auto')
            ax.set_facecolor('#edf0f2')
            ax.set_xlabel('Lateral position from tree center [m]')
            if col == 0:
                ax.set_ylabel('World height [m]')
            if row == 0:
                ax.axhline(lower_top, color='#e66a4e', linestyle='--', linewidth=1.2)
                ax.set_ylim(extent[2], extent[3])
            else:
                ax.set_xlim((lower_xmin - 1) * CELL_M, (lower_xmax + 2) * CELL_M)
                ax.set_ylim(lower_ymin * CELL_M, lower_top + 0.1)
            ax.grid(color='white', alpha=0.2, linewidth=0.4)
    for col, (label, stats) in enumerate((('Tilt ON', on), ('Tilt OFF', off))):
        axes[0, col].set_title(f'{label} | observed {stats["projected_coverage_m2"]:.2f} m² | {stats["depth_frames"]} depth frames')
        axes[1, col].set_title(f'Below {lower_top:.2f} m | area {stats["lower_coverage_m2"]:.2f} m² | ≥3 frames {stats["lower_robust_coverage_m2"]:.2f} m²')
    axes[0, 2].set_title('Coverage difference | all heights')
    axes[1, 2].set_title('Coverage difference | lower region')
    axes[0, 2].legend(handles=[Patch(color='#159895', label='ON only'),
                               Patch(color='#9aa4ac', label='Both'),
                               Patch(color='#e66a4e', label='OFF only')],
                      loc='upper right', fontsize=9)
    colorbar = fig.colorbar(axes[0, 0].images[0], ax=axes[:, :2].ravel().tolist(), shrink=0.8)
    colorbar.set_label('Distinct depth frames per 10 cm projected cell (log scale)')
    fig.suptitle(f'Surface observations: {Path(on["folder"]).name} vs {Path(off["folder"]).name}', fontsize=13)
    fig.text(0.5, 0.01, 'All observed cells are shown. Coverage and repeat observations are not ground-truth surface accuracy.',
             ha='center', fontsize=9)
    fig.savefig(output, dpi=180)
    if show:
        if os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'):
            plt.show()
        else:
            print('No display available; PNG saved without opening a window.')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description='Compare outdoor tilt-on and tilt-off depth coverage')
    parser.add_argument('on_log', nargs='?', help='flight_*_on folder (default: latest pair)')
    parser.add_argument('off_log', nargs='?', help='flight_*_off folder (default: latest pair)')
    parser.add_argument('--output', type=Path, help='JSON report path')
    parser.add_argument('--plot', type=Path, help='PNG comparison path')
    parser.add_argument('--show', action='store_true', help='open the comparison figure after saving')
    args = parser.parse_args()
    if (args.on_log is None) != (args.off_log is None):
        parser.error('Give both ON and OFF folders, or neither')
    if args.on_log:
        folders = [locate(args.on_log), locate(args.off_log)]
    else:
        candidates = [[p for root in ROOTS for p in root.glob(f'flight_*_{mode}')
                       if (p / 'events.jsonl').is_file()] for mode in ('on', 'off')]
        if not all(candidates):
            parser.error('No recorded ON/OFF pair found; pass both folder names')
        latest_on = max(candidates[0], key=lambda p: p.stat().st_mtime)
        closest_off = min(candidates[1], key=lambda p: abs(p.stat().st_mtime - latest_on.stat().st_mtime))
        if abs(closest_off.stat().st_mtime - latest_on.stat().st_mtime) > 900:
            parser.error('Latest ON/OFF recordings are over 15 minutes apart; pass both folder names')
        folders = [latest_on, closest_off]
    if any(not path.is_dir() for path in folders):
        parser.error('Both inputs must be flight_* folders')
    for path, suffix in zip(folders, ('_on', '_off')):
        if not path.name.endswith(suffix):
            parser.error(f'{path.name} must end in {suffix}')
    on, on_voxels, on_hits, reference = measure(folders[0])
    off, off_voxels, off_hits, _ = measure(folders[1], reference)
    lower_top = reference[2]
    lower_cells = lambda cells: {cell for cell in cells if (cell[1] + 0.5) * CELL_M <= lower_top}
    on_lower, off_lower = lower_cells(on_hits), lower_cells(off_hits)
    union = on_voxels | off_voxels
    report = {
        'voxel_size_m': VOXEL_M, 'projected_cell_m': CELL_M, 'sample_step': SAMPLE_STEP,
        'depth_range_m': [DEPTH_MIN_M, DEPTH_MAX_M], 'roi_radius_m': ROI_RADIUS_M,
        'lower_region_top_world_m': round(lower_top, 3), 'robust_cell_min_frames': ROBUST_CELL_FRAMES,
        'comparison_note': 'ON and OFF have different height trajectories and durations; tilt alone is not isolated.',
        'note': 'Projected coverage and observation density are not ground-truth surface accuracy.',
        'on': on, 'off': off,
        'lower_on_only_m2': round(len(on_lower - off_lower) * CELL_M * CELL_M, 3),
        'lower_off_only_m2': round(len(off_lower - on_lower) * CELL_M * CELL_M, 3),
        'lower_shared_m2': round(len(on_lower & off_lower) * CELL_M * CELL_M, 3),
        'overlap_10cm_jaccard_pct': round(100 * len(on_voxels & off_voxels) / len(union), 2) if union else 0.0,
    }
    output = args.output or folders[0].parent / ('avp_valid_' + datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%SZ') + '.json')
    plot = args.plot or output.with_suffix('.png')
    output.parent.mkdir(parents=True, exist_ok=True)
    plot.parent.mkdir(parents=True, exist_ok=True)
    render_comparison(on_hits, off_hits, lower_top, on, off, plot, args.show)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    for label, stats in (('ON ', on), ('OFF', off)):
        print(f'{label}: frames={stats["depth_frames"]}, voxels={stats["occupied_voxels_5cm"]}, '
              f'coverage={stats["projected_coverage_m2"]:.2f} m2, lower={stats["lower_coverage_m2"]:.2f} m2, '
              f'lower robust={stats["lower_robust_coverage_m2"]:.2f} m2')
    print(f'Lower region (z <= {lower_top:.2f} m): ON only={report["lower_on_only_m2"]:.2f} m2, '
          f'OFF only={report["lower_off_only_m2"]:.2f} m2, shared={report["lower_shared_m2"]:.2f} m2')
    print(f'Overlap (10 cm voxel Jaccard): {report["overlap_10cm_jaccard_pct"]:.1f}%')
    print(f'Report: {output}')
    print(f'Figure: {plot}')


if __name__ == '__main__':
    main()
