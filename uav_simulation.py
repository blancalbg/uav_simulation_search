# uav_simulation.py
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os
from uav_3d_animation import animate_uavs_3d

sns.set(style="whitegrid")

# ---------- PARAMETERS ----------
AREA_SIZE = 3000        # meters
MISSION_TIME = 900     # seconds
SENSOR_RANGE = 80      # meters
UAV_SPEED = 10          # m/s
DETECTION_PROB = 0.75
TIMESTEP = 2.0          # seconds
GRID_RESOLUTION = 20    # meters
# --------------------------------

class UAV:
    def __init__(self, uid, pos, speed=UAV_SPEED, sensor_range=SENSOR_RANGE):
        self.uid = uid
        self.pos = np.array(pos, dtype=float)
        self.speed = speed
        self.range = sensor_range
        self.path = [self.pos.copy()]

    def step_toward(self, target, dt):
        direction = np.array(target) - self.pos
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            return
        move = min(self.speed * dt, dist)
        self.pos += direction / dist * move
        self.path.append(self.pos.copy())

def generate_lawnmower_waypoints(area_size, spacing):
    x_coords = np.arange(spacing/2, area_size, spacing)
    y_top = area_size
    waypoints = []
    for i, x in enumerate(x_coords):
        if i % 2 == 0:
            waypoints.append((x, 0))
            waypoints.append((x, y_top))
        else:
            waypoints.append((x, y_top))
            waypoints.append((x, 0))
    return waypoints

def run_single_simulation(seed, n_uavs=3, n_targets=30, sensor_range=SENSOR_RANGE,
                          speed=UAV_SPEED, mission_time=MISSION_TIME, area_size=AREA_SIZE,
                          timestep=TIMESTEP):
    rng = np.random.RandomState(seed)
    
    # --- Targets ---
    targets = pd.DataFrame({
        'tx': rng.uniform(0, area_size, n_targets),
        'ty': rng.uniform(0, area_size, n_targets),
        'detected': False,
        'detect_time': np.nan
    })

    # --- UAVs: realistic start near a base ---
    uavs = []
    base_x = 0  # assume launch from left edge
    base_y_range = 50  # small random offset along y

    for i in range(n_uavs):
        # stagger UAVs along the base
        start_y = base_y_range * i + rng.uniform(0, base_y_range)
        uavs.append(UAV(i, (base_x, start_y), speed=speed, sensor_range=sensor_range))

    # --- Waypoints: lawnmower with fan-out ---
    spacing = sensor_range
    waypoints = generate_lawnmower_waypoints(area_size, spacing)

    # Assign each UAV a starting waypoint along different strips
    waypoint_indices = [i * (len(waypoints) // n_uavs) for i in range(n_uavs)]

    steps = int(np.ceil(mission_time / timestep))

    for step in range(steps):
        for i, u in enumerate(uavs):
            target_idx = waypoint_indices[i]
            target = waypoints[target_idx]
            u.step_toward(target, timestep)
            if np.linalg.norm(u.pos - target) < 1.0:
                waypoint_indices[i] = (target_idx + 1) % len(waypoints)

        # Vectorized detection
        undetected_mask = ~targets['detected'].values
        if undetected_mask.any():
            target_pos = targets.loc[undetected_mask, ['tx','ty']].values
            for u in uavs:
                dists = np.linalg.norm(target_pos - u.pos, axis=1)
                inside_mask = dists <= u.range
                rand_vals = rng.rand(len(target_pos))
                detected_now = inside_mask & (rand_vals < DETECTION_PROB)
                if detected_now.any():
                    indices = np.flatnonzero(undetected_mask)
                    targets.loc[indices[detected_now], 'detected'] = True
                    targets.loc[indices[detected_now], 'detect_time'] = step * timestep

    # --- Metrics ---
    detection_prob = targets['detected'].mean()
    avg_detect_time = targets.loc[targets['detected'], 'detect_time'].mean()
    if np.isnan(avg_detect_time):
        avg_detect_time = None

    # --- Coverage fraction ---
    grid_n = int(np.ceil(area_size / GRID_RESOLUTION))
    visited = np.zeros((grid_n, grid_n), dtype=bool)
    r_cells = int(np.ceil(sensor_range / GRID_RESOLUTION))
    for u in uavs:
        path = np.array(u.path)
        gx = np.clip((path[:,0] // GRID_RESOLUTION).astype(int), 0, grid_n-1)
        gy = np.clip((path[:,1] // GRID_RESOLUTION).astype(int), 0, grid_n-1)
        for cx, cy in zip(gx, gy):
            x_min, x_max = max(0, cx - r_cells), min(grid_n, cx + r_cells + 1)
            y_min, y_max = max(0, cy - r_cells), min(grid_n, cy + r_cells + 1)
            x_grid, y_grid = np.meshgrid(np.arange(x_min, x_max), np.arange(y_min, y_max))
            mask = np.hypot(x_grid - cx, y_grid - cy) * GRID_RESOLUTION <= sensor_range
            visited[y_grid[mask], x_grid[mask]] = True
    coverage_fraction = visited.mean()

    # --- Return both metrics and UAV objects ---
    return {
        'detection_prob': detection_prob,
        'avg_detect_time': avg_detect_time,
        'coverage_fraction': coverage_fraction,
        'targets_df': targets,
        'uavs': uavs  # <- This is key for animation
    }

def data_farm(experiments, runs_per_setting=10, out_csv='results/datafarm_results.csv'):
    rows = []
    os.makedirs(os.path.dirname(out_csv), exist_ok=True)
    for e_idx, exp in enumerate(experiments):
        for r in range(runs_per_setting):
            seed = exp.get('seed_offset', 1000) + r
            res = run_single_simulation(seed=seed,
                                        n_uavs=exp['n_uavs'],
                                        sensor_range=exp['sensor_range'],
                                        speed=exp.get('speed', UAV_SPEED),
                                        n_targets=exp.get('n_targets', 20),
                                        mission_time=exp.get('mission_time', MISSION_TIME),
                                        area_size=exp.get('area_size', AREA_SIZE),
                                        timestep=TIMESTEP)
            rows.append({
                'exp_id': e_idx,
                'n_uavs': exp['n_uavs'],
                'sensor_range': exp['sensor_range'],
                'detection_prob': res['detection_prob'],
                'avg_detect_time': res['avg_detect_time'] if res['avg_detect_time'] is not None else np.nan,
                'coverage_fraction': res['coverage_fraction']
            })
    df = pd.DataFrame(rows)
    df.to_csv(out_csv, index=False)
    return df

def example_run_and_plots():
    experiments = [
        {'n_uavs': 1, 'sensor_range': 100.0},
        {'n_uavs': 2, 'sensor_range': 100.0},
        {'n_uavs': 3, 'sensor_range': 100.0},
        {'n_uavs': 4, 'sensor_range': 100.0},
        {'n_uavs': 6, 'sensor_range': 150.0},
        {'n_uavs': 8, 'sensor_range': 200.0}
    ]
    df = data_farm(experiments, runs_per_setting=5, out_csv='results/datafarm_results.csv')

    # --- Detection probability plot ---
    plt.figure(figsize=(8,5))
    sns.lineplot(data=df, x='n_uavs', y='detection_prob', estimator='mean', ci='sd', marker='o')
    plt.title('Detection Probability vs Number of UAVs')
    plt.ylabel('Detection Probability')
    plt.xlabel('Number of UAVs')
    plt.savefig('results/MoE_detection_prob_vs_n_uavs.png', dpi=150)
    plt.close()

    # --- Coverage fraction plot ---
    plt.figure(figsize=(8,5))
    sns.lineplot(data=df, x='n_uavs', y='coverage_fraction', estimator='mean', ci='sd', marker='o')
    plt.title('Coverage Fraction vs Number of UAVs')
    plt.ylabel('Coverage Fraction')
    plt.xlabel('Number of UAVs')
    plt.savefig('results/MoE_coverage_vs_n_uavs.png', dpi=150)
    plt.close()

    # --- Detection time boxplot ---
    plt.figure(figsize=(8,5))
    sns.boxplot(data=df, x='n_uavs', y='avg_detect_time')
    plt.title('Detection Time Distribution by Number of UAVs')
    plt.ylabel('Average Detection Time (s)')
    plt.xlabel('Number of UAVs')
    plt.savefig('results/MoE_detection_time_boxplot.png', dpi=150)
    plt.close()


    # --- Summary table by number of UAVs ---
    summary = df.groupby('n_uavs').agg(
        Detection_Prob=('detection_prob', 'mean'),
        Coverage_Fraction=('coverage_fraction', 'mean'),
        Avg_Detect_Time=('avg_detect_time', 'mean'),
        Std_Detect_Time=('avg_detect_time', 'std')
    ).reset_index()

    print("=== UAV Mission Summary (Measures of Effectiveness) ===")
    print(summary)


if __name__ == '__main__':
    example_run_and_plots()

    seed = 123
    res = run_single_simulation(seed=seed, n_uavs=4, n_targets=30)
    uavs = res.get('uavs')  # make sure run_single_simulation returns 'uavs'
    
    # Create the 3D animation
    animate_uavs_3d(uavs,area_size=AREA_SIZE)