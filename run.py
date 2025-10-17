def run_single_simulation(seed, n_uavs=3, n_targets=30, sensor_range=SENSOR_RANGE,
                          speed=UAV_SPEED, mission_time=MISSION_TIME, area_size=AREA_SIZE,
                          timestep=TIMESTEP):
    rng = np.random.RandomState(seed)
    
    # Targets
    targets = pd.DataFrame({
        'tx': rng.uniform(0, area_size, n_targets),
        'ty': rng.uniform(0, area_size, n_targets),
        'detected': False,
        'detect_time': np.nan
    })

    # UAVs
    uavs = []
    for i in range(n_uavs):
        start_pos = (rng.uniform(0, 50), rng.uniform(0, area_size))
        uavs.append(UAV(i, start_pos, speed=speed, sensor_range=sensor_range))

    # Waypoints
    spacing = sensor_range
    waypoints = generate_lawnmower_waypoints(area_size, spacing)
    waypoint_indices = np.arange(n_uavs) % len(waypoints)

    steps = int(np.ceil(mission_time / timestep))

    for _ in range(steps):
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
                    targets.loc[indices[detected_now], 'detect_time'] = _ * timestep

    detection_prob = targets['detected'].mean()
    avg_detect_time = targets.loc[targets['detected'], 'detect_time'].mean()
    if np.isnan(avg_detect_time):
        avg_detect_time = None

    # Coverage fraction
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

    return {
        'detection_prob': detection_prob,
        'avg_detect_time': avg_detect_time,
        'coverage_fraction': coverage_fraction
    }