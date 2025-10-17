# UAV Search Simulation – Operational Analysis Example
# WHAT THE FUCK
## Project Overview

This project simulates a multi-UAV search mission in a defined area. UAVs follow a lawnmower search pattern and attempt probabilistic detection of randomly placed targets. The simulation evaluates key operational metrics and measures of effectiveness:

- **Detection Probability** – fraction of targets detected during the mission.
- **Coverage Fraction** – fraction of the area effectively surveyed.
- **Average Detection Time** – time to detect targets.

The project produces statistical plots and a 3D animation of UAV paths.

---

## Project Structure

uav_simulation.py # Main simulation script
results/ # Folder for generated CSV data and plots
README.md # Project description and usage

---

## How to Run

### Clone the repository:

```bash
git clone <your-repo-link>
cd <your-repo-folder>
```

### Install dependencies
```bash
pip install -r requirements.txt
```

### Run the simulation
```bash
python uav_simulation.py
```

After running, the ```bash results/ ```folder will contain:
- datafarm_results.csv – simulation results for all runs

- detection_prob_vs_n_uavs.png – detection probability plot

- coverage_vs_n_uavs.png – coverage fraction plot

- detection_time_boxplot.png – detection time distribution plot

- uav_3d_animation.gif – 3D UAV paths animation

## Simulation Parameters

| Parameter        | Value       |
|-----------------|------------|
| AREA_SIZE        | 3000 m     |
| MISSION_TIME     | 900 s      |
| SENSOR_RANGE     | 80 m       |
| UAV_SPEED        | 10 m/s     |
| DETECTION_PROB   | 0.75       |
| TIMESTEP         | 2 s        |
| GRID_RESOLUTION  | 30 m       |
| N_TARGETS        | 30         |

Parameters can be modified directly in `uav_simulation.py` or via the `experiments` list.

---

## Output

### Plots and Measures of Effectiveness summary

- **Detection Probability vs Number of UAVs**  
  ![Detection Probability](results/detection_prob_vs_n_uavs.png)

- **Coverage Fraction vs Number of UAVs**  
  ![Coverage Fraction](results/coverage_vs_n_uavs.png)

- **Detection Time Distribution**  
  ![Detection Time](results/detection_time_boxplot.png)


## 3D Animation
uav_3d_animation.gif shows UAV paths over the mission area. Each UAV moves along a lawnmower pattern; the animation shows movement over time.

## Example Metrics Table

| n_uavs | Detection_Prob | Coverage_Fraction | Avg_Detect_Time (s) | Std_Detect_Time (s) |
|--------|----------------|-----------------|-------------------|-------------------|
| 1      | 0.14           | 0.12            | 272.3             | 178.2             |
| 2      | 0.29           | 0.28            | 263.2             | 120.6             |
| 3      | 0.43           | 0.37            | 416.8             | 110.4             |
| 4      | 0.42           | 0.53            | 300.7             | 79.3              |
| 6      | 0.91           | 0.87            | 344.8             | 84.4              |
| 8      | 0.93           | 0.89            | 278.2             | 70.0              |