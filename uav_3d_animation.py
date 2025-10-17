import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation, PillowWriter

def animate_uavs_3d(uavs, area_size, save_path='results/uav_3d_animation.gif'):

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Prepare lines and points for each UAV
    lines = []
    points = []
    z_offsets = np.arange(len(uavs)) * 5  # Each UAV slightly above the previous
    for idx, u in enumerate(uavs):
        line, = ax.plot([], [], [], label=f'UAV {u.uid}')
        point, = ax.plot([], [], [], marker='o', markersize=5, color=line.get_color())
        lines.append(line)
        points.append(point)

    # Set plot limits
    ax.set_xlim(0, area_size)
    ax.set_ylim(0, area_size)
    ax.set_zlim(0, len(uavs)*5 + 10)  # accommodate stacked UAVs
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Altitude (m)')
    ax.set_title('UAV Paths 3D Animation')
    ax.legend()

    # Find max number of steps among all UAVs
    max_steps = max(len(u.path) for u in uavs)

    def update(frame):
        for idx, u in enumerate(uavs):
            path = np.array(u.path)
            z = np.full(len(path), z_offsets[idx])  # assign UAV-specific z offset
            if frame < len(path):
                lines[idx].set_data(path[:frame, 0], path[:frame, 1])
                lines[idx].set_3d_properties(z[:frame])
                points[idx].set_data(path[frame-1, 0:1], path[frame-1, 1:2])
                points[idx].set_3d_properties([z[frame-1]])
        return lines + points

    anim = FuncAnimation(fig, update, frames=max_steps, interval=50, blit=True)
    
    # Save as GIF
    anim.save(save_path, writer=PillowWriter(fps=20))
    plt.show()
