"""Static-obstacle rendering shared by the preparation map and replay views."""

from itertools import product

import numpy as np


def draw_obstacles_2d(axis, config, *, compact_labels=False):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    for index, obstacle in enumerate(config.obstacles):
        center, half = np.array(obstacle.center), np.array(obstacle.half_extents)
        low, high = center - half, center + half
        label = "Static obstacles" if index == 0 else None
        if obstacle.kind == "cylinder":
            shape = plt.Circle(center[:2], half[0], color="dimgray", alpha=0.45, label=label)
            margin = plt.Circle(center[:2], half[0] + config.obstacle_clearance, fill=False, ec="darkorange", ls="--", alpha=0.7)
        else:
            shape = Rectangle(low[:2], *(2 * half[:2]), color="dimgray", alpha=0.45, label=label)
            margin = Rectangle(low[:2] - config.obstacle_clearance,
                               *(2 * (half[:2] + config.obstacle_clearance)),
                               fill=False, ec="darkorange", ls="--", alpha=0.7)
        axis.add_patch(shape)
        axis.add_patch(margin)
        height = f"\nz={low[2]:g}–{high[2]:g} m" if config.dimensions == 3 else ""
        caption = f'O{index}' if compact_labels else f"Obstacle {index}{height}"
        axis.annotate(caption, center[:2], ha="center", fontsize=8, color="black")


def draw_obstacles_3d(axis, config):
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    for index, obstacle in enumerate(config.obstacles):
        center, half = np.array(obstacle.center), np.array(obstacle.half_extents)
        low, high = center - half, center + half
        if obstacle.kind == "cylinder":
            theta = np.linspace(0, 2 * np.pi, 33)
            bottom = np.column_stack((center[0] + half[0] * np.cos(theta),
                                      center[1] + half[0] * np.sin(theta), np.full(len(theta), low[2])))
            top = bottom.copy()
            top[:, 2] = high[2]
            faces = [bottom, top] + [np.array([bottom[k], bottom[k + 1], top[k + 1], top[k]]) for k in range(len(theta) - 1)]
        else:
            vertices = np.array(list(product(*zip(low, high))))
            faces = [vertices[np.isclose(vertices[:, k], boundary)]
                     for k in range(3) for boundary in (low[k], high[k])]
            # Order each rectangle around its center instead of drawing a bow tie.
            faces = [face[[0, 1, 3, 2]] for face in faces]
        axis.add_collection3d(Poly3DCollection(faces, facecolors="gray", edgecolors="dimgray", alpha=0.22, linewidths=0.4))
        axis.text(center[0], center[1], high[2], f"Obstacle {index}", fontsize=8)
        axis.auto_scale_xyz([low[0], high[0]], [low[1], high[1]], [low[2], high[2]], had_data=True)
