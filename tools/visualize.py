#!/usr/bin/env python3
"""Render plan_demo JSON artifacts to PNG images.

Usage:
    python tools/visualize.py out/*.json --out-dir docs/images
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def vehicle_corners(pose, vehicle):
    """World-frame corners of the vehicle body at (x, y, theta)."""
    x, y, theta = pose[0], pose[1], pose[2]
    lx_min = -vehicle["rear_overhang"]
    lx_max = vehicle["wheelbase"] + vehicle["front_overhang"]
    ly = 0.5 * vehicle["width"]
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    local = [(lx_min, -ly), (lx_max, -ly), (lx_max, ly), (lx_min, ly)]
    return [(x + lx * cos_t - lyy * sin_t, y + lx * sin_t + lyy * cos_t) for lx, lyy in local]


def render_hybrid(data, out_dir: Path) -> Path:
    width, height, res = data["width"], data["height"], data["resolution"]
    ox, oy = data["origin"]
    extent = (ox, ox + width * res, oy, oy + height * res)

    grid = np.zeros((height, width))
    for x, y in data["obstacles"]:
        grid[y, x] = 1.0

    fig, ax = plt.subplots(figsize=(10, 10 * height / width))
    ax.imshow(grid, cmap="Greys", origin="lower", interpolation="none",
              extent=extent, vmax=1.4)

    poses = data["poses"]
    # Split the path into runs of constant driving direction.
    runs, labelled = [], set()
    for pose in poses:
        if runs and runs[-1][1] == bool(pose[3]):
            runs[-1][0].append(pose)
        else:
            if runs:
                runs[-1][0].append(pose)  # share the cusp point with the next run
            runs.append(([pose], bool(pose[3])))
    for run, reverse in runs:
        arr = np.array(run)
        color = "tab:orange" if reverse else "tab:blue"
        label = "reverse" if reverse else "forward"
        ax.plot(arr[:, 0], arr[:, 1], color=color, linewidth=2.0,
                label=None if label in labelled else label)
        labelled.add(label)

    # Vehicle outlines along the maneuver.
    vehicle = data["vehicle"]
    stride = max(1, len(poses) // 14)
    for pose in poses[::stride]:
        ax.add_patch(plt.Polygon(vehicle_corners(pose, vehicle), closed=True, fill=False,
                                 edgecolor="grey", linewidth=0.7, alpha=0.6))
    for pose, color in ((data["start"], "tab:green"), (data["goal"], "tab:red")):
        ax.add_patch(plt.Polygon(vehicle_corners(pose, vehicle), closed=True, fill=False,
                                 edgecolor=color, linewidth=2.0))
        ax.arrow(pose[0], pose[1], 0.9 * np.cos(pose[2]), 0.9 * np.sin(pose[2]),
                 head_width=0.25, color=color)

    stats = data["stats"]
    ax.set_title(
        f"{data['name']}  |  cost {stats['path_cost']:.2f}  |  "
        f"{stats['gear_switches']} gear switches  |  expanded {stats['nodes_expanded']}  |  "
        f"{stats['runtime_ms']:.0f} ms"
    )
    ax.legend(loc="upper right")
    ax.set_xlim(extent[0], extent[1])
    ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal")

    out_path = out_dir / f"{data['name']}.png"
    fig.savefig(out_path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    return out_path


def render(artifact_path: Path, out_dir: Path) -> Path:
    data = json.loads(artifact_path.read_text())
    if data.get("type") == "hybrid":
        return render_hybrid(data, out_dir)
    width, height = data["width"], data["height"]

    grid = np.zeros((height, width))
    for x, y in data["obstacles"]:
        grid[y, x] = 1.0

    fig, ax = plt.subplots(figsize=(9, 9 * height / width))
    ax.imshow(
        grid,
        cmap="Greys",
        origin="lower",
        interpolation="none",
        extent=(-0.5, width - 0.5, -0.5, height - 0.5),
        vmax=1.4,  # obstacles render dark grey instead of pure black
    )

    if data["path"]:
        path = np.array(data["path"])
        ax.plot(path[:, 0], path[:, 1], color="tab:blue", linewidth=2.2, label="path")

    ax.plot(*data["start"], marker="o", color="tab:green", markersize=10,
            linestyle="none", label="start")
    ax.plot(*data["goal"], marker="*", color="tab:red", markersize=16,
            linestyle="none", label="goal")

    stats = data["stats"]
    ax.set_title(
        f"{data['name']}  |  cost {stats['path_cost']:.2f}  |  "
        f"expanded {stats['nodes_expanded']}  |  {stats['runtime_ms']:.2f} ms"
    )
    ax.legend(loc="upper right")
    ax.set_xlim(-0.5, width - 0.5)
    ax.set_ylim(-0.5, height - 0.5)
    ax.set_aspect("equal")

    out_path = out_dir / f"{data['name']}.png"
    fig.savefig(out_path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", nargs="+", type=Path, help="JSON files from plan_demo")
    parser.add_argument("--out-dir", type=Path, default=Path("docs/images"))
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    for artifact in args.artifacts:
        print(f"wrote {render(artifact, args.out_dir)}")


if __name__ == "__main__":
    main()
