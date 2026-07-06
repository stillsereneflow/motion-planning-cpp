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


def render(artifact_path: Path, out_dir: Path) -> Path:
    data = json.loads(artifact_path.read_text())
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
