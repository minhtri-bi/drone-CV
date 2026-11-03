"""Plot saved real-bag outputs without ROS, regrading or changing the raw recording."""

import argparse
import json
from pathlib import Path

import numpy as np
from matplotlib.patches import Rectangle

from plots import INK, SERIES, STATUS_COLOR, STATUS_LABEL, Status, plt


def plot_replay(recording, replay, output):
    output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((recording / "manifest.json").read_text())
    fixture = json.loads((recording / "fixture.json").read_text())
    report = json.loads((replay / "report.json").read_text())
    with np.load(replay / "preview.npz", allow_pickle=False) as saved:
        points = saved["points_world"]
        samples, status = saved["samples"], saved["status"]
    alignment = np.array(manifest["world_from_odom"])
    world_samples = samples @ alignment[:3, :3].T + alignment[:3, 3]
    visible = ((points[:, 0] >= -.5) & (points[:, 0] <= 6)
               & (np.abs(points[:, 1]) <= 2.5) & (points[:, 2] <= 1.2))
    points = points[visible]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
    for ax, vertical, label in zip(axes, (1, 2), ("World y (m)", "World z (m)")):
        ax.scatter(points[:, 0], points[:, vertical], c=points[:, 2], cmap="viridis",
                   vmin=0, vmax=1, s=2, rasterized=True)
        for target, color in zip(fixture["targets"], SERIES):
            lo, hi = np.array(target["lo"]), np.array(target["hi"])
            ax.add_patch(Rectangle((lo[0], lo[vertical]), hi[0]-lo[0], hi[vertical]-lo[vertical],
                                   fill=False, edgecolor=color, linewidth=2, label=target["name"]+" ROI"))
        ax.set(xlabel="World x (m)", ylabel=label, xlim=(-.5, 6))
        ax.grid(alpha=.3)
        ax.legend(loc="upper right", fontsize=8)
    axes[0].set_ylim(-2.5, 2.5)
    axes[1].set_ylim(-.1, 1.2)
    fig.suptitle(f"Real Gazebo depth / independent declared targets: {report['accepted_frames']} paired frames")
    fig.text(.02, .015, "Last accepted frame; finite points subsampled and display-cropped only. All target/frame errors remain in report.json.")
    fig.tight_layout(rect=(0, .04, 1, .94))
    fig.savefig(output / "pointcloud_targets.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
    mid_z = np.unique(samples[:, 2])[len(np.unique(samples[:, 2]))//2]
    middle = np.isclose(samples[:, 2], mid_z)
    for ax, vertical, label, selected in zip(axes, (1, 2), ("World y (m)", "World z (m)"),
                                            (middle, np.ones(len(samples), dtype=bool))):
        for state in (Status.UNSEEN, Status.OCCLUDED, Status.FREE, Status.OBSTACLE):
            mask = selected & (status == state)
            ax.scatter(world_samples[mask, 0], world_samples[mask, vertical], s=6,
                       color=STATUS_COLOR[state], label=STATUS_LABEL[state], alpha=.65)
        ax.set(xlabel="World x (m)", ylabel=label)
        ax.grid(alpha=.3)
    axes[0].set_aspect("equal")
    axes[1].axhline(0, color=INK, linewidth=1, label="Physical floor")
    axes[1].legend(loc="upper right", fontsize=7)
    axes[0].set_title("Middle-height slice of sampled envelope")
    axes[1].set_title("All heights: floor inside margin-inflated envelope")
    gate = report["gates"][-1]
    fig.suptitle(f"Stationary query: STOP / {gate['reason']}; UNSEEN={gate['counts']['UNSEEN']}")
    fig.text(.02, .015, "5 cm samples, not a continuous-volume proof. Near-floor returns and a rear blind zone do not authorize movement.")
    fig.tight_layout(rect=(0, .04, 1, .94))
    fig.savefig(output / "sampled_envelope.png", dpi=150)
    plt.close(fig)
    print(json.dumps({"output": str(output), "accepted_frames": report["accepted_frames"], "allowed": gate["allowed"]}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", type=Path, required=True)
    parser.add_argument("--replay", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    plot_replay(args.recording, args.replay, args.output)


if __name__ == "__main__":
    main()
