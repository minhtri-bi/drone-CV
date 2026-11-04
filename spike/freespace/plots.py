"""Figures for the spike: scenario top views and the blind-zone experiment.

Usage: python plots.py --results out/results.json --output out/
"""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Patch, Rectangle

from camera import CameraModel, Intrinsics, Pose
from capture import capture, climb, straight, yaw_scan
from evidence import EvidenceSource, Envelope, SensorProfile, Status, classify_points, decide, swept_envelope
from scene import Box, Cylinder, Scene

INK, INK2, SURFACE = "#0b0b0b", "#52514e", "#fcfcfb"
STATUS_COLOR = {Status.FREE: "#0ca30c", Status.OBSTACLE: "#d03b3b",
                Status.OCCLUDED: "#fab219", Status.UNSEEN: "#b9b8b2"}
STATUS_LABEL = {Status.FREE: "FREE: observed free", Status.OBSTACLE: "OBSTACLE: observed return",
                Status.OCCLUDED: "UNSUPPORTED: insufficient evidence",
                Status.UNSEEN: "UNSEEN: outside retained views"}
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]

CAM = CameraModel(Intrinsics.pinned_x500_depth())
PROFILE = SensorProfile(0., 0., EvidenceSource.ANALYTIC_SYNTHETIC,
                        "Exact analytic plotting fixture poses; not operational calibration", True)
WALLS = (Box((-4, 2.5, 0), (12, 2.7, 3)), Box((-4, -2.7, 0), (12, -2.5, 3)))
A, B = Pose(0.0, 0.0, 1.2), Pose(2.0, 0.0, 1.2)

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.edgecolor": INK2,
                     "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
                     "figure.facecolor": SURFACE, "axes.facecolor": SURFACE})


def draw_world(ax, scene: Scene, env: Envelope) -> None:
    for box in scene.all_boxes():
        in_band = not (box.hi[2] < env.z_lo or box.lo[2] > env.z_hi)
        ax.add_patch(Rectangle(box.lo[:2], box.hi[0] - box.lo[0], box.hi[1] - box.lo[1],
                               facecolor=INK2 if in_band else "none", edgecolor=INK2,
                               lw=0.8, hatch=None if in_band else "///", zorder=3))
    for c in scene.cylinders:
        ax.add_patch(Circle((c.cx, c.cy), max(c.radius, 0.03), facecolor=INK2, edgecolor=INK2, zorder=3))


def draw_envelope(ax, env: Envelope) -> None:
    a, b, r = np.array(env.start_xy), np.array(env.end_xy), env.radius_m
    t = np.linspace(0, 2 * np.pi, 200)
    direction = (b - a) / max(np.linalg.norm(b - a), 1e-12)
    normal = np.array([-direction[1], direction[0]])
    pts = []
    for theta in np.linspace(np.pi / 2, 3 * np.pi / 2, 60):       # rear cap
        pts.append(a + r * (np.cos(theta) * direction + np.sin(theta) * normal))
    for theta in np.linspace(-np.pi / 2, np.pi / 2, 60):           # front cap
        pts.append(b + r * (np.cos(theta) * direction + np.sin(theta) * normal))
    pts = np.array(pts + [pts[0]])
    ax.plot(pts[:, 0], pts[:, 1], color=INK, lw=1.4, zorder=5)
    ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="->", color=INK, lw=1.2), zorder=6)


def coverage_background(ax, store, env, now, extent):
    xs = np.arange(extent[0], extent[1], 0.05)
    ys = np.arange(extent[2], extent[3], 0.05)
    gx, gy = np.meshgrid(xs, ys)
    pts = np.column_stack((gx.ravel(), gy.ravel(), np.full(gx.size, B.z)))
    frames = store.retained("run", now, 60.0)
    status = classify_points(pts, frames, store.camera, store.profile).reshape(gx.shape)
    ax.contourf(gx, gy, (status == Status.FREE).astype(float), levels=[0.5, 1.5],
                colors=[STATUS_COLOR[Status.FREE]], alpha=0.12, zorder=1)


def panel(ax, title, scene, poses, env, *, extra=None, extent=(-0.6, 4.4, -2.0, 2.0)):
    store, t = capture(scene, CAM, PROFILE, poses)
    if extra is not None:
        store, t = capture(extra, CAM, PROFILE, [poses[-1]], store=store, t0=t + 0.1)
        scene = extra
    decision, samples, status = decide(store, env, epoch="run", now_s=t, ttl_s=60.0)
    coverage_background(ax, store, env, t, extent)
    mid = np.abs(samples[:, 2] - B.z) < 0.026
    for s in (Status.UNSEEN, Status.OCCLUDED, Status.FREE, Status.OBSTACLE):
        m = mid & (status == s)
        ax.scatter(samples[m, 0], samples[m, 1], s=4, color=STATUS_COLOR[s], lw=0, zorder=4)
    draw_world(ax, scene, env)
    draw_envelope(ax, env)
    cams = np.array([[p.x, p.y] for p in poses])
    ax.plot(cams[:, 0], cams[:, 1], "o", ms=3, color=SERIES[0], zorder=6)
    verdict = "ALLOW" if decision.allowed else "STOP"
    ax.set_title(f"{title}\n{verdict} — {decision.reason}", fontsize=9, color=INK, loc="left")
    ax.set_xlim(extent[0], extent[1]); ax.set_ylim(extent[2], extent[3])
    ax.set_aspect("equal"); ax.grid(color="#e6e5e0", lw=0.5, zorder=0)
    return decision


def legend_handles():
    handles = [Patch(color=STATUS_COLOR[s], label=STATUS_LABEL[s]) for s in STATUS_COLOR]
    handles += [Patch(facecolor=STATUS_COLOR[Status.FREE], alpha=0.12, label="Free sample locations (z = 1.2 m slice)"),
                Patch(facecolor=INK2, label="Obstacle intersecting the height band"),
                plt.Line2D([], [], color=INK, lw=1.4, label="Swept envelope (1 m segment + footprint)"),
                plt.Line2D([], [], marker="o", ls="", color=SERIES[0], label="Frame capture positions")]
    return handles


def fig_scenarios(out: Path) -> dict:
    clear = Scene(walls=WALLS)
    env = swept_envelope(B, 1.0)
    approach = straight(A, B)
    cases = [
        ("1. Clear corridor, 2 m approach", clear, approach, None, env),
        ("2. Same position, no history", clear, [B], None, env),
        ("3. Box across the path", Scene(walls=WALLS, boxes=(Box((2.9, -0.3, 0), (3.2, 0.3, 2.0)),)), approach, None, env),
        ("4. A 5 cm pole in the envelope", Scene(walls=WALLS, cylinders=(Cylinder(3.0, 0.0, 0.025, 0, 3),)), approach, None, env),
        ("5. New obstacle after clear history", clear, approach,
         Scene(walls=WALLS, boxes=(Box((2.9, -0.3, 0), (3.2, 0.3, 2.0)),)), env),
        ("6. Sideways into unseen space", clear, approach, None,
         Envelope((B.x, B.y), (B.x, B.y + 1.5), env.radius_m, env.z_lo, env.z_hi)),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(13, 8.6))
    results = {}
    for ax, (title, scene, poses, extra, e) in zip(axes.ravel(), cases):
        d = panel(ax, title, scene, poses, e, extra=extra)
        results[title] = {"allowed": d.allowed, "reason": d.reason, "counts": d.counts}
    fig.legend(handles=legend_handles(), loc="lower center", ncol=4, frameon=False, fontsize=8)
    fig.suptitle("Sampled free-space gating: allow supported samples, stop for obstacles or missing evidence",
                 x=0.01, ha="left", fontsize=11, color=INK, fontweight="bold")
    fig.text(0.01, 0.935, "Top view; colored points are envelope samples at z = 1.2 m. Synthetic 73 x 58 degree camera with the X500 mount.",
             fontsize=8.5, color=INK2)
    fig.subplots_adjust(left=0.04, right=0.99, top=0.85, bottom=0.11, hspace=0.42, wspace=0.12)
    fig.savefig(out / "fig_scenarios.png", dpi=150)
    plt.close(fig)
    return results


def fig_blind_topview(out: Path) -> None:
    clear = Scene(walls=WALLS)
    env = swept_envelope(B, 1.0)
    plans = [("Climb at B", climb(B.x, B.y, 0.15, B.z)),
             ("Climb, then scan 360 degrees at B", climb(B.x, B.y, 0.15, B.z) + yaw_scan(B.x, B.y, B.z)),
             ("Climb at A', then approach B by 1 m", climb(B.x - 1.0, 0, 0.15, B.z)[:-1] + straight(Pose(B.x - 1.0, 0, B.z), B))]
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.4))
    for ax, (title, poses) in zip(axes, plans):
        panel(ax, title, clear, poses, env, extent=(0.4, 3.9, -1.4, 1.4))
    fig.legend(handles=legend_handles()[:4], loc="lower center", ncol=4, frameon=False, fontsize=8)
    fig.suptitle("Takeoff blind zone: an approach history supports the rear samples in this fixture",
                 x=0.01, ha="left", fontsize=11, color=INK, fontweight="bold")
    fig.tight_layout(rect=(0, 0.09, 1, 0.93))
    fig.savefig(out / "fig_blindzone_topview.png", dpi=150)
    plt.close(fig)


def fig_blind_chart(results: dict, out: Path) -> None:
    zone = results["blind_zone"]
    names = {"pinned_73deg": "Pinned 73 degree camera", "wide_110deg": "Wide 110 degree FOV",
             "pinned_pitch20": "73 degrees, pitched down 20 degrees"}
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 4.3), gridspec_kw={"width_ratios": [2.2, 1]})
    for i, (key, label) in enumerate(names.items()):
        rows = sorted((float(k.split("_")[1][:-1]), v) for k, v in zone[key].items() if k.startswith("approach_"))
        xs = [r[0] for r in rows]
        ys = [100 * r[1]["free_fraction"] for r in rows]
        ax.plot(xs, ys, color=SERIES[i], lw=2, marker="o", ms=5, label=label, zorder=3)
    level = sorted((float(k.split("_")[1][:-1]), v) for k, v in zone["pinned_73deg"].items()
                   if k.startswith("level_"))
    ax.plot([r[0] for r in level], [100 * r[1]["free_fraction"] for r in level], color=SERIES[0],
            lw=2, ls="--", marker="o", ms=5, mfc=SURFACE, label="Pinned camera, level approach only",
            zorder=3)
    ax.axhline(100, color=INK2, lw=0.8, ls="--", zorder=1)
    ax.text(3.0, 100.6, "100% of samples supported", ha="right", va="bottom", fontsize=8, color=INK2)

    ax.set_xlabel("Approach distance from A' to B (m); solid lines include climb history")
    ax.set_ylabel("Envelope samples observed free (%)")
    ax.set_ylim(50, 103); ax.grid(axis="y", color="#e6e5e0", lw=0.5)
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(frameon=False, fontsize=8, loc="lower right")
    ax.set_title("Climb history reduces the required approach in this synthetic fixture", loc="left", fontsize=10, color=INK)

    strategies = [("climb_only", "Climb only"), ("climb_then_yaw_scan", "Climb + 360 degree scan")]
    width = 0.26
    for i, key in enumerate(names):
        vals = [100 * zone[key][s]["free_fraction"] for s, _ in strategies]
        pos = np.arange(len(strategies)) + (i - 1) * (width + 0.02)
        bars = ax2.bar(pos, vals, width=width, color=SERIES[i], label=names[key], zorder=3)
        for bar, val in zip(bars, vals):
            ax2.text(bar.get_x() + bar.get_width() / 2, val + 1, f"{val:.0f}", ha="center", fontsize=7.5, color=INK2)
    ax2.axhline(100, color=INK2, lw=0.8, ls="--")
    ax2.set_xticks(range(len(strategies)), [s[1] for s in strategies])
    ax2.set_ylim(0, 105); ax2.grid(axis="y", color="#e6e5e0", lw=0.5, zorder=0)
    ax2.spines[["top", "right"]].set_visible(False)
    ax2.set_title("Stationary views are incomplete", loc="left", fontsize=10, color=INK)
    fig.tight_layout()
    fig.savefig(out / "fig_blindzone_chart.png", dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    scenario_results = fig_scenarios(args.output)
    fig_blind_topview(args.output)
    fig_blind_chart(json.loads(args.results.read_text(encoding="utf-8")), args.output)
    (args.output / "scenarios.json").write_text(json.dumps(scenario_results, indent=2, ensure_ascii=False),
                                                encoding="utf-8")
    for title, r in scenario_results.items():
        print(f"{title}: {'ALLOW' if r['allowed'] else 'STOP'} {r['reason']}")


if __name__ == "__main__":
    main()
