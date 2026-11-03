"""Run synthetic depth through the actual G0 API; no ROS, GPU or matplotlib."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

import numpy as np

# Support a clean checkout without installing into the user's global Python.
sys.path = [str(Path(__file__).resolve().parents[1] / "src"), *sys.path]

from drone_cv.braking import BrakingLimits, MotionState, Proposal, decide_velocity
from drone_cv.depth import DepthLimits, normalize_depth
from drone_cv.frames import OPTICAL_TO_FLU
from drone_cv.histogram import HistogramConfig, LocalClearance, build_histogram
from drone_cv.projection import Camera, Pose, project_depth
from drone_cv.timing import Clock, FreshnessPolicy, Stamp


def preview_svg(points: np.ndarray, histograms: dict) -> str:
    """Actual sampled point cloud plus 72-bin free-path bars; raw data also saved."""
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 760" '
           'style="--fg: #64748b; color: var(--fg)">',
           '<g fill="currentColor" font-family="sans-serif" font-size="14">',
           '<text x="20" y="25">G0 synthetic scene, not a flight benchmark</text>',
           '<text x="20" y="55">Top view: ENU X forward (horizontal), Y left (vertical)</text>']
    # Keep every near-wire point even when downsampling the distant wall.
    subset = np.concatenate((points[::20], points[points[:, 0] < 3]))
    for x, y, _ in subset:
        svg.append(f'<circle cx="{80+x*100:.2f}" cy="{180-y*20:.2f}" r="1"/>')
    svg.append('<text x="20" y="310">Middle height band: free-path metres, directions 0..355 degrees</text>')
    for row, (label, histogram) in enumerate(histograms.items()):
        baseline = 425 + row*145
        svg.append(f'<text x="20" y="{baseline-95}">{label}</text>')
        for index, distance in enumerate(histogram.free_distance_m[:, 1]):
            svg.append(f'<rect x="{45+index*12}" y="{baseline-distance*18:.2f}" '
                       f'width="8" height="{distance*18:.2f}"/>')
        svg.append(f'<text x="45" y="{baseline+17}">0 deg</text>')
        svg.append(f'<text x="477" y="{baseline+17}">180 deg</text>')
    svg.append('</g></svg>')
    return "\n".join(svg)


def run(output: Path) -> dict:
    stamp = Stamp("g0-synthetic", 1, 1.0, 10.0)
    now, policy = Clock(stamp.epoch, 1.05, 10.05), FreshnessPolicy(0.3, 0.3)
    camera = Camera(121, 81, 60, 60, 60, 40)
    pose = Pose(OPTICAL_TO_FLU, (0, 0, 0), stamp)
    config = HistogramConfig(0.2, 0.15, 4, 0.25, True)
    limits = BrakingLimits(2.0, 0.3, 0.5, 3.0)
    rows = np.indices((camera.height, camera.width))[0]
    scenes = {
        "clear_wall": np.full(rows.shape, 6, np.float32),
        "new_thin_wire": np.where(rows == 40, 2, 6).astype(np.float32),
        "unknown_boot": np.full(rows.shape, np.nan, np.float32),
    }
    summary = {"source": "synthetic analytic fixture", "flight_evidence": False,
               "camera": asdict(camera), "footprint_config": asdict(config),
               "braking_limits": asdict(limits), "results": {}}
    histograms, clouds = {}, {}
    for label, raw in scenes.items():
        frame = normalize_depth(raw, "32FC1", DepthLimits(0.3, 10), stamp)
        # This sphere is analytically empty in the fixture, not camera-inferred.
        certificate = None if label == "unknown_boot" else LocalClearance((0, 0, 0), 1, stamp)
        histogram = build_histogram(frame, camera, pose, (0, 0, 0), now, policy, config,
                                    local_clearance=certificate, max_pose_skew_s=0.02)
        decision = decide_velocity(Proposal((5, 0, 0), 0, stamp),
                                   MotionState((0, 0, 0), (0, 0, 0), stamp),
                                   histogram, now, policy, limits)
        histograms[label] = histogram
        clouds[label] = project_depth(frame, camera, pose, now, policy, max_pose_skew_s=0.02)
        summary["results"][label] = {"forward_free_m": histogram.free_distance_m[0].tolist(),
                                     "decision": asdict(decision)}
    old = Proposal((5, 0, 0), 0, Stamp(stamp.epoch, 0, 0, 9))
    summary["stale_proposal"] = asdict(decide_velocity(
        old, MotionState((0, 0, 0), (0, 0, 0), stamp), histograms["clear_wall"],
        now, policy, limits))
    output.mkdir(parents=True, exist_ok=False)
    for label, histogram in histograms.items():
        np.savez_compressed(output / f"{label}.npz", points_enu=clouds[label],
                            free_distance_m=histogram.free_distance_m,
                            stop_reason=histogram.reason, depth_m=scenes[label])
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8")
    (output / "preview.svg").write_text(preview_svg(clouds["new_thin_wire"], histograms), encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True,
                        help="New output directory; existing output is never overwritten")
    args = parser.parse_args()
    try:
        summary = run(args.output)
    except (OSError, ValueError) as error:
        parser.exit(1, f"G0 demo failed: {error}\n")
    for label, result in summary["results"].items():
        print(f"{label}: free={result['forward_free_m']} m; "
              f"velocity={result['decision']['velocity_enu']}; {result['decision']['reason']}")
    print(f"stale proposal: {summary['stale_proposal']['reason']}")
    print(f"Artifacts: {args.output.resolve()}")


if __name__ == "__main__":
    main()
