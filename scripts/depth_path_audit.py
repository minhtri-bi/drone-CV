"""Read-only audit of declared matched stages or a historical stored receive trace."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from drone_cv.depth_path_audit import audit, historical_recording


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--recording", type=Path)
    source.add_argument("--contract", type=Path)
    parser.add_argument("--events", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("refuse to overwrite previous audit")
    if args.recording:
        manifest = json.loads((args.recording/"manifest.json").read_text(encoding="utf-8"))
        rows = [json.loads(line) for line in (args.recording/"receive_progress.jsonl").read_text(encoding="utf-8").splitlines()]
        report = historical_recording(rows, manifest)
    else:
        if args.events is None:
            parser.error("--contract requires --events JSONL")
        rows = [json.loads(line) for line in args.events.read_text(encoding="utf-8").splitlines()]
        report = audit(json.loads(args.contract.read_text(encoding="utf-8")), rows)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(report, output, indent=2, allow_nan=False)
        output.write("\n")
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
