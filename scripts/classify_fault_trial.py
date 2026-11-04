"""Classify exact frozen paired runtime observations offline; never run a trial.

Input manifest schema 1: protocols (list), profile/evaluation (file paths), trials
(list of {protocol_sha256, baseline, fault}), and source_artifacts ({sha256:path}).
Every observation artifact, blinding evidence and tracking evidence must be present
with matching original bytes. Protocols must have been frozen before execution;
this classifier cannot establish their historical freezing or observer authenticity.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from drone_cv.empirical_profile import digest
from drone_cv.fault_trial import FaultProtocol, Trace, classify_trial, summarize_trials


def _load(base, name):
    return json.loads((base / name).read_text(encoding="utf-8"))


def classify_manifest(path):
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if manifest["schema"] != 1:
        raise ValueError("UNSUPPORTED_FAULT_MANIFEST")
    base = path.parent
    profile = _load(base, manifest["profile"])
    evaluation = _load(base, manifest["evaluation"])
    protocols = [FaultProtocol.from_dict(p) for p in manifest["protocols"]]
    if len({p.sha256 for p in protocols}) != len(protocols):
        raise ValueError("DUPLICATE_PLANNED_PROTOCOL")
    trials = {trial["protocol_sha256"]: trial for trial in manifest["trials"]}
    if len(trials) != len(manifest["trials"]) or set(trials) - {p.sha256 for p in protocols}:
        raise ValueError("DUPLICATE_OR_UNPLANNED_TRIAL")
    artifacts = manifest["source_artifacts"]
    artifact_errors = []
    for expected, relative in artifacts.items():
        try:
            hasher = hashlib.sha256()
            with (base / relative).open("rb") as stream:
                while chunk := stream.read(1024 * 1024):
                    hasher.update(chunk)
            if hasher.hexdigest() != expected:
                artifact_errors.append("OBSERVATION_ARTIFACT_HASH_MISMATCH:" + relative)
        except OSError as error:
            artifact_errors.append("OBSERVATION_ARTIFACT_UNAVAILABLE:" + relative + ":" + str(error))
    required = {profile["protocol"].get(key) for key in ("blinding_evidence_sha256", "tracking_evidence_sha256")} - {"", None}
    results = []
    for protocol in protocols:
        if protocol.sha256 not in trials:
            # Planned but never attempted stays in denominator, never an exclusion.
            results.append({"scenario": protocol.scenario, "protocol_sha256": protocol.sha256,
                "test_validity": "INVALID_TEST", "fault_verdict": "NOT_EVALUATED",
                "invalid_reasons": ["PLANNED_TRIAL_NOT_RECORDED"], "failure_reasons": [],
                "safety_outcome": "UNKNOWN", "execution_outcome": "INFRA_ERROR",
                "runtime_acceptance_certified": False, "exit_code": 2})
            continue
        trial = trials[protocol.sha256]
        decoded, decode_errors = {}, []
        for side in ("baseline", "fault"):
            try:
                decoded[side] = Trace.from_dict(_load(base, trial[side]))
            except (OSError, ValueError, KeyError, TypeError) as error:
                decode_errors.append("ATTEMPTED_" + side.upper() + "_UNAVAILABLE_OR_MALFORMED:" + str(error))
        if decode_errors:
            unsafe = [r.__dict__ for trace in decoded.values() for r in trace.records if r.kind == "unsafe"]
            known_failure = any(trace.execution_outcome == "TASK_FAIL" for trace in decoded.values())
            result = {"scenario": protocol.scenario, "protocol_sha256": protocol.sha256,
                "test_validity": "INVALID_TEST", "fault_verdict": "NOT_EVALUATED",
                "invalid_reasons": [*decode_errors, *artifact_errors], "failure_reasons": [],
                "safety_outcome": "UNSAFE_OBSERVED" if unsafe else "UNKNOWN",
                "execution_outcome": "INFRA_ERROR", "unsafe_observations": unsafe,
                "decoded_trace_sha256": {side: trace.sha256 for side, trace in decoded.items()},
                "decoded_execution_outcomes": {side: trace.execution_outcome for side, trace in decoded.items()},
                "runtime_acceptance_certified": False, "exit_code": 1 if unsafe or known_failure else 2}
            results.append({**result, "classification_sha256": digest(result)})
            continue
        baseline, fault = decoded["baseline"], decoded["fault"]
        result = classify_trial(protocol, baseline, fault, profile, evaluation)
        postrun_errors = ["POSTRUN_" + side.upper() + "_TRACE_HASH_MISMATCH_OR_MISSING"
                          for side, trace in (("baseline", baseline), ("fault", fault))
                          if trial.get(side + "_sha256") != trace.sha256]
        missing = (required | {r.artifact_sha256 for trace in (baseline, fault) for r in trace.records}) - set(artifacts)
        if missing or artifact_errors or postrun_errors:
            result.update(test_validity="INVALID_TEST", fault_verdict="NOT_EVALUATED",
                invalid_reasons=[*result["invalid_reasons"], *artifact_errors, *postrun_errors,
                    *(["RETAINED_SOURCE_ARTIFACTS_MISSING:" + ",".join(sorted(missing))] if missing else [])],
                failure_reasons=[], safety_outcome="UNSAFE_OBSERVED" if result["unsafe_observations"] else "UNKNOWN")
            result["exit_code"] = 1 if result["unsafe_observations"] or "TASK_FAIL" in (result["execution_outcome"], result["baseline_execution_outcome"]) else 2
            result.pop("classification_sha256", None)
            result["classification_sha256"] = digest(result)
        results.append(result)
    summary = summarize_trials(results)
    return {"schema": 1, "manifest_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            **summary, "exit_code": 1 if any(r["exit_code"] == 1 for r in results)
            else 2 if any(r["exit_code"] == 2 for r in results) or not results else 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("refuse to overwrite historical fault classification")
    result = classify_manifest(args.manifest)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        stream.write(json.dumps(result, indent=2, allow_nan=False) + "\n")
    raise SystemExit(result["exit_code"])


if __name__ == "__main__":
    main()
