import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).parents[1]
SPEC = importlib.util.spec_from_file_location(
    "project_integrity", ROOT / "scripts/check_project.py")
PROJECT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROJECT)


@pytest.fixture
def roadmap():
    state = json.loads((ROOT / ".omp/roadmap/roadmap.json").read_text(encoding="utf-8"))
    labels = json.loads((ROOT / "docs/roadmap.en.json").read_text(encoding="utf-8"))
    return state, labels


def test_changed_native_criteria_require_a_reviewed_translation(roadmap):
    state, labels = roadmap
    task = state["tasks"][0]
    changed = {**task, "acceptance": [*task["acceptance"], "An additional sensor bound is required."]}
    updated = {**state, "tasks": [changed, *state["tasks"][1:]]}
    with pytest.raises(ValueError, match="English labels stale"):
        PROJECT.portable_roadmap(updated, labels)


@pytest.mark.parametrize("field,value", [
    ("status", "done"),
    ("dependsOn", []),
    ("evidenceIds", ["fabricated-receipt"]),
])
def test_translations_cannot_override_native_progress_or_evidence(roadmap, field, value):
    state, labels = roadmap
    task_id = state["tasks"][0]["id"]
    corrupted = {**labels, task_id: {**labels[task_id], field: value}}
    with pytest.raises(ValueError, match="display metadata only"):
        PROJECT.portable_roadmap(state, corrupted)


@pytest.mark.parametrize("missing", [True, False])
def test_translation_coverage_must_match_native_tasks(roadmap, missing):
    state, labels = roadmap
    task_id = state["tasks"][0]["id"]
    altered = ({key: value for key, value in labels.items() if key != task_id}
               if missing else {**labels, "nonexistent-task": labels[task_id]})
    with pytest.raises(ValueError, match="exactly the native task IDs"):
        PROJECT.portable_roadmap(state, altered)


def test_translation_cannot_silently_drop_an_acceptance_criterion(roadmap):
    state, labels = roadmap
    task_id = state["tasks"][0]["id"]
    incomplete = {**labels, task_id: {
        **labels[task_id], "acceptance": labels[task_id]["acceptance"][:-1]}}
    with pytest.raises(ValueError, match="incomplete English labels"):
        PROJECT.portable_roadmap(state, incomplete)


def test_evidence_checksum_detects_changed_bytes_and_omitted_payload(tmp_path):
    import hashlib
    payload = tmp_path / "report.json"
    payload.write_bytes(b'{"accepted": 152}\n')
    expected = hashlib.sha256(payload.read_bytes()).hexdigest()
    (tmp_path / "SHA256SUMS").write_text(f"{expected}  report.json\n", encoding="utf-8")
    assert PROJECT.check_evidence_bundle(tmp_path) == 1
    payload.write_bytes(b'{"accepted": 153}\n')
    with pytest.raises(ValueError, match="checksum mismatch"):
        PROJECT.check_evidence_bundle(tmp_path)
    payload.unlink()
    with pytest.raises(ValueError, match="coverage mismatch"):
        PROJECT.check_evidence_bundle(tmp_path)


def test_archive_inner_identity_is_checked_even_if_outer_hash_was_updated(tmp_path):
    import hashlib
    import io
    import tarfile
    archive_path = tmp_path / "recording.tar.gz"
    correct = b"original bag bytes"
    manifest = {"bag_files": {"bag/bag.db3": {
        "bytes": len(correct), "sha256": hashlib.sha256(correct).hexdigest()}},
        "fixture_sha256": hashlib.sha256(b"{}").hexdigest()}
    with tarfile.open(archive_path, "w:gz") as archive:
        for name, data in [("recording/manifest.json", json.dumps(manifest).encode()),
                           ("recording/fixture.json", b"{}"),
                           ("recording/bag/bag.db3", b"modified bag bytes")]:
            member = tarfile.TarInfo(name)
            member.size = len(data)
            archive.addfile(member, io.BytesIO(data))
    expected = hashlib.sha256(archive_path.read_bytes()).hexdigest()
    (tmp_path / "SHA256SUMS").write_text(f"{expected}  recording.tar.gz\n", encoding="utf-8")
    with pytest.raises(ValueError, match="archived bag identity mismatch"):
        PROJECT.check_evidence_bundle(tmp_path)


def test_entire_missing_checksum_inventory_cannot_hide_a_bundle(tmp_path):
    directory = tmp_path / "unlisted-bundle"
    directory.mkdir()
    (directory / "report.json").write_text("{}")
    with pytest.raises(ValueError, match="missing evidence checksum inventory"):
        PROJECT.check_evidence_root(tmp_path)


@pytest.mark.parametrize("changed", ["recording_policy.json", "receive_progress.jsonl",
                                     "oracle_profile.json"])
def test_oracle_archive_binds_policy_receive_trace_and_physical_profile(tmp_path, changed):
    import hashlib
    import io
    import tarfile
    payloads = {"fixture.json": b"{}", "recording_policy.json": b'{"schema":4}',
                "receive_progress.jsonl": b'{"source_ns":100}\n',
                "oracle_profile.json": b'{"schema":1}'}
    manifest = {"recording_schema": 4, "bag_files": {},
                "fixture_sha256": hashlib.sha256(payloads["fixture.json"]).hexdigest(),
                "policy_sha256": hashlib.sha256(payloads["recording_policy.json"]).hexdigest(),
                "receive_progress_sha256": hashlib.sha256(payloads["receive_progress.jsonl"]).hexdigest(),
                "oracle_profile_sha256": hashlib.sha256(payloads["oracle_profile.json"]).hexdigest()}
    archive_path = tmp_path/"recording.tar.gz"
    with tarfile.open(archive_path, "w:gz") as archive:
        values = {**payloads, changed: b"tampered", "manifest.json": json.dumps(manifest).encode()}
        for name, data in values.items():
            member = tarfile.TarInfo("recording/"+name)
            member.size = len(data)
            archive.addfile(member, io.BytesIO(data))
    with pytest.raises(ValueError, match="archived provenance mismatch"):
        PROJECT.check_evidence_archive(archive_path)


@pytest.mark.parametrize("preparation_status", [None, "pending", "in_progress", "blocked"])
def test_g2_cannot_complete_without_finished_evidence_semantics(preparation_status):
    tasks = [{"id": "g2-adapter", "status": "done", "acceptance": ["contract"]}]
    if preparation_status is not None:
        tasks += [{"id": "prep-evidence-semantics", "status": preparation_status,
                   "acceptance": ["contract"]}]
    with pytest.raises(ValueError, match="supplemental preparation gate unmet: prep-evidence-semantics"):
        PROJECT.check_roadmap({"tasks": tasks})


def test_evidence_semantics_never_blocks_g1_or_rejects_ready_g2():
    tasks = [{"id": "g1-oracle", "status": "done", "acceptance": ["contract"]},
             {"id": "g2-adapter", "status": "pending", "acceptance": ["contract"]}]
    PROJECT.check_roadmap({"tasks": tasks})
    PROJECT.check_roadmap({"tasks": [
        {**tasks[1], "status": "done"},
        {"id": "prep-evidence-semantics", "status": "done", "acceptance": ["contract"]},
        {"id": "prep-evidence-review", "status": "done", "acceptance": ["contract"]}]})


@pytest.mark.parametrize("gate,status,preparation", [
    ("g2-replay", "done", "prep-evidence-review"),
    ("g1-flight", "in_progress", "prep-flight-oracle"),
    ("g1-flight", "done", "prep-flight-oracle"),
    ("g1-calibration", "done", "prep-depth-path-audit"),
])
@pytest.mark.parametrize("preparation_status", [None, "pending", "blocked", "in_progress"])
def test_approved_supplemental_gate_rejects_missing_proof(gate, status, preparation,
                                                       preparation_status):
    tasks = [{"id": gate, "status": status, "acceptance": ["contract"]},
             {"id": "prep-evidence-semantics", "status": "done", "acceptance": ["contract"]}]
    if preparation_status is not None:
        tasks.append({"id": preparation, "status": preparation_status, "acceptance": ["contract"]})
    with pytest.raises(ValueError, match="supplemental preparation gate unmet"):
        PROJECT.check_roadmap({"tasks": tasks})
    PROJECT.check_roadmap({"tasks": [
        *[task for task in tasks if task["id"] != preparation],
        {"id": preparation, "status": "done", "acceptance": ["contract"]}]})


def test_evidence_review_does_not_block_g1_or_calibration_acquisition():
    PROJECT.check_roadmap({"tasks": [
        {"id": "g1-supervisor", "status": "in_progress", "acceptance": ["contract"]},
        {"id": "g1-calibration", "status": "in_progress", "acceptance": ["contract"]},
        {"id": "prep-evidence-review", "status": "blocked", "acceptance": ["contract"]}]})


@pytest.mark.parametrize("preparation,target,selector,statuses", [
    ("prep-evidence-semantics", "g2-", "prefix", ("done",)),
    ("prep-evidence-review", "g2-", "prefix", ("done",)),
    ("prep-flight-oracle", "g1-flight", "exact", ("in_progress", "done")),
    ("prep-depth-path-audit", "g1-calibration", "exact", ("done",)),
])
@pytest.mark.parametrize("status", ["pending", "blocked", "in_progress", "done"])
def test_supplemental_policy_status_boundaries(preparation, target, selector, statuses, status):
    task_id = target + "future-task" if selector == "prefix" else target
    tasks = [{"id": task_id, "status": status, "acceptance": ["contract"]}]
    tasks += [{"id": identity, "status": "done", "acceptance": ["contract"]}
              for identity in ("prep-evidence-semantics", "prep-evidence-review",
                               "prep-flight-oracle", "prep-depth-path-audit")
              if identity != preparation]
    if status in statuses:
        with pytest.raises(ValueError, match=f"supplemental preparation gate unmet: {preparation}"):
            PROJECT.check_roadmap({"tasks": tasks})
    else:
        PROJECT.check_roadmap({"tasks": tasks})


def translated_state(tasks):
    state = {"revision": 1, "tasks": [
        {**task, "phase": "Test phase", "title": task["id"]} for task in tasks]}
    labels = {}
    for task in state["tasks"]:
        source = {key: task[key] for key in ("title", "phase", "acceptance")}
        encoded = json.dumps(source, ensure_ascii=False, sort_keys=True,
                             separators=(",", ":")).encode("utf-8")
        labels[task["id"]] = {**source, "source_sha256": hashlib.sha256(encoded).hexdigest()}
    return state, labels


def test_generated_gate_table_covers_all_four_policies_without_geometry_dependency():
    state, labels = translated_state([])
    report = PROJECT.portable_roadmap(state, labels)
    table = report.split("| Prerequisite |", 1)[1].split(
        "Geometry contradictions", 1)[0]
    assert table.count("| `prep-") == 4
    for prerequisite, target, selector, statuses in [
            ("prep-evidence-semantics", "g2-", "prefix", ("done",)),
            ("prep-evidence-review", "g2-", "prefix", ("done",)),
            ("prep-flight-oracle", "g1-flight", "exact", ("in_progress", "done")),
            ("prep-depth-path-audit", "g1-calibration", "exact", ("done",))]:
        row = next(line for line in table.splitlines() if f"`{prerequisite}`" in line)
        assert f"{selector} `{target}`" in row
        assert ", ".join(f"`{status}`" for status in statuses) in row
    assert "prep-geometry-crosscheck" not in table
    assert "affected-profile use" in report


@pytest.mark.parametrize("policy_index", range(4))
def test_changed_policy_drives_both_enforcement_and_portable_table(monkeypatch, policy_index):
    original = PROJECT.SUPPLEMENTAL_GATES
    changed = original[policy_index]._replace(
        prerequisite="prep-changed", target="trial-", selector="prefix",
        guarded_statuses=("blocked", "in_progress"))
    monkeypatch.setattr(PROJECT, "SUPPLEMENTAL_GATES",
                        original[:policy_index] + (changed,) + original[policy_index+1:])
    tasks = [{"id": "trial-future", "status": "in_progress", "acceptance": ["contract"]}]
    with pytest.raises(ValueError, match="prep-changed for trial-future"):
        PROJECT.check_roadmap({"tasks": tasks})
    state, labels = translated_state(tasks)
    report = PROJECT.portable_roadmap(state, labels)
    row = next(line for line in report.splitlines() if "| `prep-changed` |" in line)
    assert "prefix `trial-`" in row
    assert "`blocked`, `in_progress`" in row
    assert f"| `{original[policy_index].prerequisite}` |" not in report
    PROJECT.check_roadmap({"tasks": [
        *tasks, {"id": "prep-changed", "status": "done", "acceptance": ["contract"]}]})
    PROJECT.check_roadmap({"tasks": [{**tasks[0], "status": "done"}]})
    PROJECT.check_roadmap({"tasks": [{**tasks[0], "id": "unrelated", "status": "in_progress"}]})
