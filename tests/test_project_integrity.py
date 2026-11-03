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
