# OMP project harness

## Entry point

[AGENTS.md](../AGENTS.md) is the canonical project policy. `.omp/AGENTS.md` only points to it for OMP compatibility. No mandatory skill router, automatic review pipeline or additional agent process is required.

The harness is being simplified before implementation migration. The existing code, evidence and 36-task roadmap remain intact; this change does not reset the repository, close PR #12 or establish any M0–M5 result.

## Short workflow

1. Read the user's scope and current README status; work on one concrete demo or bounded preparation.
2. Inspect nearby implementation and load only relevant skills with `skill://<name>`.
3. Make the smallest coherent change. Do not add future-stage guards or documents without a concrete need.
4. Exercise the changed path once. For documentation-only changes, use `python scripts/check_project.py` and inspect content/links.
5. Report observed results and blockers; never claim flight from offline tests.

For an external review, present BLOCKS-MILESTONE / BACKLOG / REJECT with one reason per item. Wait for approval before fixes. This is an instruction, not a claim that a `/triage` command is installed.

## Existing roadmap compatibility

- Read and update tracked work through native `project_roadmap`; do not edit canonical JSON or generated HTML.
- Small harness maintenance does not need another preparation gate.
- The existing portable view still uses `docs/roadmap.en.json` and `python scripts/check_project.py --export-roadmap` after authorized native changes.
- Replacing the v1 DAG with M0–M5 is separate migration work; do not mark blocked v1 tasks complete to make it ready.

## Available commands and limits

- `python scripts/check_project.py`: existing document, PDF, evidence and roadmap integrity.
- `python -m pytest -q`: offline behavioral tests, not flight acceptance.
- Existing simulator configuration remains under `docker/g1/`. Proposed v2 `sim/up.sh` and trial commands are not implemented.
- No model/provider/account/permission changes, dependency installation, Docker image changes, simulator startup, parameter writes or arming are authorized by this document.

Historical PLAN/SAFETY/HARNESS and results remain technical references for existing v1 behavior, not a requirement to reload all contracts before every small edit. This policy change does not disable the existing runtime guards or change historical acceptance results.
