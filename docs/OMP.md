# OMP project harness

## Entry point

[AGENTS.md](../AGENTS.md) is the canonical project policy. `.omp/AGENTS.md` only points to it for OMP compatibility. No mandatory skill router, automatic review pipeline or additional agent process is required.

Current milestone: M0. The [README milestone table and Backlog](../README.md#milestones) are the v2 status source until roadmap migration. This documentation-only change is based on main, not PR #12; it does not import that PR's code, archive it or establish any M0–M5 result.

## Short workflow

1. Read the user's scope and README milestone table; work on the current demo or its explicitly requested preparation.
2. Inspect nearby implementation and load only relevant skills with `skill://<name>`.
3. Make the smallest coherent change. Do not add future-stage guards or documents without a concrete need.
4. Exercise the changed path once. For documentation-only changes, use `python scripts/check_project.py` and inspect content/links.
5. Report observed results and blockers; never claim flight from offline tests.

For an external review, present BLOCKS-MILESTONE / BACKLOG / REJECT with one reason per item. Wait for approval before fixes. This is an instruction, not a claim that a `/triage` command is installed.

## Historical roadmap compatibility

- The native roadmap retained from main has 31 v1 tasks; it is not the work queue or status source for M0–M5.
- If native startup reads expose g1-oracle, g1-supervisor or prep-* focus, treat it as historical only. Do not start those tasks for v2 work.
- Keep v1 native JSON, receipts, translations and generated exports unchanged; do not fake completion or unlock old gates.
- A future approved migration must use supported native tooling. Until then, update v2 status only in README.md.
- The existing checker validates retained v1 integrity, not v2 milestone acceptance.

## M1 direction

After M0, implement a new minimal Offboard node for empty-world takeoff, A-to-B movement and landing. Do not require v1 supervisor.py, px4_bridge.py, permits, Home/variance/RTF admission or prep-* completion. Those names refer to the v1 architecture, including code retained only in PR #12; they are not available-command claims or required dependencies.

Use one setpoint publisher, the planned depth watchdog, speed clamp and explicit sim parameters. Evaluate actual trials separately; this document does not authorize control or flight execution.

## Available commands and limits

- `python scripts/check_project.py`: retained v1 document, PDF, evidence and roadmap integrity.
- `python -m pytest -q`: existing offline behavioral tests, not flight acceptance.
- Existing simulator configuration remains under `docker/g1/`. Proposed v2 `sim/up.sh` and trial commands are not implemented.
- No model/provider/account/permission changes, dependency installation, Docker image changes, simulator startup, parameter writes or arming are authorized by this document.

Historical PLAN/SAFETY/HARNESS and results explain retained v1 behavior. They are not v2 prerequisites or mandatory reading for every small edit. Existing runtime files are unchanged; M1 does not have to use their control architecture.
