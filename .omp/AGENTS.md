# drone-CV: project instructions

## Scope and current truth

- Simulation-first obstacle avoidance; no real drone control, purchases or hardware deployment.
- Read only the PLAN/SAFETY sections relevant to the current gate; do not reload all contracts for a small G0 change. Load SCENARIOS/HARNESS when their boundary is affected.
- G0 now has a NumPy core, behavioral tests and synthetic CLI demo. There is still no flight stack, ROS node or SITL evaluator in this repository.
- Do not claim a phase passed without evidence from that phase's real execution path.

## Language

- Communicate with the user in Vietnamese.
- Write project documentation, plans, comments, docstrings, commit messages, pull requests, reviews and other persisted authored text in English unless the user explicitly requests another language.
- Translate existing Vietnamese-authored text when editing it; preserve technical meaning, literal identifiers, source provenance, raw logs and historical evidence.

## Architecture invariants

- Pure NumPy geometry first; ROS adapters stay outside the offline core.
- Metric depth requires verified units, optical geometry, capture-time pose, clock epoch and validity.
- Unknown, stale and out-of-FOV observations never imply free space.
- Preserve new near-obstacle evidence; percentile/temporal filtering must not erase it.
- Check footprint/swept stopping volume, measured velocity and measured delay, not only requested velocity.
- Safety runs independently from planner/inference. Only one final publisher owns Offboard setpoints.
- Heartbeat is not command freshness. Reject expired proposals even when the bridge is alive.
- Never move backwards into blind space, assume yaw rotation is collision-free, or reclaim PX4 control after failsafe.
- Do not assume Collision Prevention protects Offboard.
- Ground truth is evaluator-only, except the explicitly documented simulated localization input.
- Apply phase-aware contact/clearance: designated gear/support contacts only by frozen allowlist; never exempt body/propeller collisions.
- Latch unsafe observations even if telemetry/simulator later fails. Freeze navigation/negative/stress membership before each trial.
- Signed position/velocity components are valid; do not reject them as negative distances.

## Working method

- Complete one PLAN gate slice at a time. Do not build empty modules, fake simulator outputs or a multi-agent daemon.
- Read native `project_roadmap` on start/resume. Start a ready task; arm before real verification; complete only with valid current-run evidence. Do not hand-edit roadmap JSON/HTML.
- Use `docs/OMP.md` to route existing skills by trigger with `skill://<name>`. No copied bundles, fake slash commands or automatic pipeline.
- A roadmap is not authorization to implement every pending phase. Match the current user's requested scope.
- Oracle/logger and a minimal independent supervisor must exist before exploratory G1 flight; calibration precedes G2 wall-stop. Bootstrap runs are not navigation acceptance.
- Keep bounded edits inline. No automatic agent-review/TDD/E2E pipeline for routine documentation or small changes.
- Add behavioral boundary tests where needed; run the changed path once after edits.
- Documentation-only verification uses sources, links and PDF provenance, not a new simulator installation.
- Flight/safety changes need relevant SITL/fault-injection proof; mocks and unit tests are not flight evidence.
- No acceptance-threshold weakening, cherry-picked retries, seed replacement after failure or hidden infrastructure errors.
- Stop an unproductive repair loop after three attempts without new causal evidence; report the blocker.
- Keep the original PDF unchanged. Update readable docs rather than rewriting source provenance.

## Plan-to-roadmap alignment

- Before proposing implementation, map the requested work to stable native task IDs, current status, acceptance and dependency readiness. Do not treat the default focus as authority for unrelated work.
- Every implementation plan must state its task IDs, prerequisites, scope/non-goals, verification, and which existing gates remain unmet. For a new branch, show its dependency edges explicitly; use a small diagram when it clarifies the branch.
- If a useful experiment is outside the current DAG, propose an explicit bounded preparation task and obtain user agreement before adding it through the native API. Never silently skip, remove, reopen or complete prerequisites to make a plan appear ready.
- Distinguish unarmed recording/offline analysis from armed tests, flight, calibration and navigation acceptance. Reusing preparation artifacts does not complete later gates or waive fresh verification.
- When the user requests a plan for review, leave implementation tasks pending until that review is resolved. A ready task or a published plan does not itself authorize simulator execution, installation, arming or other operational effects.
- After an approved roadmap change, update `docs/roadmap.en.json` and regenerate `docs/ROADMAP.md` using the exporter. Preserve canonical IDs, evidence and unrelated dependencies; never hand-edit native state or generated HTML.

## Git and authority

- The user authorizes commit, push and merge for requested work in `minhtri-bi/drone-CV` when relevant gates pass.
- Use a task branch and an English PR body; commit subjects and posted messages use concise natural English.
- Confirm remote/base and preserve unrelated work. Stage only task-owned paths.
- Never force-push, reset/stash unknown work, bypass required reviews/checks, use admin merge, or change protections.
- This authorization does not cover other repositories, account/security changes, tool installation or real flights.
- If a required gate cannot run, state exactly what remains unverified. Do not manufacture green evidence.

## Commands and next slice

- `python -m pytest -q`: offline behavioral regressions.
- `python scripts/check_project.py`: PDF/scenario/link/DAG integrity, published evidence/archive identities and portable roadmap freshness.
- `python scripts/check_project.py --export-roadmap`: regenerate the read-only portable view after native state updates.
- `python scripts/g0_demo.py --output artifacts/<new-name>`: actual synthetic API smoke; never overwrite earlier evidence.
- G1 environment passed an actual unarmed smoke after explicit Docker/image/derived-bridge approval; see docs/environments/g1-bridge-observation.json and docker/g1/packages.lock.json. Gazebo /clock is the only time source: UXRCE_DDS_SYNCT=0, ROS time consumers use_sim_time=true, no wall-stamp override. GZ_CONFIG_PATH=/usr/share/gz preserves native Gazebo sim discovery under ROS vendor hooks. Preserve historical failed windows/base-image evidence. Only g1-environment is complete; oracle/supervisor, flight and calibration remain pending. Never infer an arming or host-install permission.
- LocalClearance and verified pixel-cone support in the G0 demo come from analytic synthetic geometry, not forward-camera proof. Future adapters must justify them independently.
