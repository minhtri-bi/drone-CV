# drone-CV: project instructions

## Scope and current truth

- Simulation-first obstacle avoidance; no real drone control, purchases or hardware deployment.
- Read `docs/PLAN.md` and `docs/SAFETY.md`; load `docs/SCENARIOS.md` and `docs/HARNESS.md` only for the current slice.
- The initial import contains plans, research and harness specifications, not a working flight stack or evaluator.
- Do not claim a phase passed without evidence from that phase's real execution path.

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

## Git and authority

- The user authorizes commit, push and merge for requested work in `minhtri-bi/drone-CV` when relevant gates pass.
- Use a task branch and an English PR body; commit subjects and posted messages use concise natural English.
- Confirm remote/base and preserve unrelated work. Stage only task-owned paths.
- Never force-push, reset/stash unknown work, bypass required reviews/checks, use admin merge, or change protections.
- This authorization does not cover other repositories, account/security changes, tool installation or real flights.
- If a required gate cannot run, state exactly what remains unverified. Do not manufacture green evidence.

## Next implementation slice

G0/H0: offline geometry, depth validity, histogram and braking contracts. See the final task prompt in `docs/HARNESS.md`. No project test/runner command exists at the initial import; add real commands with the implementation rather than inventing an existing suite.
