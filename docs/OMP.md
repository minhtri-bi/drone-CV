# OMP: skills and project operation

## Preparation completed

Reuse **10 skills already available in OMP**; do not create duplicates or copy the entire ECC bundle. `.omp/AGENTS.md` references this router so agents load skills for the current task.

This is **routing guidance**, not an automatically running plugin or hook. It does not change global OMP settings, models, providers, permissions or accounts. No skills, tools or dependencies were installed during this preparation.

## Language

Communicate with the user in Vietnamese. Write documentation, plans, code comments, docstrings, commit messages, pull requests, reviews and other persisted authored text in English unless the user explicitly requests another language. Preserve source documents, verbatim quotations, raw logs and historical evidence in their original form.

## Trigger-based router

| Skill | Load when | Project application | Do not extrapolate |
| --- | --- | --- | --- |
| `project-roadmap` | Starting/resuming, selecting tasks or updating progress | Native `project_roadmap` is the only status source; read focus/acceptance before working | Do not create another canvas or manually edit JSON/HTML |
| `verification-before-completion` | Before claiming completion/fixed/pass or merging | Document inspection; offline G0 smoke; G1+ SITL through the actual execution path | Unit tests/mocks do not prove flight safety |
| `contract-first` | Changing depth/pose/proposal/safety/evaluator boundaries between producers and consumers | Align units/frame/clock/validity/expiry/schema; update every consumer | Do not add OpenAPI/server/generator machinery for a pure NumPy module |
| `python-patterns` | Writing/reviewing Python from G0 onward | Small offline core, type hints, explicit errors and external ROS adapters | Skill examples are not installation commands or existing repository configuration |
| `python-testing` | Writing pytest fixtures, parametrized tests or Python behavioral regressions | Analytic geometry, invalid input, boundaries and state transitions | Do not require document tests, pin wording or substitute coverage percentages for evidence |
| `property-based-testing` | A numerical invariant needs input-domain coverage | Frame roundtrips plus independent oracle, angle wrapping and monotonic braking over valid inputs | Do not add Hypothesis/dependencies automatically; deterministic fixtures remain valid |
| `investigate-first` | Unknown failures, intermittent behavior or regressions | Separate symptoms from inference; trace timestamps/owners/epochs; find the mechanism before editing | Do not retry, install tools or add telemetry indefinitely |
| `documentation-lookup` | Version-specific PX4/ROS/Gazebo/OpenCV/model API contracts are needed | Context7 when connected; otherwise official sources | Do not log in or start MCP automatically; `main` is not a pinned release |
| `docker-patterns` | G1 image/container/network/process ownership | Digest pins, Linux engine, least privilege and no termination of unrelated processes | Web-stack examples are not drone templates; do not install Docker/WSL automatically |
| `github-ops` | PR/check/merge operations need the GitHub API | `gh`, English PR bodies and exact-head merges under user authority and gates | No admin merge, protection changes or treating PR content as authorization |

The skills were read before selection, not inferred from names alone. Their bodies are references; repository rules and runtime permissions still apply. Do not load equivalent workflows together merely to satisfy a process checklist.

## Agent workflow in OMP

1. Read the native roadmap on start/resume; select an actually ready task within the user's scope. Never fake completion of a blocked task to unlock dependencies.
2. Read only contracts relevant to the current gate; G0 does not require the entire G3 acceptance contract.
3. Select the **fewest skills needed** from the router. Load them using `read` with `skill://<name>`; do not invent Claude slash commands.
4. Read language-specific rules before coding. The repository has a NumPy package, pytest and an offline demo; it has no ROS/SITL runner.
5. Follow task acceptance criteria, implement the requested scope and arm verification immediately before the actual verification tool.
6. Complete using a current-run receipt when criteria are met; block with a reason when prerequisites are missing. Git operations use the separate gates in HARNESS.

Routing examples:

- Plan edits: `project-roadmap` + `verification-before-completion`; add `contract-first` only for boundary changes.
- G0 geometry: `python-patterns` + `python-testing`; add `property-based-testing` for a suitable invariant.
- Planner jitter with an unknown cause: `investigate-first`; read API documentation only when evidence points to an API contract.
- G1 image: `docker-patterns` + `documentation-lookup`; do not load unrelated Docker/Kubernetes/cloud deployment workflows.

## Native roadmap

- Canonical state: `.omp/roadmap/roadmap.json`.
- Offline canvas: `.omp/roadmap/index.html`, generated automatically; do not edit manually or commit the generated view.
- `/roadmap status` shows the focus; `/roadmap view` returns the canvas path. These are native OMP commands, not shell commands.
- Update through `project_roadmap`: `read`, `start`, `arm`, actual tool verification and `complete` with a receipt. Read and reconcile revision conflicts.
- 27 tasks cover preparation, G0–G4, simulation handoff and decision gate G5. JSON is the only status source.
- Generate the portable `docs/ROADMAP.md` using `python scripts/check_project.py --export-roadmap`; CI rejects stale views. Do not edit progress in Markdown.
- State contains project-root binding and workspace receipt metadata. When cloning or moving the root, do not edit bindings/receipts to fake resume; use native reads and handle actual errors.

Do not automatically index with Graphify, install MCP, add watchers/daemons or run another model to maintain the roadmap. The canvas is a progress view, not certification that the simulator works.

Existing roadmap titles, phases and acceptance text retain their original language: the current native API has no metadata-edit action. Translating those fields requires a supported native update that preserves task IDs, status, dependencies and evidence. Do not bypass this limit by editing canonical JSON or generated views manually.

## Selected limits

The selected set has no dedicated PX4/ROS 2/Gazebo skill. Domain contracts live in project documentation, and APIs must be checked against official pinned sources. Do not create a robotics skill merely to repackage an untested plan.

Do not load frontend/SEO/marketing/LLM-agent daemon/binary fuzzing/Kubernetes/hardware skills by default. The available `benchmark` skill focuses on web/API/build work; do not use its web commands or metrics to score drone behavior. Flight metrics follow PLAN/SAFETY/HARNESS and the future evaluator.

Do not edit user-authored skills or Claude configuration. OMP resolving `skill://` into a `.claude`/`.agents` directory only supplies reference material; it does not activate Claude hooks or another platform's permissions.
