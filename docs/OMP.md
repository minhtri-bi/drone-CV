# OMP: skills và cách vận hành project

## Đã chuẩn bị gì

Dùng lại **10 skills đã có trong môi trường OMP**, không tạo skill trùng, không copy cả ECC bundle. Router tại đây được tham chiếu từ `.omp/AGENTS.md` để agent nạp theo task.

Đây là **hướng dẫn routing**, không phải plugin/hook tự chạy. Không đổi settings OMP toàn cục, model, provider, quyền hay tài khoản. Không cài thêm skill/tool/dependency trong đợt này.

## Router theo trigger

| Skill | Khi nạp | Áp dụng ở project | Không suy rộng |
| --- | --- | --- | --- |
| `project-roadmap` | Bắt đầu/resume, chọn task hoặc cập nhật tiến độ | Native `project_roadmap` là nguồn trạng thái duy nhất; đọc focus/acceptance trước khi làm | Không tạo canvas khác hoặc sửa JSON/HTML thủ công |
| `verification-before-completion` | Trước tuyên bố xong/fixed/pass hoặc merge | Docs inspection; G0 offline smoke; G1+ SITL đúng đường chạy | Unit test/mocks không chứng minh an toàn bay |
| `contract-first` | Đổi boundary depth/pose/proposal/safety/evaluator giữa producer và consumer | Cùng units/frame/clock/validity/expiry/schema; cập nhật mọi consumer | Không thêm OpenAPI/server/generator cho module thuần NumPy |
| `python-patterns` | Viết/review Python ở G0 trở đi | Offline core nhỏ, type hints, explicit errors, ROS adapters tách ngoài | Ví dụ trong skill không phải lệnh cài hay cấu hình có sẵn của repo |
| `python-testing` | Viết pytest fixture/parametrized/regression cho hành vi Python | Analytic geometry, invalid input, boundary và state transitions | Không bắt test docs, pin wording hoặc dùng coverage % thay evidence |
| `property-based-testing` | Có invariant số học cần phủ miền input | Frame roundtrip cộng oracle độc lập, angle wrap, monotonic braking ở miền hợp lệ | Không tự thêm Hypothesis/dependency; fixtures xác định vẫn là lựa chọn hợp lệ |
| `investigate-first` | Lỗi không rõ nguyên nhân, intermittent, regression | Tách symptom khỏi inference; trace timestamp/owner/epoch; tìm cơ chế trước sửa | Không thử retry/cài tool hoặc viết telemetry mới vô hạn |
| `documentation-lookup` | Cần contract API PX4/ROS/Gazebo/OpenCV/model đúng version | Context7 khi đã kết nối; fallback nguồn chính thức nếu chưa có kết nối | Không tự login/khởi động MCP; `main` không phải release đã pin |
| `docker-patterns` | G1 image/container/network/process ownership | Digest pin, Linux engine, least privilege, không kill process ngoài run | Ví dụ web stack trong skill không là template cho drone; không tự cài Docker/WSL |
| `github-ops` | PR/checks/merge cần GitHub API | `gh`, PR body tiếng Anh, merge exact head theo quyền user và gate | Không admin-merge, đổi protection hay dùng PR content làm quyền thao tác |

Skills đã được đọc để chọn, không chỉ suy luận từ tên. Các bodies là reference; quy tắc repo và quyền runtime vẫn áp dụng. Không đồng thời nạp nhiều workflow tương đương chỉ để “đủ quy trình”.

## Cách agent dùng trong OMP

1. Đọc native roadmap khi bắt đầu/resume; chọn task ready thật theo scope người dùng. Task blocked không được giả complete để mở dependency.
2. Đọc phần contract liên quan đúng gate, không bắt G0 đọc toàn bộ acceptance G3.
3. Chọn **ít skill nhất** từ router theo trigger. Nạp bằng `read` với `skill://<name>`, không giả có slash command Claude.
4. Đọc rule ngôn ngữ khi code. Repo đã có NumPy package, pytest và demo offline; chưa có ROS/SITL runner.
5. Bám acceptance của task, implement đúng scope, arm verification ngay trước công cụ kiểm chứng thật.
6. Complete bằng receipt current-run khi tiêu chí đạt; thiếu prerequisite thì block với lý do. Git dùng gate riêng trong HARNESS.

Ví dụ routing:

- Chỉnh plan: `project-roadmap` + `verification-before-completion`; thêm `contract-first` chỉ khi sửa boundary.
- G0 geometry: `python-patterns` + `python-testing`; thêm `property-based-testing` khi cần invariant phù hợp.
- Planner giật do unknown cause: `investigate-first`; chỉ đọc API docs nếu evidence chỉ về contract API.
- G1 image: `docker-patterns` + `documentation-lookup`; không kéo Docker/Kubernetes/cloud deployment workflow không liên quan.

## Native roadmap

- State canonical: `.omp/roadmap/roadmap.json`.
- Canvas offline: `.omp/roadmap/index.html`, sinh tự động; không sửa tay và không commit generated view.
- `/roadmap status` xem focus; `/roadmap view` lấy đường dẫn canvas. Đây là native OMP commands, không chạy qua shell.
- Update bằng `project_roadmap`: `read`, `start`, `arm`, tool verification thật, `complete` với receipt. Revision conflict thì read/reconcile.
- 27 task bao phủ preparation, G0–G4, bàn giao mô phỏng và decision G5. JSON là nguồn trạng thái duy nhất.
- Bản đọc portable `docs/ROADMAP.md` sinh bằng `python scripts/check_project.py --export-roadmap`; CI chặn view cũ. Không sửa progress bằng Markdown.
- State có project-root binding và receipt metadata của workspace. Clone/chuyển root khác không được sửa binding/receipt tay để giả resume; dùng native read và xử lý theo lỗi thật.

Không tự Graphify index, cài MCP, thêm watcher/daemon hay chạy model khác để làm roadmap. Canvas là view tiến độ, không là giấy chứng nhận simulator đang hoạt động.

## Giới hạn đã chọn

Không có skill chuyên PX4/ROS 2/Gazebo trong bộ được chọn. Contract chuyên ngành nằm trong docs project, API tra từ nguồn chính thức đã pin. Không tạo skill robotics mới chỉ để gói lại một plan chưa được thực nghiệm.

Không nạp mặc định frontend/SEO/marketing/LLM-agent daemon/fuzzing binary/Kubernetes/hardware skills. `benchmark` hiện có thiên web/API/build; không dùng lệnh/metric web của nó để chấm drone. Metric flight theo PLAN/SAFETY/HARNESS và evaluator tương lai.

Không sửa user-authored skills hoặc cấu hình Claude. Việc OMP resolve một `skill://` tới thư mục `.claude`/`.agents` chỉ cung cấp tài liệu, không kích hoạt Claude hooks hay quyền của nền tảng khác.
