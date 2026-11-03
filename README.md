# drone-CV

Drone tránh vật cản bằng computer vision, **mô phỏng trước** với PX4 SITL, Gazebo và ROS 2.

## Trạng thái

Repo đã có **G0 offline chạy được**: metric depth/validity, capture-time projection, frame/quaternion transforms, histogram 72×3 bảo thủ, braking và command freshness. Có pytest và workflow CI; **chưa có flight runtime, simulator launcher hoặc evaluator SITL**.

Mục tiêu lõi: ≥45/50 frozen navigation worlds tới đích, zero unsafe, navigation clearance ≥0.5 m theo phase contract.
Đây là mục tiêu chưa đạt, không phải kết quả benchmark.

## Đọc theo thứ tự

1. [Plan triển khai đã hiệu chỉnh](docs/PLAN.md): phạm vi, kiến trúc, contracts, gate G0–G5.
2. [Contract an toàn S01–S08](docs/SAFETY.md): contact/clearance theo phase, unsafe latch, coverage và dependency đã sửa.
3. [Phản biện và nguồn chính thức](docs/REVIEW.md): rủi ro từ plan gốc và các sửa contract tiếp theo.
4. [65 scenario và oracle](docs/SCENARIOS.md): có fixture geometry G0; các scenario bay chưa chạy.
5. [Harness phát triển và đánh giá](docs/HARNESS.md): agent workflow, runner/evaluator contract và Git gates.
6. [Skills OMP và roadmap](docs/OMP.md): 10 skills nạp theo trigger, không cài thêm bundle.
7. [Hướng dẫn agent cho repo](.omp/AGENTS.md): bất biến và phạm vi tự động hoá.

[PDF gốc, giữ nguyên byte](docs/source/drone-obstacle-avoidance-plan.pdf).
Nguồn, checksum và quan hệ giữa PDF/bản sửa được ghi trong plan.

## Roadmap trong OMP

27 task có dependency và acceptance tại [state canonical](.omp/roadmap/roadmap.json).
Dùng `/roadmap status` hoặc `/roadmap view` trong OMP; canvas offline được sinh tự động, không sửa HTML.
Trạng thái và acceptance hiện tại nằm trong JSON; [bản portable](docs/ROADMAP.md) được sinh từ cùng nguồn, không sửa tay.
Đây là roadmap hoàn chỉnh cho phạm vi mô phỏng, không phải ước lượng phần trăm công sức hay chứng nhận bay.

## Chạy G0 ngay

Python 3.12, NumPy 2.0.0, pytest 8.2.2. Từ repo root:

```sh
python -m pip install numpy==2.0.0 pytest==8.2.2
python -m pytest -q
python scripts/check_project.py
python scripts/g0_demo.py --output artifacts/g0-demo
```

Nếu dependencies đã có, bỏ lệnh install. Không cần ROS/GPU/matplotlib hoặc cài package vào Python toàn cục.
Demo không ghi đè thư mục đã tồn tại; chọn tên output mới khi chạy lại.

Mở `artifacts/g0-demo/preview.svg`; `summary.json` chứa decisions, các file NPZ chứa point cloud ENU, depth và histogram.
Ba scene thật qua API: wall 6 m, dây ngang một hàng pixel ở 2 m, và unknown boot. Proposal cũ bị từ chối riêng.
Đây là **fixture tổng hợp**, không phải dữ liệu bay hay benchmark tránh vật thật.

### Giới hạn G0

- Pixel-cone support phải được xác minh rõ; không mặc định một ray chứng minh cả bin free.
- Footprint tròn được bao bằng swept boxes cho toàn sector 5°. Cách này bảo thủ, có thể chặn nhầm.
- Vùng sát/thậm chí sau camera không được tự xem là trống. Demo cung cấp sphere 1 m được chứng minh trống bằng geometry của fixture; runtime tương lai cần evidence độc lập, fresh và không mâu thuẫn depth.
- Không dùng temporal median/percentile; vật gần mới không bị history cũ xoá.
- Braking dùng gia tốc không đổi với thông số thăm dò. Chưa đo jerk, PX4 tracking hoặc phanh thật.
- Đổi hướng khi đang chạy yêu cầu stop trước; origin lệch quá 1 µm hoặc có motion theo Z bị chặn. Chưa có motion compensation, yaw-envelope control hay hover guarantee.

### Học cùng code

1. Đọc [NumPy quickstart](https://numpy.org/doc/2.0/user/quickstart.html), chạy `tests/test_depth.py`.
2. Đọc [REP-103](https://www.ros.org/reps/rep-0103.html) và [REP-118](https://www.ros.org/reps/rep-0118.html), đối chiếu `tests/test_geometry.py`.
3. Đọc công thức braking trong PLAN rồi chạy `tests/test_safety.py` và demo; so output scene wall/wire/unknown.

## G1: đã chạy smoke không arm, gate còn blocked

Sau khi người dùng cho phép mở Docker Desktop và tải image, đã chạy đúng digest ở Linux/amd64, container `--network none`, 4 CPU/6 GiB, software rendering, không host mount và không control publisher.

- PX4 package `1.18.0~beta1-noble`, firmware commit `61907962c9220437ec01a98fa26bfbf5cbb254b5`; Gazebo Sim `8.15.0`; ROS 2 Jazzy.
- Model `gz_x500_depth` thật; depth Gazebo `R_FLOAT32` 640×480, CameraInfo và `/clock` có dữ liệu.
- DDS `/drone_cv_g1/fmu/out/vehicle_odometry` có timestamp tăng; status quan sát đều **Disarmed**. QoS publisher BEST_EFFORT/TRANSIENT_LOCAL.
- Thiếu `ros_gz_bridge` trong image: chưa có camera/CameraInfo/clock ROS, chưa xác minh camera QoS hoặc alignment frame/time. **Không đánh dấu G1 done.**
- Depth camera cấu hình 30 Hz sim-time nhưng các cửa sổ wall-time thực chỉ khoảng 6 Hz trong cấu hình CPU thử; RTF thấp. Đây là smoke, chưa là profile hoặc calibration.

[Environment evidence](docs/environments/g1-observation.json) giữ version, model hashes, sensor metadata và blocker. Không có flight acceptance hay chứng nhận `+Inf` sensor luôn là free.
`scripts/g1_probe.py` là observer chỉ đọc, chạy **bên trong container có Agent/PX4/Gazebo đang chạy**, sau khi source `/usr/local/bin/ros2-entrypoint.sh`; không publish/arm, exit 2 khi observation thiếu.
Container/process do run sở hữu đã dừng và xoá sau probe; Docker Desktop và image đã tải được giữ lại. Không cài thêm package, đổi driver/WSL hoặc quyền.
Để tiếp tục cần phê duyệt image dẫn xuất bổ sung bridge đã pin, hoặc môi trường bridge tương thích đã có; không tự cài lên host.

## Nguyên tắc

- Unknown/stale/out-of-FOV là bị chặn; không suy ra khoảng trống từ NaN.
- Safety độc lập với planner; heartbeat không thay thế freshness của lệnh.
- Evaluator dùng ground truth độc lập; planner không được đọc oracle.
- Không đưa control loop lên LLM, không bay thật trong phạm vi lõi.
- Commit/push/merge theo quyền người dùng và gate thực; không bypass checks/review.
