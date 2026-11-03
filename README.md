# drone-CV

Drone tránh vật cản bằng computer vision, **mô phỏng trước** với PX4 SITL, Gazebo và ROS 2.

## Trạng thái

Repo đã có **G0 offline chạy được**: metric depth/validity, capture-time projection, frame/quaternion transforms, histogram 72×3 bảo thủ, braking và command freshness. **G1 environment đã qua smoke không arm** với camera/clock/TF ROS và PX4 DDS trên image dẫn xuất đã pin. Có pytest và workflow CI; chưa có oracle, supervisor, flight runtime hoặc evaluator SITL.

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

## G1: environment đã qua, flight gate chưa đạt

Sau phê duyệt Docker/image và bridge, đã build [image dẫn xuất](docker/g1/Dockerfile) từ digest cố định. [Dependency lock](docker/g1/packages.lock.json) pin version và SHA-256 của 13 package mới; installer không nâng cấp package nền và từ chối archive lệch lock.

- PX4 package `1.18.0~beta1-noble`, firmware `61907962c9220437ec01a98fa26bfbf5cbb254b5`; Gazebo Sim `8.15.0`; ROS 2 Jazzy; bridge `1.0.24-1noble.20260920.234644`.
- X500 depth thật, ROS `32FC1` 640×480, 35 cặp Image/CameraInfo cùng stamp trong cửa sổ 8 giây; optical TF khớp SDF.
- Gazebo `/clock` là clock duy nhất: `UXRCE_DDS_SYNCT=0`, observer/bridge dùng `use_sim_time=true`, không thay stamp bằng wall-time. Raw PX4 odometry nằm trong cùng cửa sổ sim-time và tăng nghiêm ngặt.
- DDS status luôn **Disarmed**; không tạo control publisher hoặc gửi arm. Container network none, 4 CPU/6 GiB, software rendering, không host mount.
- Camera đạt khoảng 4.5 Hz wall-time trong cửa sổ này, không phải 30 Hz thực hoặc performance/calibration acceptance.

[Evidence mới](docs/environments/g1-bridge-observation.json) giữ pins, QoS, encoding, intrinsics, TF, clock và hai cửa sổ thất bại trước khi sửa clock policy. [Probe image nền](docs/environments/g1-observation.json) vẫn giữ nguyên lịch sử thiếu bridge.

`scripts/g1_probe.py --ros` cùng `scripts/g1_ros_observer.py` là observer chỉ đọc bên trong container đang chạy Agent/PX4/Gazebo/bridge/TF; exit 2 khi observation thiếu. [Recipe đã chạy và giới hạn](docs/HARNESS.md#recipe-smoke-dẫn-xuất-và-kết-quả-đã-quan-sát).

Container/process do run sở hữu đã dừng và xoá; Docker Desktop/images được giữ lại. Không cài package lên host, đổi driver/WSL hoặc quyền.
Chỉ `g1-environment` đạt: oracle/logger và supervisor vẫn phải có trước bootstrap flight; calibration vẫn pending. Không chứng nhận `+Inf` là free, capture-time pose interpolation hoặc tránh vật cản thành công.

## Nguyên tắc

- Unknown/stale/out-of-FOV là bị chặn; không suy ra khoảng trống từ NaN.
- Safety độc lập với planner; heartbeat không thay thế freshness của lệnh.
- Evaluator dùng ground truth độc lập; planner không được đọc oracle.
- Không đưa control loop lên LLM, không bay thật trong phạm vi lõi.
- Commit/push/merge theo quyền người dùng và gate thực; không bypass checks/review.
