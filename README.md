# drone-CV

Drone tránh vật cản bằng computer vision, **mô phỏng trước** với PX4 SITL, Gazebo và ROS 2.

## Trạng thái

Repo có plan, contract an toàn đã sửa, router skills OMP và native roadmap. **Chưa có flight runtime, simulator launcher, evaluator hay CI.**

Mục tiêu lõi: ≥45/50 frozen navigation worlds tới đích, zero unsafe, navigation clearance ≥0.5 m theo phase contract.
Đây là mục tiêu chưa đạt, không phải kết quả benchmark.

## Đọc theo thứ tự

1. [Plan triển khai đã hiệu chỉnh](docs/PLAN.md): phạm vi, kiến trúc, contracts, gate G0–G5.
2. [Contract an toàn S01–S08](docs/SAFETY.md): contact/clearance theo phase, unsafe latch, coverage và dependency đã sửa.
3. [Phản biện và nguồn chính thức](docs/REVIEW.md): rủi ro từ plan gốc và các sửa contract tiếp theo.
4. [65 scenario và oracle](docs/SCENARIOS.md): 30 case gốc + 35 bổ sung, chưa chạy.
5. [Harness phát triển và đánh giá](docs/HARNESS.md): agent workflow, runner/evaluator contract và Git gates.
6. [Skills OMP và roadmap](docs/OMP.md): 10 skills nạp theo trigger, không cài thêm bundle.
7. [Hướng dẫn agent cho repo](.omp/AGENTS.md): bất biến và phạm vi tự động hoá.

[PDF gốc, giữ nguyên byte](docs/source/drone-obstacle-avoidance-plan.pdf).
Nguồn, checksum và quan hệ giữa PDF/bản sửa được ghi trong plan.

## Roadmap trong OMP

27 task có dependency và acceptance tại [state canonical](.omp/roadmap/roadmap.json).
Dùng `/roadmap status` hoặc `/roadmap view` trong OMP; canvas offline được sinh tự động, không sửa HTML.
Chỉ phần chuẩn bị đã hoàn tất; mọi task triển khai G0–G4 và decision G5 còn pending.
Đây là roadmap hoàn chỉnh cho phạm vi mô phỏng, không phải ước lượng phần trăm công sức hay chứng nhận bay.

## Bước triển khai đầu tiên

**G0/H0:** geometry/validity/histogram/braking offline bằng NumPy, có test biên và smoke thật.
Chưa cần ROS, GPU hoặc mua drone. Không train/fine-tune model.

G1 mới mở PX4/ROS/Gazebo trên môi trường Linux đã pin và kiểm tra.
Lệnh smoke image ứng viên trong harness được ghi rõ là **chưa chạy**, không phải trình cài đặt.

## Nguyên tắc

- Unknown/stale/out-of-FOV là bị chặn; không suy ra khoảng trống từ NaN.
- Safety độc lập với planner; heartbeat không thay thế freshness của lệnh.
- Evaluator dùng ground truth độc lập; planner không được đọc oracle.
- Không đưa control loop lên LLM, không bay thật trong phạm vi lõi.
- Commit/push/merge theo quyền người dùng và gate thực; không bypass checks/review.
