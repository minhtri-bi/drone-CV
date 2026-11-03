# drone-CV

Drone tránh vật cản bằng computer vision, **mô phỏng trước** với PX4 SITL, Gazebo và ROS 2.

## Trạng thái

Repo hiện có plan, phân tích rủi ro và đặc tả harness. **Chưa có flight runtime, simulator launcher, evaluator hay CI.**

Mục tiêu lõi: ≥45/50 world test hợp lệ tới đích, không va chạm, clearance ≥0.5 m và bộ edge case bắt buộc.
Đây là mục tiêu chưa đạt, không phải kết quả benchmark.

## Đọc theo thứ tự

1. [Plan triển khai đã hiệu chỉnh](docs/PLAN.md): phạm vi, kiến trúc, contracts, gate G0–G5.
2. [Phản biện và nguồn chính thức](docs/REVIEW.md): lỗi/rủi ro còn lại trong plan gốc.
3. [65 scenario và oracle](docs/SCENARIOS.md): 30 case gốc + 35 case bổ sung, chưa chạy.
4. [Harness phát triển và đánh giá](docs/HARNESS.md): agent workflow, runner/evaluator contract, Git gates và task đầu tiên.
5. [Hướng dẫn agent cho repo](.omp/AGENTS.md): bất biến an toàn và phạm vi tự động hoá.

[PDF gốc, giữ nguyên byte](docs/source/drone-obstacle-avoidance-plan.pdf).
Nguồn, checksum và quan hệ giữa PDF/bản sửa được ghi trong plan.

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
