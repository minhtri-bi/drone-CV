# Phản biện plan và kiểm chứng nguồn

Ngày đối chiếu: 2026-11-03. Phạm vi: đọc đủ PDF 15 trang, gồm sơ đồ kiến trúc/pipeline/lộ trình, và đối chiếu tài liệu chính thức. Đây là phân tích thiết kế; chưa thực nghiệm flight stack.

## Kết luận

Giữ simulation-first, depth-first, không train, supervisor riêng và đánh giá theo seed. Sửa semantics cảm biến, geometry/filter, recovery, Offboard liveness và cách chấm trước khi viết node bay.

Các rủi ro dưới đây là **phân tích kỹ thuật**, không phải lỗi đã tái hiện trên runtime của repo. Repo chưa có runtime.

## Những điểm cần sửa trước

| Mức | Vấn đề trong PDF | Vì sao có thể fail | Quyết định | Scenario |
| --- | --- | --- | --- | --- |
| P0 | Invalid đồng loạt, chưa thấy vật gần thì coi trống 10 m | NaN/dropout không cung cấp bằng chứng free; lúc boot không có history | Validity do adapter xác minh; unknown chặn, chỉ `NO_RETURN` rõ nghĩa mới cho free tới range | A01, A02, E01 |
| P0 | Percentile 5% mỗi cột và median 3 frame | Dây ngang/vật nhỏ chiếm <5% cột có thể mất; vật mới có thể bị hai frame cũ xoá | Fast path near evidence, test vật ít pixel; temporal filter phải bù pose và không trì hoãn phanh | A04, A09, E02–E03 |
| P0 | Kẹp velocity proposal là đủ an toàn | Drone đang trượt ngang/phanh có quán tính; giảm setpoint không dừng tức thì | Kiểm tra swept stopping envelope theo measured velocity, latency và braking thực | C07, E14–E15 |
| P0 | Camera trước, RECOVER xoay 360° rồi lùi | Vùng sau vẫn có thể unknown/stale; footprint quay cũng va vào tường | Dừng, kiểm tra rotation envelope, quay nhìn hướng đi, lấy frame mới rồi mới dịch chuyển | A08, B01, E16–E17 |
| P0 | Bridge tiếp tục heartbeat dù proposal bị kẹt | ROS 2 `OffboardControlMode` sống không chứng minh `TrajectorySetpoint` còn mới | Freshness/sequence/epoch riêng cho proposal, safety result và observation; không phát lại lệnh quá hạn | C01, E12 |
| P0 | “Mọi lỗi → ABORT, hover rồi land/ngừng gửi” | Hover cần pose; land cần vùng dưới an toàn; ngừng stream mới chỉ kích hoạt failsafe đã cấu hình | Phân loại lỗi, bounded hold, handoff xác nhận, kiểm tra mode/ack; không re-arm/reclaim | C02–C04, E21–E23 |
| P1 | Chỉ nói NED/ENU, chưa rõ optical/Z và pose sync | Optical frame khác body FLU; Z-depth khác range; pose lệch timestamp gây vật giả/mất vật | Contract intrinsics/extrinsics, clock epoch, capture-time pose; đổi frame tại boundary | A03, A07, C08, E04–E08 |
| P1 | `atan(r/d)` và lọc ±0.5 m cố định | Góc có thể thiếu inflation; margin không bao gồm propellers/pose/tilt; vật lọt ngoài dải vẫn chạm thân | Inflate Cartesian footprint/swept volume; dải cao suy ra từ envelope, không gắn cứng | A05, B05, E13–E14 |
| P1 | 90%/50 world, loại infra crash khỏi tỷ lệ | 45/50 có bất định lớn; loại crash tuỳ ý làm đẹp kết quả | Bộ seed đóng băng, denominator minh bạch, CI thống kê và gate collision riêng | D01–D03, F01–F05 |
| P1 | Timeout đều sim time, chỉ một wall watchdog | Sim reset/pause làm âm data age; máy chậm che latency inference | Sim-time cho mission/physics, monotonic wall cho compute/liveness, epoch cho reset | A07, D03–D04, E08–E09 |
| P1 | DA2 Small và DA3 cùng được mô tả như metric/Apache chung | DA2 Small chuẩn là relative; DA3 license khác theo checkpoint; metric DA3 cần focal scaling | Chọn exact checkpoint, pin weights/license/preprocess; scale/focal phải kiểm tra bằng ground truth offline | E18–E19 |
| P1 | Stack đúng tên được coi đã tương thích | Firmware/messages/Agent/QoS/namespace có thể lệch dù versions “mới nhất” | Lock phiên bản sau smoke; match message definitions và kiểm tra topic/QoS thực | E10–E11, F09 |

P0 chặn mọi thử nghiệm navigation. P1 chặn tuyên bố đạt benchmark/G3. Đánh giá collision/clearance chỉ có giá trị khi evaluator thật sự quan sát ground truth.

### Chi tiết hình học dễ bỏ sót

Với điểm-vật-cản tại range `d` và footprint tròn bán kính `R < d`, nón tiếp tuyến lý tưởng có nửa góc `asin(R/d)`, không phải `atan(R/d)`. Hai góc gần nhau ở xa nhưng lệch khi gần vật. Không thay công thức mù quáng: nếu `d` là optical Z, khoảng cách tới mặt vật, hoặc đã inflate rồi, mô hình đó không còn cùng nghĩa. Chọn kiểm tra Minkowski/swept footprint Cartesian để tránh mơ hồ; `d <= R` phải chặn ngay.

Cột thẳng đứng có thể sống sót percentile theo cột; **dây ngang** vài hàng ảnh thì không. “Đã dùng percentile theo cột” không đủ để kết luận xử lý được mọi vật mảnh. Vật nhỏ hơn độ phân giải góc/pixel ở xa có thể không được sensor quan sát; harness phải công bố giới hạn, không tự gán detection rate 100%.

Median trên pixel không theo cùng điểm không gian khi drone quay/tịnh tiến. Mỗi quan sát cần pose/timestamp, hoặc không sử dụng history cho đường phanh tức thời.

## Các khẳng định của PDF đã đối chiếu

### 1. Stack và simulator

- **Đúng theo tài liệu hiện tại:** ROS 2 Jazzy + Ubuntu 24.04 được PX4 khuyến nghị. Gazebo model `gz_x500_depth` có camera trước mô phỏng OAK-D. [S1, S2]
- **Đúng nhưng cần pin:** Docker image nêu trong PDF tồn tại; nhà cung cấp mô tả đã đóng gói PX4 và message khớp. Không cần bác bỏ image vì tên lạ. Chưa pull hoặc chạy tại repo. [S3]
- Digest ứng viên trong PLAN lấy từ Docker Hub tag API, không phải hash tự đặt. Còn phải kiểm tra firmware/package version và asset depth thật trong G1. [S3]
- DDS firmware và `px4_msgs` cần cùng definitions. PX4 từ v1.16 có cơ chế translation khi khác version, nhưng lựa chọn ít phức tạp nhất ở đây là giữ khớp. [S1]

### 2. Offboard và Collision Prevention

- **Đúng:** Collision Prevention nhắm acceleration-based Position mode, không phải lớp bảo vệ mặc nhiên cho Offboard. [S4]
- **Cần nói chính xác hơn:** ROS 2 dùng `OffboardControlMode` làm proof-of-life; không phải cứ thấy setpoint 10 Hz là đủ. PX4 v1.16 yêu cầu stream trước entry trên 1 s và rate >2 Hz. Chọn mục tiêu 20 Hz để có margin, rồi đo jitter. [S5]
- `OffboardControlMode.velocity=true` không tự vô hiệu hoá position setpoint còn giá trị. Theo contract PX4, velocity control cần position fields là NaN; đây là NaN **đúng protocol**, không giống NaN depth/velocity đầu vào. [S5]
- Khi PX4 tự đổi mode, không phát lại setpoint để giành quyền. Tài liệu `main` hiện còn cảnh báo conflict giữa external/internal publishers; luôn kiểm tra contract của firmware đã pin, không dùng `main` như một release cố định. [S6]
- `ObstacleDistance` v1.16 có 72 phần tử, đơn vị cm: `0` quá gần, `max_distance+1` no obstacle, `65535` unknown. Nó không chứa 3 dải cao/metadata tuổi dữ liệu như contract nội bộ. [S7]
- `/fmu/in/obstacle_distance` có trong DDS topics v1.16.0; không mặc định phải custom-build chỉ để gửi topic này. Kiểm tra lại trên đúng image trước khi kết luận. [S8]

### 3. Cảm biến, clock và middleware

- REP-118 định nghĩa float depth là Z optical theo mét; `16UC1` phổ biến dùng mm với 0 invalid. Không scale mọi encoding giống nhau. [S9]
- REP-117 định nghĩa `-Inf` quá gần, `NaN` lỗi, `+Inf` ngoài tầm. Đây là chuẩn tham chiếu, **không chứng minh driver/model cụ thể tuân thủ**; adapter cần test thực. [S10]
- Optical axes, FLU/ENU và NED khác nhau theo REP-103. Quaternion còn khác thứ tự field giữa interfaces; phải kiểm tra message thực. [S11]
- ROS 2 best-effort publisher không nối được reliable subscriber. Sensor profile ưu tiên dữ liệu mới, queue nhỏ; health cần phát hiện cả incompatible QoS, không chỉ process chết. [S12]

### 4. Pretrained depth và license

- **DA2 Small 24.8M bản chuẩn là relative depth.** Bản metric indoor cụ thể là `depth-anything/Depth-Anything-V2-Metric-Hypersim-Small`; model card ghi Apache-2.0. Dùng đúng architecture/weights/preprocess, không lấy bản relative rồi so ngưỡng mét. [S13, S14]
- DA3 tồn tại, có `DA3METRIC-LARGE` khoảng 0.35B tham số. README yêu cầu `metric_depth = focal * net_output / 300.` cho model metric này; không mặc định raw network output đã là mét. Camera resize/crop làm đổi focal cần cập nhật. [S15]
- **Sửa câu “DA3 là Apache 2.0”**: bảng model có cả CC BY-NC 4.0 và Apache 2.0. DA3Metric-Large là Apache 2.0 theo bảng, nhưng không suy rộng cho mọi model/nested weights. [S15]
- Không chọn DA3 làm mặc định ban đầu: chưa có benchmark latency/hardware của project, model lớn hơn không tự đồng nghĩa phù hợp điều khiển thời gian thực.
- Không copy planner/VIO GPL hoặc redistribute weights trước khi kiểm tra license của exact revision và cách sử dụng. Việc dẫn link nghiên cứu không đồng nghĩa tích hợp code.

## Cải thiện cách làm, không tăng scope vô ích

1. G0 chạy CPU offline; chưa mang ROS, model hay simulator vào bài tập projection.
2. Mỗi feature có scenario ID và oracle cụ thể; không đợi cuối dự án mới thiết kế judge.
3. Đo perception trên bag trước, rồi mới đóng vòng điều khiển thật trong SITL.
4. Chấp nhận safe abort như một outcome riêng, không che nó thành navigation success.
5. Không ép local planner giải mọi mê cung. Nếu cần navigation completeness, mở nhánh global map/planner có gate riêng.
6. Giữ adapter metric depth có uncertainty/validity; đổi source không có nghĩa cả safety/planner khỏi cần đánh giá lại.
7. Giai đoạn 4 vẫn ở Gazebo: dùng RGB/stereo mô phỏng, thay depth adapter. Không “thay Gazebo bằng RGB camera” rồi mất physics/oracle.
8. Bắt đầu harness bằng workflow hữu hạn + bằng chứng độc lập; chưa xây autonomous multi-agent daemon hoặc auto-tune test set.

## Nguồn chính thức

Các nguồn web `main` có thể đổi; khi triển khai phải pin revision. Nguồn release-tagged dùng để kiểm tra contract cụ thể, không phải tuyên bố đã chạy release đó.

| ID | Nguồn | Dùng để đối chiếu |
| --- | --- | --- |
| S1 | [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html) | Jazzy, Ubuntu, DDS/messages |
| S2 | [PX4 v1.16 Gazebo vehicles](https://docs.px4.io/v1.16/en/sim_gazebo_gz/vehicles.html) | Depth/lidar models |
| S3 | [Docker image API](https://hub.docker.com/v2/repositories/px4io/px4-sitl-gazebo-ros2/) và [tag/digest API](https://hub.docker.com/v2/repositories/px4io/px4-sitl-gazebo-ros2/tags/latest/) | Nội dung image theo nhà cung cấp, digest |
| S4 | [PX4 Collision Prevention](https://docs.px4.io/main/en/computer_vision/collision_prevention.html) | Position mode, sensing coverage, tracking delay |
| S5 | [PX4 v1.16 Offboard](https://docs.px4.io/v1.16/en/flight_modes/offboard.html) | Entry, heartbeat, setpoint, failsafe |
| S6 | [PX4 main Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) | Cảnh báo publisher/control conflict hiện tại |
| S7 | [ObstacleDistance v1.16.0](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/msg/ObstacleDistance.msg) | cm, unknown, no-return, angles |
| S8 | [DDS topics v1.16.0](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/uxrce_dds_client/dds_topics.yaml) | Topic input/output thực của release |
| S9 | [REP-118](https://www.ros.org/reps/rep-0118.html) | Depth image encoding/Z |
| S10 | [REP-117](https://www.ros.org/reps/rep-0117.html) | Invalid/near/far semantics |
| S11 | [REP-103](https://www.ros.org/reps/rep-0103.html) | Frames và SI units |
| S12 | [ROS 2 Jazzy QoS](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html) | Sensor freshness và compatibility |
| S13 | [Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2) | Relative model, size |
| S14 | [DA2 metric implementation](https://github.com/DepthAnything/Depth-Anything-V2/tree/main/metric_depth) và [Small indoor model card](https://huggingface.co/depth-anything/Depth-Anything-V2-Metric-Hypersim-Small) | Metric weights, preprocess, license |
| S15 | [Depth Anything 3 README/model cards](https://github.com/ByteDance-Seed/Depth-Anything-3) | Metric scaling, parameters, per-model license |

Context7 được thử nhưng server chưa kết nối; nghiên cứu dùng trực tiếp tài liệu/repo/API chính thức. Không có khảo sát cộng đồng hay thực nghiệm latency để suy ra “stack tốt nhất”.
