# Danh mục scenario và oracle

**Trạng thái: 65 scenario được đặc tả, chưa có scenario nào được chạy trong repo.** A–D là 30 case của PDF; E–F là 35 case bổ sung. Mỗi hàng phải trở thành test/fixture hoặc SITL scenario ở gate phù hợp, không phải một dấu tick mặc định.

## Quy tắc chung

- `PASS` theo oracle từng case, không theo log “success” do planner tự in.
- Mỗi scenario phải pin geometry, footprint, seed, thời điểm inject, config, clock và deadline trước khi chạy.
- Unknown không cho phép tịnh tiến vào corridor đó; quay tại chỗ chỉ được phép khi swept rotation envelope đủ clearance.
- “Brake/hold” nghĩa là yêu cầu dừng trong stopping envelope **khả thi** đã đo; không hứa tránh va chạm với vật spawn bên trong quãng phanh.
- “Fail closed” là reject dữ liệu/lệnh không hợp lệ và đi theo supervisor contract; không nhất thiết tắt động cơ hay dừng heartbeat ngay.
- Negative test pass khi lỗi được phát hiện và xử lý đúng; không cộng vào navigation success.
- Thời gian nhiệm vụ dùng sim time; health/compute dùng monotonic wall time. Reset clock bắt đầu epoch mới.
- Case kính/dây/thiếu sáng cần sensor model/noise phù hợp. Chỉ thay material trong simulator không chứng minh mô phỏng đúng lỗi camera thật.
- Áp dụng [SAFETY.md](SAFETY.md): contact ledger toàn trial, navigation clearance theo phase, support-contact allowlist, unsafe latch và suite membership trước attempt.

## A. Perception — 10 case gốc

| ID | Kích thích | Oracle bắt buộc |
| --- | --- | --- |
| A01 | Tường cách mũi camera 0.2 m, dưới min range | Bin quá gần/unknown bị chặn; không diễn giải NaN là free; không arm khi spawn unsafe |
| A02 | World trống ngoài max range | Chỉ clear tới range khi adapter chứng minh `NO_RETURN`; unknown không trở thành clear |
| A03 | Tăng tốc làm pitch/roll, sàn phẳng | Floor ngoài envelope không sinh obstacle giả; vật thật trong envelope vẫn được giữ |
| A04 | Cột Ø5 cm cách 3 m | Với cấu hình sensor quan sát được cột, evidence sống qua reduction và hướng va chạm bị chặn |
| A05 | Hai vách tạo khe 0.6 m | Nếu khe nhỏ hơn envelope + margin thì không đi; kiểm tra biên rộng hơn để tránh chặn tất cả |
| A06 | Camera ngừng/rớt frame khi bay | Data age vượt deadline kích hoạt brake/bounded hold; stale frame không kéo dài quyền điều khiển |
| A07 | Timestamp ngược, future stamp, một node sai `use_sim_time` | Reject khác clock/epoch; không có data age âm được coi fresh |
| A08 | Goal ngang 90° hoặc sau lưng | Dừng/quay quan sát trước; không bay vào FOV chưa được kiểm chứng |
| A09 | Depth flicker qua bag replay | Giữ phản ứng gần tức thời; không tạo free xuyên vật khi filter dùng history |
| A10 | Vách ít texture, kính, tối với sensor fault phù hợp | Dropout/low confidence chặn hoặc giảm quyền di chuyển; không tuyên bố nhìn được kính chỉ vì sim depth sạch |

## B. Planner — 7 case gốc

| ID | Kích thích | Oracle bắt buộc |
| --- | --- | --- |
| B01 | Ngõ cụt/chữ U | Sau 5 s không tiến triển vào RECOVER; tối đa 3 lần; không lùi mù; safe abort nếu không tìm được lối |
| B02 | Cột giữa hai lối đối xứng | Hysteresis giảm đảo trái/phải; case đảo >3 lần/2 s phải được phát hiện, không che oscillator |
| B03 | Goal trong hộp kín | Reject bởi validator hoặc bounded safe abort ở negative suite; không success/fake goal |
| B04 | Đến goal rồi chịu nhiễu pose nhỏ | Goal hold theo distance/speed/dwell; latch state, không bay vòng quanh |
| B05 | Xà ngang/vật thấp chạm envelope | Mọi dải cắt swept volume đều chặn hướng; không chỉ đọc dải giữa |
| B06 | Spawn hộp khi đang bay | Nếu ngoài stopping envelope thì brake trước planner; nếu bất khả thi thì ghi unsafe outcome, không claim guarantee |
| B07 | Goal qua yaw ±π | Sai số góc shortest-path đúng, không quay gần 2π do wrap; biên ±π nhất quán |

## C. Safety và điều khiển — 8 case gốc

| ID | Kích thích | Oracle bắt buộc |
| --- | --- | --- |
| C01 | Kill perception hoặc planner | Supervisor độc lập phát hiện mất progress/observation; không lặp proposal cũ |
| C02 | Kill supervisor hoặc Offboard bridge | PX4 mất proof-of-life và thực hiện failsafe đã cấu hình; log đo thời điểm, mode và kết quả |
| C03 | PX4 rời Offboard vì battery/geofence/RC | Mission nhường quyền; không re-enter/arm tự động hoặc tiếp tục setpoint can thiệp |
| C04 | Arm/takeoff bị từ chối | Ack/status có lý do; retries hữu hạn; không gửi lặp vô hạn hay đánh dấu airborne giả |
| C05 | EKF reset/odometry nhảy >1 m | Reset planner/history/goal transform thích hợp; không phát spike velocity hoặc dùng stale map |
| C06 | Spawn cách tường 0.3 m | PREFLIGHT chặn nếu envelope cất cánh không an toàn; không chỉ xét tâm drone |
| C07 | Goal xa, obstacles dày, command tốc độ cao | Speed bị bound theo latency/stopping distance; actual velocity và side-slip cũng được giám sát |
| C08 | Trộn NED/ENU và FRD/FLU | Fixture vector/quaternion có đáp án hình học độc lập; positive-up/forward/yaw đúng cả output PX4 |

## D. Đánh giá — 5 case gốc

| ID | Kích thích | Oracle bắt buộc |
| --- | --- | --- |
| D01 | Va chạm thân/propeller, kể cả sau goal/đang land, planner không báo | Latch unsafe toàn trial; chỉ gear/support pair đúng phase/zone được allow; success hoặc infra error không ghi đè collision |
| D02 | Random world có A* grid path nhưng segment cắt góc/khe không đủ footprint | Validator kiểm tra continuous swept route/start/goal trước trial; ghi rejection/seed, không loại theo planner outcome |
| D03 | Gazebo/PX4 treo/crash, kể cả ngay sau collision | Wall watchdog bounded collect/cleanup process sở hữu; giữ cả unsafe latch và infra reason; batch thiếu trial không pass |
| D04 | RTF tụt do tải máy | Sim age và monotonic wall progress báo riêng; workload vượt calibrated bound không tiếp tục motion theo dữ liệu cũ |
| D05 | Drone ra biên hoặc nghiêng >60° | Evaluator fail trial, dừng có kiểm soát; không chờ timeout rồi gán infra error |

## E. Lỗ hổng runtime bổ sung — 25 case

| ID | Kích thích | Oracle bắt buộc |
| --- | --- | --- |
| E01 | Boot với toàn NaN/0, không có frame history | Unknown chặn; “chưa từng thấy vật gần” không được suy ra free |
| E02 | Dây ngang/obstacle chiếm <5% số pixel mỗi cột | Nếu sensor quan sát được, near evidence không bị percentile xoá; công bố giới hạn độ phân giải |
| E03 | Vật mới gần ở frame thứ ba sau hai frame trống | Fast path chặn trong frame đầu có evidence, không đợi median đạt đa số |
| E04 | Đổi `32FC1` mét sang `16UC1` mm; encoding lạ | Chuẩn hoá đúng đơn vị/invalid; encoding không hỗ trợ bị reject, không scale âm thầm |
| E05 | Resize/crop ảnh nhưng CameraInfo cũ | Contract phát hiện mismatch; projection dùng intrinsics đã cập nhật, không tạo chiều sâu/khe giả |
| E06 | Missing/stale TF, đổi extrinsics hoặc `frame_id` | Không xử lý với transform mặc định identity; fresh epoch hoặc invalid observation |
| E07 | Camera/odom lệch thời gian trong lúc yaw nhanh | Interpolation đúng capture time hoặc reject; không dùng pose mới nhất tuỳ tiện |
| E08 | Sim reset về t=0, rosbag seek/loop | Flush history, commands, sequence và TF thuộc epoch cũ; mission không tiếp tục ngoài ý muốn |
| E09 | Pause sim trong khi process/heartbeat còn sống | Không dùng wall tick để giả tạo frame mới; watchdog nhận pause có chủ đích khác hang, resume phải tái xác minh |
| E10 | QoS subscriber reliable gặp publisher best effort | Health báo không kết nối/incompatible; PREFLIGHT không qua chỉ vì thấy tên topic |
| E11 | XRCE Agent restart, packet reorder, namespace/domain trùng | Discovery lại có kiểm soát; reject dữ liệu run/vehicle cũ; không tự arm khi reconnect |
| E12 | Planner freeze nhưng bridge/safety thread còn heartbeat | Proposal hết hạn, không refresh timestamp cho lệnh cũ; safe behavior có trace nguồn lệnh |
| E13 | Bin boundary/wrap, một ray free nhưng phần còn lại unknown, cm saturation | Không clear cả corridor từ một ray; sampling support/inflation không tạo khe, sentinel không overflow thành free |
| E14 | Roll lớn, camera lệch tâm, propellers rộng hơn thân | Swept/rotation envelope và clearance dùng geometry thật, không chỉ sphere tại camera |
| E15 | Side-slip, jerk limit, tracking chậm hơn cấu hình | Phanh theo actual motion và bound đo được; báo vi phạm envelope, không chỉ kẹp proposal |
| E16 | Yaw scan sát tường, mặt trước hở nhưng cánh quạt chạm khi quay | Không scan nếu rotation envelope không đủ; abort thay vì “quay luôn an toàn” |
| E17 | Quét 360° xong, vật đi vào hướng đã nhìn lúc đầu | Per-bin timestamp hết hạn bị chặn; không coi scan cũ là coverage mới |
| E18 | Depth model relative hoặc metric sai scale/focal 2× | Offline gate không qua; không cho bật navigation theo ngưỡng mét dù ảnh depth nhìn hợp lý |
| E19 | Model đo tốt AbsRel toàn ảnh nhưng quá xa ở vật gần/OOD | Đánh giá overestimation trong corridor/near-field và uncertainty; lỗi nguy hiểm không bị average che |
| E20 | NaN/Inf trong pose/velocity/command, distance âm, `a_brake<=0`; đối chứng vector có thành phần âm hợp lệ | Reject invalid physical input/config, chấp nhận signed-axis values đúng frame; NaN unused PX4 fields chỉ ở protocol adapter |
| E21 | Land command bị từ chối hoặc không có ack | Không DONE; bounded retry/timeout, ghi outcome; không ngừng kiểm soát trước handoff thích hợp |
| E22 | Camera mất và odometry cũng invalid | Không giả định zero-velocity là hover; thực hiện PX4 fallback đã test, ghi mất khả năng giữ vị trí |
| E23 | Hai publisher setpoint, bridge restart khi vẫn airborne | Phát hiện owner conflict/restart; không tự nhận quyền/re-arm hoặc phát default position=(0,0,0) |
| E24 | Sàn dốc, bậc cao, gờ cửa, ceiling thấp | Filter không xoá vật cắt envelope; cố định altitude không được coi là clearance khỏi sàn |
| E25 | Sensor lỗi theo mảng tương quan, motion blur, saturation | Unknown/uncertainty không thành free; noise Gaussian độc lập không là test duy nhất |

## F. Evaluator và tính trung thực — 10 case

| ID | Kích thích | Oracle bắt buộc |
| --- | --- | --- |
| F01 | Cùng seed nhưng đổi model/world/plugin/PX4/config | Manifest phân biệt inputs; không gộp như cùng experiment hoặc cùng baseline |
| F02 | Trial lỗi rồi retry nhiều lần | Giữ mọi attempt; report first-attempt và retry riêng; không chỉ chọn run thành công |
| F03 | Thiếu seed, duplicate trial, telemetry mất cuối run | Không aggregate pass; batch incomplete có exit khác 0 và lý do, không fill bằng kết quả giả |
| F04 | Contact sensor thiếu output/propeller mesh; landing support và body-wall contact đồng thời | Oracle health fail nếu thiếu coverage; gear/support đúng phase được allow, body/propeller-wall vẫn unsafe; clearance không double-count planning margin |
| F05 | Goal gần nhưng route vòng dài hoặc time reference không khả thi động lực học | Continuous route/constraints được xác minh; chỉ report detour nếu chưa có time reference đúng, không tự bỏ gate time-efficiency |
| F06 | Inference chậm, RTF thấp nhưng sim latency đẹp | Report wall compute/transport, sim age và RTF riêng; timeout/drop frame thật theo contract |
| F07 | Ground-truth obstacle topic/map lọt vào planner | Topic/interface audit fail benchmark; oracle chỉ evaluator được đọc, không “sửa” bằng giấu trong report |
| F08 | Agent sửa seed/threshold/oracle sau khi thấy fail | Dừng acceptance, yêu cầu quyết định người dùng và baseline mới; không tự làm dễ gate |
| F09 | Máy cũ còn Gazebo/Agent, namespace trùng, asset tải thiếu | Preflight chỉ rõ conflict/asset; không kill process không sở hữu hoặc tải model mới âm thầm |
| F10 | Dynamic obstacle spawn trong quãng phanh không thể tránh, suite đã frozen | Stress report giữ unsafe outcome; test classifier có thể pass nhưng không cộng navigation success; không chuyển navigation failure sang stress sau run |

## Độ bao phủ theo giai đoạn

Mapping gate nằm trong [PLAN.md](PLAN.md). Case E18–E19 chỉ bắt buộc khi có source depth học máy; F06 cần khi benchmark latency và đặc biệt ở G4. Các case còn lại được mở dần nhưng phải hoàn tất trước khi công bố G3 trong phạm vi tĩnh.

Các bài test A–C pass về xử lý lỗi không làm tăng số world navigation thành công. Nhóm D/F kiểm tra chính bộ chấm; evaluator hỏng thì cả batch không có giá trị acceptance.
