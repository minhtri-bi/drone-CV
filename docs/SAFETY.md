# Contract an toàn và acceptance đã sửa

**Đây là sửa thiết kế, chưa phải vá runtime:** repo chưa có flight code. Mọi kiểm chứng bay vẫn pending trong native roadmap. Không được dùng việc merge tài liệu để đánh dấu G0–G4 đạt.

Tài liệu này là contract bổ sung cho [PLAN](PLAN.md), [HARNESS](HARNESS.md) và [scenario](SCENARIOS.md). Các deadline/bounds cần đo được lưu cùng effective config khi triển khai; thiếu bound thì không cấp quyền autonomous motion.

## S01. Contact và clearance theo phase

Quy tắc “clearance ≥0.5 m cả chuyến bay, mọi contact là fail” không thể cùng đúng với cất/hạ cánh trên sàn.

- **Contact ledger theo toàn bộ run**, từ preflight tới disarmed và flush log. Không bỏ collision sau goal hoặc lúc shutdown.
- Landing support contact được phép chỉ giữa **landing gear được chỉ định** và **support surface đã khai báo**. Pair allowlist, surface bounds và phase phải có trong manifest trước trial.
- Chỉ cho phép support pair lúc đứng trên mặt đỡ, chuyển tiếp takeoff, hoặc landing có chủ đích. Contact thân, propeller envelope, tường, trần, đồ vật và mặt ngoài landing zone luôn unsafe.
- Lật hoặc clearance violation không được miễn trừ vì có một support pair hợp lệ đồng thời.
- **Navigation clearance ≥0.5 m** tính từ envelope vật lý thân/propellers đến mọi surface liên quan trong `NAVIGATE`, `RECOVER`, `GOAL_HOLD`. Floor/ceiling cũng tính nếu gần envelope.
- Takeoff/landing có envelope, known-clear volume và giới hạn riêng đã đóng băng; không áp 0.5 m xuống support surface mà chân đang chạm. Không vì thế miễn trừ vật cản khác.
- Lấy minimum clearance riêng theo phase. G3 không pass nếu thiếu phase/contact telemetry.

Oracle dùng hình học vật lý chưa inflate để đo surface clearance. Planner có thể inflate thêm tracking/localization margins; **không trừ cùng margin hai lần trong evaluator**. Propeller collision meshes không đầy đủ thì bổ sung oracle geometry envelope, không mặc định contact sensor sẽ nhìn thấy cánh quạt.

**Chứng minh khi triển khai:** D01/F04 nhận support contact hợp lệ, đồng thời phát hiện body-wall contact; C06/E14 kiểm tra takeoff envelope. Task `g1-oracle`, `g1-flight`.

## S02. Không còn dependency vòng

Thứ tự bắt buộc trước navigation:

1. G0: geometry, validity, braking/freshness core offline.
2. G1 environment: simulator/camera/odom/clock/QoS, **chưa arm**.
3. G1 oracle/logger và supervisor/bridge tối thiểu. Kiểm tra contact/ownership/freshness tĩnh trước flight.
4. Mission thăm dò tốc độ thấp trong known-clear test volume, có supervisor, observer và operator stop.
5. Calibration braking/delay/tracking, cấu hình deadlines/bounds, fault handoff.
6. G2 perception replay, rồi wall-stop **qua supervisor đã có**.
7. G3 planner/recovery/runner/batch; không xây oracle lần đầu sau khi đã dùng nó chấm G2.

Biết trước volume trống chỉ là **fixture thăm dò của harness**, được ghi `bootstrap` trong manifest; không đưa bản đồ thật vào planner, không cộng run đó vào navigation acceptance. Trước calibration, chỉ cho phép kịch bản thăm dò bị chặn bởi volume/low-speed/operator stop riêng, không tuyên bố đã có stopping guarantee.

**Chứng minh khi triển khai:** H1 có oracle/supervisor trước mission; G2 phụ thuộc `g1-calibration`. Roadmap DAG thể hiện dependencies này.

## S03. Lỗi an toàn không bị infra error ghi đè

Kết quả gồm hai trục độc lập:

- `safety_outcome`: `NO_VIOLATION_OBSERVED`, `UNSAFE_OBSERVED`, hoặc `UNKNOWN` khi bằng chứng không đủ.
- `execution_outcome`: `COMPLETED`, `SAFE_ABORT`, `TASK_FAIL`, `INFRA_ERROR`, hoặc `INVALID_WORLD` trước trial.

`UNSAFE_OBSERVED` được latch vĩnh viễn cho attempt. Ví dụ đã va tường rồi Gazebo crash: giữ collision và infra error; không đổi thành một infra run có thể bỏ khỏi mẫu số.

- SUCCESS cần execution completed, goal/termination đúng, observer đủ và không unsafe event; “không thấy” không đủ nếu observer chết.
- `SAFE_ABORT` cần đủ bằng chứng không vi phạm và handoff/dừng đúng contract. Mất oracle không đủ điều kiện gán safe.
- Một event có thể sinh nhiều reason codes; primary report không xoá reason khác.
- Mọi nhánh terminal, kể cả start/health/infra lỗi, đều đi qua **best-effort bounded collect**, sau đó cleanup process của run. Collect lỗi được lưu riêng, không nuốt lỗi ban đầu.
- Exit `1` khi đã quan sát safety/task failure; `2` khi không có failure đã biết nhưng evidence invalid/incomplete; `0` chỉ khi gate đầy đủ đạt.

**Chứng minh khi triển khai:** D01/D03/F02/F03/F04 kiểm tra collision rồi crash, telemetry mất và flush failure. Tasks `g1-oracle`, `g3-runner`, `g3-batch`.

## S04. Tách navigation, negative safety và stress

Suite membership, expected outcome, geometry, fault timing và deadlines phải đóng băng **trước** attempt.

| Suite | Mục tiêu | Acceptance |
| --- | --- | --- |
| `navigation` | Vật tĩnh, route/start/goal hợp lệ, không inject fault bất khả thi | 50 frozen test worlds; G3 ≥45 success, zero unsafe, đủ telemetry và navigation clearance gate |
| `safety-negative` | Từ chối arm, stale input, crash, unreachable goal, mất quyền | PASS theo oracle lỗi/handoff; không cộng numerator navigation, unsafe không được gọi safe abort |
| `stress-diagnostic` | Vật động spawn không thể phanh, giới hạn sensor kính/dây dưới pixel | Ghi cả unsafe/unknown; chấm detector/classifier đúng, không yêu cầu tránh tình huống vật lý bất khả thi |

F10 có thể pass **test phân loại** khi evaluator ghi đúng unavoidable collision. Collision đó vẫn hiện trong report stress, nhưng không làm giả rằng navigation suite đã crash hoặc đã tránh được vật. Không được chuyển một navigation run fail sang stress sau khi thấy kết quả.

**Chứng minh khi triển khai:** B03/B06/F08/F10; task `g3-faults` và frozen manifest.

## S05. Freshness và latency có clock domain rõ

- Giữ capture stamp/epoch theo sim clock; receive/processing/progress theo monotonic wall clock. Không trừ hai timestamp khác domain.
- `sim_age` kiểm tra thế giới mô phỏng đã thay đổi bao lâu; `wall_age`/progress watchdog phát hiện compute/transport bị kẹt. Báo hai giá trị riêng.
- Pause sim chỉ hợp lệ khi runner đã xác nhận pause. Không giả frame mới bằng timer heartbeat; resume phải tái xác minh epoch/pose/coverage và command ownership.
- Deadline cho observation, pose, proposal, supervisor result, handoff và logger phải finite, explicit trong effective config. Không đặt mặc định “vô hạn”.
- Dùng measured conservative delay/braking/jerk bound cho safety; latency p95 là metric, không làm bound.
- Không cộng hai lần cùng khoảng delay: capture-to-safe-publish trace và publish-to-tracking trace có endpoints riêng. Nếu cộng margin, ghi rõ margin ngoài phần đã đo.
- Calibrate ở workload/RTF/config xác định. Load/latency vượt bound làm giảm quyền motion hoặc abort theo policy, không tiếp tục dựa vào calibration cũ.
- Command freshness dựa trên nguồn sequence/epoch và expiry gốc. Bridge không được gia hạn lệnh chỉ bằng đóng dấu publish mới.

**Chứng minh khi triển khai:** A06/A07/E08/E09/E12/E15/F06; task `g0-safety`, `g1-calibration`.

## S06. Free-space evidence có coverage thật

Một ray `NO_RETURN` không chứng minh toàn bộ bin 5° hoặc toàn hành lang thể tích là free.

- Coverage phải phản ánh pixel/ray support, FOV, near/far bounds, timestamp và vùng bị occlusion; không chỉ một boolean “bin có dữ liệu”.
- Corridor chỉ được cấp quyền đến range mà **toàn bộ footprint/swept volume cần kiểm tra** có support theo sensor sampling model đã chốt. Cell/bin chưa support vẫn unknown.
- Tính inflation/độ rộng support trước reduction. Một pixel gần không bị average/percentile hoà vào ray xa trong cùng bin.
- `NO_RETURN` chỉ clear đoạn trong tầm tin cậy của ray đã xác minh; không clear bóng sau vật gần hoặc vùng giữa các ray tuỳ ý.
- Công bố minimum resolvable obstacle size theo range/FOV/resolution. Safety bound không bao gồm vật sensor không thể quan sát; dây dưới pixel thuộc stress suite.

**Chứng minh khi triển khai:** A04/A05/A08/E02/E13/E17; task `g0-geometry`, `g2-adapter`.

## S07. Input hợp lệ không đồng nghĩa mọi số phải không âm

- Vị trí, velocity, yaw và quaternion components có thể âm hợp lệ trong ENU/NED/FLU/FRD.
- Reject NaN/Inf ở physical pose/velocity/command; kiểm tra quaternion norm/order, units và frame theo contract.
- Distance, latency và margin phải trong miền của chúng; `a_brake` phải dương. Negative distance không phải reverse velocity.
- Chỉ adapter PX4 được ghi NaN vào **unused setpoint fields theo protocol**, không cho NaN sensor/actual velocity đi qua.
- Tolerances hình học/số học cần có unit, rationale và fixture độc lập; không mặc định clip quaternion/velocity để che input sai.

**Chứng minh khi triển khai:** C08/E20 phải có valid negative-axis fixtures lẫn invalid physical values. Tasks `g0-contracts`, `g0-geometry`.

## S08. Route validator và efficiency không là oracle toàn năng

- A* grid-centre connectivity chưa chứng minh drone đi qua được khe hoặc chỗ cắt góc. Phải validate liên tục các segment theo footprint, margin, altitude và geofence.
- Start/goal và volume takeoff/landing hợp lệ được kiểm tra riêng, không chỉ tìm route giữa hai tâm cell.
- Oracle route có cùng motion constraints cần cho phép so sánh: speed bound, stopping/turn assumptions và protocol NAVIGATE-to-goal-hold.
- Nếu chưa có reference time khả thi theo dynamics, chỉ report path length/detour ratio; **không chấm time-to-goal ≤2× một con số tự nhận tối ưu**. Gate time-efficiency chưa có reference thì chưa đạt, không bỏ gate.
- Validator được đóng băng trước test seed generation; giữ mọi rejection và thử lại generator có cap. Không loại world vì local planner fail.

**Chứng minh khi triển khai:** D02/F01/F05/F08 và A05; task `g3-worlds` rồi `g3-batch`.

## Trạng thái đóng lỗ hổng

S01–S08 được đóng ở **mức contract** trong đợt chuẩn bị này. Runtime proof được phân bổ tới task/scenario nêu trên và vẫn pending. Deadline numerical, environment lock, stopping calibration và oracle geometry không được coi đã có trước khi task tương ứng chạy thật.
