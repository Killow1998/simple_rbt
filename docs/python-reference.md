# simple_rbt 的 MuJoCo 原型

目前可在 Jammy 運行 **七個配置**，共用雷達、IMU、相機和 ROS 2 介面。
`run.sh` 預設啟動 **C++17** 執行程式：物理調度、七種控制器、感測器、ROS 2 和桌面視窗
均在原生程式中執行。GPU 雷達使用預先編譯的 CUDA kernel；執行時不啟動 Python、Warp 或 LLVM。
Python 版本保留為模型準備及測試對照，用 `reference/python/run.sh` 明確選擇。
[C++ 遷移與資源驗證](cpp-runtime-validation-20261013.md)保存遷移階段的完整驗收和 CPU／RAM 對照；
本輪[架構整理與消融驗證](cpp-architecture-ablation-20261013.md)記錄模組拆分、等價性、故障攔截及交錯 A/B 實測。
三台輪式車的程序峰值 RSS 均值減少 8.7–13.2 MiB，CPU 差異不足以宣稱明確加速。
上述資源數據對應腳輪修正前的優化快照。雙輪參考模型的正常腳輪接地誤判現已修正，
仍保留車殼撞地與原安全門檻。最終七台各通過 31/31 正式檢查，CTest 10/10、
2026-10-13 版本的 Python 392 tests＋65 subtests 全數通過，無 skip；完整版本身份和結果見上方階段紀錄。
[原生建置說明](runtime.md)列出依賴與目前模組。

Go2／Go2W 已加入平地、坡道、樓梯和碎石式起伏地形；Tracer、Limo 等尚未移植。
[輪式底盤部署記錄](wheel-chassis-validation-20261013.md)保存 FR-mini、Ridgeback、Ranger Mini
及 Scout Mini 的連續地形、運動、ROS 和資源驗證。
純輪式底盤使用 `flat`、`ramp`、`hills`；不接受有階躍的 `stairs` 或方塊式 `rough`。
各車依公開規格建立程序幾何，輪胎／馬達／質量分布是工程近似，原廠獨立懸架未校準。
[Scout Mini 第一階段](scout-mini-validation-20261013.md)保留之前的歷史結果。

| `--robot` | 運動結構 | `auto` 控制器 | 支援側移／原地轉向 |
|---|---|---|---|
| `go2` | 四足 | `robot_lab` | 是／是 |
| `go2w` | 輪足 | `rl_sar` | 是／是 |
| `diffdrive` | 雙輪差速參考 | `none` | 否／是 |
| `scout_mini` | Scout Mini 四輪差速 | `skid_steer` | 否／是 |
| `fr_mini` | 前阿克曼、兩後輪驅動 | `ackermann` | 否／否 |
| `ridgeback` | 四輪轂＋40 個被動麥輪滾子 | `mecanum` | 是／是 |
| `ranger_mini` | 四個舵關節＋四個驅動關節 | `swerve` | 是／是 |
[第一階段記錄](validation-20261013.md)保留最初介面驗證；
[多地形與資源實測](terrain-resources-20261013.md)記錄本階段結果和限制。
[較高地形測試](hard-terrain-20261012.md)進一步測了 12–22 cm 台階及 15–25° 長坡，
54 案中 34 案走完全程，兩台能力差異與失敗原因均保留。
高難度對照的指令前進速度為 **0.3、0.6、1.0 m/s**，實際速度另行測量。
目前適配器下，同場景 Go2 HIMLoco 補測完成 6/27 路線，RobotLab 為 10/27；
[策略對照紀錄](hard-terrain-20261012.md#go2-himloco-同場景補測)保留逐案結果及失敗演示。
[部署與完整聯合驗收](deployment-validation-20261013.md)進一步確認起身／扭矩／材料因素，
Go2W 9 組完整 ROS＋感測器案例有 8 組全部合格，低抓地坡道失敗亦保留。
上述歷史案例使用舊版快照雷達；目前 MID360 改用逐點運動掃描和獨立內置 IMU，
本階段的資料規格、重新驗證與資源結果見 [MID360 對齊記錄](mid360-validation-20261013.md)。

在原始 `standard` 低難度地形，預設控制器加回授的兩台機器人，合計 30/30 組完整 6 m 路線通過，
並各通過原地轉向、側移、倒退、邊走邊轉和停車的平地檢查。
Go2 在 0.6／1.0 m/s 仍有步態偏航擺動；1.0 m/s 指令的平均實速約 0.87–0.91 m/s。
這些測試使用理想狀態回授，不能視為實機驗收。
本機 [六格地形演示](../output/stage2-20261013/terrain-preview/terrain-preview.gif)
顯示兩台機器人的真實物理運行，1.5 倍速回放；輸出資料不納入 Git。

## 在 Jammy 直接運行

從倉庫根目錄執行；Jammy 已完成原生建置。新環境需要 `git`、`uv`、Python 3.10 作離線準備，
以及原生建置說明中的 C++／CUDA／ROS／圖形開發套件，再執行：

```bash
./setup.sh
```

只重編譯 C++ 時執行 `./setup.sh --native-only`。Python 對照仍可用
`./reference/python/run.sh`，參數與原入口相同。

啟動 Go2W 上下樓梯，同時開啟完整感測器與 10 Hz 雙相機：

```bash
./run.sh --robot go2w --terrain stairs --backend warp \
  --cmd .6 0 0 --settle 2 --duration 15 --sensors full --camera-hz 10 --realtime
```

在桌面查看機器人（會開啟 MuJoCo 視窗）：

```bash
./run.sh --robot go2 --terrain ramp --backend warp \
  --cmd .6 0 0 --settle 2 --duration 15 --sensors full --camera-hz 10 --viewer
```

切換成雙輪底盤；不使用 GPU 雷達時選 `cpu`：

```bash
./run.sh --robot diffdrive --backend cpu --cmd .3 0 .2 --duration 10
```

啟動 Scout Mini 完整感測器、原生輪速控制和實時時鐘：

```bash
./run.sh --robot scout_mini --terrain hills --backend warp \
  --control-mode native --cmd .3 0 0 --settle 2 --duration 25 \
  --sensors full --camera-hz 10 --realtime
```

三台新底盤使用相同入口；例如 FR-mini 走連續波浪、Ridgeback 做平地複合運動、Ranger Mini 走坡道：

```bash
./run.sh --robot fr_mini --terrain hills --terrain-level moderate \
  --cmd .6 0 0 --settle 2 --duration 14 --backend warp --sensors full --camera-hz 10 --realtime
./run.sh --robot ridgeback --terrain flat --cmd .3 .2 .3 \
  --settle 2 --duration 10 --backend warp --sensors full --camera-hz 10 --realtime
./run.sh --robot ranger_mini --terrain ramp --cmd .6 0 0 \
  --settle 2 --duration 14 --backend warp --sensors full --camera-hz 10 --realtime
```

`--cmd` 的三個值依序是前進速度、側移速度、轉向角速度，單位 m/s、m/s、rad/s。
Go2／Go2W 的指令範圍為 vx ±1、vy ±0.6、wz ±1。
雙輪底盤、Scout Mini 和 FR-mini 不支援側移；FR-mini 也不接受原地轉向。
Scout Mini 的範圍為 vx ±1.5、vy=0、wz ±1；雙輪底盤和 FR-mini 為 vx ±1、vy=0、wz ±1。
Ridgeback／Ranger Mini 為 vx ±1、vy ±0.6、wz ±1。FR-mini 的轉向還受最小半徑 1.1 m 的模型解讀限制，
實際可發給輪組的 `|wz| <= |vx|/1.1`，限幅會記錄。FR 的 `base_link` 與速度作用點位於後軸中心上方，
車殼中心在其前方 0.2 m；其他底盤的參考點見 manifest。
CLI 非法輸入報錯；C++／Python `Simulation` 非法速度清零後報錯；
ROS 非法輸入清零並繼續仿真，後續有效指令可恢復。
這些是輸入限幅，不能視為任意地形、速度組合都能準確追蹤。
Go2 與 Go2W 預設使用各自的 rl_sar RobotLab 策略；指令不等於實際速度。
CLI 預設 `--control-mode tracked`，用仿真的理想機體速度與朝向回授修正發給策略的指令，
減少原生策略的側偏；沒有直接移動機體或額外施力。這是仿真輔助，不能作為實機控制驗收。
`--control-mode native` 可關閉外層理想機體速度／朝向回授；Python `Simulation` 預設不啟用它。
C++ 的 `SimulationOptions` 預設與 CLI 一樣是 `tracked`；直接使用 C++ API 時也應明確選擇控制模式。
Scout Mini 的原生 `skid_steer` 仍使用理想陀螺儀 z 角速度 PI 修正輪速，補償轉向側滑，
只控制輪子、保留接觸摩擦與扭矩限幅；這不是松靈原廠韌體。`tracked` 在其外層再加入理想機體回授。
Go2 可用 `--controller legacy` 或 `himloco` 對照其他策略。
Go2 的 `--go2-torque-mode configured|upstream` 和 `--go2-startup crouched|upstream_getup`
提供部署對照，預設仍為 `configured + crouched`。
GetUp 約 3.005 秒，CLI 的 `--duration` 包含起身及 `--settle`；指令窗口從完成起身並穩定後開始。
GetUp 曾讓 12 cm 樓梯走完，但速度仍不合格，因此保留為實驗選項，見上方部署紀錄。

`--terrain flat|ramp|hills|stairs|rough` 選擇地形，`--seed` 固定方塊式起伏排列。
`hills` 是 4 m 路段上的兩個連續波浪，48 個斜面接縫高度一致；它是分段平面近似，沒有垂直台階，
斜率在接縫會改變。高度查詢、真實碰撞與 CPU／CUDA 雷達使用同一份幾何。
`--terrain-level` 選擇一組明確的地形尺寸；每條樓梯與坡道都包含上行、平台和下行。

| level | 樓梯每階 × 級數／總高 | 坡度 × 單側水平長度／頂高 | `rough`／`hills` 峰高 |
|---|---|---|---|
| `standard` | 6 cm × 4／0.24 m | 8° × 2 m／0.28 m | 2.5 cm |
| `moderate` | 12 cm × 6／0.72 m | 15° × 3 m／0.80 m | 5 cm |
| `hard` | 18 cm × 6／1.08 m | 20° × 4 m／1.46 m | 8 cm |
| `extreme` | 22 cm × 8／1.76 m | 25° × 4 m／1.87 m | 12 cm |

新增樓梯踏面深度 30 cm；原始踏面 40 cm。高難度長坡的完成線延伸至 x=10.6 m，
評估器會相應延長時間上限，要求完整下坡後越過終點。
`--difficulty` 仍可縮放 `standard` 坡度與高度（0–3，不含 0）；其他 level 固定為 1，避免雙重縮放。
`--friction MU` 可測明確的地面材料，MU 為 (0,2]：floor／course 使用 priority2、condim6，
係數為 `(MU,0.02,0.01)`；省略時保留模型原材料混合。實際輪／腳摩擦可能覆蓋地形名義值，
材料對照需檢查 contact.friction。已測 25° 坡道低抓地時會超時，不能只看坡度判定能否通過。
這些是測試場景尺寸，不代表控制器已通過每一檔。
本機演示：[18 cm 樓梯／20° 坡道](../output/hard-terrain-20261012/preview-hard-v06-final/terrain-preview.gif)、
[22 cm 樓梯／25° 坡道](../output/hard-terrain-20261012/preview-extreme-v03-final/terrain-preview.gif)，成功與失敗均保留。

例如測試更高的樓梯：

```bash
./run.sh --robot go2w --terrain stairs --terrain-level hard \
  --cmd .6 0 0 --settle 2 --duration 20 --backend warp --viewer
```

`--sensors physics|lidar|full` 控制負載；`full` 啟用雷達、IMU、普通和全景相機。
相機預設 1 Hz，完整運行建議明確指定 `--camera-hz 10`。
`--summary-only` 仍計算所選感測器，只保存摘要以限制磁碟輸出。

每次運行會建立新的 `output/mujoco/native-<識別碼>/`，包含：

- `summary.json`：位移、控制模式、地形、感測器耗時、CPU／RAM 和模型限制。
- `trajectory.csv`：10 Hz 地面真值軌跡和機體座標速度。
- `mid360-imu-si.csv`：200 Hz 內置 IMU，比力用 m/s²、角速度用 rad/s；沒有姿態估計。
- `lidar-xyz.npy`、`lidar-offsets-ns.npy`、`lidar-lines.npy`、`lidar-ray-indices.npy`：
  最後一幀原始點雲、每點時間／線號與 CSV 索引；`lidar-metadata.json` 保存開始時間和幀末參考位姿。
  `lidar-capture-position-world.npy` 和 `lidar-capture-quaternion-world-xyzw.npy` 保存每點採集位姿。
  Python 對照版本仍保存 `last-scan.npz`。
- `--cameras` 另存普通／全景 RGB PNG、深度 NPY 和 `camera-metadata.json`，包含時間、位姿與標定。

不同感測器的保存時間可能不同，應讀取各自的 metadata，不能只按檔名視為同步幀。
普通相機深度是沿光軸的 Z 距離，全景深度是從相機中心到表面的徑向距離；均以公尺表示，無表面為 `inf`。

## ROS 2

使用 Humble 與本專案 `ros2_interfaces/` 建置的 Livox 訊息库；未載入硬體驅動。
其他機器可用 `SIMPLE_RBT_LIVOX_PREFIX` 指定套件安裝前綴，建置時也可用
`SIMPLE_RBT_LIVOX_SETUP` 指定 `local_setup.bash`。目前完整 C++ 執行程式建置／載入需要 ROS 庫；
不加 `--ros2` 時不建立 ROS 節點，仍可獨立運行仿真。

終端一：

```bash
./run.sh --robot go2w --terrain rough --backend warp --ros2 \
  --sensors full --camera-hz 10 --duration 60
```

終端二：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=83
export ROS_LOCALHOST_ONLY=1
ros2 topic pub -r 10 /go2w/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}, angular: {z: 0.1}}'
```

控制話題按 `--robot` 自動選擇：Go2 用 `/go2/cmd_vel`，Go2W 用 `/go2w/cmd_vel`，
Scout Mini 用 `/scout_mini/cmd_vel`，FR-mini 用 `/fr_mini/cmd_vel`，Ridgeback 用 `/ridgeback/cmd_vel`，
Ranger Mini 用 `/ranger_mini/cmd_vel`，雙輪底盤用 `/diffdrive/cmd_vel`。節點也在相同的機器人命名空間中。
收到所選底盤不支援的側移或原地轉向 ROS 指令時，整條速度指令立即清零，仿真繼續運行；後續有效指令可恢復。
停止發送指令後，0.5 秒超時會把速度指令設為零。`--ros2` 模式由對應話題控制，`--cmd` 不生效。
兩個終端應使用相同 domain；預設為 localhost / domain 83。下游節點使用 `use_sim_time:=true`。

| 話題 | 內容／頻率 |
|---|---|
| `/<robot>/cmd_vel` | 上表所選機器人的平面速度輸入，`geometry_msgs/msg/Twist` |
| `/clock` | 每個物理步：Go2 RobotLab／HIM／四台實名輪式底盤 200 Hz、Go2W 500 Hz、legacy／雙輪 250 Hz |
| `/livox/lidar` | `livox_ros_driver2/CustomMsg`，10 Hz，每幀最多 20,000 點 |
| `/livox/points` | 同一點雲的 `PointCloud2`，10 Hz |
| `/livox/imu` | 200 Hz，本機 Livox ROS 2 驅動相容：加速度 **g**、角速度 rad/s；header 為 `livox_frame` |
| `/livox/imu_si` | 同一 IMU，200 Hz，標準 SI：m/s²、rad/s；header 為真實內置 IMU 的 `imu_frame` |
| `/odom` | 地面真值里程計，10 Hz；不是估計器輸出 |
| `/tf`、`/tf_static` | `odom → base_link → livox_frame / imu_frame / camera_optical_frame` |
| `/camera/image_raw`、`/camera/depth/image_raw` | 普通 RGB／Z 深度，320×240，`--camera-hz` 指定頻率 |
| `/camera/camera_info` | 普通相機針孔標定 |
| `/camera/panorama/image_raw`、`/camera/panorama/depth/image_raw` | 360°×180° RGB／徑向深度，720×360，同一相機頻率 |

全景影像不是針孔模型，因此不發布假的 `CameraInfo`。
控制話題按車名隔離；感測器話題與 TF frame 仍是共用絕對名稱，目前驗證的是單機器人運行，
不是同一 ROS domain 下的完整多機器人感測器隔離。
感測器外參來自實際 MuJoCo site。Go2 的 MID360 放在頂部，並非上游示例的倒置前雷達位置。
原廠驅動的 raw IMU header 名稱沿用雷達 frame，但加速度的作用點仍在內置 IMU 原點。
`/livox/imu` 使用本機 ROS 2 驅動未填姿態時的固定 identity 占位，covariance 為零，**不是姿態量測**。
`/livox/imu_si` 明確標示沒有姿態（orientation covariance 首項 -1）；ROS 真值姿態由 `/odom` 和動態 TF 提供。
原本接收 `/livox/imu` 並假設 m/s² 的程式，應改訂閱 `/livox/imu_si`。

## 雷達模型和目前邊界

沿用倉庫的 `mid360-real-centr.csv`（800,000 條方向），200 kHz、4 線、10 Hz。
CSV 第一欄的 `Time/s` 標題不準確，實際內容是連續點序號；時間由 200 kHz 採樣率生成。
`CustomMsg.timebase` 保存完整模擬奈秒時間，漏點過濾會保留原始點的時間偏移和線號。

預設 `--lidar-motion rolling`：每個點按自身採集時刻的雷達位姿射線求交，
原始 XYZ 保持在該時刻的雷達局部座標，未預先去畸變。位姿採原生物理步節點，平移內插、旋轉短弧 SLERP；不外推。
`--lidar-motion snapshot` 是幀末除錯快照，全部點的時間偏移為零，不冒充運動掃描。
逐點扫描共用一份靜態環境，排除整台機器人的幾何；尚未模擬自身遮擋或動態外部障礙。
CSV 方向、200 kHz 均勻時間和四線索引是模擬採樣約定，不是原始硬體時間記錄。

MID360 內置 IMU 在雷達座標中位於 **[+0.011,+0.02329,-0.04412] m**，座標軸平行，
按 [官方手冊](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf) 的標稱幾何實作。
獨立 MuJoCo 感測點自然包含旋轉杆臂加速度，控制器仍讀原機身 IMU。
IMU 使用共用模擬時鐘、零時刻起的 5 ms 栅格；Go2W 的 2 ms 物理步用相鄰讀值內插，最多延遲 1 ms 才發出。
這是時間內插近似。預設理想、無噪聲／偏置／晶片濾波，亦不截斷硬體量程；Python sampler 可選 ±4g／±2000°/s 截斷。
尚未復現完整光學、UDP／網路延遲、逐台實測標定；反射強度固定。
本階段沒有執行 FAST-LIO 建圖，也沒有與 Gazebo 做同場景控制品質或速度對照。

## 重新驗證

原生 Release 傳輸、隨機序列和驗收邊界測試（先 source Humble 與 Livox）。目前 CTest 共 10 項：
七台各自的指令進程、一個感測器進程、隨機序列、路線門檻。ROS 測試共用 domain 89 資源鎖。
開發時可先用 `-L course` 或 `-L ros` 選擇受影響模組，再按介面變更重跑真實聯合驗收：

```bash
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 ctest --test-dir build/native --output-on-failure
```

完整 C++ ROS＋感測器路線（先 source Humble 與 Livox）：

```bash
.venv/bin/python tools/validate_full_run.py --runtime cpp \
  --robot ridgeback --terrain hills --terrain-level moderate --control-mode native \
  --speed .6 --output output/cpp-full-new
```

以下 Python 測試及地形矩陣是參考模型驗證；不代表執行了 C++ 版本。
核心、CSV、相機和傳輸的 Python 對照檢查：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MUJOCO_GL=egl SIMPLE_RBT_TEST_RENDER=1 SIMPLE_RBT_TEST_WARP=1 SIMPLE_RBT_TEST_CUDA=1 SIMPLE_RBT_ROS_TESTS=1 \
  .venv/bin/python -m pytest -q reference/python/tests
```

CPU／CUDA 在同一批移動場景射線上的對照：

```bash
.venv/bin/python tools/benchmark_lidar.py \
  --frames 100 --warmup 5 --rays 20000 --gpu-device cuda:0 \
  --output output/lidar-benchmark.json
```

詳細結果包含所有誤差，不以平均速度掩蓋命中差異。`--require-agreement` 要求命中索引完全一致，
舊快照測試曾有一個牆角邊界判定差異；新 rolling 模式的獨立結果見上方 MID360 對齊記錄，兩者不能混用。

完整 ROS 2 實際發布／接收驗證命令見腳本 `tools/check_ros2.py --help` 及本輪結果記錄。
輪式的所有支援動作、坡道和連續波浪可用統一入口重跑；需要新的輸出目錄：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python \
  tools/evaluate_wheels.py --control-modes native \
  --output output/wheel-matrix-new
```

預設四台實名輪式底盤，0.3／0.6／1 m/s、8°／15° 坡道、2.5／5 cm 波浪；
FR-mini 不生成側移與原地轉向案例。逐項失敗保留於結果，矩陣完成不代表每案通過。
當前 Ridgeback 15° 坡入口會觸底，FR-mini 原生 15° 下坡會超速；
全程平均通過也不能取代坡段速度檢查，詳見上方部署記錄。

分別量測物理、CPU／CUDA 雷達及完整雙相機的資源和速度（使用新的輸出目錄）：

```bash
.venv/bin/python tools/profile_resources.py \
  --runtime cpp --duration 6 --output output/resource-profile-new
```

每種設定使用獨立程序，RAM 不會混入前一種設定的峰值。CPU 100% 表示一個邏輯核心；
paced 結果反映實時負載，unpaced 結果反映不等待時鐘的運行速度。
腳本預設 ONNX／BLAS 單執行緒；不需要安裝 Torch。專案磁碟統計不含共用快取或系統 ROS。
在上述資源命令加入 `--robots scout_mini --terrain hills --control-mode native --speed 1.5` 可單獨量測 Scout；
預設仍為 `tracked`。速度上限按所選機器人檢查，混合 Go2／Go2W 的批次仍限 1 m/s。

重跑兩台機器人完整路線與獨立速度檢查：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python \
  tools/evaluate_terrains.py --go2-controllers robot_lab \
  --control-modes native tracked --output output/terrain-matrix-new
```

## 程式入口與來源

目前執行來源：

| 檔案 | 責任 |
|---|---|
| `../src/core.cpp`、`scene.cpp` | 物理步進與模型／地形裝配 |
| `../src/robots.cpp` | `RobotSpec`：車型能力、限幅與預設控制器 |
| `../src/controllers/`、`controller_policy.cpp`、`tracking.cpp` | 七種控制實作、ONNX 策略與外層回授 |
| `../src/runtime.cpp` | 共用時鐘、200 Hz IMU、10 Hz rolling 雷達、相機排程 |
| `../src/artifacts.cpp`、`resources.cpp` | 輸出檔案與程序資源量測 |
| `../src/lidar.cpp`、`lidar_cuda.cu` | MID360 CSV、位姿內插、CPU／CUDA 射線 |
| `../src/cameras.cpp`、`viewer.cpp` | 普通／全景感測相機與桌面視窗 |
| `../src/ros_bridge.cpp` | 真實 DDS 收發、命名空間與超時停車 |
| `../src/course.cpp`、`course_geometry.cpp` | 路線／速度、安全包絡、接觸與實際摩擦門檻 |
| `../src/full_run.cpp` | 同次物理／感測器運行的證據、雜湊與 DDS 握手 |
| `scripts/validate_full_run.py`、`scripts/full_run_*.py` | 獨立 Python 驗收入口、參考 worker、DDS 接收、31 項判定與證據檔案 |

以下保留為 Python 模型準備與對照來源：

| 檔案 | 責任 |
|---|---|
| `simple_rbt_mujoco/core.py` | 機器人、控制器、物理步進、理想 IMU |
| `simple_rbt_mujoco/go2.py`、`go2w.py` | 各機器人的原生觀測、動作與 PD 控制合約 |
| `simple_rbt_mujoco/scout_mini.py` | Scout Mini 參數模型、輪速與陀螺儀轉向控制 |
| `simple_rbt_mujoco/fr_mini.py`、`ridgeback.py`、`ranger_mini.py` | 阿克曼、麥輪滾子、舵輪的接觸驅動模型與輪組控制 |
| `simple_rbt_mujoco/robots.py` | 车型選擇、輸入限幅、運動與地形契約 |
| `simple_rbt_mujoco/wheel_geometry.py` | 真實車體／輪組包絡、世界走廊投影與非輪組接觸 |
| `simple_rbt_mujoco/command_tracking.py` | 可關閉的理想速度與朝向回授 |
| `simple_rbt_mujoco/terrains.py` | 可重現地形與對應高度查詢 |
| `simple_rbt_mujoco/lidar.py` | CSV 掃描方向與幀末快照 |
| `simple_rbt_mujoco/rolling_lidar.py` | 原生位姿歷史、逐點移動射線、CPU／Warp 共用靜態環境 |
| `simple_rbt_mujoco/mid360.py` | 內置 IMU 幾何、200 Hz 採樣與明確單位 |
| `simple_rbt_mujoco/cameras.py` | 普通相機、六面拼接全景 |
| `simple_rbt_mujoco/ros2.py` | ROS 2 訊息轉換與指令超時 |
| `simple_rbt_mujoco/cli.py` | 可執行迴圈與資料保存 |

雷達與 legacy Go2 模型／ONNX 控制器固定於
[`e6b879df`](https://github.com/discoverse-dev/MuJoCo-LiDAR/tree/e6b879df4b96daaed30ef10fe02650d29048fa9b)。
legacy Go2 接口沿用其 `examples/unitree_go2.py`；模型只在記憶體中加裝感測器和測試場景。
來源保留在 `.vendor/`，MIT 授權副本見 [LICENSE-MuJoCo-LiDAR.txt](LICENSE-MuJoCo-LiDAR.txt)。
`requirements.lock` 固定本輪 Python 3.10 的依賴，`setup.sh` 固定上游提交。
目前預設 Go2／Go2W 的來源、逐檔雜湊與 ONNX 匯出驗證在 `assets/go2_manifest.json`、
`assets/go2w_manifest.json` 及 [Go2W 來源說明](../assets/manifests/GO2W_ASSETS.md)。只下載所需模型檔；
模型庫未找到根目錄授權，mesh 保留在被 Git 忽略的本機 vendor，未重新發布。
Scout Mini 使用原創程序幾何，不下載 vendor mesh 或控制權重；尺寸來源、工程假設、
被捨棄的舊模型參數和版本差異記於 `assets/scout_mini_manifest.json`。
FR-mini、Ridgeback、Ranger Mini 同樣使用原創程序幾何，來源與工程假設記於各自的 `*_manifest.json`。
