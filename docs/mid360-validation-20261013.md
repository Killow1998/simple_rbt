# MID360 關係、逐點運動掃描與完整驗收

日期：2026-10-13。此階段在 Jammy 宿主執行；GPU／CUDA／渲染未在沙盒驗證。

目前 Go2 與 Go2W 都能接受速度指令，由控制器和物理仿真產生運動，
輸出實際位置、姿態、機體速度，以及 IMU、雷達、普通與全景相機資料。
`/odom` 和動態 TF 是仿真真值；速度指令不等於瞬時實速。
本階段完成 MID360 標稱內置 IMU 幾何、共用時間與本機驅動格式的對齊。
原生控制器、物理步長和原控制器 IMU 保持原實作，未重新訓練策略。

## 修正的問題

舊版輸出的是機身 IMU；其與雷達的距離並非 MID360 內置 IMU 的位置。
雷達也在幀末一次求交，卻給每點附上較早的時間；融合器去畸變會對這種資料作錯誤補償。
現在新增獨立感測點，並用每個雷達點採集時刻的位姿求交。快照除錯模式改成幀末時標、零偏移。

原 `/livox/imu` 曾使用 SI 加速度和真值姿態，PointCloud2 也曾是自訂 24-byte 格式。
目前明確分成原廠 raw 相容與標準 SI 兩話題；沒有把真值姿態當作 IMU 量測。

## 固定幾何與資料規格

內置 IMU 在雷達點雲座標中位於 `[+0.011,+0.02329,-0.04412] m`，兩者軸平行：

```text
p_lidar = p_imu + [0.011, 0.02329, -0.04412]
p_imu   = p_lidar + [-0.011, -0.02329, 0.04412]
```

第一行用於安裝 IMU；第二行是 FAST-LIO 的 LiDAR→IMU `extrinsic_T` 方向。
此為 [MID360 官方手冊](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf)
印刷頁 15 的標稱幾何，非逐台實測標定。方向亦對照
[FAST-LIO 配置](https://github.com/hku-mars/FAST_LIO/blob/main/config/mid360.yaml)
與其 README 的外參定義。模型安裝與輸出 metadata 共用同一常數，並保存校準 SHA256。
加速度在新 site 直接求值，包含杆臂的向心／切向加速度。

| 輸出 | 規格 |
|---|---|
| `/livox/imu` | 200 Hz；角速度 rad/s、加速度 g；float32 線上精度再轉 ROS double；header `livox_frame` |
| `/livox/imu_si` | 同一感測器 200 Hz；rad/s、m/s²；header `imu_frame`；沒有姿態，covariance 首項 -1 |
| raw 姿態 | 固定 identity `[0,0,0,1]`、covariance 全零，匹配本機 Humble C++ 驅動未填姿態的預設；只是占位 |
| `/livox/lidar` | CustomMsg、10 Hz、每幀 20,000 條射線；正常點 tag=0，line 0–3 |
| `/livox/points` | 同批點；26-byte packed PointCloud2；x/y/z/intensity float32，tag/line uint8，timestamp float64 為絕對 ns |
| 相機 | 普通 320×240 RGB／Z 深度；全景 720×360 RGB／徑向深度；本次均 10 Hz |
| `/odom`／動態 TF | 10 Hz 真值位姿與機體速度；兩者互相驗證 |

g 與 SI 的換算使用 `g0=9.80665 m/s²`。
raw header 沿用本機驱動慣例，但作用點仍在內置 IMU 原點；實際原點由 `imu_frame` TF 表示。
不能把 raw 占位 quaternion 當作姿態估計，也不能把 g 當作 m/s²。
依據：[官方 MID360 協議](https://github.com/Livox-SDK/livox_wiki_en/blob/6b030b73a7fa1e035e44bf1b262d7a9cbebe0728/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md)、
[Livox ROS 2 驅動](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/src/lddc.cpp)、
[ROS Imu 規格](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg)。

本機驅動 `kLineNumberMid360=4`，線號按包內點索引 modulo 4 生成；
仿真的 CSV 索引同樣 modulo 4。不同驅動版本的未填姿態預設需重新核對，不能套用 ROS 1 的常識。

## 時間与幾何驗證

採用 MID360 的 200 Hz 配置，從 0 開始每 5 ms 一條 IMU，沒有以物理步數取整來冒充採樣率。
Go2W 物理步為 2 ms，在 5、15…ms 的讀值由相鄰步內插，等待右端樣本最多 1 ms；
Go2 RobotLab／HIM 的 5 ms 格點直接讀取。每條样本保存原始時間括號及是否內插。
這是時間重採樣近似，不是晶片濾波模型。

逐點雷達在每個 `scan_start+offset` 時刻取得世界位姿，用該點的 origin／direction 對靜態環境求交。
raw XYZ 保留在該時刻的雷達局部座標，未提前去畸變。
位姿歷史使用原生物理步節點、平移線性內插和短弧 SLERP，保留至少 250 ms，禁止外推。
一份場景與 BVH 共用全部射線，沒有為 20,000 點複製 20,000 份環境。

解析平面測試用 20,000 點、1 m/s 平移與 0.5 rad/s 轉向，逐點回投誤差不超過 1 µm；
舊快照加合成時間的負例產生超過 9 cm 的誤差。
旋轉杆臂、200 Hz 格點、缺歷史拒絕、miss 索引、CSV wrap 和幀末快照亦有獨立測試。
Go2 RobotLab、Go2W、雙輪、legacy Go2 各 100 步，新增感測器前後 qpos/qvel/ctrl 與原 IMU 逐元素完全相等。

回歸套件：224 tests 與 24 subtests 通過；唯一跳過的 CUDA 用例補開 `SIMPLE_RBT_TEST_CUDA=1` 後通過，
合計 225 個常規測試與 24 個子測試。包含真 ROS DDS 收發、非 identity 真值不得洩漏到 IMU，以及宿主 CUDA／相機測試。

[CPU／CUDA rolling 對照](../output/mid360-validation-20261013/rolling-cpu-cuda.json)：
12 幀、240,000 條射線，108,612 個共同回波，命中索引、時間與線號完全一致；最大距離差 3.20 µm。
完整每幀耗時中位 CPU **16.75 ms**、CUDA **3.05 ms**，包含內插、求交、readback 和過濾，不含物理及相機。
2 幀 snapshot 對照亦通過，JSON 時標核實為幀末 100／200 ms。
性能工具保留最大 800,000 點選項，長幀歷史窗口隨時長擴大；52,000 點／260 ms 補測亦通過。
此選項是性能對照，正式 ROS 運行仍為 20,000 條射線／100 ms。

## 同次完整 ROS 與地形驗收

每個案例使用新仿真程序與獨立接收程序，由 `/cmd_vel` 輸入指令。
每組 **30/30 項通過**：原完整路線／速度／摩擦門檻、兩 IMU payload hash 和精確 200 Hz 格點、
每幀掃描的完整 IMU 及姿態歷史覆蓋、點雲／雙相機／里程計／TF、實時係數至少 0.95、靜默後停車。
可靠話題無漏收；`/clock` 是 best effort，允許中間消息被合併或丟失，仍驗證單調和最終時間。

| 案例／報告 | 指令／平均實速 m/s | 完成時間 s | 實時係數 | 仿真 RAM 峰值 MiB | 仿真 CPU 平均核心數 |
|---|---:|---:|---:|---:|---:|
| [Go2 平地](../output/mid360-validation-20261013/full-go2-flat-v03/report.json) | 0.3／0.298 | 22.060 | 1.0000 | 803.75 | 0.622 |
| [Go2W 18 cm 樓梯](../output/mid360-validation-20261013/full-go2w-stairs18-v06/report.json) | 0.6／0.598 | 12.686 | 1.0000 | 931.95 | 0.823 |
| [Go2W 25° 長坡](../output/mid360-validation-20261013/full-go2w-ramp25-v06/report.json) | 0.6／0.604 | 20.602 | 0.9974 | 926.09 | 0.833 |
| [Go2W 12 cm 起伏、seed 2](../output/mid360-validation-20261013/full-go2w-rough12-seed2-v06/report.json) | 0.6／0.591 | 12.114 | 0.9971 | 924.91 | 0.929 |

樓梯完整上行、平台、下行後到 x=6 m；長坡上／下各 4 m、頂高約 1.87 m，到 x=10.6 m。
完成時間含 2 s 穩定；另持續運行 2 s 驗證停車。平均實速是移動窗口的機體前向速度，不是每時刻保證值。
未重跑歷史全部地形矩陣；Go2 在更高地形的能力限制仍以既有紀錄為準，不能把本次 Go2 平地通過擴大為全部地形通過。

RAM 是完整程序高水位，含初始化；CPU 是實時迴圈區間。
獨立接收／驗收程序另需 288–306 MiB，平均 0.384–0.440 核；其 CPU 窗口不同，不直接與仿真平均值相加。
單機單機器人加接收程序的峰值總和上界約 1.21 GiB，實用預留可按 1.5–2 GiB、2 個 CPU 核做起點，
不含 OS、桌面和其他節點。此為本機測量後的容量建議，不是跨硬體保證。
GPU 的 1 Hz compute-process 採樣為 514–524 MiB；不視為完整圖形 VRAM 或保證峰值。

本階段 25 個驗證檔合計約 **6.09 MiB**，完整案例保存 hash 和必要真值，不持續保存所有圖像。
本機 `.venv`、`.vendor`、`mujoco_sim` 與 CSV 的檔案配置量合计約 **1.22 GiB**，不含系統 ROS、共用快取或歷史輸出。
一般 CLI 保存 200 Hz IMU CSV、10 Hz 軌跡與最後點雲／相機；`--summary-only` 僅保存摘要。

## 邊界與下一步

已對齊標稱位置、軸向、共用時間、選定 200 Hz 配置、單位和本機 ROS 2 資料布局。
尚未模擬 IMU 噪聲／偏置／晶片濾波；預設不截斷量程，Python sampler 可選 ±4g／±2000°/s 截斷，
但只是輸出截斷近似。沒有完整光學強度／量測噪聲、網路延遲或逐台校準。
CSV 第一欄是點序號，200 kHz 時間為合成；不是硬體 packet 記錄。
rolling 場景排除整台機器人的自遮擋，並拒絕外部動態物體；目前只驗證靜態地形環境。

下一步可在這份資料規格上接融合器，驗證 LIO 去畸變／估計軌跡。
本階段未執行 FAST-LIO 建圖，不能把介面與幾何測試等同定位精度驗收。

## 重跑

### 同日控制命名空間更新

目前速度入口由 `--robot` 自動選擇：`/go2/cmd_vel`、`/go2w/cmd_vel` 或 `/diffdrive/cmd_vel`，
ROS 節點也放在對應機器人命名空間。Python `RosBridge` 必須明確提供 `robot_name`。
上方初始 MID360 四組基準使用裸 `/cmd_vel`，保留其原始報告；重跑腳本現在使用機器人命名話題。

命名空間修正的 28 tests＋40 subtests 通過，包括兩個 ROS bridge 各自接收不同指令、
裸 `/cmd_vel` 無訂閱者且不覆蓋指令，以及獨立 0.5 s 超時計時。
[Go2 CLI](../output/namespace-validation-20261013/go2-cli.json) 和
[Go2W CLI](../output/namespace-validation-20261013/go2w-cli.json) 各有 19/19 檢查通過，
使用實際啟動入口、外部發布者、全傳感器與靜默停車。
[Go2W 18 cm 完整樓梯](../output/namespace-validation-20261013/full-go2w-stairs18-v06/report.json)
31/31 通過，新增父程序／worker／ROS 狀態／bridge 的控制話題一致性檢查；原路線和速度門檻照常驗收。

### 命令

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros_ws/to_migrate_ws/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MUJOCO_GL=egl SIMPLE_RBT_TEST_RENDER=1 SIMPLE_RBT_TEST_WARP=1 \
  SIMPLE_RBT_TEST_CUDA=1 SIMPLE_RBT_ROS_TESTS=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=83 \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m pytest -q mujoco_sim/tests

.venv/bin/python mujoco_sim/scripts/validate_full_run.py \
  --robot go2w --controller rl_sar --terrain ramp --terrain-level extreme --speed .6 \
  --output output/mid360-full-run-new
```

測試命令在 Jammy 宿主執行。每次完整驗收需新的輸出路徑；報告保存完整配置和程式／模型／策略 SHA256。
