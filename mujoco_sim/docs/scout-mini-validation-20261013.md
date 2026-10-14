# Scout Mini 第一階段部署與驗證 — 2026-10-13

Scout Mini 已接入 Jammy 的 MuJoCo 核心、CLI 和 ROS 2；可以輸入速度、觀察接觸驅動的運動，
並接收共用 MID360、IMU、普通／全景相機。平地運動、8° 坡道和 2.5 cm 起伏地面通過；
4、5、6 cm 連續台階仍超時。這是可用的第一階段模型，尚不能用來驗證實車越障性能。

本階段從 `12fb741` 開始，保留既有 Go2、Go2W 和雙輪底盤。現在有四個可啟動配置：

| `--robot` | 底盤 | 控制輸入 |
|---|---|---|
| `go2` | 四足 | `/go2/cmd_vel` |
| `go2w` | 輪足 | `/go2w/cmd_vel` |
| `diffdrive` | 簡單雙輪差速參考模型 | `/diffdrive/cmd_vel` |
| `scout_mini` | 松靈 Scout Mini 四輪差速／滑移轉向 | `/scout_mini/cmd_vel` |

## 模型與控制邊界

使用的是普通 **Scout Mini**，不是全尺寸 Scout，也不是 Mini Omni。
[松靈當前產品規格](https://www.agilex.ai/page/68930913f34d776a0e9d147e)給出
612×580×245 mm 外形、451 mm 軸距、490 mm 輪距、115 mm 離地間隙、23 kg 空載質量和 1.5 m/s 最大速度。
程序幾何與來源雜湊記於 [`scout_mini_manifest.json`](../assets/scout_mini_manifest.json)。
舊官方 URDF 的質量、輪徑與插件參數彼此不一致，沒有直接照搬。

當前版本 `scout-mini-current-spec-rounded-rigid-v2` 使用箱體和圓肩橢球輪胎；
80 mm 輪半徑、每輪質量、慣量分配、摩擦、8 N·m 扭矩上限與伺服增益均為明示工程估計。
真實獨立懸架尚未實作。機體只有輪子接觸力驅動，沒有外力補推、指定機體速度或改寫位姿。
輪胎兩個切向的滑動摩擦均為 0.8，沒有以近零側摩擦冒充滑移轉向。

`auto` 選擇 `skid_steer`，200 Hz 更新四輪速度。原生控制包含理想陀螺儀 z 軸的轉向 PI，
補償側滑，但不是原廠韌體。`native` 關閉的是外層理想機體速度／朝向 tracker；
`tracked` 另加該 tracker。所有下述正式案例均使用 **native**。

輸入限幅為 vx ±1.5 m/s、vy=0、wz ±1 rad/s；複合指令仍受輪速和扭矩約束，不能保證全範圍準確追蹤。
CLI／Python 拒絕非零 vy。ROS 收到非零 vy 或非有限指令時，整條速度指令立即清零，
仿真繼續運行；後續有效指令恢復。停止發布後沿用 0.5 秒單調時鐘超時清零。
感測器話題及 TF frame 仍是共用絕對名稱，這一階段沒有實作完整多機器人感測器／TF 隔離。

## 可觀察運動驗收

[12 案逐項結果](../../output/scout-mini-20261013/rounded-native-v2/RESULTS.md)使用獨立程序、
相同控制／摩擦／扭矩和原門檻，9 案通過，3 案台階超時。
[最高速補測](../../output/scout-mini-20261013/rounded-max-speed-v2/RESULTS.md)另行通過。

| 動作／路線 | 指令 | 穩態實测 | 運動、速度與停車 |
|---|---|---|---|
| 平地前進 | 0.3 / 0.6 / 1.0 m/s | 0.298 / 0.596 / 0.993 m/s | 通過 |
| 平地最高速 | 1.5 m/s | 1.490 m/s | 通過 |
| 倒退 | −0.3 m/s | −0.298 m/s | 通過 |
| 左／右原地轉向 | ±0.5 rad/s | ±0.493 rad/s | 通過 |
| 邊走邊轉 | 0.3 m/s、0.3 rad/s | 0.311 m/s、0.283 rad/s | 通過 |
| 8° 上坡／平台／下坡 | 0.3 m/s | 0.297 m/s | 全程與停車通過 |
| 2.5 cm 起伏、seed 0 | 0.3 m/s | 0.295 m/s | 全程與停車通過 |
| 4 / 5 / 6 cm 連續台階 | 0.3 m/s | 約 0.033 m/s | 上行超時；未執行停車階段 |

穩態欄排除開始運動的第一秒。速度誤差門檻為 `max(0.05, 15% × |目標|)`，
側移絕對值均值 ≤0.1 m/s，直行偏航絕對值均值 ≤0.1 rad/s；停車末秒平面速度與偏航絕對值均值 ≤0.05。
另檢查真實底盤接觸、45° 傾斜與按車體包絡計算的走廊，沒有為新模型降低通過門檻。

台階失敗沒有底盤撞地或翻倒；前軸越過第一階，但後軸未通過，不能稱整車完成一階。
原廠 5 cm 空載／4 cm 載重指標指單障礙，連續樓梯是不同測試；兩個低台階案例都使用
23 kg 空載模型，沒有驗證載重或單障礙性能，也不能由此判定實車不符合原廠規格。

原圓柱 v1 的 [10 案基線](../../output/scout-mini-20261013/native/RESULTS.md)、
[4／5 cm 補測](../../output/scout-mini-20261013/within-spec-steps/RESULTS.md)和
[圓肩接觸對照](../../output/scout-mini-20261013/rounded-tire-probe/RESULTS.md)全部保留。
對照只改接觸形狀，保留原慣量、質量、摩擦及控制；轉向改善，但台階未修復。
v2 單元測試的 1 rad/s 指令末段約 0.9005 rad/s，同時保留輪速飽和記錄；不是額外正式多地形案例。
最高速案的飽和比例按嚴格 `scale < 1` 計數，極小偏航修正會觸及邊界；該比例不表示等比例減速。

## ROS、傳感器與實時聯合驗收

[實際 CLI／DDS 驗證](../../output/scout-mini-20261013/ros-cli-v2.json)的 20 項檢查全部通過。
下列三個完整路程案例各 **31／31 項通過**，且來源雜湊一致、運行中未修改源碼：

| 完整 ROS 場景 | 平均 vx | 實時倍數 | 仿真峰值 RAM | 仿真平均 CPU | 接收驗收程序 RAM／CPU |
|---|---|---|---|---|---|
| [8° 坡道，0.3 m/s](../../output/scout-mini-20261013/full-ramp8-v03-v2/report.json) | 0.297 m/s | 0.9991 | 504.2 MiB | 0.606 核 | 287.7 MiB／0.403 核 |
| [2.5 cm 起伏，0.3 m/s](../../output/scout-mini-20261013/full-rough-v03-v2/report.json) | 0.294 m/s | 1.0000 | 506.9 MiB | 0.645 核 | 290.6 MiB／0.405 核 |
| [平地，1.5 m/s](../../output/scout-mini-20261013/full-flat-v15-v2/report.json) | 1.445 m/s | 0.9999 | 501.0 MiB | 0.598 核 | 289.0 MiB／0.410 核 |

此表速度窗口包含起步加速，與上表排除一秒的穩態值不同。停車末端均值分別約
0.0000047、0.0000047、0.000415 m/s。
CPU 1 核表示一個邏輯核心的 100%；RAM 是每個新仿真程序的整段高水位，包含載入／暖機。
受測主機有 20 個邏輯核心。接收驗收程序另列，不能把它的 CPU／RAM算進或漏報成仿真核心開銷。
資源數據是這些短時單機案例，沒有宣稱長期穩定性或多機器人容量。

正式循環包含物理、所有門檻、GPU rolling LiDAR、普通／全景 RGB＋深度和 DDS；
設定／JIT／renderer 暖機在實時倍數之外。兩種相機達 10 Hz 模擬頻率，牆鐘約 9.94–9.98 Hz。
仿真 PID 的 1 Hz compute 顯存採樣為 500 MiB，並非完整 graphics／桌面顯存或保證峰值。

沿用目前 MID360：雷達座標內 IMU 位移 `[+0.011,+0.02329,−0.04412] m`，座標軸平行；
Scout 的 5 ms 物理步恰好對齊 200 Hz IMU，10 Hz 雷達按逐點採集時刻使用真實運動位姿。
ROS 實際收到的 IMU、點雲和影像雜湊與同一次物理輸出一致，可靠訊息全部收齊。
原始 IMU 為 g／rad/s，SI companion 為 m/s²／rad/s，均沒有姿態估計；`/odom` 是真值。
雷達仍排除機器人自遮擋，僅對靜態环境追蹤，不包含完整硬體噪聲、UDP 或標定誤差。

物理評估的 12 個獨立單執行緒程序 RAM 為 60.7–67.0 MiB，無等待時鐘的倍數為 22.2–29.7；
這些數字不含雷達／相機／ROS 工作負載，但物理與控制仍使用原生 IMU；
沒有預留獨佔 CPU，不能拿來冒充完整仿真的资源或硬體性能對比。
獨立 [CLI 實時純物理設定](../../output/scout-mini-20261013/paced-physics-v15-v2/results.json)
亦確認新資源腳本的 `native` 與 1.5 m/s 參數生效：6 秒窗口 RAM 高水位 89.7 MiB、
平均 0.0195 核、實時倍數 1.0000；程序樹採樣峰值 90.1 MiB。
CLI 載入路徑與純評估 worker 不同，兩者 RAM 不應混為同一口徑。
`--sensors physics` 仍計算原生 IMU 並取樣 MID360 IMU（此案 1,201 筆），省去的是雷達、相機和 ROS 傳輸。

## 磁碟與重現

五個新增模型／manifest／評估／測試檔合計 63,637 bytes，實際磁碟分配 73,728 bytes；
沒有新增 Python 依賴、下載網格或控制權重。驗收輸出被 Git 忽略；容量見
`output/scout-mini-20261013/stage-resource-summary.json`，其中包含保留的 v1、對照和 v2 證據。
既有 `.venv` 分配約 809.6 MiB、`.vendor` 約 411.4 MiB，均沿用本階段之前的依賴／資產；
來源與各目录的分配／邏輯容量亦保留於彙總中，不含共用快取與系統 ROS。
正式路線保存摘要與訊息雜湊，不持續錄製全部影像；不能用本次輸出容量預測 rosbag 錄製開銷。

從倉庫根目錄啟動：

```bash
./mujoco_sim/run.sh --robot scout_mini --terrain rough --backend warp \
  --control-mode native --cmd .3 0 0 --settle 2 --duration 25 \
  --sensors full --camera-hz 10 --realtime
```

外部 ROS 控制改用 `--ros2`，在另一終端發布 `/scout_mini/cmd_vel`；兩端使用相同 domain。
桌面查看可增加 `--viewer`。GPU 執行及驗收均在已授權 Jammy 宿主機完成。

```bash
.venv/bin/python mujoco_sim/scripts/evaluate_scout_mini.py \
  --cases all stairs_4cm stairs_5cm forward_1.5 --control-mode native \
  --output output/scout-mini-evaluation-new

source /opt/ros/humble/setup.bash
source /home/user/ros_ws/to_migrate_ws/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash
.venv/bin/python mujoco_sim/scripts/validate_full_run.py --robot scout_mini \
  --controller skid_steer --control-mode native --terrain ramp \
  --terrain-level standard --speed .3 --output output/scout-mini-full-new
```

完整測試開啟 render／Warp／CUDA／ROS，**249 tests + 47 subtests 通過，9.64 秒**，無跳過。
新增 DDS 行為測試涵蓋 Scout Mini／雙輪的有效指令、非法側移清零與有效恢復；
既有 Go2／Go2W 命名空間與側移行為仍通過。

## 下一步的車型與門檻

尚未部署的順序如下；每次先完成運動、传感器、ROS 和資源驗收，再記錄結果。

| 順序 | 車型 | 新的底盤運動 | 首要驗證 |
|---|---|---|---|
| 2 | [煜禾森 FR-mini](https://www.yuhesen.com/cn/Product/FR-mini) | 前輪阿克曼 | 轉向角、轉彎半徑、倒車、轉向限幅；不接受原地旋轉 |
| 3 | [Clearpath Ridgeback R100](https://clearpathrobotics.com/ridgeback-indoor-robot-platform/) | 四麥輪 | 真實輪子／滾子接觸下的側移、旋轉、複合運動與 CPU 成本 |
| 4 | [松靈 Ranger Mini 3.0](https://www.agilex.ai/page/6908c1285e78cfa260412be9) | 四輪轉向／舵輪 | 舵角、輪速、轉向模式切換、側移與輪組對齊 |

Ridgeback 有官方 ROS 2 [R100 麥輪描述](https://github.com/clearpathrobotics/clearpath_common/blob/69b6da109b14117b5fb693789747317a52ff973f/clearpath_platform_description/urdf/r100/drivetrain/wheels/mecanum.urdf.xacro)。
小型替代是 [Husarion ROSbot XL](https://husarion.com/manuals/rosbot-xl/)，有官方 CAD、普通輪與麥輪版本。
官方 Gazebo 麥輪碰撞使用圓柱與方向摩擦／slip；直接搬圓柱到 MuJoCo 不會自然產生麥輪側移。
下一階段需明示並驗證滾子物理或接觸近似，不能用機體速度指定冒充完成部署。
Scout Mini 的連續台階問題另保留為懸架／輪胎標定工作，需可靠參數及可觀察驗收，不能靠放寬門檻。
