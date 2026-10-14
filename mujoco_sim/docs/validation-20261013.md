# MuJoCo 第一階段結果（2026-10-13 UTC／Jammy 本地 10 月 12 日）

## 目標與已完成範圍

把附件中 simple_rbt 的構想推進到可實際運行的核心：機器人可切換，ROS 只負責收發，
提供 MID360、IMU、普通相機和全景相機。本輪新增 `mujoco_sim/`，不改寫舊 Gazebo 套件。

目前包含 Go2 現成 ONNX 控制器，以及一個 MuJoCo 物理雙輪底盤。兩者共用感測器及可選 ROS 2 介面。
在 Jammy 宿主機完成 CUDA、EGL 和 ROS 2 的實際驗證，測試結果不是由沙盒設備可見性推斷。

## 可觀察結果

| 驗證 | 結果 |
|---|---|
| 全部功能測試 | **29 項通過，0 跳過**，含 EGL 渲染、DDS 收發、CSV 循環、漏點時間對齊、機器人前進／轉向和保存時間 |
| Go2 獨立運行 | 10 秒內前進 x=2.020 m、側向漂移 y=-0.436 m；最低機體高度 0.272 m、最大傾角 4.37° |
| GPU＋ROS 2＋相機 | 8 秒模擬的主迴圈耗時 8.151 秒，實時比 0.982；含整個驗收程式啟動和收尾為 9.848 秒 |
| 實際 ROS 收到 | 80 幀 Livox、80 幀 PointCloud2、2,000 筆 IMU、80 幀里程計／TF、普通／全景各 8 組 RGB 與深度 |
| 速度輸入與停車 | 0.3 m/s 指令發送約 4 秒，移動 1.058 m；停止發送後指令歸零，末秒平均速度 0.00103 m/s |
| 相機無物體深度 | 天空回傳 `+inf`；普通／全景最後一幀比例分別 16.14%／39.09%，無 NaN 或負無限值 |

Go2 獨立前進有可見側向漂移，現階段只證明現成控制器能運行，未證明控制效果優於舊 Gazebo。

### 雷達 CPU／CUDA 對照

RTX 5060 Ti，MuJoCo 3.15.0、Warp 1.18.0。相同 Go2 移動狀態，每幀 20,000 射線，
各預熱 5 幀，再比較 100 幀。耗時包括取回完整結果與點過濾，排除物理步進、建模及首次編譯。

| 後端 | 中位數 | p95 |
|---|---:|---:|
| CPU | 3.333 ms | 3.434 ms |
| CUDA | 0.811 ms | 0.838 ms |

該測試場景下 CUDA 約快 4.11 倍。這不是對 Gazebo 的性能對照，也不是大型地圖的性能保證。

共同命中的 980,035 點，最大距離差為 **3.49 微米**，所有點的時間和線號一致。
2,000,000 條射線中有 **1 條命中判定不同**：CSV row 74259 命中牆頂邊界
`[-7.899998665, 1.959900498, 3.0]`，CUDA 判為命中、CPU 判為未命中。
差异符合邊界浮點交點精度的表現；沒有為此加特判或修改上游。
原始報告保留 `numerical_agreement_passed=false`，因此不能說兩個後端完全一致。

## 實作中修正的問題

- CSV 的 `Time/s` 欄實際是連續序號。按 200 kHz 生成奈秒偏移，避免誤當秒。
- 新 ROS 接口使用完整模擬時間；`header.stamp` 和 `CustomMsg.timebase` 一致，跨秒不回零。
- 漏掉無命中的射線時，保留原始射線索引、線號和偏移，避免點雲與時間錯位。
- 普通深度和全景深度分別保存光軸 Z 與徑向距離，不混用；全景不套用針孔 `CameraInfo`。
- MuJoCo 遠裁切深度的浮點轉換會把天空寫成約 1418 m。現在用幾何分割遮罩判斷無命中，
  不靠「接近 far」的數值猜測；多組 near/far 比例和真實開放場景已驗證。
- 最後一幀 LiDAR 與相機可能不同時刻，輸出各自時間、位姿和標定。

## 證據與重跑

本機完整證據在 `output/validation-20261013/`：

- `tests-final.txt`：最終 29 項測試。
- `lidar-benchmark.json`：逐幀 CPU／GPU 數據、誤差與上游來源。
- **`ros2-end-to-end-final.json`**：修正相機後的 CUDA 完整鏈路驗收，11 項判定全部通過。
- `ros2-cli-20261013T043552-912798Z/`：該輪相機、軌跡、點雲和摘要。
- `preview.gif`、`preview.png`、`preview.json`：5 秒、60 幀的真實 Go2 仿真預覽。

早期 `go2-cpu/` 和 `ros2-end-to-end.json` 保留作為修正前記錄，其背景深度有已知問題。
應以 **final** 報告和對應的影像資料夾作為目前驗收依據。

全部測試：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/ros_ws/to_migrate_ws/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MUJOCO_GL=egl SIMPLE_RBT_TEST_RENDER=1 \
  SIMPLE_RBT_ROS_TESTS=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=83 \
  .venv/bin/python -m pytest -q mujoco_sim/tests
```

獨立進程的完整 GPU／ROS 驗收：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/ros_ws/to_migrate_ws/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash"
MUJOCO_GL=egl .venv/bin/python mujoco_sim/scripts/check_ros2.py \
  --backend warp --domain 84 --output output/ros2-new-validation.json
```

安裝與日常運行入口見 [README](../README.md)。新輸出應選新名稱，保留舊證據。
依賴固定在 `requirements.lock`，MuJoCo-LiDAR 固定提交 `e6b879df4b96daaed30ef10fe02650d29048fa9b`。

## 下一階段

優先把 FAST-LIO 接到目前已驗證的接口，先做靜止／低速建圖驗收；再加入幀內逐時刻取樣的雷達，
用有已知真值的運動驗證去畸變。之後依實際需求移植 Go2W 和其他底盤，分別驗收控制器。

目前雷達是幀末快照、IMU 無噪聲和偏置、反射強度固定。尚未聲稱具備高速 LIO 的真實感測器精度，
未進行實機部署、FAST-LIO 建圖或 Gazebo 品質對照。第一階段的「可運行核心與薄 ROS 接口」已完成。
