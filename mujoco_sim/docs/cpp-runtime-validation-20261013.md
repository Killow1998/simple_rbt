# C++ 遷移與 CPU／RAM 驗證（2026-10-13）

`mujoco_sim/run.sh` 現在啟動 C++17 的 `simple-rbt-native`。
七種機器人的控制器、MuJoCo 步進、感測器調度、MID360 雷達／IMU、雙相機、ROS 2 收發與 GUI
已移入 `mujoco_sim_cpp/`。Python 保留作離線模型準備及獨立測試對照，入口是 `run-python.sh`。
原生程式使用 MuJoCo／ONNX Runtime 的 C／C++ API，CUDA 射線在建置時編譯，執行時不啟動 Python。
建置與操作見 [原生 README](../../mujoco_sim_cpp/README.md)；現行操作入口仍是 [仿真 README](../README.md)。

## 功能與驗收

保留 Go2、Go2W、diffdrive、Scout Mini、FR-mini、Ridgeback R100、Ranger Mini 3.0。
Go2 的 RobotLab／HIMLoco／legacy、起身和扭矩選項，以及 Go2W 的 50 Hz 策略／200 Hz PD 時鐘均保留。
纯輪式底盤接受平地、坡道與連續起伏；Go2／Go2W 保留樓梯和方塊式起伏。
`tracked` 仍使用理想模擬器速度／航向回授；`native` 關閉這層回授，未添加外力或直接改姿態。

外部控制仍用 `/<robot>/cmd_vel`，0.5 秒未收到新指令清零；非法指令清零而仿真繼續。
Livox、PointCloud2、raw／SI IMU、普通／全景 RGB＋depth、odom、clock、TF 的介面和資料單位保留。
MID360 的內置 IMU 標稱位移仍是雷達座標中的 `[0.011,0.02329,-0.04412] m`，軸平行。
200 Hz IMU 使用同一模擬时鐘；Go2W 的 2 ms 物理步仍對 5 ms 栅格做相鄰內插。
雷達仍為完整 800,000 條 CSV、200 kHz 點時序、10 Hz／20,000 原始射線，每點使用其採集時刻位姿。
透視 320×240、全景 720×360、10 Hz 完整相機設定未降低。

獨立對照結果：

- 30 組核心控制案通過，覆蓋七機器人、native／tracked、Go2 三種策略、起身及扭矩部署。
  初次對照 26/30 組逐位一致，其餘最大感測器誤差約 `4e-12`；另用種子 42／最大 uint64
  比較非預設 rough 模型與步進，兩案通過。PCG64 的七種種子／五種縮放共 2,100 個 double 與 NumPy 逐位一致。
- 16 組相機對照通過：七機器人各兩姿態，另有空場景和封閉彩色房間。
  32 個深度陣列、4,730,652 個深度像素逐位一致，有限值／無命中 mask 一致；RGB 最大差值為 1。
  GUI＋完整感測器實際聯跑通過，修正後再次跑這 16 組相機對照亦通過。
- 原生 CPU `mj_ray` 與原生 CUDA 在四場景共 320 萬射線上的命中索引、時間、線號和 XYZ 逐位一致。
  C++ 對 Python 的 CSV 角度逐位一致；三角函數方向最多相差 3 ULP，受測點雲 XYZ 最大差約 1.91 µm。
  不把這些有限場景說成任意邊界都能逐位相同。
- Release 真實 DDS 測試通過 3,213 項斷言；14 種訊息與 Python oracle 全字段相同。
  CDR padding 不作資料字段比較。最終命令隔離測試改為每機器人獨立進程，三輪各通過 3,318 項斷言。
  三個 CTest 目標通過，保留的 Python 全套為 385 passed、65 subtests，無跳過。

完整 C++ 路線均在同次物理運行中發布／接收真實 ROS 資料，檢查載荷 SHA、時鐘、外參、速度、接觸、終點、停車與實時速度。
原有 31 項判定及速度門檻保留：

| 機器人／控制模式 | 路線與速度 | 結果 |
|---|---|---|
| FR-mini／native | 8° 上下坡、5 cm 連續起伏，0.6 m/s | 各 31/31 |
| Ridgeback／native | 同上 | 各 31/31 |
| Ranger Mini／native | 同上 | 各 31/31 |
| Go2W／tracked | 12 cm × 6 級，上下樓梯，0.6 m/s | 31/31 |
| Go2／tracked | 8° 上下坡，0.3 m/s | 31/31 |
| Go2／tracked | 8° 上下坡，0.6 m/s | 30/31；走完但轉向擺動超標 |

三台新底盤的 5 cm 起伏又各重複一次，仍各 31/31。
Go2 0.6 m/s 的平均絕對 yaw rate 為 0.137 rad/s，超過原門檻 0.1；
同條 Python 路線為 0.128，也未通過。0.3 m/s 的 C++ 為 0.083，通過。
這次沒有重新訓練策略；語言遷移不等於控制能力提高。
原生入口另通過 20/20 項實際 CLI／DDS 驗證。

## CPU、RAM 與磁碟

下表使用相同車型、地形、命令、完整 10 Hz 雙相機、10 Hz rolling 雷達、200 Hz IMU、實際 ROS 和實時節奏。
CPU 是仿真程序主循環的 user＋system CPU；**100% 表示一個邏輯核心**。
RAM 是每案新程序的 `ru_maxrss`，含共享庫／初始化；不含獨立 checker。受測路線的 RTF 約 1.000。

| 機器人 | Python CPU → C++ CPU | Python RAM → C++ RAM |
|---|---|---|
| FR-mini | 69.1% → 9.4% | 504.8 → 240.5 MiB |
| Ridgeback | 72.8% → 11.8% | 501.9 → 242.5 MiB |
| Ranger Mini | 68.3% → 9.5% | 504.7 → 240.2 MiB |
| Go2 | 43.1% → 10.1% | 800.3 → 521.2 MiB |
| Go2W | 64.7% → 18.5% | 918.5 → 602.1 MiB |

前三行為原始 Python `989ee60` 與 C++ 各兩次 5 cm 起伏測量的均值：CPU 下降 84–86%，RAM 下降約 52%。
Go2／Go2W 則是現有已優化 Python oracle 與 C++ 各一次同路線對照，不能混稱同一版本基線：
Go2 為上表 0.6 m/s 坡道，兩版均未通過轉向品質門檻；Go2W 為 12 cm 樓梯，兩版均通過。
兩台的 RAM 仍明顯高於程序幾何底盤，不宣稱七機器人都只需要 240 MiB。

Ridgeback 的 60 秒完整感測器＋ROS 持續靜止運行，RTF 0.99999；
第 10–20 秒與第 50–60 秒的 `/proc` RSS 均為 230.88 MiB，受測窗口未見持續增長。
此案沒有下游感測器接收器，不代替上述完整 DDS 路線負載。
實際 `/proc/<pid>/exe` 為原生 executable，無子程序，載入清單沒有 libpython／Warp／LLVM／NVRTC。
這是 60 秒的觀察，不是無限期無洩漏的保證。

CUDA compute 記憶體的逐 PID 抽樣：三台新底盤約 381 MiB（Python 原始基線約 500 MiB），
Go2 約 395 MiB、Go2W 約 406 MiB。`nvidia-smi` 此字段不保證包含全部 EGL 圖形分配，也不是 GPU 使用率。

新增原生來源約 1.90 MiB，third-party headers／許可約 1.28 MiB、模型 catalog／XML 約 0.34 MiB。
建置目錄約 30.07 MiB，主 executable 約 1.82 MiB；忽略 build 的來源才納入 Git。
重用已安裝的 MuJoCo／ONNX Runtime 共享庫和既有策略／mesh，未增加整套 SDK 或模型權重。
普通輸出只保存最後一幀感測器；`--summary-only` 可保留完整計算而不輸出逐步 CSV／圖像／點雲。
驗收證據目前約 48 MiB，保留在被 Git 忽略的 `output/`。

## 設計與本次修正

主要 RAM 收益來自不再載入 Python／Warp／LLVM 編譯器，雷達只持有靜態環境 plane／box 資料與可重用射線緩衝。
相機使用實際視口大小的 framebuffer、重用背景 ID，逐 face 拼接全景；ROS 使用有界緩衝並保留完整點數。
Python 對照也保留本次低風險優化：CSV 讀取不再多複製，感測影像／點雲重用有界緩衝。
CLI／ROS 的觀測頻率、尺寸、控制節奏和驗收門檻均未降低。

Release GCC 11.4 的受測 IMU 迴圈曾將 `double → float → double` 的部分軸合併，省略必要的 float32 舍入。
修正為明確 float 儲存點；raw IMU 符合 Livox 的 float32 wire 規格，SI 保留原 double。
用獨立 IEEE 位元期望值及 Python `struct.pack('<f')` 實收 trace 驗證，而不是讓期望值複製同一段被最佳化的寫法。
同類 wire 精度契約應在 Release 編譯與真實接收端驗證。

桌面 viewer 的 GLX 與感測相機 EGL 曾在同一執行緒保留 current context，造成跨 API 切換衝突。
兩者現於各次構造／渲染後釋放自己的 context，繪製／清理時重新取得；實際 GUI＋完整相機已驗證。
輸出流現在檢查寫入與關閉錯誤，避免磁碟寫失敗仍回報成功。

DDS 測試原先在同進程建立八個 ROS context，間歇性出現端點實際匹配數量不對稱：
發送端為 1、接收端為 0，而 graph 仍為 1，另一次多 context 構造曾崩潰。
尚未定位中介軟體的內部原因，不把它說成重複端點或網路故障。
正式產品是每仿真進程一個機器人／ROS context，完整路線和 60 秒運行均通過。
測試現以每機器人 fresh 子進程驗證真實 DDS；要求自己話題恰有一個接收者，另外六個車名話題
及全局 `/cmd_vel` 無接收者，實際發送外來命令不能覆寫，所有拒絕／恢復／停車判定保留。
測試預設獨立 domain 89 並尊重環境設定；生產程式未因這項測試修正而改動。

## 邊界與下一步

既有 Ridgeback 15° 坡入口觸底、FR-mini 原生 15° 下坡超速，以及 Go2 高速／高地形限制仍保留。
本次驗證不擴大硬體能力聲明。CUDA 目前只接受靜態環境的 plane／box；其他幾何明確報錯。
感測器仍為理想模型：沒有完整光學、MID360 噪聲／偏置／硬體 UDP 延遲或實機標定。
控制話題按車名隔離，感測器話題和 TF frame 仍是單機器人設計。
完整執行程式目前需要 ROS／CUDA toolkit 的建置依賴，即使執行時不啟用 ROS 或選 CPU 雷達。
下一步若新增底盤、策略或地形，应更新離線 catalog 並重跑核心及完整 C++ 路線；
若要提高 Go2 的高速品質，另作控制器任務，不能放寬此處門檻。

## 本機證據

輸出不納入 Git；路徑按此 checkout 相對定位：

- [CPU／RAM 對照與 executable SHA](../../output/cpp-migration-20261013/cpu-ram-comparison.json)
- [完整路線批次](../../output/cpp-migration-20261013/acceptance-batch.json)、
  [Go2 0.3 m/s](../../output/cpp-migration-20261013/go2-ramp-v03-final/report.json)、
  [C++ 入口 20 項](../../output/cpp-migration-20261013/cli-ridgeback-final.json)
- [60 秒 RSS／實際載入庫](../../output/cpp-migration-20261013/stationary-60s-resources.json)、
  [磁碟數據](../../output/cpp-migration-20261013/disk-footprint.json)
- [最終核心 30 案](../../output/cpp-core-20261013/final-physics/summary.json)、
  [PCG64](../../output/cpp-native-20261013/random-validation/numpy-parity.json)
- [相機 context 修正後對照](../../output/cpp-camera-20261013/context-release-oracle/summary.json)、
  [LiDAR CPU／CUDA](../../output/cpp-native-20261013/lidar-validation/native-cuda.json)、
  [Release DDS](../../output/cpp-migration-20261013/ros-transport-release-fixed/validation.json)
- [原生測試](../../output/cpp-migration-20261013/final-native-suite-fixed.log)、
  [Python 全套](../../output/cpp-migration-20261013/final-python-suite.log)
