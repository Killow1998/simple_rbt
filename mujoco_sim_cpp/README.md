# simple_rbt C++ runtime

`../mujoco_sim/run.sh` 現在啟動 `build/simple-rbt-native`。物理、控制器、LiDAR、相機及 ROS 2 都在 C++ 程序內執行，不啟動 Python，也不使用 Warp／LLVM runtime。原 Python 實作由 `../mujoco_sim/run-python.sh` 保留，供獨立對照。

支援 7 台機器人：`go2`、`go2w`、`diffdrive`、`scout_mini`、`fr_mini`、`ridgeback`、`ranger_mini`。Go2 的 RobotLab／HIMLoco／legacy 是控制器選項，不另算機器人。輪式場景使用 `flat`、`ramp`、`hills`；`tracked` 預設模式含理想模擬器速度／航向回授，`native` 關閉這層外部回授，保留各底盤自身控制器。

以下命令從 repository 根目錄執行：

```bash
# Jammy 已有 SDK、模型及策略資產時，直接編譯
./mujoco_sim_cpp/setup.sh

# 透視＋全景相機、逐點運動 LiDAR，以及 GUI
./mujoco_sim/run.sh --robot fr_mini --cmd .3 0 0 \
  --sensors full --backend cuda --camera-hz 10 --realtime --viewer --duration 10

# 純 CPU LiDAR；不開相機或 GUI
./mujoco_sim/run.sh --robot scout_mini --cmd .3 0 0 \
  --sensors lidar --backend cpu --duration 10

./mujoco_sim/run.sh --help
```

首次準備 SDK／模型可執行 `./mujoco_sim/setup.sh`，它會準備鎖定的 `.venv` 套件與策略資產，再建置 C++。C++ 直接連結 `.venv` 內的 MuJoCo／ONNX Runtime 共享庫與 MuJoCo headers；套件目錄需要保留，但執行時不需要 Python 解譯器。

建置使用 Jammy 現有 GCC（C++17）、CMake ≥3.22、CUDA 12.9、ROS 2 Humble／`livox_ros_driver2` 訊息套件，以及 EGL／OpenGL、GLFW、PNG、TinyXML2、OpenSSL、pkg-config。預設 CUDA compiler 是 `/usr/local/cuda-12.9/bin/nvcc`、host compiler 是 `/usr/bin/g++`，GPU 架構為 `120`（本機 RTX 5060 Ti）。其他 GPU 可在已載入 ROS／Livox 環境的 CMake 配置中，以 `-DCMAKE_CUDA_ARCHITECTURES=<架構號>` 修改 cache；compiler 路徑也可由同名 CMake cache 變數覆寫。目前建置包含 CUDA，即使執行時選擇 CPU LiDAR 也需要 CUDA toolkit。

`setup.sh` 使用 `SIMPLE_RBT_LIVOX_SETUP` 指向 Livox 的 `local_setup.bash`，以及 `SIMPLE_RBT_LIVOX_PREFIX` 指向其 install prefix；兩者預設位於 `$HOME/ros_ws/to_migrate_ws/install/livox_ros_driver2`。`SIMPLE_RBT_BUILD_JOBS` 預設為 2。啟動腳本使用該 prefix 設定共享庫／ROS 搜尋路徑，`SIMPLE_RBT_CPU_THREADS` 預設為 1，`ROS_DOMAIN_ID` 預設為 83，`ROS_LOCALHOST_ONLY` 預設為 1。

加入 `--ros2` 後，速度指令改由 `/<robot>/cmd_vel` 接收，取代 `--cmd`；超過 0.5 s 沒有新指令就停車。GUI 支援滑鼠調整視角及 Esc 關閉。`--realtime` 控制即時節奏，並不保證任意場景都能達到即時性能。

`--sensors physics|lidar|full` 分別選擇物理／加 LiDAR／再加雙相機；IMU 都保留。預設是 `lidar`、CPU backend、rolling 掃描；相機開啟後預設 1 Hz，可用 `--camera-hz 10`。MID360 設定為 10 Hz、每幀 20,000 條原始 CSV 射線、200 kHz 合成點時序，IMU 為 200 Hz。`rolling` 逐點插值剛體 pose，輸出各點採集當刻的局部座標，尚未 deskew；場景僅含靜態環境且排除完整機器人自身遮擋。CUDA 支援目前地形的 plane／box，其他幾何會拒絕，不會悄悄略過。`snapshot` 是幀末快照，offset 全零，與 rolling 在 metadata 中分開標示；`--backend warp` 僅是原生 CUDA 的相容名稱。

| 檔案 | 職責 |
| --- | --- |
| `src/core.cpp`、`src/scene.cpp` | 物理步進、模型／地形裝配與感測點 |
| `src/robots.cpp` | `RobotSpec` 登記、命令能力／限幅與預設控制器 |
| `src/controller.cpp`、`src/controllers/` | 控制器建立入口及七個控制實作 |
| `src/controller_policy.cpp`、`src/tracking.cpp` | ONNX 策略執行、可關閉的外層速度／航向回授 |
| `src/random.cpp` | 與 NumPy 1.26.4 單一 uint64 seed 相容的 SeedSequence＋PCG64 |
| `src/lidar.cpp`、`src/lidar_cuda.cu` | CSV、pose history、獨立 `mj_ray` CPU 參照與原生 CUDA 射線 |
| `src/cameras.cpp` | EGL 透視 RGB／Z-depth、全景 RGB／徑向深度 |
| `src/ros_bridge.cpp` | rclcpp、Livox／IMU／影像訊息、命名空間與超時停車 |
| `src/runtime.cpp`、`src/viewer.cpp` | 感測器排程與 GLFW GUI |
| `src/artifacts.cpp`、`src/resources.cpp` | 輸出檔案與程序資源量測 |
| `src/course.cpp`、`src/course_geometry.cpp` | 無 DDS 的路線／速度／接觸／摩擦判定 |
| `src/full_run.cpp` | 實際運行的證據聚合、雜湊及驗收握手 |
| `scripts/`、`tests/` | 離線模型匯出及獨立 oracle；不由模擬 runtime 啟動 |

CMake 分別建立 robots、random、core、LiDAR、cameras、ROS、course、evidence 函式庫。
相機與雷達測試各自連結所需庫；路線判定測試不建立 DDS、不依賴圖形或 CUDA。
`Controller` 保留控制實作邊界，PImpl 隔離 SDK／資源生命週期，`RunObserver` 接收真實運行證據；
這些仍有用途。新增採用現有關節／控制器 schema 的同型模型可登記 `RobotSpec` 與模型 catalog；
新增控制律仍需對應實作和 factory 登記，不能視為任意車型只改資料即可接入。

模型／策略來源與 SHA 保存在 `assets/catalog.json` 及既有 `../mujoco_sim/assets/` manifests。ONNX Runtime 1.19.2、nlohmann/json 3.11.3 headers 的版本、來源、SHA 與 MIT 許可見 [third_party/manifest.json](third_party/manifest.json) 及相鄰 LICENSE；SeedSequence／PCG 的 MIT 與 NumPy glue 的 BSD 許可保留在 `src/random.cpp`。

輸出預設放在新建的 `output/mujoco/native-*`，或由 `--output NEW_DIRECTORY` 指定；已存在的目錄會拒絕。包含 `summary.json`、`compiled-model.xml`、trajectory／SI IMU CSV，以及最後一幀的 LiDAR、相機資料。LiDAR 欄位分別儲存為 `.npy`，深度亦為獨立 `.npy`，RGB 為 PNG，外參及時序見相鄰 metadata JSON；格式與 Python 的 NPZ 封裝不同。`--summary-only` 略過逐步 CSV 與感測器檔案，保留 summary 和模型。

最終版本七台各通過 31/31 正式完整 ROS 路線檢查；CTest 10/10、Python 392 tests＋65 subtests 通過，無 skip。
雙輪參考模型的正常腳輪接地誤判已修正：依既有車型能力與明確腳輪角色分類，沒有按機器人名字分支。
真正車殼接觸仍被攔截，原 50° 傾角與 0.1 m 高度門檻保留；87 條原生邊界斷言驗證正常支撐與錯誤碰撞。
最終 binary SHA、凍結源碼身份和逐案結果見下方階段紀錄。
修正前優化快照另有相機 16/16 fixtures 通過，32 組深度逐位元相同；RGB 最大差 1 灰階。四場景共 320 萬條
CUDA／獨立 CPU 射線結果一致；FR-mini 完整感測器 GUI 亦完成實際運行。
範圍、三台輪式車交錯 A/B 資源數據、兩個實際故障變異及擴展實驗見
[架構整理與消融紀錄](../mujoco_sim/docs/cpp-architecture-ablation-20261013.md)。
[C++ runtime 遷移紀錄](../mujoco_sim/docs/cpp-runtime-validation-20261013.md)保留前一階段結果。
