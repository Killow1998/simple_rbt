# simple_rbt

MuJoCo、C++17、ROS 2 Humble 機器人模擬器。輸入速度指令，輸出物理運動、MID360 雷達／IMU、雙相機、里程計和 TF。
目前支援 Go2、Go2W、雙輪差速參考模型、Scout Mini、FR-mini、Ridgeback、Ranger Mini。
純輪式底盤使用平地、坡道、連續起伏；Go2／Go2W 另有樓梯及階躍起伏。

## 從這裡開始

```bash
# 首次準備 SDK、模型及建置
./setup.sh

# 已有 SDK／資產時，只建置本專案訊息與 C++
./setup.sh --native-only

# 完整感測器、即時運行及桌面視窗
./run.sh --robot fr_mini --cmd .3 0 0 --sensors full \
  --backend cuda --camera-hz 10 --realtime --viewer --duration 10

./run.sh --help
```

加上 `--ros2` 後，速度由 `/<robot>/cmd_vel` 接收，超過 0.5 秒沒有新指令即停車。
正常執行入口 `run.sh` 啟動 C++；Python 留作離線準備與獨立對照。
完整依賴、ROS 介面、控制模式和已知運動限制見 [操作說明](docs/python-reference.md)及[原生建置](docs/runtime.md)。

## 資料夾分工

```text
simple_rbt/
├── CMakeLists.txt          C++ 建置入口
├── setup.sh / run.sh       準備與啟動入口
├── src/                    物理、控制器、感測器、ROS 與排程
├── include/simple_rbt/     C++ 模組介面
├── assets/                 共用模型、策略、參數與扫描資料
├── tests/                  C++ 邊界與傳輸測試
├── tools/                  資產準備、量測與聯合驗收
├── reference/python/       獨立 Python 對照實作及其測試
├── ros2_interfaces/        相容的 Livox 訊息定義，沒有硬體驅動
├── third_party/            固定版本 headers 與原授權
├── docs/                   操作、設計與已完成驗證
├── build/                  本機編譯結果（不進 Git）
└── output/                 實驗、量測及測試輸出（不進 Git）
```

[assets/README.md](assets/README.md)說明共用資產；`.vendor/` 存放按 manifest 準備的模型網格，
`.venv/` 提供 MuJoCo／ONNX Runtime 的 SDK 與對照工具。兩者不進 Git。
新增同控制器的模型修改登記及資料；新增控制律放在獨立控制器實作，見[模組說明](docs/runtime.md)。

## 驗證與舊版本

```bash
source tools/ros_env.sh
ctest --test-dir build/native --output-on-failure
PYTHONPATH=reference/python .venv/bin/python -m pytest reference/python/tests
.venv/bin/python tools/validate_full_run.py --runtime cpp --robot fr_mini \
  --terrain ramp --control-mode native --speed .6 --output output/full-fr-new
```

輸出目錄須是新目錄。GPU 與 live ROS 對照測試需要相應環境；完整配置與結果見[驗證文件](docs/README.md)。
[結構整理記錄](docs/repository-layout-20261014.md)說明此次搬遷與重新驗證。

舊 Gazebo／ROS 1 的 `simple_rbt/`、MID360 插件及硬體 driver 保存在
[archive_gazebo](https://github.com/Killow1998/simple_rbt/tree/archive_gazebo)。
目前程式只保留其中實際使用的扫描 CSV，以及原樣的 `CustomMsg`／`CustomPoint` 訊息定義。
`livox_ros_driver2` 的訊息包名保留是為了相容現有 ROS 使用者，並不代表模擬器啟動了硬體驅動。
