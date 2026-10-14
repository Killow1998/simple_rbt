# 倉庫結構整理 — 2026-10-14

本輪只調整工程結構與依賴範圍。輸入速度、機器人物理、控制器、傳感器格式、ROS topic 和安全門檻維持原行為。
舊版基準為 `823a581ac2c32cbf75f708127307104920850d8d`；Gazebo 來源另由 `archive_gazebo`（`64385758c95ba729b41faa0a9a7c72657f112c49`）保存。

## 為何原目錄仍存在

先前建 archive 分支只保存歷史，沒有從 main 拿掉舊資料夾。三個舊目錄合計 279 份追蹤檔：

| 舊目錄 | 新程式實際需要 | 此次歸屬 |
| --- | --- | --- |
| `simple_rbt/` | 不需要其 Gazebo 模型／ROS 1 插件 | Git 歷史與 archive 分支 |
| `mid360_sim/` | 800,000 行 MID360 扫描 CSV | 原樣移到 `assets/lidar/`；其他插件／資料留歷史 |
| `livox_ros_driver2/` | `CustomMsg` 和 `CustomPoint` 定義 | `ros2_interfaces/` 最小訊息套件；硬體 driver 留歷史 |

原掃描檔 25,399,988 bytes，SHA256 `aa486c66092af45eab59fccb1119421470ba01981434b75d04367dd374576553`；來源與舊 Git blob 見
[資產來源](../assets/lidar/source.json)。沒有重算、抽點或更換資料，也未替未明確授權的原資料新增授權聲明。

## 現行分工

C++ 工程在根目錄建置：`src/` 實作，`include/` 介面，`tests/` 原生測試。
`assets/` 是共用資料，不再把 C++ 權重放在 Python 對照目錄。
`tools/` 是準備／驗證入口；`reference/python/` 是獨立對照。
Python 透過 `paths.py` 和工具 `project.py` 共用位置，避免每支工具自行猜目錄深度。

`ros2_interfaces/livox_ros_driver2` 只建訊息，保留原 ROS 型別名稱、原字段、原 generator target 和原授權。
沒有編譯硬體 driver、Livox SDK、PCL 或 APR。它安裝到專案自己的 `build/ros_interfaces/`，不覆寫外部 ROS workspace。
`run.sh` 直接啟動 `build/native/simple-rbt-native`；`setup.sh --native-only` 可重建訊息與 C++，不重新安裝 SDK。

`.venv/`、`.vendor/` 是必要 SDK／模型，`build/`、`output/` 是本機產物；這四者不進 Git。
沒進 Git 的舊編譯結果、Python cache／egg-info／實驗輸出在切換時另存本機 Archive，不依靠 Git 恢復它們。

## 重新驗證

已完成的草稿驗證：

- 新 prefix 建置訊息；兩份 `.msg`、LICENSE 及生成 IDL 與原版相同。
- 原安裝 Python 與新 C++ 訊息包在獨立進程以真實 DDS 雙向互通，3 幀全部字段及非 padding CDR bytes 相同。
  CDR alignment padding 不作數值等價斷言；實際載入位置亦核對，避免两端誤用同一套生成物。
- 新根目錄 fresh CMake 全目標編譯成功。
- 29/29 Python core 對照及 29/29 舊原生逐步對照：qpos、qvel、控制和感測值逐 bytes 相同。
  452 個非名稱／檔案位置的物理陣列及 29 個 option 欄位相同；27 份 MJB 完全相同，2 份 legacy Go2 僅檔案位置資訊不同。
- 實際重新匯出的九份模型 XML 完全相同；catalog 全部值相同，只有 JSON key 順序不同。
- source 指紋保留原 151 項輸入並擴充到 206 項，包括扫描、訊息來源、工具、launcher 與測試。

新結構的正式驗收全部通過：七台各 31/31 完整 ROS 路線；CTest 10/10；Python 394 tests＋65 subtests，四個 render／Warp／CUDA／live ROS 旗標均啟用、零 skip。
全部案例用預設 `build/native` worker，沒有用環境覆寫繞過入口。每例維持 200 Hz IMU、10 Hz ×20,000 rolling 射線及 10 Hz 雙相機，RTF 0.999996311–0.999998749。
FR-mini GUI full＋CUDA 1.2 s 實際通過，241 IMU、12 LiDAR、每相機 12 幀；RTF 0.999954683。

草稿 binary SHA256：`a79423d934f91044120bc8e15fbacbbf9dfa0d36b666c35807105d7816432de8`；206 項源碼／資源集合前後相同，SHA256
`0cc1539379c4183686fa47475b1bf03cc2455aab38d22d868513aa4c7e5f1bb0`。正式目錄已另行重新建置；binary 包含工作路徑資訊，不能沿用草稿 binary SHA。

本機證據：[原生數據等價](../output/repository-layout-20261014/evidence/native-layout/summary.json)、
[原訊息／新包互通](../output/repository-layout-20261014/evidence/ros-interface-compat/results.json)、
[七台與全套驗收](../output/layout-e2e-20261014/compact-summary.json)。
量測是各自一次完整負載，CPU 7.74–18.34%（100% 為單核心）、程序峰值 RSS 220.86–586.13 MiB。
這不是資源 A/B；sampled compute VRAM 381–406 MiB 也不代表所有 graphics allocation。

此次建置沿用原實際 CUDA cache `52`。原 CMake 在 `project()` 啟用 CUDA 後才宣告 cache 預設 `120`，不會覆蓋已初始化的值；
以實際 cache 與生成命令確認配置，沒有把文字預設當成測量事實。本輪不改 GPU 架構，使用者可明確設定 CMake 選項。

## 正式目錄切換

正式工作目錄已切換到新結構，五個舊頂層目錄 `simple_rbt/`、`mid360_sim/`、`livox_ros_driver2/`、
`mujoco_sim/`、`mujoco_sim_cpp/` 均已移出。三份舊源碼在本機 `~/Archive/simple-rbt/repository-layout-20261014/gazebo/`
另存完整副本，279 份原追蹤檔逐一核對 SHA256；未追蹤的舊原生／Python 工作資料保存於同一 Archive 的 `previous-workdirs/`。
原 `.venv/`、`.vendor/`、`output/` 及外部 ROS workspace 保留。

正式目錄重新執行 `./setup.sh --native-only` 後，CTest 10/10；FR-mini 坡道和 Go2W 中等樓梯各 31/31 完整 ROS 驗收，
使用預設正式 worker，沒有 binary 路徑覆寫。兩例 RTF 分別為 0.999996364 與 0.999996396。
FR-mini 收到 2,849 幀 IMU、142 幀 LiDAR 及每相機 142 幀；Go2W 收到 2,883 幀 IMU、144 幀 LiDAR 及每相機 144 幀，
維持 20,000 rays/frame。原生 binary SHA256 為 `f043fb68db8417ebf8cd131f0d411c5492e63d8068170f6d34ef089bba1ea709`，
206 項 source 指紋前後仍為上述值；本機證據見[正式目錄驗收](../output/repository-layout-20261014/evidence/primary-activation/summary.json)。

最小 Livox 訊息包實際從正式專案的 `build/ros_interfaces/install/livox_ros_driver2` 載入。
Python editable 安裝改為 `reference/python`，普通 `.venv/bin/python` 可直接匯入；42 個已裝套件的名稱、版本均未改變，
Python launcher 與兩個驗收工具入口全部通過，見[安裝與入口記錄](../output/repository-layout-20261014/evidence/primary-activation/python-package.json)。

## 工作目錄大小

此次追蹤檔案由 448 份減為 181 份，約 394.5 MiB 減為 29.4 MiB。
這只計目前 checkout 的檔案，不含 `.git` 歷史、SDK、模型網格、編譯／實驗結果和本機 Archive；沒有宣稱整台機器釋放同等空間。

## 恢復

需要舊 Gazebo 時使用 `archive_gazebo`；需要結構整理前的原生版本時檢出上述 `823a581`。
目前操作入口和相應新命令見 [README](../README.md)及[原生建置](runtime.md)。
已驗證的歷史測量保留當時的版本和數據，只修正文件搬遷所需的相對連結。
