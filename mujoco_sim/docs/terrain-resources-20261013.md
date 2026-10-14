# Go2／Go2W 多地形與資源驗證 — 2026-10-13 UTC

本階段按 Xian 的要求，優先完整功能與實時速度，同時量測 RAM、CPU 和磁碟占用。
執行位置為 Jammy 宿主機，20 個邏輯 CPU、RTX 5060 Ti 16 GB。
GPU 雷達與 EGL 相機均在宿主環境驗證。沒有修改外部 rl_sar／f1reRL 工作區，沒有刪除既有資料。

## 實作範圍

- Go2：預設改為固定版本的 RobotLab ONNX；保留 `legacy` 與 `himloco` 對照選項。
  RobotLab 與 HIM 均完成 15/15 路線，但 RobotLab 另通過原地轉向與側移，故選為預設。
- Go2W：導入固定版本 rl_sar RobotLab ONNX、16 關節輪足模型；50 Hz 策略、200 Hz PD、500 Hz 物理。
- 平地、8° 上下坡、4 級 6 cm 上下樓梯、0–2.5 cm 方塊起伏地形；地形同時具有碰撞與雷達幾何。
- CLI 預設 `tracked`：從理想機體速度及朝向回授修正策略指令。`native` 保留原生策略表現。
  不修改權重、不直接設置機體位姿、不施加額外推力；回授仍不等於可部署到實機的估計器。
- 可選 physics／lidar／full 負載，雙相機支援連續 10 Hz，摘要加入 CPU／RAM／輸出磁碟統計。
- 不在主環境安裝 Torch；新模型只取所需檔案。BLAS／ONNX 預設單執行緒。

## 完整路線測試

每台機器人、每種控制模式各 15 組：速度 0.3／0.6／1.0 m/s，
平地、坡道、樓梯，以及 seed 0／1 的起伏地形。每組使用新程序。
先完成 HIM／Go2W 的 60 組原生及回授對照，再補測 Go2 RobotLab 的 15 組回授路線。
通過條件是走到 x=6 m；進入地形後保持 |y|≤0.95 m、傾斜≤50°、
機體距當地表面至少 0.10 m，且軀幹不能撞地形。每組有依速度設定的時間上限。

| 機器人／模式 | 走完整段 | 獨立速度檢查 | 兩項均通過 |
|---|---:|---:|---:|
| Go2 原生 HIM | 2/15 | 2/15 | 0/15 |
| Go2 HIM 加回授 | 15/15 | 5/15 | 5/15 |
| Go2 RobotLab 加回授（最終預設） | 15/15 | 5/15 | 5/15 |
| Go2W 原生 | 1/15 | 14/15 | 1/15 |
| Go2W 加回授 | 15/15 | 15/15 | 15/15 |

27 組原生路線失敗均因側偏離開走廊，沒有跌倒。加回授的 30 組均走完整段。
追加的 RobotLab 15 組也全部走完，因此最終預設的兩台機器人合計 **30/30 完整路線通過**。
速度檢查與路線完成分開計算：平均前進誤差≤max(0.05 m/s, 指令的15%)、
平均絕對側移速度≤0.1 m/s、平均絕對偏航角速度≤0.1 rad/s，觀察窗至少 1 秒。
RMSE 另行保留，沒有用作通過門檻。指令開始後的加速過程也包含在平均值中。

| 指令速度 | Go2 RobotLab 加回授平均實速 | Go2W 加回授平均實速 |
|---|---:|---:|
| 0.3 m/s | 0.298–0.299 m/s | 約 0.300 m/s |
| 0.6 m/s | 0.593–0.596 m/s | 0.592–0.594 m/s |
| 1.0 m/s | 0.874–0.912 m/s | 0.870–0.907 m/s |

Go2 RobotLab 的 10 組速度檢查失敗全部來自 0.6／1.0 m/s 時的偏航擺動，
平均絕對角速度 0.128–0.165 rad/s；最大累計朝向偏差僅 2.47°，
雖然沒有偏離路線，仍未通過這項平順性門檻。HIM 也有同類問題（0.110–0.162 rad/s）。
1.0 m/s 時回授經常觸及指令上限，不能宣稱穩定達到 1.0 m/s。
本結果只適用於預設難度、固定摩擦係數與上述路線，沒有證明任意坡度、越障或實機能力。

完整資料：`output/stage2-20261013/terrain-matrix/final-summary.json`。
最終 Go2 RobotLab：`output/stage2-20261013/terrain-robot-lab-tracked/summary.json`。
最初 Go2W 評估器誤用了 Go2 的機體名稱，尚未步進就失敗；修正為依 `base_origin` site
尋找機體後，只重跑受影響的 30 組。原錯誤保留在原報告，不計作機器人運動失敗。
合併報告逐組保留來源路徑、時間、姿態、速度、回授飽和、CPU 和 RSS。

## CPU、RAM、顯存與速度

下表 Go2 使用最終 RobotLab 預設，並在選定後重跑全部 6 種資源設定；
先前 HIM 測量另行保留，不混用控制器數據。

`scripts/profile_resources.py` 對兩台機器人各執行 6 種設定，每組新程序、6 秒模擬，
起伏地形 seed 0，站立 2 秒後前進指令 0.3 m/s。感測器實際計算；`--summary-only` 僅限制落盤。
每個雷達幀 20,000 條射線、10 Hz；普通 RGB／Z 深度 320×240，全景 RGB／徑向深度 720×360。

| 機器人／負載 | 程序樹 RAM 峰值 | 實時運行 CPU（100%=1 核） | 實時倍速 RTF |
|---|---:|---:|---:|
| Go2 物理 | 329 MiB | 3.9% | 1.000 |
| Go2 CPU 雷達 | 363 MiB | 8.9% | 1.000 |
| Go2 CUDA 雷達 | 633 MiB | 5.0% | 1.000 |
| Go2 完整感測器／相機 1 Hz | 731 MiB | 10.0% | 0.997 |
| Go2 完整感測器／相機 10 Hz | 729 MiB | 27.0% | 0.997 |
| Go2W 物理 | 418 MiB | 19.7% | 1.000 |
| Go2W CPU 雷達 | 453 MiB | 18.2% | 0.999 |
| Go2W CUDA 雷達 | 725 MiB | 14.8% | 1.000 |
| Go2W 完整感測器／相機 1 Hz | 828 MiB | 20.1% | 0.997 |
| Go2W 完整感測器／相機 10 Hz | 830 MiB | 37.0% | 0.997 |

在 20 邏輯核心主機上，完整 10 Hz 設定分別占整機 CPU 的約 1.35%／1.85%。
同規格關閉實時等待後，Go2 為 **4.00×**、Go2W 為 **2.79×**，CPU 約用滿一核。
這是包含物理、雷達和雙相機渲染的速度；表格不含 ROS 傳輸。ROS 的獨立結果另列。
短測有系統排程波動，例如 Go2W 物理模式 CPU 高於 CPU 雷達模式，不能據此認為加雷達更省 CPU。

CUDA 雷達的程序計算顯存取樣峰值為 162 MiB；完整相機時 Go2／Go2W 為 514／524 MiB。
顯存來自每 500 ms 的 `nvidia-smi` 程序計算配額，可能漏掉 graphics-only 配額，
不是整張顯卡占用率。RAM 為 50 ms 程序樹 RSS 取樣；另保留程序自身 `ru_maxrss`。
CPU 只統計模擬程序，不含外部量測程序；迴圈 RAM 包含第一次 EGL 初始化，未稱為穩態值。
程序總耗時包含匯入，`main()` CPU 計時不含頂層匯入；LiDAR 預熱在迴圈外，首次相機初始化在迴圈內。

原始資料：`output/stage2-20261013/resources-go2-robot-lab/results.json`（最終 Go2）和
`output/stage2-20261013/resources/results.json`（Go2W 及早期 HIM）。

## ROS 全感測器實時驗證

兩台機器人各 8 秒模擬，在 rough 地形使用 tracked 模式、CUDA 雷達、雙相機 10 Hz，
真正發布和接收 ROS 訊息。初次測試功能通過，但 RTF 只有 Go2 HIM 0.609、Go2W 0.557。
檢查 Humble 的訊息 setter 發現，直接傳入 `bytes` 會逐位元組跑 Python 斷言；
改為相同內容的 `array('B')` 後走原生支援的陣列路徑，沒有關閉斷言、降解析度或降頻。
Image 與 PointCloud2 均使用此路徑，測試另驗證非連續影像、float 深度及 `+inf` 的逐位元組一致性。

| 完整 ROS／感測器 | Go2 HIM | Go2W |
|---|---:|---:|
| 修復前 RTF | 0.609 | 0.557 |
| 修復後 RTF | 0.993 | 0.995 |
| 修復後雙相機生成及發布中位耗時 | 19.45 ms | 19.73 ms |
| 模擬程序平均 CPU 核數 | 0.741 | 0.895 |
| 程序 RAM 峰值 | 764 MiB | 865 MiB |

兩台各 14 項通過，包含明確的 **RTF≥0.95** 門檻。
每路相機、雷達、里程計收到 80 幀；IMU 分別收到 1600／4000 幀，對應 200／500 Hz。
驗證了 TF、時間、影像／深度內容、速度指令和超時停車。
這是短時間傳輸與實時測試，並非完整 6 m 路線，不能取代上面的地形驗證。
相機耗時計入渲染與 ROS 發布；此前的 115–117 ms 不能誤稱為純 GPU 渲染耗時。

證據：`output/stage2-20261013/ros2-go2-himloco-rough-10hz-fastpayload.json`、
`output/stage2-20261013/ros2-go2w-rough-10hz-fastpayload.json`；修復前同名但不帶 `-fastpayload`。

選定 RobotLab 後另外重跑最終 Go2 ROS 驗收，仍是 14/14 通過：RTF **0.995**，
相機實際 **9.95 Hz**，生成及發布中位 **19.40 ms**；平均 **0.741 核**，程序峰值 **760.6 MiB**。
收到全部 80 幀／流和 1600 幀 IMU，超時停車末速 0.00045 m/s。
證據：`output/stage2-20261013/ros2-go2-robotlab-rough-10hz-final.json`。
ROS 報告的速度取自 10 Hz 里程計樣本，地形報告取自所有物理步；步態角速度的平均值不能混用。

## 轉向、側移、倒退及停車

`scripts/check_motion.py` 使用新程序分別測原地轉向、側移、倒退、邊走邊轉和零指令站立。
每組為 2 秒站立、8 秒指令、3 秒零指令。下列均為 tracked 模式、平地、指令後半段平均值。

| 功能／指令 | Go2 HIM | Go2 RobotLab | Go2W |
|---|---:|---:|---:|
| 原地轉向 0.5 rad/s | 0.003，失敗 | 0.513 | 0.497 |
| 側移 0.3 m/s | 0.204，偏弱 | 0.297 | 0.299 |
| 倒退 −0.3 m/s | 通過 | −0.298 | −0.299 |
| 邊走邊轉 0.3 m/s、0.3 rad/s | 通過 | 0.300、0.326 | 0.297、0.308 |
| 零指令最大位移 | 0.013 m | 0.001 m | 0.028 m |

RobotLab 與 Go2W 各 5/5 通過，HIM 4/5。通過門檻是平均 vx／vy 誤差≤0.10 m/s、
wz 誤差≤0.15 rad/s，並維持直立；不是每個瞬間都滿足這個精度。
HIM 原地轉向指令有 92.1% 時間觸及上限，側移修正有 45.3% 時間觸及上限，
因此其側移雖勉強通過門檻，仍不適合作為完整平面運動能力的結論。
RobotLab 五組指令沒有觸及指令／修正上限。

停車門檻查看零指令階段最後一秒，不能據此宣稱瞬時停止；
HIM／RobotLab／Go2W 停車最大額外移動分別為 0.112／0.020／0.056 m。
這些只證明短時間平地指令行為，不是地形中轉向或長時間可靠性測試。
原始資料：`output/validation-20261013/planar-motion.json` 和
`output/stage2-20261013/planar-motion-go2-robot_lab.json`。

## 磁碟

本輪資源量測時，下列為實際配置區塊容量；統計不跟隨符號連結，同目錄內硬連結去重。
不同目錄或 uv 共用快取仍可能共用檔案，因此不能把各項當成刪除後必然釋放的空間。

| 目錄 | 占用 |
|---|---:|
| `.venv` | 約 810 MiB |
| `.vendor`，含前階段上游庫及本輪兩台模型 | 約 411 MiB |
| `.git`，含舊專案歷史 | 約 158 MiB |
| `mujoco_sim` 程式／ONNX／文件 | 約 3 MiB |
| `output`，本輪量測當下 | 約 14 MiB |
| 原有 `mid360_sim` | 約 183 MiB |
| 原有 `simple_rbt` Gazebo 資料 | 約 206 MiB |

這不是另裝一套 Isaac／Torch 環境；新增兩台機器人的本機來源模型約 60 MiB、三個 ONNX 約 2.4 MiB。
未刪除舊專案或共用快取。普通運行只保存最後一組感測器檔與流式軌跡；
大量重複性能測試可使用 `--summary-only`，每次輸出新目錄以保留證據。

## 尚未完成的能力

LiDAR 仍是在幀末場景快照中射線取樣，沒有逐點運動畸變；IMU 沒有偏置／噪聲；
里程計與 tracked 控制使用理想狀態。未進行 FAST-LIO 建圖、真機驗證或完整 Gazebo 對照。
Tracer、Limo 和多機器人仍未移植。

下一步應處理 Go2 快速步態的偏航擺動，並用含偏置／噪聲的估計器替換理想回授，
再擴大到不同摩擦、坡度、障礙尺寸和完整 LIO 閉環；保留本輪原生／回授兩組基準。

## 最終回歸

本機完整地形演示：`output/stage2-20261013/terrain-preview/terrain-preview.gif`，
另有同名 PNG 和 `manifest.json`。6/6 段實際物理運行走到 x≥6 m，
明確標註理想狀態回授及 1.5 倍速回放；不是控制器插值動畫。
合成 GIF 83 幀、4.81 MiB，所有預覽資料約 10.50 MiB。
完成全部驗證與預覽後，整個專案（含虛擬環境、vendor、舊模型與 Git 歷史）約 **1.8 GiB**。

最終預設控制器與 ROS 資料路徑下：**63 tests passed、2 subtests passed**，沒有跳過項目。
啟用真實 EGL 相機、CUDA 地形射線及 ROS 發布／接收；4 種地形的 CPU／CUDA／幾何高度一致性均通過。
模型 bootstrap 重新驗證 Go2 23 檔／30,201,326 bytes、Go2W 22 檔／32,839,932 bytes 的固定雜湊。
依賴相容性、shell 語法與 `git diff --check` 通過。

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros_ws/to_migrate_ws/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 MUJOCO_GL=egl SIMPLE_RBT_TEST_RENDER=1 \
  SIMPLE_RBT_TEST_WARP=1 SIMPLE_RBT_ROS_TESTS=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=83 \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -m pytest -q mujoco_sim/tests
```
