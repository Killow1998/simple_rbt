# Go2 部署對照與 Go2W 聯合驗收

本階段接續 [較高地形測試](hard-terrain-20261012.md)，完成部署因素對照、材料／多 seed 測試，
以及同一次物理運行中的完整 ROS、感測器、速度與資源驗收。
資料沿用實驗開始時的 `output/deployment-validation-20261012/`；本紀錄於 2026-10-13 收尾。
命令中的速度都是 **指令目標 m/s**，實際前進、側移和偏航速度分別測量。

## 目前的決定

Go2 與 Go2W 仍使用各自的 RobotLab 策略。Go2 保留原 `configured + crouched` 預設，
新增 `upstream` 扭矩路徑和 `upstream_getup` 起身流程作可選部署對照。
完整驗收沒有支持改預設：起身讓 Go2 多完成一條 12 cm 樓梯，但速度仍不合格，
另外一條起伏地形失去偏航合格結果。提高到上游扭矩路徑沒有淨增通關。

Go2W 在已測條件下具備完整實時 ROS／感測器路線能力；接下來應保留這些條件作回歸基準，
而非宣稱任何路面或高速台階都可通過。陡坡對實際接觸材料敏感；22 cm 台階的較高速仍碰到軀幹。
Go2 下一個控制工作應集中於速度與步態偏航穩定性，並明確報告環境材料。

## Go2：分開測扭矩與起身

固定來源為 rl_sar `36bac3930bca95239de9bcfade283832e6ee4409` 和
rl_sar_zoo `d4cca961c70c9265e92a58639f1fe9aaccb2ad0f`，權重、關節映射及模型電機範圍保留。
先測 HIM／RobotLab × 兩種扭矩 × 兩種啟動 × 六個代表案例，共 **48 案**。
六案為 15°／20° 坡道、12／18 cm 樓梯各 0.3 m/s，以及 12 cm 起伏的 0.3／1.0 m/s。

- `configured`：依策略配置截斷 PD 扭矩，保留既有行為。
- `upstream`：按固定版本上游 MuJoCo 路徑把 PD 結果寫入 ctrl，仍受原 MJCF 電機限幅。
- `crouched`：原啟動方式，直接從 keyframe 運行策略。
- `upstream_getup`：重放首次 Passive→GetUp 的插值，完成後下一物理步才啟動新策略。

GetUp 只使用原電機控制；沒有搬動姿態、加外力或重設模擬時鐘。
1 秒預起身與 2 秒站起插值使用上游 float32 累加和同 tick 轉段，實際 **601 步／3.005 秒**。
起身期間策略 history 與外層 tracker 不更新。起身完成後另給足原 2 秒穩定期及路線預算。
啟動中的軀幹接觸和間隙另外記錄；正式路線仍使用原傾斜、間隙、碰撞、走廊及時限門檻。
這是確定性的模擬轉場，沒有重放人工按鍵等待或上游非同步執行緒相位。

48 案的單因子對照中，HIM 四種模式均為 0/6 完整路線；RobotLab 的起身因素讓路線 3/6→4/6，
扭矩因素沒有新增完整路線。這個六案樣本均沒有路線與速度雙合格案例。
12 個重測的原預設案例與歷史結果的共享物理數值、門檻及取樣軌跡完全一致；
另有 9 組舊核心／新核心的 5 秒預設 rollout，qpos／qvel／ctrl 逐位元組一致。

對有路線改善的 RobotLab `configured + upstream_getup` 再測完整 **27 案**：

| 指標 | 原 crouched | GetUp |
|---|---:|---:|
| 完整路線 | 10/27 | 11/27 |
| 路線與速度均合格 | 2/27 | 1/27 |
| 平地轉向／側移／倒退／邊走邊轉／停車 | 5/5 | 5/5 |

新增成功路線是 12 cm 樓梯、目標 0.3 m/s：真實均速約 **0.232 m/s**，
平均絕對偏航速度 **0.151 rad/s**，兩項速度門檻都失敗。
完成時只剩 **66.7 ms** 路線時限餘量，不能作為穩定的攀爬通關保證。
8 cm 起伏、0.3 m/s 雖仍完成，但偏航 0.1043 rad/s 超過原 0.1 門檻，造成雙合格數下降。
[真實樓梯演示](../output/deployment-validation-20261012/preview-go2-getup-stairs/terrain-preview.gif)
顯示完成 GetUp 後的整段上下樓梯；其綠色終點只表示路線完成，速度判定以矩陣為準。

證據：`RESULTS-GO2.md`、`go2-deployment-summary.json`、`go2-ramp-stairs/`、`go2-rough/`、
`go2-robot-lab-getup-full/`、`go2-getup-motion.json` 和 `baseline-parity.json`，均在本階段 output 下。

## 材料與地形排列

原 Go2 腳接觸優先級為 1、滑動摩擦 0.4；Go2W 輪接觸為 1、滑動摩擦 0.8。
單改地形名義摩擦會被輪／腳設定覆蓋。
新 `--friction` 明確指定 floor 與 course 的材料：priority 2、condim 6、
三係數為 `(mu, 0.02, 0.01)`，並用實際 contact.friction 核對。
省略此選項時原混合行為保留；default→explicit 也改變混合優先級及材料轉動係數，
因此公平的 mu 對照應固定相同 explicit 設定，只替換 mu。

Go2W 做 **45 案**，指令速度為 0.3／0.6／1.0 m/s：

| 分組 | 完整路線 | 路線與速度均合格 |
|---|---:|---:|
| 12 cm 起伏，seed 1／2／3，原材料 | 9/9 | 8/9 |
| 25° 坡道，mu 0.2／0.4／0.8 | 3/9 | 3/9 |
| 18／22 cm 樓梯，mu 0.2／0.4／0.8 | 8/18 | 7/18 |
| 12 cm 起伏，0.6 m/s，三 seed × 三 mu | 9/9 | 9/9 |
| 合計 | 29/45 | 27/45 |

16 個路線失敗為 8 次超時、8 次確認的軀幹接觸，沒有執行錯誤。
25° 坡道只有 mu 0.8 的三個速度都通過，mu 0.2／0.4 均在上坡超時。
22 cm 樓梯的 0.3 m/s 在三種材料都走完，0.6／1.0 m/s 則在上行軀幹接觸。
原材料起伏 seed 2、1.0 m/s 的偏航 0.10176 rad/s 略超門檻；
22 cm、mu 0.2、0.3 m/s 的偏航 0.10650 亦失敗。門檻沒有放寬。

Go2 另做 **12 案** RobotLab 材料對照：0.3 m/s、兩啟動、15／20／25°、mu 0.4／0.8。
15° 在兩材料均走完；20° 在 mu 0.8 的兩啟動走完，而 mu 0.4 均超時；25° 全部超時。
合計路線 6/12、路線與速度雙合格 0/12，所有完成案的前進均速接近指令，但偏航仍超標。
較高抓地材料確實影響結果；它不是原預設模型的通關，也不能當成策略能力已完全改善。

證據：`go2w-reliability/RESULTS.md`、`go2-ramp-friction/RESULTS.md` 及各自 compact JSON。
這些指定案例不是可靠性概率估計。

## Go2W：同一次物理運行的完整聯合驗收

新 [validate_full_run.py](../tools/validate_full_run.py) 使用 fresh worker 與獨立 DDS 接收程序。
運動指令只經 `/cmd_vel`；原路線門檻每個物理步檢查，速度窗只涵蓋穩定期後到終點／失敗，
不包含通關後失聯停車。雙方用 run ID、PID、配置與來源 SHA、時間戳核對同一執行。
所有可靠訊息數量及實際 payload hash 必須吻合，odom／TF／相機外參也交叉核對。

負載為每幀 20,000 射線的 10 Hz 雷達、500 Hz IMU、10 Hz 普通及全景 RGB／深度，
普通相機 320×240、全景 720×360。通關後接收方停止發指令，另驗證 deadman 實際停止。
總合格要求 **完整路線、原速度門檻、ROS 契約／收發、實時係數≥0.95、失聯停車全部通過**。

| Go2W 場景 | 指令 m/s | 全部 23 項檢查 | RTF |
|---|---:|---|---:|
| 25° 坡道，原材料 | 0.6 | 通過 | 0.9975 |
| 18 cm 樓梯，原材料 | 0.6 | 通過 | 1.0000 |
| 22 cm 樓梯，原材料 | 0.3 | 通過 | 0.9991 |
| 12 cm 起伏，seed 1／2／3，原材料 | 0.6 | 三案均通過 | 0.9973–0.9992 |
| 25° 坡道，mu 0.8 | 0.6 | 通過 | 0.9890 |
| 25° 坡道，mu 0.4 | 0.6 | 上坡超時、速度不合格 | 0.9981 |
| 22 cm 樓梯，mu 0.8 | 0.3 | 通過 | 0.9989 |

**9 案中 8 案全部合格**。唯一未通過案停在 x=1.273／10.6 m、無軀幹接觸，
ROS 資料和實時性仍合格；未到終點導致終點／deadman 派生條件無法成立，不能據此說停止機制故障。
18 cm 樓梯的平均絕對偏航為 0.09847 rad/s，距 0.1 門檻很近，仍需要重複起步條件驗證。
9 案來源 SHA 全程一致；各次材料對照實際接觸係數零 mismatch。

| 資源 | worker：物理、全部感測器、ROS、驗收取樣 | 獨立接收驗收程序 |
|---|---:|---:|
| 程序 RSS 高水位 | 863–868 MiB | 130–308 MiB |
| 平均 CPU，1 核＝一個滿載邏輯核 | 0.785–0.892 核 | 0.396–0.421 核 |
| worker 計算 VRAM，1 Hz 取樣 | 524 MiB | 未合併 |

接收程序 CPU 的量測區間包含啟動／收尾，比 worker loop 更長，兩個平均值不能直接相加冒充同步總量。
GPU 數字是取樣計算程序配置量，不是完整顯卡占用或保證峰值。
相機 render＋publish 中位數 19.7–20.6 ms，雷達約 1.05–1.07 ms；實際牆鐘相機率 9.85–9.98 Hz。
warmup 和 DDS discovery 在 loop RTF 外，整個程序資源高水位仍包含它們。

Go2 GetUp 的 12 cm 樓梯另測完整感測器、不含 ROS：33 秒、每路 330 幀，
前進位移 6.455 m、RTF 0.9996、RAM 730.5 MiB、平均 CPU 0.233 核。
這個資源腳本不檢查正式通關；相應矩陣案例只通過路線，速度仍失敗。

證據：`full-ros/summary.json` 及每案 report／worker／compiled-model／log；
Go2 資源為 `resources-go2-getup-stairs/results.json`。
本階段新結果與演示約 **36 MiB**，感測器只保存摘要、雜湊與有界軌跡，不保存連續影像／點雲流。

## 重跑與時間語義

在專案根目錄、Jammy 宿主機執行；新目錄可保留既有證據：

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros_ws/to_migrate_ws/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 MUJOCO_GL=egl \
  .venv/bin/python mujoco_sim/scripts/validate_full_run.py \
  --robot go2w --controller rl_sar --terrain ramp --terrain-level extreme \
  --speed .6 --camera-hz 10 --backend warp --domain 85 --output output/full-run-new
```

`--camera-hz` 在此嚴格聯合工具只接受 1 或 10 Hz；本階段使用 10 Hz。
普通 CLI 的 `--duration` 是包括 GetUp 和 settle 的總模擬時長。
例如 GetUp 3.005 秒＋settle 2 秒，8 秒總時長只有約 2.995 秒移動窗口。
terrain／motion／preview 評估則在完成 GetUp 後另給完整測試時間；絕對時鐘不重設。
CLI 分開記錄排定與真正開始的指令時間，總時長不足時實際開始欄位為 null。

回歸：宿主 CUDA／EGL／完整 ROS 的全套 **181 tests＋17 subtests** 通過；
最後的 CLI 時間欄位修正另有 **2 項短／完整指令窗口檢查**通過。
沒有為文件或不影響物理的時間欄位再重跑已完成矩陣。

## 下一個控制工作

保留這套幾何、門檻、材料和完整 ROS 驗收作固定回歸。
Go2 優先改善前進速度與步態偏航，並在固定材料下衡量台階能力；
Go2W 優先擴充 18 cm 樓梯的重複起步條件，以及 22 cm 的較高速落腳。
現有結果仍使用理想狀態回授、無偏置 IMU、無運動畸變雷達；尚無真機驗證。
