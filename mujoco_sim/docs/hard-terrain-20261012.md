# 較高地形測試 — 2026-10-12（洛杉磯時間）

Xian 指出原有 6 cm 台階和 8° 短坡太容易。本輪增加明確的 moderate／hard／extreme
尺寸，保持原有 standard 作對照，並實測完整上行、平台、下行及離開地形的能力。
首輪機器人模型、控制器權重、回授增益與通過門檻均未調整；Go2 與 Go2W 使用各自的 rl_sar RobotLab 策略，
兩者使用 tracked 理想狀態回授。這是仿真測試，沒有實機結果。

## 場景與測試方法

三檔樓梯為每階 12／18／22 cm、6／6／8 級，踏面 30 cm，總高 0.72／1.08／1.76 m。
坡道為 15°／20°／25°，單側水平長度 3／4／4 m，平台長 1 m，頂高 0.804／1.456／1.865 m。
起伏塊最大高度為 5／8／12 cm，保留同一個 seed 0 排列。
尺寸入口見 [使用指南](../README.md)，完整幾何保存在每個案例的 `terrain` metadata。

坡道終點隨實際尺寸延長至 x=8.6／10.6／10.6 m，樓梯終點 x=6／6／7.1 m。
時間上限為 `2 秒站立 + max(8 m, finish_x + 2 m) / 指令速度`，
所以長坡不會沿用舊的 x=6 m 提前通過或因舊時限提前結束。
穿過走廊、機體傾斜≤50°、機體原點距當地表面≥0.10 m、軀幹不得碰到地形等條件保持原樣。
每案使用新程序；速度為 0.3／0.6／1.0 m/s，共 2 台 × 3 檔 × 3 地形 × 3 速度 = **54 案**。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  .venv/bin/python mujoco_sim/scripts/evaluate_terrains.py \
  --robots go2 go2w --go2-controllers robot_lab --control-modes tracked \
  --levels moderate hard extreme --terrains ramp stairs rough \
  --speeds .3 .6 1 --rough-seeds 0 --output output/hard-terrain-new
```

## 結果

**34/54 完整路線通過：Go2 10/27，Go2W 24/27。**
下表列出「走完整段」的指令速度，不代表瞬時速度精度或可靠性已獲證明。

| 地形 | Go2 完整通過的速度檔 | Go2W 完整通過的速度檔 |
|---|---|---|
| 15°、高 0.80 m 坡道 | 0.3 m/s | 0.3／0.6／1.0 m/s |
| 20°、高 1.46 m 坡道 | 本輪無 | 0.3／0.6／1.0 m/s |
| 25°、高 1.87 m 坡道 | 本輪無 | 0.3／0.6／1.0 m/s |
| 12 cm × 6 樓梯 | 本輪無 | 0.3／0.6／1.0 m/s |
| 18 cm × 6 樓梯 | 本輪無 | 0.3／0.6 m/s |
| 22 cm × 8 樓梯 | 本輪無 | 0.3 m/s |

5／8／12 cm 的起伏地形：兩台各 9/9 路線通過。僅測了一個固定 seed，不能當成任意碎石地形保證。

速度門檻仍獨立計算：平均前進誤差≤max(0.05 m/s, 指令的15%)、
平均絕對側移速度≤0.1 m/s、平均絕對偏航角速度≤0.1 rad/s，觀察至少 1 秒。
**同時通過路線和速度檢查為 25/54：Go2 2/27、Go2W 23/27。**
失敗前的局部運動也可能符合速度門檻，因此不能拿局部速度合格冒充完成路線。
Go2 15°、0.3 m/s 雖走完全程，仍未過偏航擺動門檻；
Go2W 12 cm 樓梯的 1.0 m/s 檔走完全程，但未過獨立速度檢查。

20 個未完成案例包含 11 個超時與 9 個確認的軀幹接觸，沒有因傾斜、走廊或低間隙單獨觸發的案例。
Go2 在 15° 的較快檔位仍停留上坡段；20°／25° 時主要卡在坡道入口附近。
Go2 的 12 cm 樓梯在 0.6 m/s 檔曾到達平台，但沒有按時完成下行，不能算通過。
Go2W 的 18 cm、1.0 m/s 檔已離開樓梯，但在 x=5.975 m 時到達時限，距 x=6 m 終點尚差約 2.5 cm；
仍按原門檻記為超時，不能描述成「爬不上」。22 cm 的 0.6／1.0 m/s 檔在上行段出現軀幹接觸。
「超時未完成」只描述本輪時限內的結果，不能直接推論延長時間也永遠無法通過。

評估器新增 approach／ascent／top／descent／exit／complete 階段記錄。
較高台階邊緣的地表高度會跳變，低間隙不等於已跌倒；結果保留較高表面、
前一步地表高度、越過的台階邊緣及同時接觸情況，並將低間隙與確認接觸／大傾斜分開。
舊 `fall_detected` 欄位保留相容性並加定義，不把它直接稱為實際摔倒次數。

原始資料：`output/hard-terrain-20261012/matrix/summary.json`，每個案例另有完整結果、
有上限的軌跡取樣與 log。`matrix/RESULTS.md` 提供速度檔、失敗階段和來源雜湊。

## Go2 HIMLoco 同場景補測

Xian 要求測試另一套已接入的 rl_sar Go2 策略 HIMLoco。
指令前進速度明確為 **0.3、0.6、1.0 m/s**；這是目標值，實際速度另行測量。
只替換 Go2 策略，沿用首輪的 tracked 回授、幾何、seed 0、站立時間、路線時限及兩類通過門檻。
共 3 檔地形尺寸 × 3 種地形 × 3 個指令速度 = **27 案**，逐案使用獨立程序。
所有配對案例的配置（除策略名稱）、地形 metadata、路線門檻及速度門檻均一致；
核心、地形與評估器的來源 SHA 也與首輪一致。

| Go2 策略 | 15／20／25° 坡道 | 12／18／22 cm 樓梯 | 5／8／12 cm 起伏地形 | 完整路線合計 | 路線與速度均合格 |
|---|---:|---:|---:|---:|---:|
| RobotLab（首輪） | 1/9 | 0/9 | 9/9 | 10/27 | 2/27 |
| HIMLoco（補測） | 0/9 | 0/9 | 6/9 | 6/27 | 2/27 |

HIMLoco 完整通過的指令速度：5 cm 起伏為 0.3／0.6／1.0 m/s，
8 cm 起伏為 0.3／0.6 m/s，12 cm 起伏為 0.6 m/s。
同時通過速度門檻的完整路線只有 5 cm 與 8 cm 起伏的 0.3 m/s，和 RobotLab 相同。
相對 RobotLab 沒有新增成功路線，少了 15° 坡道的 0.3 m/s、8 cm 起伏的 1.0 m/s，
以及 12 cm 起伏的 0.3／1.0 m/s。

21 個未完成案例為 **5 個超時、16 個確認的軀幹接觸**；沒有初始化、非有限狀態或程序錯誤，
也沒有僅因低間隙、傾斜或走廊門檻停止的案例。
15° 坡道三個速度檔均卡在上坡入口附近，最終 x 約 1.32–1.35 m（終點 8.6 m）。
12 cm 樓梯的 0.3 m/s 在 x=1.869 m 上行時發生軀幹接觸；18／22 cm 樓梯常在機體原點
抵達第一階前已碰到台階，因軀幹向前伸出，不能只按原點所在階段判定「尚未接觸地形」。
本次結果支持保留 RobotLab 作預設，不能據此推論所有 Go2 策略或實機都無法通過。

部署核對：manifest 中 23 個來源檔案的大小與 SHA 均符合記錄；ONNX 與保留的 3 組
Torch 輸出參考最大絕對差為 1.13e-6。45 維觀測 × 6 幀、最新幀在前、關節映射及
5 ms 物理／20 ms 策略週期與固定版本上游一致。
但目前適配器依各策略配置截斷 PD 扭矩：HIM 為 ±33.5 N·m，RobotLab 為 ±23.5 N·m。
模型電機本身的限幅為髖／大腿 ±23.7、膝 ±45.43 N·m，因此 HIM 膝關節在本機更受限。
上游活躍 MuJoCo 路徑的 `RLFSMState::RLControl` 將 tau 設為 0，再由 `SetCommand`
重新計算位置 PD 寫入 ctrl，沒有使用 `ComputeOutput` 的軟體截斷扭矩。
這是既有部署差異，其對通關的影響尚未實測；不能稱當前控制路徑為上游的完整復現。
當前亦從 crouched keyframe 啟動，未重放 GetUp FSM。
下一步應先排查部署差異，再比較策略能力；本次結果限定為固定權重在當前適配器下的表現。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  .venv/bin/python mujoco_sim/scripts/evaluate_terrains.py \
  --robots go2 --go2-controllers himloco --control-modes tracked \
  --levels moderate hard extreme --terrains ramp stairs rough \
  --speeds .3 .6 1 --rough-seeds 0 --output output/himloco-hard-terrain-new
```

資料位於 `output/himloco-hard-terrain-20261012/matrix/`：`summary.json` 保存所有案例，
`RESULTS.md` 與 `compact-summary.json` 提供配對比較、實際均速、失敗階段和來源／權重 SHA。
[15° 坡道與 12 cm 樓梯真實演示](../../output/himloco-hard-terrain-20261012/preview-moderate-v03/terrain-preview.gif)
均使用 0.3 m/s 指令；保留直立卡住後超時和軀幹接觸兩種不同結果。

完整感測器另測 HIMLoco 的 12 cm 起伏地形、0.6 m/s，運行 13 秒：
每路相機與雷達各 130 幀（10 Hz、每幀 20,000 射線），前進位移 6.452 m，
RTF **0.9989**，RAM 峰值 **731.7 MiB**，平均 CPU **0.253 個邏輯核心**，
取樣的 GPU 計算記憶體峰值 514 MiB。這次未開啟 ROS 傳輸；資源腳本不執行正式通關門檻，
相應獨立矩陣案例通過路線、未通過速度門檻。只保存摘要，避免連續感測器資料佔用磁碟。
資源證據：`output/himloco-hard-terrain-20261012/resources-extreme-rough-v06/results.json`。
本次補測與演示新增磁碟用量約 9 MiB，沒有下載或新增權重。

## 長坡上的完整感測器與資源

另外在宿主機開啟每幀 20,000 射線的 CUDA 雷達、IMU、普通及全景 RGB／深度相機，
全部相機 10 Hz。這兩次為獨立程序、實時運行、只保存摘要，**不含 ROS 傳輸**。

| 場景 | 模擬時長／相機幀數 | 最終 x／完成線 | 實時係數 | RAM 峰值 | 平均 CPU |
|---|---|---|---:|---:|---:|
| Go2：15° 長坡，0.3 m/s | 32 s／320 幀 | 8.864／8.6 m | 0.9996 | 730 MiB | 0.237 核 |
| Go2W：25° 長坡，0.6 m/s | 21 s／210 幀 | 10.816／10.6 m | 0.9992 | 828 MiB | 0.340 核 |

兩次均真正跨過相應完成線，完整感測器仍保持接近實時。幀數是每一路相機，雷達亦為同樣幀數。
資源測量腳本不是通關評估器；正式路線／碰撞／速度結論仍以獨立 54 案矩陣為準。
CPU 1 核表示佔滿一個邏輯核心，RAM 為程序樹取樣峰值。
Go2W 最初的 20 秒資源測試僅走到 x=10.282 m，尚未跨線；保留它並另跑了完整的 21 秒案例，
沒有把不完整片段當成全程測試。

資料：`resources-go2-moderate-ramp/results.json` 與
`resources-go2w-extreme-ramp-complete/results.json`，均位於 `output/hard-terrain-20261012/`。
`profile_resources.py` 新增 `--terrain-level`、`--speed` 和 `--profiles`，可只測必要負載：

```bash
.venv/bin/python mujoco_sim/scripts/profile_resources.py --robots go2w \
  --terrain ramp --terrain-level extreme --speed .6 --duration 21 \
  --profiles full_10hz --output output/hard-ramp-resources-new
```

## 回歸檢查

**133 tests、2 subtests 全部通過**，包含宿主 CUDA 地形射線、真實碰撞／接觸、
EGL 相機、既有 ROS 介面以及延長路線的評估器檢查。
標準場景的 4 種地形 × difficulty 0.5／1／3，共 12 份 MJCF 與修改前逐位元組相同。
新高地形的高度查詢與 MuJoCo 實際射線一致；CPU／CUDA 誤差門檻維持 0.1 mm。
坡頂和下坡接觸測試涵蓋新尺寸，沒有用只有視覺而無碰撞的假地形。

## 可視化證據

兩組四格演示都保留 Go2 的失敗及 Go2W 的完整上下行，和矩陣結果一致；
停止畫面會明確寫出超時或軀幹接觸，不會把失敗片段剪掉。

- `output/hard-terrain-20261012/preview-hard-v06-final/terrain-preview.gif`：20° 坡道、18 cm 樓梯，0.6 m/s。
- `output/hard-terrain-20261012/preview-extreme-v03-final/terrain-preview.gif`：25° 坡道、22 cm 樓梯，0.3 m/s。

每組另有 `terrain-preview-final.png` 和 `manifest.json`，包含實際尺寸、命令、
結束位置、失敗原因、姿態與時間。相機視角已檢查，長坡終點沒有被房間牆體遮擋。
GIF 按累積時間取整到 10 ms，避免播放速度偏移；結果畫面另停留 1.5 秒。
全部 10 個 GIF 的時間修正前後逐幀畫面一致，核對記錄保存在各目錄的 `gif-timing-verification.json`。

## 範圍與下一步

本輪沒有提高馬達輸出、改模型質量／摩擦、換策略、放寬通過門檻，或刪除舊測試。
新增幾何仍使用少量 box，沒有下載新的模型或權重。
本輪原始結果、演示與保留的舊編碼合計佔用約 62 MiB 磁碟，未保存連續感測器資料流。
下一步應針對 Go2 的陡坡爬升、高台階落腳與 Go2W 高台階的高速上行檢查策略能力；
保持這組固定場景作回歸依據，再加入不同摩擦、起步位置與多 seed，避免只記住單一路線。
