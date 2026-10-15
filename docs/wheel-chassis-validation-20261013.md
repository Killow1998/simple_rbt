# 輪式底盤部署與連續地形驗收 — 2026-10-13

FR-mini、Ridgeback R100、Ranger Mini 3.0 已在 Jammy 接入 MuJoCo 核心、CLI 和 ROS 2。
連同 Go2、Go2W、雙輪參考底盤及 Scout Mini，目前共有七個可啟動配置。
本階段從 `cd6a1bd` 開始，在項目現有文件目錄保存結果，
沒有推送外部倉庫、下載網格或控制權重，也没有刪除既有實驗。

## 本輪要求與實作

Xian 明確要求純輪式底盤只考慮連續表面：`flat`、`ramp`、`hills`。
Scout Mini、FR-mini、Ridgeback、Ranger Mini 及雙輪參考模型均拒絕 `stairs` 和方塊式 `rough`；
Go2／Go2W 的既有地形與歷史記錄另行保留。台階通行不是本輪未完成目標。

`hills` 是 4 m 路段上的兩個波浪，以 48 個真實傾斜盒體近似；相鄰頂面高度一致，沒有垂直階躍。
峰高為 standard 2.5 cm、moderate 5 cm、hard 8 cm、extreme 12 cm，波長 2 m。
表面高度連續，接縫的斜率仍有變化；本輪運動矩陣使用前兩檔，不宣稱後兩檔通過。
所有接縫兩側的真實 MuJoCo 垂直射線檢查通過，CPU／CUDA 雷達與高度查詢亦使用同一份碰撞幾何。

| 新配置 | 真實關節與控制 | ROS 速度輸入 | 輸入範圍 vx／vy／wz |
|---|---|---|---|
| `fr_mini` | 前輪阿克曼轉向、兩後輪驅動；前輪滾動被動 | `/fr_mini/cmd_vel` | ±1 m/s／0／±1 rad/s，另受轉彎半徑限制 |
| `ridgeback` | 四輪轂驅動＋40 個斜置被動滾子 | `/ridgeback/cmd_vel` | ±1／±0.6／±1 |
| `ranger_mini` | 四個舵關節＋四個驅動關節 | `/ranger_mini/cmd_vel` | ±1／±0.6／±1 |

FR-mini 不接受側移或原地旋轉；模型按原廠最小半徑 1.1 m 的後軸解讀限幅，
有效 `|wz| <= |vx| / 1.1`。其 `base_link`／速度參考點位於後軸中心上方，車殼中心在前方 0.2 m。
Ridgeback、Ranger 支援前後、左右、原地轉向和複合運動。轉向、輪速及力矩均有界；
機體運動只來自真實接觸與關節驅動，沒有外力補推、指定機體速度或改寫位姿。
麥輪兩個接觸切向都保留正常摩擦；側移由被動滾子旋轉實現。

`auto` 分別選 `ackermann`、`mecanum`、`swerve`，均為 5 ms 物理步／200 Hz 控制。
`native` 使用本底盤控制器；`tracked` 另加理想仿真機體速度／朝向回授，CLI 預設 tracked。
FR／Ridgeback／Ranger 的 native 不含額外機體回授；Scout native 仍包含其既有陀螺轉向 PI。
Ranger 的通用 swerve 複合模式不是原廠韌體模式的精確重播。

非法或非有限輸入會清零：ROS 繼續仿真，直接 API 清零後報錯；新合法指令可恢復。
FR 的極小 vx 配非零 wz 亦拒絕，避免原地轉向進入輪組。0.5 秒單調時鐘超時停車維持不變。
控制節點與話題按車名隔離；感測器話題／TF frame 仍共用絕對名稱，尚不是完整多機器人感測器隔離。

## 規格來源與模型近似

| 模型版本 | 採用的原廠事實與記錄 |
|---|---|
| `fr-mini-ackermann-rigid-v1` | [FR-mini 產品頁](https://www.yuhesen.com/cn/Product/FR-mini)：18 kg、400 mm 軸距、312 mm 輪距、140 mm 輪徑；官方 V1.0.3 手冊確認前轉向、後雙驅，見 [manifest](../assets/manifests/fr_mini_manifest.json) 的固定提交／PDF 雜湊 |
| `ridgeback-r100-passive-rollers-v1` | [Ridgeback 產品頁](https://clearpathrobotics.com/ridgeback-indoor-robot-platform/)的 135 kg；官方 ROS 2 固定描述的軸位／輪徑；旧 URDF 約 195.8 kg 的差異明確保留，見 [manifest](../assets/manifests/ridgeback_manifest.json) |
| `ranger-mini-3-current-spec-swerve-rigid-v1` | [Ranger Mini 3.0 現頁](https://www.agilex.ai/page/6908c1285e78cfa260412be9)：73 kg、494 mm 軸距、370 mm 輪距；未混用 V1/V2，見 [manifest](../assets/manifests/ranger_mini_manifest.json) |

三台均採原創程序幾何。原廠未公開或未可靠確認的殼體細節、質量分布、舵軸偏置、輪胎曲線、
馬達限制、滾子形狀等皆標為工程估計；真實懸架、載重、電池／熱限制及韌體沒有校準復現。
輸入範圍是保守的仿真接口限幅，不能當作全地形或任意組合都能精確追蹤的承諾。

## 原生運動矩陣

[81 案詳細結果](../output/wheel-chassis-20261013/native-matrix/RESULTS.md)全部完成，**77 通過、4 未通過**。
每案使用新的單執行緒 CPU 程序、相同凍結模型與原數值門檻：

| 底盤 | 通過／全部 | 未通過 |
|---|---|---|
| Scout Mini | 19／19 | 無 |
| FR-mini | 17／18 | 原生 15° 坡道、0.3 m/s，速度誤差 |
| Ridgeback | 19／22 | 15° 坡道三檔速度，坡入口車殼接觸 |
| Ranger Mini | 22／22 | 無 |

四台的所有支援平地動作、8° 坡道、2.5／5 cm 波浪，在 0.3／0.6／1 m/s 下均通過。
FR 不生成側移與原地轉向案例；Ridgeback／Ranger 另測正反側移、轉向及複合運動。
速度穩態窗口排除運動第一秒；vx、wz 誤差為 `max(0.05, 15% × |目標|)`。
合法非零 vy 使用同樣的目標誤差規則；零 vy 仍要求絕對值均值 ≤0.1 m/s。
直行偏航絕對值均值 ≤0.1 rad/s；停車末秒平面速度與偏航絕對值均值 ≤0.05。

安全判斷使用 45° 傾角、實際非輪組接觸和每步的世界座標碰撞包絡，走廊留 0.1 m 餘量。
轉向輪、甲板、被動滾子及 FR 後軸原點偏移均納入幾何；車殼撞地不會被當成正常輪胎接觸。
機體參考點高度只是診斷，沒有沿用四足 0.1 m 門檻當作不同車身的真實離地間隙。
完整 ROS 的輪式驗收使用同一安全規則；Go2／Go2W 既有門檻不變。

### 15° 邊界與 FR 回授對照

Ridgeback 的近似低車殼名義離地 25.9 mm、前懸 161 mm；15° 入口在 X≈0.6167 m 擦底，
三案實際 X=0.6133–0.6187 m。接觸時仍正常前進，不能稱為電機失速或推定實車能力。
[接觸分析](../output/ridgeback-20261013/native-slope-limit-analysis.md)保留推導與同場 8° 通過證據。

FR 原生 15° 下坡有制動滑移／接觸動態造成的超速跡象，原生全程均速為
0.354／0.670／1.090 m/s。後兩案雖通過全程平均門檻，亦有坡段超速，不能把平均通過當成速度始終準確。
保存軌跡中的下坡內段均速約 0.753／0.987／1.370 m/s；0.3 案觀察到 ±4 N·m 後輪制動限幅與輪速振盪。
未記錄接觸法向力，不能分別量化摩擦、载荷轉移及離散伺服的影響。

模型、摩擦與門檻不改的 [tracked 同場六案](../output/wheel-chassis-20261013/fr-mini-tracked-moderate-v03/RESULTS.md)
全部通過；15°、0.3 m/s 全程穩態均速 0.303 m/s，抽樣下坡內段均速 0.332、最大 0.532 m/s。
它改善速度追蹤，仍有瞬態超速，且依賴理想狀態回授，不能當作實車制動驗收。

## 完整 ROS、傳感器與資源

三台的實際 CLI／DDS 驗證各 **20／20 項通過**：
[FR-mini](../output/wheel-chassis-20261013/ros-cli-fr-mini.json)、
[Ridgeback](../output/wheel-chassis-20261013/ros-cli-ridgeback.json)、
[Ranger Mini](../output/wheel-chassis-20261013/ros-cli-ranger-mini.json)。
它們使用預設 tracked 模式、連續波浪、GPU 雷達、10 Hz 双相機；檢查實際控制器、車名話題、
外參、訊息規格、IMU 200 Hz／雷達 10 Hz 時間、普通／全景 RGB＋深度及指令停止後制動。

下列七場每場 **31／31 項通過**，來源雜湊一致且精確匹配最終運行源碼：

| 完整路線 | 平均 vx | 實時倍數 | 仿真 RAM 高水位 | 仿真平均 CPU |
|---|---|---|---|---|
| [FR 8°，native，0.6](../output/wheel-chassis-20261013/full-fr-mini-ramp8-v06/report.json) | 0.591 m/s | 0.9965 | 497.7 MiB | 0.647 核 |
| [FR 5 cm 波浪，native，0.6](../output/wheel-chassis-20261013/full-fr-mini-hills5-v06/report.json) | 0.590 | 1.0000 | 505.0 | 0.685 |
| [Ridgeback 8°，native，0.6](../output/wheel-chassis-20261013/full-ridgeback-ramp8-v06/report.json) | 0.585 | 0.9969 | 508.6 | 0.672 |
| [Ridgeback 5 cm 波浪，native，0.6](../output/wheel-chassis-20261013/full-ridgeback-hills5-v06/report.json) | 0.586 | 0.9987 | 502.2 | 0.729 |
| [Ranger 8°，native，0.6](../output/wheel-chassis-20261013/full-ranger-mini-ramp8-v06/report.json) | 0.590 | 0.9981 | 503.0 | 0.633 |
| [Ranger 5 cm 波浪，native，0.6](../output/wheel-chassis-20261013/full-ranger-mini-hills5-v06/report.json) | 0.592 | 0.9989 | 506.0 | 0.690 |
| [FR 15°，tracked，0.3](../output/wheel-chassis-20261013/full-fr-mini-ramp15-tracked-v03/report.json) | 0.303 | 0.9989 | 506.7 | 0.649 |

此表速度窗口包含起步，截止正式終點，不含其後超時制動；整體驗收另包含制動檢查。
它不等同排除第一秒的穩態矩陣。
每個新仿真 PID 的 RAM 高水位包含載入／暖機；CPU 1 核是一个邏輯核心的 100%。
實時倍數包含正式循環的全部門檻、物理、雷達、雙相機和 DDS，設定／JIT／renderer 暖機在循環外。
接收驗收程序另用約 291–308 MiB／0.39–0.41 核；不是仿真進程內的開銷。
仿真 PID 的 1 Hz compute 顯存採樣為 500 MiB，不是完整 graphics／桌面显存或保證峰值。
這是短時單機案例，沒有宣稱長期無洩漏或多機器人容量。

沿用 [MID360 對齊規格](mid360-validation-20261013.md)：內置 IMU 在雷達座標中
`[+0.011,+0.02329,−0.04412] m`、軸平行；各新車 5 ms 物理步精確對齊 200 Hz IMU。
雷達逐點使用採集時刻位姿，200 kHz／10 Hz；實際 DDS 收到的 IMU、點雲、影像雜湊與同一次物理输出一致。
Raw IMU 為 g／rad/s，SI companion 為 m/s²／rad/s，沒有姿態估計；`/odom` 為真值。
原生 IMU 無噪聲／偏置，雷達仍使用靜態環境且排除機器人自遮擋，沒有變成完整硬體重現。

## 磁碟與重現

十五個新增實作／manifest／評估／測試檔共 **134,069 bytes（約 131 KiB）**、分配 **160 KiB**，
不含本階段文件及既有檔案的修改。依賴與 vendor 數量／容量維持原值：`.venv` 分配約 809.6 MiB，
`.vendor` 約 411.4 MiB。沒有新依賴、網格、權重或供應商程式複製。
保存的本階段矩陣、CLI、完整驗收資料約 **63.3 MiB 邏輯／63.6 MiB 分配**（彙總生成前），
全部在被 Git 忽略的 `output/wheel-chassis-20261013/`；另保留 Ridgeback 原始探針／接觸分析。
詳見 [階段資源彙總](../output/wheel-chassis-20261013/stage-summary.json)。
此容量是驗收證據；沒有持續錄製所有影像／點雲，不能用來預測 rosbag 容量。

從倉庫根目錄重跑：

```bash
./mujoco_sim/run.sh --robot ranger_mini --terrain hills --terrain-level moderate \
  --cmd .6 0 0 --settle 2 --duration 14 --backend warp --sensors full --camera-hz 10 --realtime

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python \
  mujoco_sim/scripts/evaluate_wheels.py --control-modes native --output output/wheel-matrix-new

source /opt/ros/humble/setup.bash
source /home/user/ros_ws/to_migrate_ws/install/livox_ros_driver2/share/livox_ros_driver2/local_setup.bash
.venv/bin/python mujoco_sim/scripts/validate_full_run.py --robot ridgeback --controller mecanum \
  --control-mode native --terrain hills --terrain-level moderate --speed .6 --output output/ridgeback-full-new
```

外部 ROS 控制加 `--ros2`，向相應車名話題發布 Twist；同一 domain、localhost，資料消費節點用模擬時鐘。
一般 `run.sh` 預設 tracked；詳細啟動例與話題表在 [使用說明](https://github.com/Killow1998/simple_rbt/blob/8684c5d927c5ae869aa433c6754ea7d86088bfe9/docs/python-reference.md)。
桌面查看可加 `--viewer`，本輪資源表沒有計入額外 GUI 視窗。

完整回歸在已授權 Jammy 宿主機啟用 render／Warp／CUDA／ROS：
**366 tests＋60 subtests 通過，14.73 秒，無跳過**。
測試含三新車接觸驅動、失去地面時質心、非法指令清零、五車命名話題隔離、
MID360 裝載前後物理不變性、世界投影包絡、連續接縫及 GPU／CPU 射線一致性。

三台部署範圍已完成。後續改進應針對明確需求：例如 FR 原生陡坡制動／估計器、
有資料支持的懸架／輪胎標定、完整多機器人感測器／TF 隔離；台階不列入輪式驗收。
