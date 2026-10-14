# C++ 架構整理與消融驗證 — 2026-10-13

本輪整理 C++ 模組、移除已證明多餘的資料／流程，保留七台機器人的控制、感測器、ROS 介面與原驗收門檻。
架構整理快照的物理與控制分階段對照保持逐位元一致；三台輪式車完整 ROS 交錯 A/B 的程序峰值 RSS 均值下降
8.7–13.2 MiB。CPU 差異小，尚不足以宣稱明確加速。
後續已修正正常腳輪接地的誤分類，最終七台各通過 31/31 正式完整 ROS 檢查。

架構整理提交 `427649f` 的量測用 `simple-rbt-native` SHA256（腳輪修正前）：
`6c912bb2808769beebd2ad9937628e105a850f01fda68ae9147dd446ac6cac3b`。
以下 A/B、消融及感測器結果屬於此快照；後續腳輪修正的 binary／驗收另列，不沿用這個 SHA。
對照原生版本 SHA256：`98a487baec27d0a6dee16435ce460c5beb901a523c94b5bce56c6eb308542c6d`。
本機 `output/` 證據被 Git 忽略；下列相對連結適用於保留實驗資料的工作區。
前一階段的 [C++ 遷移紀錄](cpp-runtime-validation-20261013.md)及其他日期文件保持原樣。

## 分工和保留的邊界

| 入口 | 整理前行數 | 本輪行數 | 拆出的責任 |
| --- | ---: | ---: | --- |
| `core.cpp` | 923 | 268 | scene、RobotSpec、七個 controller、policy、tracking |
| `runtime.cpp` | 572 | 359 | artifacts、resources；排程仍在 runtime |
| `full_run.cpp` | 536 | 248 | CourseMonitor、geometry／friction；觀測器保留真實證據與握手 |
| `validate_full_run.py` | 961 | 287 | course、worker、recorder、gates、evidence |

這是縮小單一檔案的修改範圍，並非把移出的程式算成刪除。完整模組入口見
[原生 README](runtime.md)。`RobotSpec` 集中車型能力、命令限幅、預設控制器和安全門檻選擇；
CLI、ROS 及路線判定使用同一份原生登記。Python 仍保留獨立模型／訊息驗收對照。

`Controller` 表達實際不同的控制律；PImpl 管理 SDK 隱藏與資源生命週期；`RunObserver` 只收實際 runtime hooks。
這三項保留。沒有加入通用插件框架，也沒有把物理判定和訊息序列化合併成一份自我驗證的實作。
CourseMonitor 測試使用真實編譯模型與邊界姿態，已不需要啟動 DDS 或填入虛構的資源數字。

CMake 分成 robots、random、core、LiDAR、cameras、ROS、course、evidence 函式庫。
ROS sensor test 與七台 command probe 由 CTest 各自啟動獨立進程，保留 namespace、非法指令、恢復及 deadman 檢查，
移除測試內自行 `posix_spawn`、pipe 和 JSON 加總的流程；domain 89 由 CTest 資源鎖串行使用。

## 實際刪除及等價性

- `PatternRay` 不再永久保存只在解析時計算方向所需的角度與原始 sample ID；每筆由 32 降至 12 bytes。
  800,000 條方向逐位元相同，方向快取由 25,600,000 降至 9,600,000 bytes，未減少射線或抽點。
  [方向與快取證據](../output/code-ablation-20261013/sensor-cache/summary.json)。
- 移除未使用的 camera config／LiDAR backend getters 和重複 metadata 常數；CUDA 私有型別移到內部 header。
- diffdrive 沒有 outer tracker，原 native／tracked 對照的模型、每步資料及 metadata 相同。
  預設核心矩陣由 30 減為 29 案；明確選擇 diffdrive tracked 單模式仍保留 1 案。
  [案例裁剪證據](../output/architecture-ablation-20261013/core-case-pruning.json)。

核心先做機械拆分 A，再做配置讀取／記憶體配置相關整理 B。原版本→A 與 A→B 各 30 案，完整編譯模型、
每個物理 tick 的狀態／控制／感測值及 metadata 都逐位元相同，沒有以容差掩蓋差異。
這是架構整理快照所列短程情境的等價性證據，不代表所有可能輸入的數學證明；
不適用於後續腳輪命名修正的整份 MJB／名稱 metadata 位元比較。
[核心三階段對照](../output/core-refactor-20261013/stage-comparison.json)。

rolling 和 snapshot 的原生 runtime 對照也保留模型、trajectory、IMU、點雲、點序、offset、line、採集位姿及 metadata 的檔案位元一致；
只排除 PID、輸出位置、耗時和資源等每次本來就不同的 summary 欄位。
[輸出對照](../output/code-ablation-20261013/sensor-cache/runtime-artifact-comparison.json)。

## 測試有沒有失去攔錯能力

以現有 Release `-O3` 編譯選項和不變的 ROS 測試物件，在獨立輸出目錄建立變異版本，實際經 DDS 收發；
正式源碼、函式庫和主程式 SHA 前後相同。使用 domain 91、每次 30 秒 timeout，沒有 GPU 或物理仿真。

| 版本 | 觀測結果 |
| --- | --- |
| 正常 sensor oracle | 3099 條斷言通過 |
| 只去掉 IMU binary32 儲存的 `volatile`，保留 float cast | GCC 11.4 Release 在第 26 條 `raw IMU units/float32 wire precision` 失敗 |
| PC2 byte 18 的絕對奈秒時間改為 `% 1e9` | 第 25 條 `absolute per-point timestamp crosses second` 失敗 |

因此保留實際 float32 儲存、獨立預期位元與跨秒時間斷言；「看似多餘」不能只以行數判斷。
本次兩個注入實驗只證明這兩類缺陷仍會被攔截，不代表所有測試都已完成消融。
[結果與 binary SHA](../output/architecture-ablation-20261013/mutants/run-20261013T164347817183Z/results.json)、
[可重現腳本](../output/architecture-ablation-20261013/mutants/run_ros_mutants.py)保留完整命令及 stdout／stderr。

## 架構整理快照的驗收與問題發現

[CTest](../output/architecture-ablation-20261013/ctest-components.log) 10/10 通過；
[Python 全套](../output/architecture-ablation-20261013/python-regression.log) 385 tests＋65 subtests 通過，無 skip。
正式完整路線要求原有 route、速度、ROS payload／頻率／pose、實時 RTF ≥0.95、實際接觸摩擦及停車全部成立。
每案使用 200 Hz IMU（含 t=0）、10 Hz ×20,000 原始射線 rolling LiDAR、10 Hz 雙相機，以及實際 DDS 發送／接收。

| 機器人 | 場景／指令速度 | 控制模式 | 正式結果 |
| --- | --- | --- | --- |
| Go2 | standard 8° ramp／0.3 m/s | tracked | 31/31 |
| Go2W | moderate 12 cm stairs／0.6 m/s | tracked | 31/31 |
| Scout Mini | moderate 5 cm hills／0.6 m/s | native | 31/31 |
| FR-mini | standard 8° ramp／0.6 m/s | native | 31/31 |
| Ridgeback | standard 8° ramp／0.6 m/s | native | 31/31 |
| Ranger Mini | standard 8° ramp／0.6 m/s | native | 31/31 |
| diffdrive | standard flat／0.3 m/s | native | 26/31，未正式通關 |

[逐案 E2E 證據](../output/architecture-ablation-20261013/e2e-ablation.json)。diffdrive 的球形支撐腳輪與 base body 共用，
原有 torso-contact 規則把它的正常接地也算成 `torso_environment_contact`；新舊版本都在約 2.06 s 被同一規則攔下。
由於未走完全程，速度窗口、終點與終點後 deadman 也未通過；這不是本輪新增的跌倒或傳輸失敗。
此階段尚未修接觸分類，也沒有放寬門檻。[舊版反證](../output/architecture-ablation-20261013/baseline-diffdrive-route/report.json)、
[優化快照結果](../output/architecture-ablation-20261013/e2e-diffdrive/report.json)均保留。
另用真實 CLI 與 DDS 驗證其動作、完整感測器、時間、外參及超時停車，
[20/20 通過](../output/architecture-ablation-20261013/diffdrive-cli-e2e.json)，與正式路線結果分開記錄。

該優化快照的感測器回歸有 16/16 相機 fixtures：32 個深度陣列逐位元一致，32 個 RGB 中 26 個完全一致，其餘最大差 1 灰階，
有限值／背景遮罩一致。四場景各 800,000 條、合計 320 萬條 CUDA 射線對照獨立 CPU，XYZ 最大誤差 0。
FR-mini GUI 實際運行 full＋CUDA 1.2 s：241 筆 IMU、12 幀 LiDAR、各相機 12 幀，RTF 0.999955733。
這些是感測器及視窗正確性證據；短程 fixture 不用來推論性能。
[最終感測器與 GUI 摘要](../output/architecture-ablation-20261013/sensor-regression-final-summary.json)。

## 腳輪誤判修正與最終驗收

問題已修到正式程式。diffdrive 兩個支撐球明確命名為 `front_caster_collision`、`rear_caster_collision`；
尺寸、位置、質量、摩擦及關節物理參數不變。C++ CourseGeometry 把滾動支撐接觸分類與 `wheel_safety` 分開，
原生分類使用既有 `RobotSpec.continuous_terrain` 能力，Python 使用既有 `WHEELED_ROBOTS`，不按機器人名字分支。
可選腳輪角色對所有純輪式模型解析；沒有腳輪的車型仍可使用實際輪組接觸分類。
只排除明確的輪組／腳輪接觸，保留車殼及其他碰撞；diffdrive 的 50° 傾角、0.1 m 高度和速度門檻不變。
Python 的 `wheel_geometry.py`、`full_run_worker.py`、`evaluate_terrains.py` 同類路徑同步修正。
未新增通用框架或能力旗標，也不忽略所有 base 球體。

命名會改變編譯模型的名稱表，不能沿用「整份 MJB 逐位元一致」的舊結論。
已比較 456 個非名稱／儲存布局的模型陣列和 29 個物理 option 欄位，全數一致；11 s 站立、直行、弧線、倒車、
原地轉向及停止，共 2751 筆初始與逐步狀態，qpos／qvel／ctrl／感測值和運行 metadata 位元一致。
[物理對照](../output/diffdrive-contact-fix-20261013/physics-parity.json)。

原生邊界測試 87 條斷言通過：兩顆指定腳輪實際接地不再判成 torso；高度 0.14 m、roll 45° 的車殼真實碰地仍判失敗，
即使高度／傾角本身合格。另以新編譯測試 XML 加入同半徑、同摩擦的普通 `body_sphere`，其碰地也必須失敗，
證明沒有略過所有球體。[接觸反例](../output/diffdrive-contact-fix-20261013/formal-boundaries-generic/results.json)。
只缺一個角色、錯誤形狀、錯誤所屬 body 或停用碰撞遮罩都拒絕初始化；兩個可選角色都不存在的輪式模型正常工作。
局部驗證詳見[修正摘要的 generic_followup](../output/diffdrive-contact-fix-20261013/summary.json)，完整回歸則以下方最終結果為準。

最終修正版 `simple-rbt-native` SHA256：
`a9c970f8953209c5abeab1a58fb84891d54aae0c8752d70750fabb4e02e8e1dd`。
驗收前後 145 份源碼／配置及 binary 均未改變；源碼集合雜湊為
`36cad6fb606823e800ce828325828f3812aa8ad075b8bbda059a1b23fdc29f00`。

| 最終驗收 | 實際結果 |
| --- | --- |
| 七台完整 ROS 路線 | 上方七組相同車型／地形／速度／模式全部重跑，各 31/31 通過；diffdrive 也正式通關 |
| CTest | 10/10 通過 |
| Python | 392 tests＋65 subtests 通過，0 skip；render／Warp／CUDA／live ROS 四個測試旗標全啟用 |
| 實時性 | 七案 RTF 0.999833803–0.999997904，維持完整感測器頻率與原 ≥0.95 門檻 |

[最終摘要](../output/diffdrive-contact-fix-20261013/generic-final/final-validation-summary.json)、
[七台逐案 E2E](../output/diffdrive-contact-fix-20261013/generic-final/final-e2e.json)、
[全套測試與旗標](../output/diffdrive-contact-fix-20261013/generic-final/final-tests.json)。
修正版可執行檔仍為 2,061,448 bytes，與架構整理快照相同；以下 RSS／CPU 表仍是修正前的交錯 A/B 量測，未用最終結果替換測量版本。

## 完整 ROS 負載的資源 A/B

以下量測在腳輪分類修正之前完成，對應上方兩個原生 binary SHA；不是修正版的重新性能量測。
三台車各跑三組交錯 before／after，共 18 案，全部 31/31 通過。每次是新的原生程序，場景及完整感測器配置相同，
按實時時鐘節奏運行；獨立 checker 不計入 worker CPU／RSS。RSS 是整個程序的峰值，包含載入與初始化，表中列三次均值。
CPU 100% 代表一個邏輯核心；此處並非無等待的最高吞吐速度。

| 機器人 | 峰值 RSS 均值：前→後（MiB） | 減少（MiB） | 平均 CPU：前→後（單核心 %） |
| --- | ---: | ---: | ---: |
| FR-mini | 238.5534 → 225.5495 | 13.0039 | 9.591 → 9.477 |
| Ridgeback | 235.6276 → 226.9128 | 8.7148 | 11.888 → 11.857 |
| Ranger Mini | 238.8828 → 225.6901 | 13.1927 | 9.684 → 9.576 |

這 18 案最低 RTF 為 0.999996316。CPU 變化約 0.03–0.11 個單核心百分點，試次範圍相互重疊，不能宣稱明確 CPU 加速。
目前可支持的是這三台、這些完整負載設定下的 RSS 減少，以及未失去實時能力；不推廣為所有車型的性能保證。
[資源比較與每次範圍](../output/architecture-ablation-20261013/resource-comparison.json)。
原生可執行檔由 1,913,240 增至 2,061,448 bytes（約增加 145 KiB）；模型／權重重用，沒有複製。
RAM 減少不等於磁碟減少。[檔案大小證據](../output/architecture-ablation-20261013/disk-comparison.json)。

## 依賴和擴展實驗

在乾淨建置狀態只暫改 `cameras.cpp` 的 mtime，GNU Make dry-run 只增加 camera 編譯與 archive recipe，沒有 LiDAR／CUDA 編譯。
乾跑不更新 archive 時間，因此沒有觀測到 camera oracle 的下游 relink；該依賴由生成規則確認。
這不是實際重編譯成功或編譯加速的證據。原 mtime 已還原，54 個建置產物的時間／大小不變。

既有 ELF 的實際 `ldd` 顯示：random oracle 不連 MuJoCo／ROS／GPU，course oracle 不連 DDS／圖形／CUDA，
camera oracle 不連 CUDA，LiDAR oracle 不連 EGL／GLFW。這不涵蓋未來路徑的動態 `dlopen`，完整程式建置仍需已列出的 SDK。
[依賴隔離證據](../output/architecture-ablation-20261013/dependency-isolation.json)。

另在 ignored scratch 建立 `fr_probe`，只加入 RobotSpec 及 model／controller catalog 登記，沿用同一 FR 驅動和關節／感測器 schema；
既有 core、控制器、main、runtime、ROS、course 物件未重新編譯，只有登記物件編譯及私有程式重新連結。
native／tracked 各 601 個狀態取樣（含初始狀態）模型和軌跡位元一致；metadata 僅允許機器人身份變化。
flat、ramp、hills 三組 CLI 的模型／軌跡／IMU 檔案也一致，實際 `/fr_probe/cmd_vel` DDS probe 41 條斷言通過。
正式版本仍只有七台。[擴展實驗](../output/architecture-ablation-20261013/extension-probe/summary.json)。

這證明相同控制律與 schema 的模型可以只登記；新控制律仍需新增實作／factory，其他關節結構仍需適配。
模組拆分不能保證修改永不影響其他功能，也不能取代整合驗收。

## 後續開發和版本收尾

日常先編譯／測試受影響模組，例如：

```bash
# 倉庫根目錄，先載入建置所需的 Humble／Livox 環境
cmake --build mujoco_sim_cpp/build --target full_run_oracle
ctest --test-dir mujoco_sim_cpp/build -L course --output-on-failure
ctest --test-dir mujoco_sim_cpp/build -L ros --output-on-failure

# 控制器修改可先只對照相關車型；輸出使用新目錄
.venv/bin/python mujoco_sim_cpp/scripts/validate_core.py \
  --binary mujoco_sim_cpp/build/core_oracle --robots fr_mini \
  --output output/core-fr-new

# 控制／感測／傳輸邊界變更後，以同次真實程序驗證完整鏈路
.venv/bin/python mujoco_sim/scripts/validate_full_run.py --runtime cpp \
  --robot fr_mini --terrain ramp --terrain-level standard --control-mode native \
  --speed .6 --camera-hz 10 --output output/full-fr-new
```

跨模組介面變動需要對應的真實 E2E；改動範圍大時再跑整套回歸。保留有攔錯證據的位元、時序、world-Y 包絡與命令隔離測試，
避免靠刪斷言或縮小感測器負載換取通過。此次整理沒有尋求或聲稱數學上的全局最優。

版本安排為：`archive_gazebo` 保存原 `main`（`64385758c95ba729b41faa0a9a7c72657f112c49`），
`main` 使用完成最終驗收的原生修正版；實際發布狀態以遠端 ref 為準。
架構整理提交 `427649f` 的 A/B 證據保持其原版本身份，不以後續修正的結果覆蓋。
