# 執行資產

此目錄是 C++ 模擬器與 Python 對照共用的資料；執行 C++ 不需要 Python 對照程式碼。

| 路徑 | 用途 |
| --- | --- |
| `catalog.json` | 模型、控制器參數、地形與檔案雜湊 |
| `models/` | 離線匯出的 MuJoCo 模型 XML |
| `policies/` | Go2／Go2W 的 ONNX 策略權重 |
| `manifests/` | 原模型／策略來源、物理參數、版本與授權紀錄 |
| `lidar/` | 原樣保存的 MID360 掃描 CSV 與來源紀錄 |

模型網格及 legacy Go2 策略仍按 manifest 放在 `.vendor/`，由 `./setup.sh` 準備。
它們不來自已歸檔的 Gazebo `simple_rbt/model/`。

掃描資料的原位置與 SHA 見 [lidar/source.json](lidar/source.json)。原資料沒有明確的本地 LICENSE，沒有替它新增授權聲明。
現有模型／權重的各自授權仍以 manifests 為準。
