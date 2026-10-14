# 执行资产

此目录是 C++ 模拟器与 Python 对照共用的数据；执行 C++ 不需要 Python 对照代码。

| 路径 | 用途 |
| --- | --- |
| `catalog.json` | 模型、控制器参数、地形与文件哈希 |
| `models/` | 离线汇出的 MuJoCo 模型 XML |
| `policies/` | Go2／Go2W 的 ONNX 策略权重 |
| `manifests/` | 原模型／策略来源、物理参数、版本与授权纪录 |
| `lidar/` | 原样保存的 MID360 扫描 CSV 与来源纪录 |

模型网格及 legacy Go2 策略仍按 manifest 放在 `.vendor/`，由 `./setup.sh` 准备。

扫描数据的原位置与 SHA 见 [lidar/source.json](lidar/source.json)。原数据没有明确的本地 LICENSE，没有替它新增授权声明。
现有模型／权重的各自授权仍以 manifests 为准。
