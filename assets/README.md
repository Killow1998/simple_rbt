# 执行资产

此目录是 C++ 模拟器与 Python 对照共用的数据；执行 C++ 不需要 Python 对照代码。

| 路径 | 用途 |
| --- | --- |
| `catalog.json` | 模型、控制器参数、地形与文件哈希 |
| `models/` | 离线汇出的 MuJoCo 模型 XML |
| `policies/` | Go2／Go2W 的 ONNX 策略权重 |
| `manifests/` | 原模型／策略来源、物理参数、版本与授权纪录 |
| `visual/` | 官方 CAD 转换、文件哈希与只影响外观的安装变换 |
| `lidar/` | 原样保存的 MID360 扫描 CSV 与来源纪录 |

模型网格及 legacy Go2 策略仍按 manifest 放在 `.vendor/`，由 `./setup.sh` 准备。

扫描数据的原位置与 SHA 见 [lidar/source.json](lidar/source.json)。原数据没有明确的本地 LICENSE，没有替它新增授权声明。
现有模型／权重的各自授权仍以 manifests 为准。

Ranger Mini V3、普通四轮 Scout Mini、FR-mini 和 Ridgeback 的外观使用官方 URDF／CAD 资源。
外观网格挂在现有机身／关节上，质量与碰撞掩码为零；原碰撞体、惯量、执行器和控制参数保留。
`visual/*/provenance.json` 记录来源 commit、原始及转换文件哈希、转换参数和与物理近似的差异。
`tools/prepare_visual_assets.py` 核验已有资源，缺失时从固定来源离线准备；
`tools/export_models.py` 汇出时通过 `tools/visual_models.py` 应用同一外观变换。

Official visual CAD is separate from validated dynamics. Preparation scripts verify source and
derived hashes; runtime C++ loads the exported model directly and never invokes these scripts.
