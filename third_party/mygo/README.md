# MyGo source and local patch / MyGo 源码与本地补丁

`manifest.json` pins the upstream archive and Go toolchain. `tools/setup-gui.py`
verifies the archive, patch, and affected source hashes before building. Applying
an already verified patch is a no-op; an unexpected source edit stops preparation.

`manifest.json` 锁定上游源码和 Go 工具链。`tools/setup-gui.py` 在构建前验证归档、
补丁及受影响源码的 SHA-256；已应用的补丁不重复执行，源码异常改动会中止准备。

`pango-lcd-text-color.patch` fixes normal Linux text being mistaken for color
emoji when desktop font rendering uses LCD antialiasing. A Pango font face can
retain RGB coverage in an ARGB32 surface even when its Cairo scaled font reports
grayscale antialiasing. Monochrome probes now use A8 masks; Fontconfig color fonts
and actual LCD rendering retain ARGB32. The requested text color is preserved
without disabling GPU rendering or changing desktop settings.

`pango-lcd-text-color.patch` 修复 Linux 的 LCD 抗锯齿将普通文字误判为彩色 emoji 的问题。
即使 Cairo 缩放字体报告灰度抗锯齿，Pango 字体仍可能在 ARGB32 中产生 RGB 覆盖率。
普通文字的灰度探测改用 A8 掩码；Fontconfig 标记的彩色字体和实际 LCD 绘制仍使用
ARGB32。文字按指定颜色显示，无需关闭 GPU 渲染或修改桌面设置。

Regression checks / 回归检查（使用准备脚本提供的 Go 工具链和缓存环境）：

```bash
go test -C .vendor/mygo ./internal/text -run '^TestPango'
```

The patch includes gray/LCD text tests on opaque and transparent backgrounds,
and preservation of real color emoji when a color emoji font is installed.

补丁包含灰度与 LCD 文字在不透明、透明背景上的回归测试；安装彩色 emoji 字体时，
同时验证其原始颜色得到保留。
