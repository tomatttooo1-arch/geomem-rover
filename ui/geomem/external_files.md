# 不入库的原包文件（超过仓库 1 MiB 准入上限）

仓库准入规则（`tools/admin/workspace.py`：单文件 ≤ 1 MiB，例外须由管理员写进公共 `policy.json`）拒收下面三个文件。
另存，仓里只留本清单。原包整个 zip 也一并存档，便于复核来源。


| 仓内路径 | 字节 | sha256 | 取回 |
|---|---|---|---|
| ~~`app/vendor/three.core.js`~~（**已作废**，见文末） | 1 403 455 | `eb077d2417f61d3e6d9264c317cabc4ea35769ed6b0ab533067292a550784c20` | 存档目录同名文件；或官方 https://cdn.jsdelivr.net/npm/three@0.180.0/build/three.core.js （10-15 验收核过逐字节相同） |
| `界面预览.png` | 1 083 198 | `f8d76499af77eb24add4682d111ffc28677a098f783349ab4b548bb7d1c8a47f` | 存档目录同名文件（只是静态预览图，不影响运行） |
| `runtime/node.exe` | 92 825 416 | `3602f2bb1a10f2cbab4c36886218a33c1ab3db87290e73b033c46c77147d0237` | 见 `runtime/README.md`（官方下载地址） |
| （原包）`GeoMem-M8-Windows-x64-20261015.zip` | 37 340 986 | `efccc0beb06a77f1d9c9b5816b42996792196c12a79981a5c3ae5d4a16d0bbca` | 存档目录 |

在服务器上补齐（缺 `three.core.js` 时页面三维视图起不来）：

```bash
cp "$A/three.core.js" ui/geomem/app/vendor/ && cp "$A/界面预览.png" ui/geomem/
sha256sum ui/geomem/app/vendor/three.core.js ui/geomem/界面预览.png   # 对上表
```

后续改动计划（`docs/PROTOCOL_V2_DESIGN.md` 第 4 节）把 three.js 换成官方压缩版（`three.core.min.js` 等，单个 < 1 MiB）入库，
届时本表的第一行作废。

**10-15 更新（阶段 1 前端）**：已按上句换成 three@0.180.0 官方压缩版入库，界面不再需要 `three.core.js`：

| 仓内路径 | 字节 | sha256 | 来源核对 |
|---|---|---|---|
| `app/vendor/three.core.min.js` | 381 124 | `61ba0df005b05991361d040d8ff670e1aadfd0ce7aeebd1fdb0725957a8957de` | npm 包 `three-0.180.0.tgz`（sha512 integrity 与 registry 元数据一致）内 `package/build/` 原文件；与 cdn.jsdelivr.net/npm/three@0.180.0/build/ 同名文件逐字节一致 |
| `app/vendor/three.module.min.js` | 338 908 | `e2b5ee6bccd38fd6d8a2428546b83c5f2426d84b152ef82be8055556e3b40eb6` | 同上 |

`app/vendor/three.module.js`（未压缩版，603 113 B）已删除；`orbitcontrols.js` 与 npm 包内 `examples/jsm/controls/orbitcontrols.js` 逐字节一致，未改。
`界面预览.png` 仍不入库（外存已有）。

