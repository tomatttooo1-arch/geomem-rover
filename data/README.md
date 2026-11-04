# data/ —— 数据的指针，不是数据本身

本目录**内容整体被 .gitignore 忽略**（`/data/**` 根锚定）。真正的数据集与录制流放在
`${FPGA_SHARED}/data/`，git 里只留两样东西：

1. 一句指针：哪份数据、在 shared 下的哪个目录、怎么来的、谁录的、什么时候；
2. 用 `tools/manifest.sh <目录>` 生成的 `MANIFEST.sha256`（文本，会被反向豁免规则放行）。

清单请放在本目录**第一层**：git 不会走进被忽略的子目录，放深了救不回来。
# 本轮可直接使用的数据（2026-09-07）

完整 TUM 官方桌面序列：
下载来源与压缩包哈希见上一级 `SOURCE.json`。原始文件不改。
原包 `desk-complete.tgz` 的 SHA256：`e983d6830916e66dc4a46a71368046b149b283de87769690e7aa4e0b9483530c`。
官方发布地址：<https://webshare.cvg.cit.tum.de/g/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk.tgz>。
本轮使用每隔 15 帧抽样的前 30 个有效 RGB-D 配对；具体帧与每张图哈希见 `runs/README.md` 指向的 `tum-v2/input.jsonl`。
这是公开真实数据，位姿为数据集 mocap 真值插值，不是本队 SLAM 输出。未采集本队硬件数据。
正常代理返回 503 后改为获授权的直连；下载残片与分段文件保留，只有 `desk-complete.tgz` 是完整包。
