# models/ —— 模型权重的指针，不是权重本身

本目录**内容整体被 .gitignore 忽略**（`/models/**` 根锚定）。权重文件放在
`${FPGA_SHARED}/models/`，git 里只留指针 + `MANIFEST.sha256`。

每个模型请写清：来源（自己训的 / 官方发布 / 第三方）、许可、输入输出格式、
量化与编译状态（如是否已转成 .xmodel）、以及对应的评测数字出自哪次实验。

清单请放在本目录**第一层**（同 `data/`，git 不进被忽略的子目录）。
# 本轮软件参考权重（2026-09-07）

文件为其下 `checkpoints/ssdlite320_mobilenet_v3_large_coco-a79551df.pth`。
SHA256：`a79551df90c79834bcd3bb3845ef9d966b5449a3a9b2833ae8404778ca5d65d2`，已核对发布方文件名的哈希前缀。
来源：<https://download.pytorch.org/models/ssdlite320_mobilenet_v3_large_coco-a79551df.pth>。
COCO_V1 预训练原权重，没有训练或量化；执行实现为 Torchvision SSDLite320 MobileNetV3 Large、CPU、阈值 0.4。
不是旧草案的 MobileNetV2 300，也没有证据说明它支持本届 FPGA 工具链。
