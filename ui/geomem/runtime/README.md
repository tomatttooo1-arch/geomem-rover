# runtime/ —— 便携包自带的 Node.js 运行环境（不入库）

它是官方原样二进制，不入 git（见 `../.gitignore`）；仓里只留许可证 `NODE-LICENSE.txt` 和本说明。

## 取回方法

| 项 | 值 |
|---|---|
| 版本 | v24.19.0，win-x64 |
| 下载地址 | https://nodejs.org/dist/v24.19.0/win-x64/node.exe |
| 官方哈希清单 | https://nodejs.org/dist/v24.19.0/SHASUMS256.txt （其中 `win-x64/node.exe` 一行） |
| sha256 | `3602F2BB1A10F2CBAB4C36886218A33C1AB3DB87290E73B033C46C77147D0237` |

包内文件与上表一致（10-15 验收时核过：包里的 `package-info.json` 记的哈希、实际文件哈希、官方清单三者相同）。

Windows 上核对：

```powershell
Invoke-WebRequest https://nodejs.org/dist/v24.19.0/win-x64/node.exe -OutFile runtime\node.exe
(Get-FileHash runtime\node.exe -Algorithm SHA256).Hash   # 必须等于上表
```

Linux 上核对：`sha256sum runtime/node.exe`（小写同值）。哈希不符就删掉，不要运行。


重新打便携 zip 时（例如给评委/队友一个双击即用的包）：

1. 从干净的 git 树导出 `ui/geomem/`（`git archive`），不带工作区里的临时文件。
2. 下载 `node.exe` 到 `runtime/`，按上表核 sha256，不符即中止。
3. 重新生成 `package-info.json`：逐个文件算 sha256（路径用 `\` 分隔，与原包一致），`build` 写打包日期。
4. 打 zip 时文件名用 UTF-8 并置 zip 的 UTF-8 标志位（原包四个中文文件名里三个是 GBK、一个是 UTF-8 未置标志，
   Linux 上 `unzip` 会出乱码；本次入库是按 `package-info.json` 的路径与 sha256 对回原名的）。
5. 打完在一个空目录解压，逐个文件对 `package-info.json` 复核一遍。

`start_demo.cmd` 找不到 `runtime\node.exe` 时会提示「Runtime missing」并退出，不会去联网下载。
