# Nano 联网稳定性（2026-10-13）

车载 Orin Nano 靠校园 WiFi 上网，服务器经反向隧道 `ssh nano` 连它，KV260 再经 Nano 的网口跳转。
10-12 晚到 10-13 凌晨 70 分钟内断了三次，每次十到二十分钟，正式趟的取数和收尾都没送到板上。

## 查到的原因

| 现象 | 证据（Nano `/var/log/syslog`） | 处理 |
|---|---|---|
| WiFi 显示连着，但网关不回 ARP | 三次断网期间隧道一直报 `No route to host`，`wpa_supplicant` 无断开记录，内核无报错；三次都在重新关联的那一秒恢复（00:02:22、00:22:36 各一次漫游） | 看门狗：网关二层不通 30 s 就重新关联 |
| WiFi 省电是开的 | `/etc/NetworkManager/conf.d/default-wifi-powersave-on.conf` 为 3；`iw get power_save` 为 on | 关掉（最可能的诱因，未单独证实） |
| 上电约一小时后 KV260 网线地址失效 | Nano 无时钟电池，开机后第一次对时把时钟往前跳（10-13 跳了约 11.5 h），dnsmasq 把一小时租约判为过期并删除，此后 KV260 每分钟续约都被拒 | KV260 租约改为永久 |
| WiFi 恢复后隧道还要再等两分多钟 | 断网期间发起的连接在重新关联后不会自己接上，要等内核约 130 s 的连接超时 | 隧道的 ssh 加 `ConnectTimeout=10` |
| 一趟测量中间时钟被回拨 270 ms | 系统镜像只配了 pool.ntp.org 的服务器，从这里往返 35–430 ms 来回跳，对时结果跟着跳 | 对时源换成 `time.edu.cn`、`ntp.tencent.com`（见下「对时源怎么选的」） |

系统日志本来就由 rsyslog 落盘（`/var/log/syslog`、`kern.log`），不需要再改 journald。

## 装了什么

`sudo bash install_nano_net.sh`（可重复执行；`--undo` 撤销）：

1. `/etc/NetworkManager/conf.d/zz-fpga-wifi-powersave-off.conf`：`wifi.powersave = 2`。
2. `fpga-wifi-watchdog.service` + `/usr/local/sbin/fpga-wifi-watchdog.sh`，日志 `/var/log/fpga-wifi-watchdog.log`。
3. `/etc/NetworkManager/dnsmasq-shared.d/kv260.conf` 末尾加 `,infinite`（原件备份在 `/root/fpga-nano-net-backup/`）。
4. `/etc/systemd/system/revtunnel-super-test.service.d/10-fpga-connect-timeout.conf`。
5. `/etc/systemd/timesyncd.conf.d/zz-fpga-ntp.conf`：`NTP=time.edu.cn ntp.tencent.com`（镜像自带的 pool 服务器仍留作后备）。

## 看门狗做什么

每 10 s 探一次：先 ping 远端（服务器、公网各一个），通就算好；不通再看网关在二层是否还活着。

| 状态 | 含义 | 动作 |
|---|---|---|
| OK | 远端可达 | 无 |
| DEAD | 有默认路由但网关不回 ARP | 30 s 重新关联；90 s 关开无线；210 s 重载驱动（30 分钟最多一次）；之后从头再来 |
| UPSTREAM | 网关活着但远端不通 | 只记录（现场离线演示的热点就是这个样子，不能打扰） |
| DOWN | 没有默认路由（NetworkManager 正在连） | 超过 2 分钟才催一次自动连接 |
| OFF | 无线被人关掉 | 无 |

绝不重启整机。日志每行带墙钟和开机秒数（时钟会跳），并记录换接入点、换地址。

## 验收（10-13 12:00，板上实测）

人为把网关的邻居表项改成假地址（与真实故障同一特征），150 s 后自动撤销作保险。

| | 改之前 | 改之后 |
|---|---|---|
| 看门狗发现 | — | 18 s |
| 重新关联 | —（昨晚靠漫游，10–20 min） | 50 s 时动作，64 s 恢复 |
| 服务器侧 `ssh nano` 恢复 | 约 3 min 40 s（第一次测，隧道无连接超时） | 75 s |
| 重连后省电 | — | 仍为 off |
| KV260 地址 | 一小时租约，被拒续 | `valid_lft forever` |

## 对时源怎么选的（10-13 14:12–15:01，板上实测）

在 Nano 上每 30 s 向四个服务器各问一次时间，和板上不受对时影响的原始时钟比（49 分钟，各约 95 次）。看的是「去掉固定快慢之后还剩多少晃动」：

| 服务器 | 往返中位 ms | 晃动（标准差 / 峰峰值 ms） | 相邻两次最大跳变 ms | 备注 |
|---|---|---|---|---|
| `time.edu.cn`（教育网） | 37.5 | 2.1 / 11.6 | 9.4 | 采用，排第一 |
| `ntp.tencent.com` | 37.7 | 2.1 / 14.7 | 9.4 | 采用，排第二 |
| `ntp.aliyun.com` | 24.9 | 取往返较短的一半 8.8 / 32.0 | — | 有一次应答晚了 2.4 s |


## 还没解决的

- 省电是不是真正的诱因没有单独证实；之后看 `/var/log/fpga-wifi-watchdog.log` 里还有没有 DEAD。
- 断电重启后服务器上旧隧道会话占住端口的问题不在这里（要在服务器普通终端跑 `jetson_tunnel_unstick.sh`）。
- KV260 的 DHCP 客户端收到拒绝后不重新申请（只会每分钟再续一次），原因没查；永久租约绕开了它。
