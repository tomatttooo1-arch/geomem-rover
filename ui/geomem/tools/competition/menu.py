#!/usr/bin/env python3
"""Interactive laptop entry; never starts a car just by opening the menu."""
import json
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
config = HERE / "session.json"
if not config.exists():
    shutil.copyfile(HERE / "session.example.json", config)
    print("已生成现场配置：" + str(config))
    print("请先填写 Nano 地址、两板部署路径；示例值不能用于实车。")
    if sys.platform == "win32":
        subprocess.Popen(["notepad.exe", str(config)])
    sys.exit(0)
items = [
    ("start", "一键启动或接续实车（电脑直接连接 Nano）"),
    ("check", "检查直连 SSH、两板和 KV 启动权限"),
    ("kv-up", "起 KV260 服务（车栈未运行时）"),
    ("mapping", "第一趟：起车栈、取口令、起桥并打开控制台"),
    ("down", "收车（先在界面确认建图/任务结果）"),
    ("collect", "复制第一趟数据库和记忆"),
    ("basemap", "笔记本 Docker 生成 1 cm 底图"),
    ("demo", "第二趟：带地图和记忆起栈并打开控制台"),
    ("bridge", "已有车栈：取本趟口令并连接"),
]
while True:
    print("\nGeoMem 比赛现场入口（执行所选操作；0 退出）")
    for i, (_, label) in enumerate(items, 1):
        print("%d. %s" % (i, label))
    answer = input("选择：").strip()
    if answer == "0":
        break
    if not answer.isdigit() or not 1 <= int(answer) <= len(items):
        continue
    c = json.loads(config.read_text(encoding="utf-8-sig"))
    if c.get("nano_ip") == "198.18.1.100":
        print("仍是示例 IP；请先编辑 " + str(config))
        continue
    action = items[int(answer) - 1][0]
    try:
        subprocess.run([sys.executable, str(HERE / "session.py"), action, "--config", str(config), "--execute"], check=True)
    except KeyboardInterrupt:
        print("本机命令被中断；请核对车辆状态。退出桥不会自动收车。")
    except subprocess.CalledProcessError as exc:
        print("操作没有完成，退出码 %d；请按上方原因处理。" % exc.returncode)
