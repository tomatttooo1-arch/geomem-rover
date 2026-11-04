"""脱敏工具：规则表（正则 → 替换）放在提交包之外，由调用方用 --rules 传入，
因此本脚本里不含任何需要脱敏的字面量。规则按顺序应用（先长路径、后短名字）。"""
import json, re

class Sanitizer:
    def __init__(self, rules_path):
        self.rules = [(re.compile(p), r) for p, r in json.load(open(rules_path, encoding="utf-8"))["regex"]]
    def __call__(self, s):
        if not s:
            return s
        for p, r in self.rules:
            s = p.sub(r, s)
        return s
