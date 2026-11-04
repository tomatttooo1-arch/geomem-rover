

- 只保留原始D16码；证据值为 RN32(float(code)*unit)，不可换成观测的 code/1000。
- roi_fx_ps：旧状态优先级、精确单调二分cut、55位RNE比值中点。
- roi_fx.cpp：只读覆盖整行、全局坐标、8组私有4深BRAM、真实RAW距离4。
- 旧浮点镜像槽为空；其余旧offset固定；新增c00..d38。
- CPU全量规定集为原B1200及B2/rec30 310帧；AP C/RTL只两组。C边界只做固定单变量，不跑笛卡尔积。


