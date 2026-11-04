# 80×80规划建场 HLS


- `frame_based/`：纯C参考包装及对照入口。
- `rearchitect/v1/src/`：9位entry/walk入图、24位势场、共享正反扫描II1、两个AXI master的实际核和接口头；`tb/`逐位testbench。
- `rearchitect/v1/run_hls.tcl`：2022.2工程模板，part xck26-sfvc784-2LV-c、10ns。
- `../../tools/planner_model/hls/`：黄金/冻结向量生成、边界、单槽与分批RTL工具。

原始产物和可复现输入保存在：






历史基线：单核OOC实际4411LUT/4829FF/2DSP/7RAMB36+4RAMB18；100MHz WNS3.982ns/TNS0。RAM共9个BRAM36等容量，比建议8多1；441条OOC外部I/O延迟警告保留至整板集成。未执行板上驱动/cache/DDR/整机验证，不据此声明CPU或系统加速比。


本版实际3509LUT/3538FF/1DSP/7RAMB36+1RAMB18=7.5等容量，URAM0；100MHz WNS2.114ns/TNS0、WHS0.047ns。唯一两组C/RTL均38552字节逐位一致，16轮/20轮分别239545/295928拍。唯一OOC在900秒止于功耗报告，布线已完成；最终资源和时序只读同一routed DCP取得，没有重做实现。HLS export_impl汇总缺失如实保留，勿将QoR CSV的空值/估计回退误当实际值。

