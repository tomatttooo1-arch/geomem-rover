# A53BENCH0 board results (camkv1_1104)

Times are wall-clock per unit of the software reference call only (CLOCK_MONOTONIC), hashing and input staging excluded. Single = 1 instance pinned to one core; par4 = 4 instances on cores 0-3 started together.

| item | what | unit | single median ms | single p99 ms | single units/s (1/mean) | par4 units/s | par4 / single | bit-exact | FIFO |
|---|---|---|---|---|---|---|---|---|---|
| geo_fx | geometry, fixed-point (final PL algorithm) | frame (rec30 640x360) | 56.140 | 107.312 | 16.9 | 69.6 | 4.11 | yes/yes | yes |
| geo_ref_d1 | geometry, old float reference, Dlat2 on | frame | 79.605 | 130.879 | 12.0 | — | — | yes | yes |
| geo_ref_d0 | geometry, old float reference, Dlat2 off (297.6 ms era) | frame | 185.331 | 237.473 | 5.1 | — | — | yes | yes |
| fuse_r10 | rolling-map fusion (c3b core source) | record (cosim_r10) | 0.362 | 1.484 | 3006.8 | 4859.5 | 1.62 | yes/yes | yes |
| grid_formal | same core, formal trace incl. GRID cost map | record (grid_formal) | 0.470 | 1.337 | 1200.2 | 3234.6 | 2.69 | yes/yes | yes |
| roi19 | depth evidence (ROI batch) | frame (19 boxes) | 70.289 | 121.353 | 13.5 | 54.8 | 4.06 | yes/yes | yes |
| plan_key2 | planner field, 2 key cases | request (K_max 64) | 9.857 | 12.936 | 101.2 | 381.6 | 3.77 | yes/yes | yes |
| plan_213 | planner field, 213-case set | request | 2.487 | 12.868 | 445.8 | 1450.6 | 3.25 | yes/yes | yes |
| prep69 | DPU pre-processing (stream model + YUYV ref) | frame (mixed configs) | 16.132 | 70.655 | 62.6 | 120.6 | 1.93 | yes/yes | yes |
| post_ref | detection post-processing, detpost_ref | vector | 1.234 | 2.368 | 815.5 | 3014.9 | 3.70 | yes/yes | yes |
| post_fast | same algorithm, NEON scan | vector | 0.327 | 1.683 | 2261.9 | 7173.8 | 3.17 | yes/yes | yes |
| post_vai | Vitis AI yolov8 port (float, not bit-exact) | frame (model) | 52.564 | 103.670 | 18.2 | — | — | n/a | yes |

## Groups (single-core run)

| item | group | n | median ms | p99 ms | mean ms |
|---|---|---|---|---|---|
| fuse_r10 | dump | 120 | 0.051 | 0.059 | 0.043 |
| fuse_r10 | integrate | 1080 | 0.414 | 1.484 | 0.427 |
| fuse_r10 | op7 | 20 | 0.000 | 0.000 | 0.000 |
| fuse_r10 | query | 160 | 0.008 | 0.030 | 0.008 |
| fuse_r10 | reset | 80 | 0.232 | 0.470 | 0.224 |
| grid_formal | dump | 210 | 0.051 | 0.059 | 0.050 |
| grid_formal | grid | 3750 | 1.307 | 1.342 | 1.172 |
| grid_formal | integrate | 1985 | 0.351 | 0.770 | 0.377 |
| grid_formal | op7 | 5 | 0.001 | 0.001 | 0.001 |
| grid_formal | query | 220 | 0.008 | 0.030 | 0.008 |
| grid_formal | reset | 25 | 0.232 | 0.422 | 0.223 |
| plan_key2 | r16 | 10 | 12.895 | 12.936 | 12.887 |
| plan_key2 | r20 | 10 | 6.875 | 6.886 | 6.875 |
| plan_213 | r1 | 69 | 0.120 | 0.923 | 0.156 |
| plan_213 | r11 | 3 | 8.919 | 8.921 | 8.903 |
| plan_213 | r12 | 9 | 9.737 | 9.770 | 9.705 |
| plan_213 | r13 | 9 | 10.578 | 10.644 | 10.532 |
| plan_213 | r14 | 3 | 11.360 | 11.426 | 11.382 |
| plan_213 | r15 | 3 | 12.191 | 12.210 | 12.193 |
| plan_213 | r16 | 6 | 12.893 | 12.976 | 12.897 |
| plan_213 | r17 | 3 | 13.745 | 13.752 | 13.742 |
| plan_213 | r2 | 84 | 1.517 | 2.043 | 1.479 |
| plan_213 | r20 | 3 | 6.872 | 6.875 | 6.872 |
| plan_213 | r3 | 105 | 0.476 | 0.501 | 0.471 |
| plan_213 | r4 | 294 | 2.585 | 2.856 | 2.610 |
| plan_213 | r5 | 18 | 2.984 | 2.990 | 2.983 |
| plan_213 | status2 | 6 | 0.004 | 0.004 | 0.004 |
| plan_213 | status3 | 12 | 0.004 | 0.004 | 0.004 |
| plan_213 | status6 | 6 | 0.004 | 0.004 | 0.004 |
| plan_213 | status7 | 6 | 1.545 | 2.069 | 1.545 |
| prep69 | bgr_640x480_to_416x416 | 3 | 16.073 | 16.132 | 16.092 |
| prep69 | bgr_640x480_to_512x512 | 3 | 20.145 | 20.217 | 20.160 |
| prep69 | rgb_1000x750_to_416x416 | 3 | 22.746 | 73.793 | 39.760 |
| prep69 | rgb_1000x750_to_512x512 | 3 | 27.242 | 27.253 | 27.238 |
| prep69 | rgb_1000x750_to_640x640 | 3 | 37.204 | 37.223 | 37.206 |
| prep69 | rgb_1280x720_to_416x416 | 3 | 22.536 | 22.539 | 22.529 |
| prep69 | rgb_1280x720_to_512x512 | 3 | 27.283 | 27.319 | 27.288 |
| prep69 | rgb_1280x720_to_640x640 | 3 | 32.655 | 32.725 | 32.675 |
| prep69 | rgb_480x640_to_416x416 | 3 | 15.637 | 15.658 | 15.644 |
| prep69 | rgb_480x640_to_512x512 | 3 | 19.657 | 19.667 | 19.654 |
| prep69 | rgb_480x640_to_640x640 | 3 | 3.199 | 3.213 | 3.202 |
| prep69 | rgb_640x480_to_416x416 | 30 | 15.648 | 62.638 | 17.212 |
| prep69 | rgb_640x480_to_512x384 | 30 | 19.630 | 70.677 | 21.334 |
| prep69 | rgb_640x480_to_512x512 | 36 | 19.657 | 70.655 | 21.076 |
| prep69 | rgb_640x480_to_640x480 | 30 | 3.287 | 3.362 | 3.292 |
| prep69 | rgb_640x480_to_640x640 | 30 | 3.301 | 3.386 | 3.304 |
| prep69 | uyvy_640x480_to_640x640 | 3 | 12.581 | 12.591 | 12.581 |
| prep69 | yuyv_640x480_to_416x416 | 3 | 25.288 | 25.292 | 25.282 |
| prep69 | yuyv_640x480_to_512x384 | 3 | 30.510 | 30.598 | 30.528 |
| prep69 | yuyv_640x480_to_512x512 | 6 | 28.511 | 28.956 | 28.500 |
| prep69 | yuyv_640x480_to_640x640 | 3 | 11.780 | 11.788 | 11.783 |
| post_ref | ka | 30 | 1.221 | 1.284 | 1.235 |
| post_ref | layout | 10 | 0.239 | 0.268 | 0.243 |
| post_ref | model | 120 | 1.235 | 1.324 | 1.242 |
| post_ref | random | 10 | 2.348 | 2.378 | 2.351 |
| post_ref | scene | 10 | 1.759 | 1.791 | 1.762 |
| post_ref | scene6 | 10 | 0.337 | 0.369 | 0.339 |
| post_fast | ka | 30 | 0.324 | 0.430 | 0.332 |
| post_fast | layout | 10 | 0.264 | 0.309 | 0.270 |
| post_fast | model | 120 | 0.328 | 0.414 | 0.336 |
| post_fast | random | 10 | 1.652 | 1.690 | 1.661 |
| post_fast | scene | 10 | 1.068 | 1.127 | 1.077 |
| post_fast | scene6 | 10 | 0.357 | 0.453 | 0.362 |
| post_vai | model | 100 | 52.564 | 103.670 | 55.065 |

## Per pass (single) and par4 detail

| item | passes | pass median ms | per-unit max ms | par4 wall s | par4 per-unit median ms | par4 per-unit p99 ms |
|---|---|---|---|---|---|---|
| geo_fx | 1 | 18291.9 | 107.599 | 17.81 | 56.347 | 60.663 |
| geo_ref_d1 | 1 | 25903.2 | 131.477 | — | — | — |
| geo_ref_d0 | 1 | 60337.2 | 238.145 | — | — | — |
| fuse_r10 | 20 | 23.9 | 7.332 | 1.20 | 0.384 | 1.511 |
| grid_formal | 5 | 1069.8 | 52.352 | 7.66 | 0.476 | 1.390 |
| roi19 | 1 | 22120.5 | 121.380 | 21.81 | 70.422 | 74.846 |
| plan_key2 | 10 | 19.8 | 12.936 | 0.21 | 10.647 | 13.030 |
| plan_213 | 3 | 477.8 | 13.752 | 1.76 | 2.491 | 12.914 |
| prep69 | 3 | 1134.1 | 73.793 | 6.86 | 16.363 | 36.830 |
| post_ref | 10 | 23.3 | 2.378 | 0.25 | 1.228 | 2.371 |
| post_fast | 10 | 8.4 | 1.690 | 0.11 | 0.367 | 1.690 |
| post_vai | 10 | 549.8 | 103.755 | — | — | — |


## Environment

```
date 2026-11-03T18:49:44+00:00
Linux 5.15.0-1027-xilinx-zynqmp aarch64
nproc 4
0.85 0.95 0.92 1/365 196647
cpu0 gov=userspace cur_khz=1333333 max_khz=1333333
cpu1 gov=userspace cur_khz=1333333 max_khz=1333333
cpu2 gov=userspace cur_khz=1333333 max_khz=1333333
cpu3 gov=userspace cur_khz=1333333 max_khz=1333333
/sys/class/thermal/thermal_zone*/temp 
sched_rt_runtime_us 950000
--- top cpu processes
    PID %CPU %MEM COMMAND
 196575  2.2  0.2 sshd
    921  1.0  0.2 polkitd
 196630  0.6  0.0 run_a53bench.sh
 195429  0.5  0.2 systemd
    907  0.2  0.1 dbus-daemon
 195443  0.2  0.1 user-session-he
      1  0.1  0.3 systemd
    813  0.1  0.0 systemd-oomd
    908  0.1  0.4 NetworkManager
  79721  0.1  2.9 gnome-shell
      2  0.0  0.0 kthreadd
--- free
               total        used        free      shared  buff/cache   available
Mem:            3911         444        1056           4        2410        3271
Swap:              0           0           0
4.32 2.90 1.73 1/365 197446
end cpu0 cur_khz=1333333
end cpu1 cur_khz=1333333
end cpu2 cur_khz=1333333
end cpu3 cur_khz=1333333
end /sys/class/thermal/thermal_zone*/temp
```
