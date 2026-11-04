#!/bin/bash
# Step 0: direct SDK probe after the camera was moved. s0_probe.sh OUT [extra probe args]
# tools/gemini2_probe) and the SDK under ~/fpga/orbbec/OrbbecSDK_v2.9.3_*_linux_arm64.
cd ~/fpga/camprobe/build; export LD_LIBRARY_PATH=$(ls -d ~/fpga/orbbec/OrbbecSDK_v2.9.3_*_linux_arm64/lib)
O=$1; shift; mkdir -p $O
n=$(ls $O | grep -c '\.json$'); i=$((n+1))
timeout 60 ./fpga_cam_probe --depth 640x400@30:Y16 --color 640x480@30:RGB --seconds 6 "$@" > $O/p$i.json 2> $O/p$i.err; echo "p$i rc=$? args=$*"
grep -a -o -E '"laser": [-0-9]+|"ldp": [0-9]+|"ldp_triggered": [0-9]+|"connection": "[^"]+"|"depth": \{[^}]+\}|"color": \{[^}]+\}' $O/p$i.json | tr '\n' ' '; echo
grep -a DEPTH_VALUES $O/p$i.json; grep -a -c 'does not match' $O/p$i.json $O/p$i.err
