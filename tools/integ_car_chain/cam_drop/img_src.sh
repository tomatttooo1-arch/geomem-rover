#!/bin/bash
# Copy the driver's source (already in the image) out for reading. No camera, no network.
IMG=fpga-ros:humble-nav-orbbec-scipy-20261017
D=$HOME/fpga/dev1_camdrop/drvsrc
mkdir -p "$D"
docker run --rm --name fpga-dev1-inspect --network none --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$D":/out --entrypoint bash $IMG -c '
cd /opt/orbbec_ws/src/orbbec_camera
tar cf /out/orbbec_camera_src.tar --exclude=SDK --exclude="*.so*" --exclude="*.bag" --exclude="*.png" --exclude="*.jpg" src include launch CMakeLists.txt package.xml 2>/dev/null
ls -la /out; du -sh /opt/orbbec_ws/src/orbbec_camera/* | head -30
head -5 /opt/orbbec_ws/src/orbbec_camera/package.xml; grep -m1 "<version>" /opt/orbbec_ws/src/orbbec_camera/package.xml
'
