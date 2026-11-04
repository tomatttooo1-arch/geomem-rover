#!/usr/bin/env bash
# Run the ROS userland in an owner-owned rootfs; never use sudo or host Docker.
set -euo pipefail
if [[ $# -lt 3 ]]; then
    echo "Usage: bash tools/agx_rootfs.sh ROOTFS PROJECT COMMAND [ARG ...]" >&2
    echo "Optional FPGA_AGX_WORKSPACE is mounted at /workspace for IPC and run artifacts." >&2
    exit 2
fi
rootfs=$(realpath -e -- "$1")
project=$(realpath -e -- "$2")
shift 2
[[ "$rootfs" != / && -d "$rootfs/opt/ros/humble" && -x "$rootfs/bin/bash" ]] || {
    echo "Expected a separate Ubuntu/ROS Humble rootfs" >&2; exit 2;
}
[[ -d "$project/host" && -d "$project/perception" ]] || {
    echo "PROJECT must contain the FPGA host and perception source directories" >&2; exit 2;
}
dds_transport=${FPGA_DDS_TRANSPORT:-udp}
case "$dds_transport" in
    udp|shm_udp) ;;
    *) echo "FPGA_DDS_TRANSPORT must be udp or shm_udp" >&2; exit 2 ;;
esac
# Only service replies need time for the reverse DDS endpoint to match.
# Keep normal publishers and the shared ROS installation unchanged.
service_reply_profile=${FPGA_SERVICE_REPLY_PROFILE:-legacy}
service_reply_xml=""
case "$service_reply_profile" in
    legacy) ;;
    bounded)
        service_reply_xml='<publisher profile_name="service"><qos><reliability><kind>RELIABLE</kind><max_blocking_time><sec>1</sec><nanosec>0</nanosec></max_blocking_time></reliability></qos></publisher>'
        ;;
    *) echo "FPGA_SERVICE_REPLY_PROFILE must be legacy or bounded" >&2; exit 2 ;;
esac
workspace=""
if [[ -n "${FPGA_AGX_WORKSPACE:-}" ]]; then
    workspace=$(realpath -e -- "$FPGA_AGX_WORKSPACE")
    [[ "$workspace" != / ]] || { echo "Refuse to mount the host root" >&2; exit 2; }
fi
shm_descriptor=""
shm_user_transport=""
shm_backing=""
if [[ "$dds_transport" == shm_udp ]]; then
    [[ -n "$workspace" && -n "${FPGA_DDS_SHM_DIR:-}" ]] || {
        echo "shm_udp requires FPGA_AGX_WORKSPACE and FPGA_DDS_SHM_DIR" >&2; exit 2;
    }
    ipc_dir=$(realpath -e -- "$workspace/ipc")
    shm_dir=$(realpath -e -- "$FPGA_DDS_SHM_DIR")
    shm_name=${shm_dir##*/}
    [[ "$ipc_dir" == "$workspace/ipc" && "${shm_dir%/*}" == "$ipc_dir" &&
       "$shm_name" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*-dds$ &&
       -d "$shm_dir" && -O "$shm_dir" && -w "$shm_dir" ]] || {
        echo "SHM directory must be an owned existing workspace/ipc/<session>-dds directory" >&2
        exit 2
    }
    if [[ -n "${FPGA_DDS_SHM_BACKING:-}" ]]; then
        shm_backing=$(realpath -e -- "$FPGA_DDS_SHM_BACKING")
        [[ "$shm_backing" == "/dev/shm/fpga-${workspace##*/}-$shm_name" &&
           -d "$shm_backing" && -O "$shm_backing" && -w "$shm_backing" &&
           "$(stat -f -c %T -- "$shm_backing")" == tmpfs ]] || {
            echo "RAM backing must be this member/session's owned tmpfs subdirectory" >&2
            exit 2
        }
    fi
    # Fast DDS 2.6 transport XML. A segment must comfortably exceed a complete
    # RGB-D message; do not rely on the small middleware default for large frames.
    # https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/xml_configuration/transports.html
    shm_descriptor='<transport_descriptor><transport_id>fpga_shm</transport_id><type>SHM</type><segment_size>16777216</segment_size></transport_descriptor>'
    shm_user_transport='<transport_id>fpga_shm</transport_id>'
fi
command -v bwrap >/dev/null
# Default: private /dev/shm and UDP only. Opt-in: the caller creates one new
# session directory shared by every ROS launch. Never mount host /dev/shm.
# Shared ROS dependencies and the release remain read-only. HOME and /tmp are
# private scratch space; only the member workspace and session IPC are writable.
# Keep the profile ephemeral and read-only inside the namespace.
exec {dds_profile_fd}<<XML
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>fpga_udp</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>
    </transport_descriptor>
    $shm_descriptor
  </transport_descriptors>
  $service_reply_xml
  <participant profile_name="fpga_${dds_transport}" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>fpga_udp</transport_id>$shm_user_transport</userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
mounts=(--ro-bind "$rootfs" / --proc /proc --dev /dev --tmpfs /tmp --tmpfs /root
        --ro-bind-data "$dds_profile_fd" /etc/fpga-fastdds-udp.xml
        --ro-bind /etc/resolv.conf /etc/resolv.conf --ro-bind "$project" /fpga)
if [[ -n "$workspace" ]]; then
    mounts+=(--bind "$workspace" /workspace)
fi
if [[ "$dds_transport" == shm_udp ]]; then
    if [[ -n "$shm_backing" ]]; then
        # Preserve exact project/session marker for process-ownership validation.
        mounts+=(--ro-bind "$shm_dir" /tmp/fpga-session --bind "$shm_backing" /dev/shm)
    else
        mounts+=(--bind "$shm_dir" /dev/shm)
    fi
fi
exec bwrap --unshare-user --uid 0 --gid 0 --unshare-pid --unshare-ipc \
    --die-with-parent --new-session "${mounts[@]}" --chdir /fpga \
    /usr/bin/env -i HOME=/root USER=root LANG=C.UTF-8 \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    PYTHONPATH=/fpga ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fpga-fastdds-udp.xml \
    FPGA_DDS_TRANSPORT="$dds_transport" \
    FPGA_SERVICE_REPLY_PROFILE="$service_reply_profile" \
    FPGA_DDS_LOOPBACK_ONLY=1 ROS_LOCALHOST_ONLY=0 \
    /bin/bash -c 'set -e; source /opt/ros/humble/setup.bash; exec "$@"' fpga-rootfs "$@"
