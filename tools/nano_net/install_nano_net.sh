#!/bin/bash
# Install the Nano network-stability changes. Run as root on the Nano, from the
# directory that holds this script:  sudo bash install_nano_net.sh
#
#   1. WiFi power save off (NetworkManager default on this image is on).
#   2. fpga-wifi-watchdog service.
#   3. KV260 lease on the shared wired link made infinite: the Nano has no clock
#      battery, its clock jumps forward at the first NTP sync after power-up, dnsmasq
#      then drops the one-hour lease and NAKs every renewal from the KV260.
#   4. ConnectTimeout for the reverse tunnel's ssh: a connect started while the WiFi was
#      dead does not survive the re-association and otherwise blocks for the kernel's
#      ~130 s connect timeout after the WiFi is already back (measured 2026-10-13).
#   5. Time source: time.edu.cn, then ntp.tencent.com. The image only lists pool.ntp.org
#      servers, whose round trip from here swings between 35 and 430 ms; timesyncd pulled
#      the clock back 270 ms in the middle of a run on 2026-10-13. Measured that day
#      against the board's raw clock (49 min, one query per 30 s): these two stay within
#      (8 ms) but wanders 62 ms peak to peak and steps 40 ms, ntp.aliyun.com had one
#      reply 2.4 s late. See README.md.
#
# Undo: sudo bash install_nano_net.sh --undo
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
WIFI_IF="${FPGA_WD_IF:-wlP1p1s0}"
KV_CON="${FPGA_KV_CON:-kv260-link}"
KV_CONF=/etc/NetworkManager/dnsmasq-shared.d/kv260.conf
PS_CONF=/etc/NetworkManager/conf.d/zz-fpga-wifi-powersave-off.conf
BACKUP=/root/fpga-nano-net-backup
TUNNEL="${FPGA_TUNNEL_UNIT:-revtunnel-super-test.service}"
TUNNEL_DROPIN="/etc/systemd/system/$TUNNEL.d/10-fpga-connect-timeout.conf"
NTP_CONF=/etc/systemd/timesyncd.conf.d/zz-fpga-ntp.conf
NTP_SERVERS="${FPGA_NTP_SERVERS:-time.edu.cn ntp.tencent.com}"

[ "$(id -u)" -eq 0 ] || { echo "run as root" >&2; exit 2; }

if [ "${1:-}" = "--undo" ]; then
    systemctl disable --now fpga-wifi-watchdog.service 2>/dev/null || true
    rm -f /etc/systemd/system/fpga-wifi-watchdog.service /usr/local/sbin/fpga-wifi-watchdog.sh \
        /etc/logrotate.d/fpga-wifi-watchdog "$PS_CONF" "$TUNNEL_DROPIN" "$NTP_CONF"
    systemctl daemon-reload
    systemctl restart systemd-timesyncd.service
    [ -f "$BACKUP/kv260.conf" ] && cp "$BACKUP/kv260.conf" "$KV_CONF"
    nmcli general reload conf
    echo "undone; power save, the KV260 lease time and the tunnel change back at their next (re)start"
    exit 0
fi

# Backups live outside the conf directories: dnsmasq and NetworkManager read every file there.
mkdir -p "$BACKUP"
[ -f "$BACKUP/kv260.conf" ] || cp "$KV_CONF" "$BACKUP/kv260.conf"

# 1. power save off: 2 = disable (3 = enable, the image default)
printf '[connection]\nwifi.powersave = 2\n' >"$PS_CONF"
nmcli general reload conf
iw dev "$WIFI_IF" set power_save off
echo "power save now: $(iw dev "$WIFI_IF" get power_save)"

# 2. watchdog
install -m 0755 "$HERE/fpga-wifi-watchdog.sh" /usr/local/sbin/fpga-wifi-watchdog.sh
install -m 0644 "$HERE/fpga-wifi-watchdog.service" /etc/systemd/system/fpga-wifi-watchdog.service
cat >/etc/logrotate.d/fpga-wifi-watchdog <<'EOF'
/var/log/fpga-wifi-watchdog.log {
    size 2M
    rotate 4
    missingok
    notifempty
    copytruncate
}
EOF
systemctl daemon-reload
systemctl enable fpga-wifi-watchdog.service
systemctl restart fpga-wifi-watchdog.service
echo "watchdog: $(systemctl is-active fpga-wifi-watchdog.service)"

# 3. infinite lease for the KV260
if grep -q ',infinite' "$KV_CONF"; then
    echo "KV260 lease already infinite"
else
    sed -i -E 's/^(dhcp-host=[^#]*[^,[:space:]])[[:space:]]*$/\1,infinite/' "$KV_CONF"
    echo "KV260 conf: $(cat "$KV_CONF")"
    # Re-activating the shared profile restarts its dnsmasq and bounces the wired link,
    # so the KV260 asks again from scratch.
    nmcli con up id "$KV_CON"
fi

# 4. tunnel ConnectTimeout. The session that runs this script usually arrives through
#    that very tunnel, so the restart is done by a detached job that puts the old unit
#    back if the new one is not holding a connection after 60 s.
if [ -f "/etc/systemd/system/$TUNNEL" ] && [ ! -f "$TUNNEL_DROPIN" ]; then
    cur=$(sed -n 's/^ExecStart=//p' "/etc/systemd/system/$TUNNEL")
    case "$cur" in
    *ConnectTimeout*) echo "tunnel already has a ConnectTimeout" ;;
    *"-o BatchMode=yes"*)
        mkdir -p "$(dirname "$TUNNEL_DROPIN")"
        printf '[Service]\nExecStart=\nExecStart=%s\n' \
            "${cur/-o BatchMode=yes/-o BatchMode=yes -o ConnectTimeout=10}" >"$TUNNEL_DROPIN"
        systemctl daemon-reload
        systemd-run --unit=fpga-tunnel-dropin-guard --collect bash -c "
            sleep 3
            systemctl restart $TUNNEL
            sleep 60
            pid=\$(systemctl show -p MainPID --value $TUNNEL)
            age=\$(ps -o etimes= -p \$pid 2>/dev/null | tr -d ' ')
            if [ \"\$(systemctl is-active $TUNNEL)\" = active ] && [ \"\${age:-0}\" -ge 20 ]; then
                logger -t fpga-nano-net 'tunnel drop-in kept (ssh alive '\$age' s)'
            else
                rm -f $TUNNEL_DROPIN
                systemctl daemon-reload
                systemctl restart $TUNNEL
                logger -t fpga-nano-net 'tunnel drop-in REVERTED'
            fi"
        echo "tunnel drop-in written; the tunnel restarts in 3 s (this session may drop)"
        ;;
    *) echo "tunnel ExecStart not recognised, left alone" ;;
    esac
fi

# 5. time source. The file name sorts after the image's nv-fallback-ntp.conf; the pool
#    servers listed there stay as the fallback (an offline demo network reaches none of
#    them and timesyncd then simply does not adjust the clock).
rm -f /etc/systemd/timesyncd.conf.d/zz-fpga-campus-ntp.conf # name used on 2026-10-13 14:11
printf '[Time]\nNTP=%s\n' "$NTP_SERVERS" >"$NTP_CONF"
systemctl restart systemd-timesyncd.service
for i in 1 2 3 4 5 6 7 8 9 10; do
    timedatectl timesync-status 2>/dev/null | grep -q 'Packet count: [1-9]' && break
    sleep 2
done
echo "time source: $(timedatectl timesync-status 2>/dev/null | sed -n 's/^ *Server: //p')"
echo "done"
