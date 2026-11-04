#!/bin/bash
# fpga-wifi-watchdog: re-associate the Nano's WiFi when it is associated but dead.
#
# Why (syslog of 2026-10-12/13): three outages in 70 min. In each one wpa_supplicant
# stayed associated, the tunnel's ssh said "No route to host" (the gateway no longer
# answered ARP), nothing was logged by the kernel, and the outage ended at the second
# the interface re-associated. So: when the gateway stops answering at layer 2, force
# a re-association instead of waiting for a roam.
#
# Ladder while the gateway is dead: 30 s re-associate, 90 s radio off/on, 210 s reload
# the driver (at most once per 30 min), then start over. Never reboots the board.
# "Gateway alive but nothing behind it" (UPSTREAM) is only logged: an offline demo
# network looks exactly like that and must not be disturbed.
set -u

IF="${FPGA_WD_IF:-wlP1p1s0}"
FAR="${FPGA_WD_FAR:-198.18.200.1 223.5.5.5}"
LOG="${FPGA_WD_LOG:-/var/log/fpga-wifi-watchdog.log}"
PERIOD="${FPGA_WD_PERIOD:-10}"
MODULE="${FPGA_WD_MODULE:-rtl8822ce}"
T_REASSOC="${FPGA_WD_T_REASSOC:-30}"
T_RADIO="${FPGA_WD_T_RADIO:-90}"
T_RELOAD="${FPGA_WD_T_RELOAD:-210}"
T_CYCLE="${FPGA_WD_T_CYCLE:-510}"
RELOAD_MIN_GAP="${FPGA_WD_RELOAD_MIN_GAP:-1800}"

now() { cut -d. -f1 /proc/uptime; }
# The Nano has no clock battery: wall time jumps at boot, so every line also carries uptime.
log() { printf '%s up=%s %s\n' "$(date '+%F %T')" "$(now)" "$*" >>"$LOG"; }

gateway() {
    ip -4 route show default dev "$IF" 2>/dev/null |
        awk '{for (i = 1; i < NF; i++) if ($i == "via") {print $(i + 1); exit}}'
}
bssid() { iw dev "$IF" link 2>/dev/null | awk '/Connected to/ {print $3}'; }
addr() { ip -4 -o addr show dev "$IF" 2>/dev/null | awk '{print $4; exit}'; }

# Layer-2 liveness of the gateway. ICMP may be filtered (a laptop hotspot), ARP is not,
# and ARP failure is the signature of the outages this script exists for.
l2_alive() {
    local g="$1" i st
    ping -c1 -W1 -I "$IF" "$g" >/dev/null 2>&1 && return 0
    for i in 1 2 3 4 5 6 7 8; do
        st=$(ip -4 neigh show "$g" dev "$IF" 2>/dev/null | awk '{print $NF}')
        case "$st" in
        REACHABLE) return 0 ;;
        FAILED | INCOMPLETE | "") [ "$i" -ge 3 ] && return 1 ;;
        esac
        ping -c1 -W1 -I "$IF" "$g" >/dev/null 2>&1 && return 0
    done
    return 1
}

probe() { # OK | UPSTREAM | DEAD | DOWN | OFF
    local g f
    [ "$(nmcli radio wifi 2>/dev/null)" = "enabled" ] || { echo OFF; return; }
    g=$(gateway)
    [ -n "$g" ] || { echo DOWN; return; }
    for f in $FAR; do
        ping -c1 -W2 "$f" >/dev/null 2>&1 && { echo OK; return; }
    done
    if l2_alive "$g"; then echo UPSTREAM; else echo DEAD; fi
}

snapshot() {
    {
        echo "--- snapshot: $1"
        iw dev "$IF" link 2>&1
        echo "power_save: $(iw dev "$IF" get power_save 2>&1)"
        ip -4 addr show dev "$IF" 2>&1 | sed -n 2p
        ip -4 route show 2>&1
        ip -4 neigh show dev "$IF" 2>&1
        nmcli -t -f DEVICE,STATE,CONNECTION dev 2>&1 | grep -E "^($IF|enP)"
        dmesg 2>/dev/null | tail -8
        echo "---"
    } >>"$LOG"
}

reassociate() {
    local c
    c=$(nmcli -g GENERAL.CONNECTION dev show "$IF" 2>/dev/null)
    log "ACTION reassociate profile='${c:-none}'"
    # "con up" on the active profile tears down and re-activates in one step. Never
    # "dev disconnect": that switches autoconnect off for the device.
    if [ -n "$c" ]; then
        timeout 60 nmcli con up id "$c" ifname "$IF" >>"$LOG" 2>&1 ||
            timeout 60 nmcli dev connect "$IF" >>"$LOG" 2>&1
    else
        timeout 60 nmcli dev connect "$IF" >>"$LOG" 2>&1
    fi
    nmcli dev set "$IF" autoconnect yes >/dev/null 2>&1
}

radio_cycle() {
    log "ACTION radio off/on"
    nmcli radio wifi off >>"$LOG" 2>&1
    sleep 3
    nmcli radio wifi on >>"$LOG" 2>&1
    sleep 5
    nmcli dev set "$IF" autoconnect yes >/dev/null 2>&1
}

reload_driver() {
    log "ACTION reload driver $MODULE"
    timeout 30 modprobe -r "$MODULE" >>"$LOG" 2>&1
    sleep 2
    timeout 30 modprobe "$MODULE" >>"$LOG" 2>&1
    sleep 10
    nmcli dev set "$IF" autoconnect yes >/dev/null 2>&1
}

state=INIT
bad_since=0
stage=0
last_reload=-999999
last_nudge=-999999
last_hb=$(now)
last_bssid=""
last_addr=""
log "START if=$IF far='$FAR' period=$PERIOD power_save='$(iw dev "$IF" get power_save 2>&1)'"

while :; do
    s=$(probe)
    t=$(now)
    b=$(bssid)
    a=$(addr)
    if [ -n "$b" ] && [ "$b" != "$last_bssid" ]; then
        log "BSSID '${last_bssid}' -> '$b'"
        last_bssid=$b
    fi
    if [ -n "$a" ] && [ "$a" != "$last_addr" ]; then
        log "ADDR '${last_addr}' -> '$a'"
        last_addr=$a
    fi

    if [ "$s" != "$state" ]; then
        log "STATE $state -> $s gw=$(gateway) bssid=$b"
        case "$s" in
        OK)
            [ "$bad_since" -gt 0 ] && log "RECOVERED after $((t - bad_since)) s, stage reached $stage"
            bad_since=0
            stage=0
            ;;
        *)
            if [ "$bad_since" -eq 0 ]; then
                bad_since=$t
                snapshot "left OK as $s"
            fi
            ;;
        esac
        state=$s
    fi

    d=$((t - bad_since))
    case "$s" in
    DEAD)
        log "PROBE DEAD for $d s stage=$stage"
        if [ "$stage" -eq 0 ] && [ "$d" -ge "$T_REASSOC" ]; then
            reassociate
            stage=1
        elif [ "$stage" -eq 1 ] && [ "$d" -ge "$T_RADIO" ]; then
            radio_cycle
            stage=2
        elif [ "$stage" -eq 2 ] && [ "$d" -ge "$T_RELOAD" ]; then
            if [ $((t - last_reload)) -ge "$RELOAD_MIN_GAP" ]; then
                reload_driver
                last_reload=$(now)
            else
                log "SKIP reload driver (last one $((t - last_reload)) s ago)"
            fi
            stage=3
        elif [ "$stage" -eq 3 ] && [ "$d" -ge "$T_CYCLE" ]; then
            snapshot "still dead after the whole ladder, starting over"
            bad_since=$t
            stage=0
        fi
        ;;
    DOWN)
        # NetworkManager is already trying; only nudge it if it has given up.
        if [ "$d" -ge 120 ] && [ $((t - last_nudge)) -ge 300 ]; then
            log "ACTION nudge autoconnect (no default route on $IF for $d s)"
            nmcli dev set "$IF" autoconnect yes >/dev/null 2>&1
            timeout 60 nmcli dev connect "$IF" >>"$LOG" 2>&1
            last_nudge=$t
        fi
        ;;
    UPSTREAM)
        [ $((d % 300)) -lt "$PERIOD" ] && log "PROBE UPSTREAM for $d s (gateway alive, far side silent; no action)"
        ;;
    esac

    if [ $((t - last_hb)) -ge 3600 ]; then
        log "HB state=$s bssid=$b addr=$a power_save='$(iw dev "$IF" get power_save 2>&1)'"
        last_hb=$t
    fi
    # A dead probe already takes about 13 s; do not add a full period on top of it.
    if [ "$s" = OK ]; then sleep "$PERIOD"; else sleep 3; fi
done
