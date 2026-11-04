#!/bin/bash
# Nano 40-pin UART for the chassis command link (protocol v0.1, CHASSIS_UART_PROTOCOL_20261012.md).
#   nano_uart_setup.sh check       read-only: which ttyTHS* is the 40-pin UART, who else holds it, permissions
#   sudo nano_uart_setup.sh install    udev rule -> /dev/fpga_chassis (group dialout, 0660), stop any getty on it,
#                                      add $SUDO_USER to dialout. Idempotent; prints what it changed.
#   sudo nano_uart_setup.sh uninstall  remove the udev rule (getty masks are reported, not undone automatically)
# Jetson Orin Nano Developer Kit (T234, carrier P3768) 40-pin header: pin 8 = UART1_TXD, pin 10 = UART1_RXD,
# pin 6/9/14/... = GND; 3.3 V levels, NOT 5 V tolerant. UART1 on the header is the T234 controller UARTA at
# 0x03100000, i.e. the tty whose device is 3100000.serial (the ttyTHS number differs between JetPack
# releases, so it is looked up, never assumed). Override with PIN_UART_NODE=<addr>.serial if a loopback
# test (tools/chassis_uart/uart_bench.py loopback, pin 8 jumpered to pin 10) says otherwise.
set -u
MODE=${1:-check}
NODE=${PIN_UART_NODE:-3100000.serial}
LINK=fpga_chassis
RULE=/etc/udev/rules.d/99-fpga-chassis-uart.rules

find_tty() {
  for d in /sys/class/tty/ttyTHS*; do
    [ -e "$d/device" ] || continue
    case "$(readlink -f "$d/device")" in */$NODE) basename "$d"; return 0;; esac
  done
  return 1
}

echo "== ttyTHS devices and their controllers"
for d in /sys/class/tty/ttyTHS* /sys/class/tty/ttyTCU*; do
  [ -e "$d" ] || continue
  n=$(basename "$d"); dev=$(readlink -f "$d/device" 2>/dev/null)
  echo "$n -> ${dev##*/} $(ls -l /dev/$n 2>/dev/null | awk '{print $1, $3, $4}')"
done
TTY=$(find_tty) || { echo "NO tty for $NODE: 40-pin UART not enabled in this device tree"; exit 2; }
echo "40-pin UART (pins 8/10) = /dev/$TTY  ($NODE)"

echo "== users of /dev/$TTY"
grep -o 'console=[^ ]*' /proc/cmdline | sed 's/^/kernel cmdline: /'
grep -q "console=$TTY" /proc/cmdline && echo "WARNING: kernel console is on $TTY (edit APPEND in /boot/extlinux/extlinux.conf)"
for u in "serial-getty@$TTY.service" nvgetty.service; do
  printf '%-32s enabled=%s active=%s\n' "$u" "$(systemctl is-enabled "$u" 2>/dev/null)" "$(systemctl is-active "$u" 2>/dev/null)"
done
grep -s "$TTY" /etc/systemd/nvgetty.sh /etc/systemd/nv*.sh 2>/dev/null | sed 's/^/nvgetty script: /'
holders=$(sudo -n fuser "/dev/$TTY" 2>/dev/null || fuser "/dev/$TTY" 2>/dev/null)
echo "processes holding it: ${holders:-none}"
echo "user $(id -un) groups: $(id -Gn)"
[ -e /dev/$LINK ] && echo "/dev/$LINK -> $(readlink -f /dev/$LINK)" || echo "/dev/$LINK absent"
stty -F "/dev/$TTY" -a 2>/dev/null | head -1

case "$MODE" in
check) exit 0;;
install)
  [ "$(id -u)" = 0 ] || { echo "install needs sudo"; exit 2; }
  want="KERNEL==\"$TTY\", SUBSYSTEM==\"tty\", SYMLINK+=\"$LINK\", GROUP=\"dialout\", MODE=\"0660\""
  if [ "$(cat $RULE 2>/dev/null)" != "$want" ]; then
    printf '%s\n' "$want" > $RULE; echo "wrote $RULE"
  fi
  udevadm control --reload-rules && udevadm trigger --action=add "/sys/class/tty/$TTY"
  sleep 1
  for u in "serial-getty@$TTY.service"; do
    if [ "$(systemctl is-active "$u" 2>/dev/null)" = active ] || [ "$(systemctl is-enabled "$u" 2>/dev/null)" = enabled ]; then
      systemctl stop "$u"; systemctl mask "$u"; echo "stopped+masked $u"
    fi
  done
  if grep -qs "$TTY" /etc/systemd/nvgetty.sh && [ "$(systemctl is-active nvgetty.service)" = active ]; then
    systemctl stop nvgetty.service; systemctl disable nvgetty.service; echo "stopped+disabled nvgetty (it served $TTY)"
  fi
  u=${SUDO_USER:-}
  if [ -n "$u" ] && ! id -nG "$u" | grep -qw dialout; then usermod -aG dialout "$u"; echo "added $u to dialout (new logins only)"; fi
  ls -l "/dev/$LINK" "/dev/$TTY";;
uninstall)
  [ "$(id -u)" = 0 ] || { echo "uninstall needs sudo"; exit 2; }
  rm -fv $RULE; udevadm control --reload-rules && udevadm trigger --action=add "/sys/class/tty/$TTY"
  systemctl list-unit-files 2>/dev/null | grep -E "serial-getty@$TTY|nvgetty" | sed 's/^/left as is: /';;
*) echo "usage: $0 check|install|uninstall"; exit 2;;
esac
