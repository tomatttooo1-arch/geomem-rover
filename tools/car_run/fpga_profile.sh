#!/usr/bin/env bash
# Source me: fpga_profile [name] exports, from tools/car_run/fpga_profiles.env,
#   FPGA_PROFILE (v3fc2|v3fc2_166|v3fc3_187|v3fc|a2|m1_166|m1|m2_166|m2; empty name = $FPGA_PROFILE, else the file's default)
#   FPGA_APP FPGA_BUILD_ID FPGA_KV_ROLLING FPGA_NANO_ROLLING FPGA_KV_FAST (E7, off|native)
#   FPGA_AR (SW_M1 2026-11-03): nano = the profile's bitstream has no AR core (<P>_AR=nano, m1 / m1_166 / m2 / m2_166: the
#   car screen is drawn on the Nano); kv = every profile without that line (as before)
# An unknown profile name only warns and uses the file's default (owner 10-26: no new refusals); a profile with
# <P>_USE=<other> in the file (paused) warns and uses that other profile (10-26: v3fc3_187 -> v3fc2);
# returns 2 only when the profile file itself is missing. Go-live 2026-10-26.
FPGA_PROFILES_ENV=${FPGA_PROFILES_ENV:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/fpga_profiles.env}
fpga_profile() {
  local name=${1:-${FPGA_PROFILE:-}} key val P
  local -A kv=()
  [ -f "$FPGA_PROFILES_ENV" ] || { echo "fpga profile: $FPGA_PROFILES_ENV missing" >&2; return 2; }
  while IFS='=' read -r key val; do
    case "$key" in ''|'#'*) continue;; esac
    kv[$key]=$val
  done < "$FPGA_PROFILES_ENV"
  name=${name:-${kv[FPGA_PROFILE_DEFAULT]:-}}
  case "$name" in
    v3fc2) P=V3FC2;;
    v3fc2_166) P=V3FC2_166;;
    v3fc3_187) P=V3FC3_187;;
    v3fc) P=V3FC;;
    a2) P=A2;;
    m1_166) P=M1_166;;
    m1) P=M1;;
    m2_166) P=M2_166;;
    m2) P=M2;;
    *) echo "WARNING fpga profile: unknown FPGA_PROFILE '$name' (v3fc2 | v3fc2_166 | v3fc3_187 | v3fc | a2 | m1_166 | m1 | m2_166 | m2); using the default ${kv[FPGA_PROFILE_DEFAULT]:-a2}" >&2
       name=${kv[FPGA_PROFILE_DEFAULT]:-a2}; P=${name^^};;
  esac
  # a paused profile (<P>_USE=<other>, 10-26: v3fc3_187 until its 187.5 MHz clock is fixed) warns and runs the other one
  if [ -n "${kv[${P}_USE]:-}" ]; then
    echo "WARNING fpga profile: '$name' is paused (${P}_USE=${kv[${P}_USE]} in $FPGA_PROFILES_ENV); using ${kv[${P}_USE]}" >&2
    name=${kv[${P}_USE]}; P=${name^^}
  fi
  export FPGA_PROFILE=$name FPGA_APP=${kv[${P}_APP]} FPGA_BUILD_ID=${kv[${P}_BUILD_ID]}
  export FPGA_KV_ROLLING=${kv[${P}_KV_ROLLING]} FPGA_NANO_ROLLING=${kv[${P}_NANO_ROLLING]}
  export FPGA_AR=${kv[${P}_AR]:-kv}
  # E7 2026-11-02: kv_server --kv-fast (off|native), one switch for every profile; absent = off
  export FPGA_KV_FAST=${kv[FPGA_KV_FAST_DEFAULT]:-off}
}
# fpga_profile_of_build <BUILD_ID>: prints the registered profile whose BUILD_ID equals it
# (numeric compare; a 0 placeholder never matches; v3fc2 and v3fc2_166 share 0x03c087c9 -> the first listed,
# v3fc2, same rolling backends; m1 and m1_166 share 0x08051f22 -> m1, m2 and m2_166 share 0x07da3d4a -> m2; same rolling backends and AR), returns 1 when none does.
fpga_profile_of_build() {
  local want key val n
  want=$(( ${1:-0} )) 2>/dev/null || return 1
  [ "$want" != 0 ] || return 1
  while IFS='=' read -r key val; do
    case "$key" in *_BUILD_ID) ;; *) continue;; esac
    [ $(( val )) = "$want" ] || continue
    n=${key%_BUILD_ID}; echo "${n,,}"; return 0
  done < "$FPGA_PROFILES_ENV"
  return 1
}
