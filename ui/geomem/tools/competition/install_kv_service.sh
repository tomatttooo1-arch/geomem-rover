#!/usr/bin/env bash
# Administrator install of the competition KV startup snapshot (run on the KV260 with sudo). Does not load a
# bitstream or start a server (geomem-kv-start does that).
#   sudo bash install_kv_service.sh <snapshot>            first install; refuses (exit 3) if one is installed
#   sudo bash install_kv_service.sh --update <snapshot>   10-26 (comp chain, deploy_kv.sh): replace the installed
#        snapshot; refuses (exit 4) while a kv_server runs or a start holds start.lock (stop the KV service first)
#   sudo bash install_kv_service.sh <snapshot> --replace [--app-dir D]...   go-live 2026-10-26 (FPGA profiles):
#        as --update, but a running kv_server is stopped first (kv_board.sh stop <its run>) and the start lock is
#        waited for; each D (the package_app.sh app directory, e.g. .../m7_pl_v3fc2; repeatable: m7_pl_v3fc2,
#        m7_pl_v3fc2_166, m7_pl_v3fc3, m7_pl_v3fc, m7_pl_m1_166, m7_pl_m1, m7_pl_m2_166, m7_pl_m2) is copied to /lib/firmware/xilinx/<name> (an existing one is
#        kept as <name>.prev-<stamp>).
#   sudo bash install_kv_service.sh --rollback            undo the last --update/--replace: stop the server, put
#        the newest kept previous snapshot (releases/*-prev-*, or an older source.prev-*) back as source, reinstall
#        its helper (firmware apps are left as they are).
# --update/--replace: the new snapshot is copied into releases/ (root-owned), the old source/ is moved into
#   releases/<stamp>-prev-<commit7> (kept for rollback; --update <that directory> also rolls back), then the new one
#   is renamed into source/ while start.lock is held (geomem-kv-start cannot run in between);
#   /usr/local/sbin/geomem-kv-start and the sudoers rule are reinstalled.
# sudoers: ubuntu may run exactly geomem-kv-start with no argument, v3fc2, v3fc2_166, v3fc3_187, v3fc, a2, m1_166, m1, m2_166, m2 or stop.
# No symbolic links anywhere in the installed paths. Prints "INSTALLED COMMIT=<sha>".
# GEOMEM_KV_TEST_ROOT (tests only): prefix for every installed path; sudo resets the environment, so the real
#   administrator run never sees it.
set -euo pipefail
mode=install source_dir="" app_dirs=()
while [ $# -gt 0 ]; do
  case "$1" in
    --update) mode=update; shift ;;
    --replace) mode=replace; shift ;;
    --rollback) mode=rollback; shift ;;
    --app-dir) app_dirs+=("$(realpath "${2:?--app-dir needs the app directory}")"); shift 2 ;;   # repeatable
    -*) echo "unknown option $1" >&2; exit 2 ;;
    *) source_dir=$(realpath "$1"); shift ;;
  esac
done
test "$(id -u)" = 0 || { echo 'Run this installer with sudo.' >&2; exit 2; }
T=${GEOMEM_KV_TEST_ROOT:-}
base=$T/opt/geomem-dev3-kv
state=$T/var/lib/geomem-dev3-kv
sbin=$T/usr/local/sbin
sudoers=$T/etc/sudoers.d
fw=$T/lib/firmware/xilinx
helper=/usr/local/sbin/geomem-kv-start   # the path sudoers names (the real one, also under a test root)
stamp=$(date +%Y%m%d-%H%M%S)-$$
test ! -L "$base" && test ! -L "$base/source" && test ! -L "$base/releases"
mkdir -p "$base/releases" "$state/runs" "$sbin" "$sudoers"

stop_running() {   # every kv_server through its own run directory (kv_board.sh stop), whatever its profile
  local pid out board
  board="$base/source/tools/kv_link/kv_board.sh"
  for pid in $(pgrep -f '[k]v_server.py' || true); do
    out=$(tr '\0' '\n' < "/proc/$pid/cmdline" 2>/dev/null | sed -n '/^--out$/{n;p;q}')
    if [ -n "$out" ] && [ -f "$out/ready.json" ] && [ -f "$board" ]; then
      echo "stopping kv_server pid $pid (run $out)"
      bash "$board" stop "$out"
    else
      echo "kv_server pid $pid has no run directory with ready.json; stop it by hand, then rerun." >&2
      exit 4
    fi
  done
}

install_helper_and_policy() {
  local policy
  install -o root -g root -m 755 "$base/source/ui/geomem/tools/competition/kv_service.py" "$sbin/geomem-kv-start"
  policy=$(mktemp)
  printf 'ubuntu ALL=(root) NOPASSWD: %s v3fc2, %s v3fc2_166, %s v3fc3_187, %s v3fc, %s a2, %s m1_166, %s m1, %s m2_166, %s m2, %s stop, %s ""\n' \
    "$helper" "$helper" "$helper" "$helper" "$helper" "$helper" "$helper" "$helper" "$helper" "$helper" "$helper" > "$policy"
  visudo -cf "$policy"
  install -o root -g root -m 440 "$policy" "$sudoers/geomem-dev3-kv"
  rm -f "$policy"
}

exec 9>"$state/start.lock"
if [ "$mode" = rollback ]; then
  # newest kept snapshot by its stamp: releases/<YYYYmmdd-HHMMSS>-<pid>-prev-<c7> or (before the merge) source.prev-<stamp>
  prev=$(for d in "$base"/releases/*-prev-* "$base"/source.prev-*; do
           [ -d "$d" ] || continue; b=$(basename "$d"); s=${b#source.prev-}; echo "${s:0:15} $d"
         done | sort | tail -1 | cut -d' ' -f2-)
  test -n "$prev" || { echo 'No kept previous snapshot (releases/*-prev-* or source.prev-*) to roll back to.' >&2; exit 3; }
  flock -w 120 9 || { echo 'A KV start holds start.lock for over 120 s; not rolling back.' >&2; exit 4; }
  stop_running
  old=$(head -1 "$base/source/COMMIT" 2>/dev/null || echo unknown)
  mv -T "$base/source" "$base/releases/$stamp-rolledback-${old:0:7}"
  mv -T "$prev" "$base/source"
  install_helper_and_policy
  echo "Rolled back to $prev (now $base/source; the replaced one is releases/$stamp-rolledback-${old:0:7}). No server started."
  echo "INSTALLED COMMIT=$(head -1 "$base/source/COMMIT")"
  exit 0
fi

test -n "$source_dir" || { echo 'pinned source snapshot required' >&2; exit 2; }
test -f "$source_dir/COMMIT"
test -f "$source_dir/tools/kv_link/kv_server.py"
test -f "$source_dir/ui/geomem/tools/competition/kv_service.py"
case "$source_dir/" in "$base/source/"*) echo 'The snapshot to install cannot be the installed source itself.' >&2; exit 2;; esac
for app_dir in ${app_dirs[@]+"${app_dirs[@]}"}; do
  test "$mode" = replace || { echo '--app-dir only with --replace' >&2; exit 2; }
  app=$(basename "$app_dir")
  for f in "$app.bit.bin" "$app.dtbo" shell.json; do
    test -f "$app_dir/$f" || { echo "app directory $app_dir lacks $f" >&2; exit 2; }
  done
  if [ -f "$app_dir/../SHA256SUMS" ]; then   # package_app.sh output: verify before touching anything
    ( cd "$app_dir/.." && sha256sum --quiet -c SHA256SUMS ) || { echo "SHA256SUMS of the package does not verify" >&2; exit 2; }
  fi
done
if [ "$mode" = install ] && test -e "$base/source"; then
  echo 'An installed snapshot already exists; preserving it. Update with --update (KV service stopped) or --replace.' >&2
  exit 3
fi
case "$mode" in
  update)
    if ! flock -n 9; then echo 'A KV start is in progress (start.lock held); not updating.' >&2; exit 4; fi
    if pgrep -f '[k]v_server.py' >/dev/null; then
      echo 'A kv_server is running; stop it before updating (the running service keeps its old code). Not updating.' >&2
      exit 4
    fi;;
  replace)
    flock -w 120 9 || { echo 'A KV start holds start.lock for over 120 s; not replacing.' >&2; exit 4; }
    stop_running;;
  *) flock -n 9 || true;;
esac
commit=$(head -1 "$source_dir/COMMIT")
release="$base/releases/$stamp-${commit:0:7}"
mkdir "$release"
# Source is a supplied git archive (plus the cross-built AR helper), not a developer checkout or runtime directory.
cp -a "$source_dir/." "$release/"
if find "$release" -type l | grep -q .; then
  rm -rf "$release"; echo 'The snapshot contains symbolic links; refusing.' >&2; exit 5
fi
chown -R root:root "$release"
chmod -R go-w "$release"
chmod -R a+rX "$release"   # staged files inherit the host umask (dirs 700 / files 600); keep the snapshot world-readable like 10-19
prev=""
if test -e "$base/source"; then
  old=$(head -1 "$base/source/COMMIT" 2>/dev/null || echo unknown)
  prev="$base/releases/$stamp-prev-${old:0:7}"
  mv -T "$base/source" "$prev"
fi
if ! mv -T "$release" "$base/source"; then
  [ -n "$prev" ] && mv -T "$prev" "$base/source"
  echo 'Could not move the new snapshot into place; previous snapshot restored.' >&2; exit 6
fi
install_helper_and_policy
for app_dir in ${app_dirs[@]+"${app_dirs[@]}"}; do
  mkdir -p "$fw"
  app=$(basename "$app_dir")
  if [ -e "$fw/$app" ]; then mv "$fw/$app" "$fw/$app.prev-$stamp"; echo "previous $fw/$app kept as $app.prev-$stamp"; fi
  cp -r "$app_dir" "$fw/$app"
  chown -R root:root "$fw/$app"
  echo "FPGA app installed: $fw/$app"
done
[ -n "$prev" ] && echo "Previous snapshot kept for rollback: $prev"
echo "INSTALLED COMMIT=$(head -1 "$base/source/COMMIT")"
echo "Installed: fixed root-owned KV startup snapshot; sudo $helper [v3fc2|v3fc2_166|v3fc3_187|v3fc|a2|m1_166|m1|m2_166|m2|stop] (no argument = a2). Running hardware unchanged, no server started."
