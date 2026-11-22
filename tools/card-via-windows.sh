#!/bin/bash
# Kilo's SD card from WSL when it is NOT mounted in Linux (no sudo password at
# hand, or the /mnt/e mount went stale): the same thing as `kilo card`, with
# Windows doing the copying. docs/laptop-setup.md, "Windows and macOS".
#
#   tools/card-via-windows.sh write [NAME=VALUE ...]   build and write (e.g. KILO_REAR_UP=1)
#   tools/card-via-windows.sh status                   what the Pi left on the card
#   tools/card-via-windows.sh fetch                    recordings -> ~/kilo-logs
#   DRIVE=F tools/card-via-windows.sh ...              another drive letter (default E)
#
# How: the card's files are copied to a staging folder, `kilo card --force`
# runs on that folder (--force because it is not a mount point), and for
# `write` the four files it made go to the card in `kilo card`'s own order,
# each compared by SHA-256 with the staged one: kernel7.new, then kernel7.new
# moved over kernel7.img (no previous kernel is kept; a kernel7.old or
# kernel7.bad left from earlier is removed), then body.bin and body.json;
# body-result.txt is removed, as `kilo card` does.
#
# `write` builds the Pi kernel from an empty build folder: on 2026-11-21 an
# incremental build put a kernel on the card whose pieces disagreed about a
# header (docs/laptop-setup.md, "If the new kernel misbehaves").
set -euo pipefail

REPO="$(cd "$(dirname "$0")/.." && pwd)"
DRIVE="${DRIVE:-E}"
KILO="${KILO:-$REPO/.venv/bin/kilo}"
cmd="${1:-}"; shift || true

ps() { powershell.exe -NoProfile -ExecutionPolicy Bypass "$@" | tr -d '\r'; }

ps -Command "if (-not (Test-Path ${DRIVE}:\\kernel7.img)) { exit 3 }" \
  || { echo "drive ${DRIVE}: has no kernel7.img: is Kilo's card in the reader? (DRIVE=F for another letter)" >&2; exit 1; }

STAGE="$(mktemp -d "${TMPDIR:-/tmp}/kilo-card.XXXXXX")"
trap 'rm -rf "$STAGE"' EXIT
mkdir -p "$STAGE/kilorec"
WSTAGE="$(wslpath -w "$STAGE")"

# the card as `kilo card` wants to see it; the recordings only when they are read
ps -Command "\$ErrorActionPreference='Stop'
  foreach (\$f in 'kernel7.img','config.txt','bootcode.bin','body.bin','body.json','body-result.txt') {
    if (Test-Path ${DRIVE}:\\\$f) { Copy-Item ${DRIVE}:\\\$f '$WSTAGE' } }
  if (Test-Path ${DRIVE}:\\kilorec\\SAVE_ON) { Copy-Item ${DRIVE}:\\kilorec\\SAVE_ON '$WSTAGE\\kilorec' }"

case "$cmd" in
status|fetch)
  ps -Command "if (Test-Path ${DRIVE}:\\kilorec) { Copy-Item ${DRIVE}:\\kilorec\\* '$WSTAGE\\kilorec' -Force }"
  "$KILO" card "$STAGE" "--$cmd" --force "$@"
  ;;
write)
  cp "$STAGE/kernel7.img" "$STAGE/.card-kernel"
  rm -rf "$REPO/kernel/build"
  "$KILO" card "$STAGE" --force "$@"
  cat > "$STAGE/write.ps1" <<EOF
\$ErrorActionPreference = 'Stop'
function FHash(\$p) { (Get-FileHash -Algorithm SHA256 \$p).Hash }   # not "H": that is Get-History
\$s = '$WSTAGE'; \$d = '${DRIVE}:'
if ((FHash "\$d\\kernel7.img") -ne (FHash "\$s\\.card-kernel")) { throw 'the card changed since it was staged' }
if ((FHash "\$s\\kernel7.img") -ne (FHash "\$s\\.card-kernel")) {
  Copy-Item "\$s\\kernel7.img" "\$d\\kernel7.new"
  if ((FHash "\$d\\kernel7.new") -ne (FHash "\$s\\kernel7.img")) { throw 'kernel7.new on the card differs from the build' }
  Move-Item -Force "\$d\\kernel7.new" "\$d\\kernel7.img"
} else { 'kernel: the card already has this one' }
foreach (\$f in 'kernel7.old','kernel7.bad') {   # no previous kernel is kept
  if (Test-Path "\$d\\\$f") { Remove-Item "\$d\\\$f"; "\$f: removed" }
}
Copy-Item "\$s\\body.bin" "\$d\\body.bin" -Force
Copy-Item "\$s\\body.json" "\$d\\body.json" -Force
foreach (\$f in 'kernel7.img','body.bin','body.json') {
  if ((FHash "\$d\\\$f") -ne (FHash "\$s\\\$f")) { throw "\$f on the card differs from the build" } else { "\$f: on the card, verified" }
}
if (Test-Path "\$d\\body-result.txt") { Remove-Item "\$d\\body-result.txt" }
EOF
  ps -File "$(wslpath -w "$STAGE/write.ps1")"
  echo "Done. Eject drive ${DRIVE}: in Windows (the tray's \"Safely Remove\") before taking the card out."
  ;;
*)
  sed -n '2,10p' "$0" | sed 's/^# \?//' >&2; exit 2 ;;
esac
