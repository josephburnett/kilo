#!/usr/bin/env bash
# Every check of the mind core on the Linux host:
#   1. the C++ unit tests (sanitizers): formatter, framing vectors, names, link state machine,
#      planner, walker, and the planner against the Python planner's vectors (float and double);
#   2. the planner's cost per tick on this machine;
#   3. the freestanding build (no libc or libstdc++ beyond memcpy/memset/memcmp, libm's
#      float sinf cosf sincosf tanf atanf atan2f);
#   4. kilo-mind-host against the virtual body (the real body core on a pty), and
#      against a silent pty and a protocol-1 fake body; kilo --host (status, monitor, posture,
#      log, the bridge, kilo reboot: exit 3, kilo update into --card DIR: exit 3 and the
#      card's files checked), all over the Noise channel with a throwaway key (kilo key init),
#      and refused: a key the mind does not know, no key file, a plaintext frame;
#      walk-demo trotting the virtual body;
#   5. body flashing (P4): kilo flash --host through kilo-mind-host to a fake optiboot
#      (tests/fake_optiboot.py), a refused CRC, a sync timeout, and kilo flash --sd.

#   5. P3: the generated trick and personality tables and the perception and behaviour golden
#      vectors up to date with the Python mind (their unit tests run in 1.); IR keys, the
#      behaviour runner and perception against the virtual body (vbody.core --control).
#
#   kernel/host/test.sh
#   PYTHON=/path/to/python   interpreter with the mind's kilo package (default ~/OpenCat/.venv/bin/python)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
PYTHON="${PYTHON:-$HOME/OpenCat/.venv/bin/python}"
HOST="$HERE/build/kilo-mind-host"
OUT="$HERE/build/integration"
mkdir -p "$OUT"
JOBS="$(nproc 2>/dev/null || echo 2)"

echo "== vendored protocol copies up to date"
python3 "$REPO/protocol/vendor.py" --check

echo "== build"
make -s -C "$HERE" -j"$JOBS" all

echo "== unit tests"
make -s -C "$HERE" -j"$JOBS" check

echo "== planner cost per tick"
make -s -C "$HERE" -j"$JOBS" bench

echo "== freestanding core"
make -s -C "$HERE" freestanding

echo "== Noise vectors up to date (noiseprotocol, the published cacophony and snow vectors)"
"$PYTHON" "$HERE/tests/gen_noise_vectors.py" | cmp -s - "$HERE/tests/noise_vectors.txt" ||
    { echo "FAILED: stale; run: $PYTHON kernel/host/tests/gen_noise_vectors.py > kernel/host/tests/noise_vectors.txt"; exit 1; }

# Start a body that prints its pty; sets BODY_PID and PTY.
start_body() {
    local log="$1"; shift
    "$@" > "$log" 2>&1 &
    BODY_PID=$!
    PTY=""
    for _ in $(seq 1 150); do
        PTY="$(grep -o '/dev/pts/[0-9]*' "$log" | head -1 || true)"
        [ -n "$PTY" ] && return 0
        kill -0 "$BODY_PID" 2>/dev/null || break
        sleep 0.2
    done
    echo "body did not start:"; cat "$log"; return 1
}

stop_body() {
    kill "$BODY_PID" 2>/dev/null || true
    wait "$BODY_PID" 2>/dev/null || true
}

expect() {  # expect <log> <grep -E pattern> <what>
    if ! grep -Eq "$2" "$1"; then
        echo "FAILED: $3 (no /$2/ in $1):"; cat "$1"; exit 1
    fi
}

echo "== integration: kilo-mind-host against the virtual body (python -m kilo.vbody.core)"
start_body "$OUT/vbody.log" "$PYTHON" -u -m kilo.vbody.core
trap stop_body EXIT
set +e
"$HOST" --port "$PTY" --duration 3 2> "$OUT/host.log"
status=$?
set -e
stop_body
trap - EXIT
cat "$OUT/host.log"
[ "$status" -eq 0 ] || { echo "FAILED: exit status $status"; exit 1; }
expect "$OUT/host.log" "HELLO proto 2\.[0-9]+ build 45524f43 reset [A-Z_]+ config OK" "HELLO with protocol major 2"
expect "$OUT/host.log" "link: telemetry [1-9][0-9]*/s" "telemetry received"
expect "$OUT/host.log" "body STATS: uptime" "a STATS_REQ answered"
if grep -Eq "decode errors [1-9]" "$OUT/host.log"; then echo "FAILED: decode errors"; exit 1; fi

echo "== integration: the virtual body with its config defaulted (--config-state 2)"
start_body "$OUT/vbody_defaulted.log" "$PYTHON" -u -m kilo.vbody.core --config-state 2
trap stop_body EXIT
set +e
"$HOST" --port "$PTY" --duration 1 2> "$OUT/host_defaulted.log"
status=$?
set -e
stop_body
trap - EXIT
[ "$status" -eq 0 ] || { cat "$OUT/host_defaulted.log"; echo "FAILED: exit status $status"; exit 1; }
expect "$OUT/host_defaulted.log" "HELLO proto 2\.[0-9]+ .* config DEFAULTED" "HELLO config_state DEFAULTED"
expect "$OUT/host_defaulted.log" "event CONFIG_DEFAULTED" "EVENT CONFIG_DEFAULTED"

echo "== integration: a silent pty (no body) -> exit 1, no HELLO"
start_body "$OUT/silent.log" python3 -u "$HERE/tests/fake_body.py" --silent
trap stop_body EXIT
set +e
"$HOST" --port "$PTY" --duration 1 2> "$OUT/host_silent.log"
status=$?
set -e
stop_body
trap - EXIT
[ "$status" -eq 1 ] || { cat "$OUT/host_silent.log"; echo "FAILED: exit status $status, want 1"; exit 1; }
expect "$OUT/host_silent.log" "no HELLO from the body" "the no-HELLO error"

echo "== integration: a protocol-1 body -> exit 2, refusing to drive"
start_body "$OUT/proto1.log" python3 -u "$HERE/tests/fake_body.py" --proto-major 1
trap stop_body EXIT
set +e
"$HOST" --port "$PTY" --duration 2 2> "$OUT/host_proto1.log"
status=$?
set -e
stop_body
trap - EXIT
[ "$status" -eq 2 ] || { cat "$OUT/host_proto1.log"; echo "FAILED: exit status $status, want 2"; exit 1; }
expect "$OUT/host_proto1.log" "protocol major 1 != expected 2: refusing to drive; flash the body" "the refusal"

# ---- the laptop interface (docs/fast-boot.md P2): kilo --host against kilo-mind-host --listen ----
# The CLI and vbody come from this checkout's mind/ (PYTHONPATH), not the venv's editable install.
echo "== integration: kilo --host against kilo-mind-host --listen and the virtual body"
KPY() { PYTHONPATH="$REPO/mind${PYTHONPATH:+:$PYTHONPATH}" "$PYTHON" "$@"; }
# A throwaway key for this run (never the owner's ~/.config/kilo/host_key): kilo key init makes
# it, kilo key show gives the line kilo-mind-host --authorized-keys reads, as the kernel build does.
KEYDIR="$OUT/keys"
rm -rf "$KEYDIR" "$KEYDIR.line"
KPY -m kilo.cli --key "$KEYDIR/host_key" key init --name test@kernel-host 2> "$OUT/key_init.txt" > "$KEYDIR.line"
expect "$OUT/key_init.txt" "host_key made; fingerprint [0-9a-f]{16}" "kilo key init made the key"
[ "$(stat -c %a "$KEYDIR/host_key")" = 600 ] && [ "$(stat -c %a "$KEYDIR")" = 700 ] \
    || { echo "FAILED: the key's modes"; ls -la "$KEYDIR"; exit 1; }
KPY -m kilo.cli --key "$KEYDIR/host_key" key show 2> /dev/null > "$KEYDIR/authorized_keys"
cmp -s "$KEYDIR.line" "$KEYDIR/authorized_keys" || { echo "FAILED: key init and key show differ"; exit 1; }
export KILO_HOST_KEY="$KEYDIR/host_key"
KEYS="$KEYDIR/authorized_keys"
start_body "$OUT/vbody_host.log" env PYTHONPATH="$REPO/mind" "$PYTHON" -u -m kilo.vbody.core
"$HOST" --port "$PTY" --authorized-keys "$KEYS" --listen 127.0.0.1:0 2> "$OUT/host_listen.log" &
MIND_PID=$!
stop_all() { kill "$MIND_PID" 2>/dev/null || true; wait "$MIND_PID" 2>/dev/null || true; stop_body; }
trap stop_all EXIT
HPORT=""
for _ in $(seq 1 50); do
    HPORT="$(grep -o 'listening on port [0-9]*' "$OUT/host_listen.log" | grep -o '[0-9]*$' || true)"
    [ -n "$HPORT" ] && break
    sleep 0.1
done
[ -n "$HPORT" ] || { echo "FAILED: kilo-mind-host did not listen"; cat "$OUT/host_listen.log"; exit 1; }
H="127.0.0.1:$HPORT"
echo "   mind on $H"
sleep 1  # HELLO and the first telemetry

KPY -m kilo.cli --host "$H" status > "$OUT/cli_status.txt"
cat "$OUT/cli_status.txt"
expect "$OUT/cli_status.txt" "firmware +proto 2\.[0-9]+ +build 45524f43" "status: the body's HELLO over --host"
expect "$OUT/cli_status.txt" "link +connected=True alive=True" "status: the link alive"
KPY -m kilo.cli status --host "$H" --json > "$OUT/cli_status.json"
expect "$OUT/cli_status.json" '"mind_build": "[0-9a-f]{8}"' "status --json: the mind build"
expect "$OUT/cli_status.txt" "^guard +on +zone (green|blue|red)" "status: the distance guard's line"
KPY -m kilo.cli guard off --host "$H" > "$OUT/cli_guard_off.txt"
KPY -m kilo.cli guard --host "$H" --json > "$OUT/cli_guard.json"
KPY -m kilo.cli guard on --host "$H" > "$OUT/cli_guard_on.txt"
expect "$OUT/cli_guard_off.txt" "^guard +OFF" "kilo guard off --host"
expect "$OUT/cli_guard.json" '"enabled": 0' "kilo guard --json: still off"
expect "$OUT/cli_guard_on.txt" "^guard +on" "kilo guard on --host"
expect "$OUT/host_listen.log" "distance guard off" "the mind logs the guard switched"
expect "$OUT/host_listen.log" "noise: client [0-9]+: handshake with key [0-9a-f]{16} \(test@kernel-host\) in [0-9]+\.[0-9] ms, slowest step [0-9]+ us" \
    "the mind's handshake line"
grep -o "noise: client [0-9]*: handshake .*" "$OUT/host_listen.log" | head -1 | sed 's/^/   /'

echo "== integration: refused over --host (an unknown key, no key file, a key others can read, a plaintext frame)"
KPY -m kilo.cli --key "$KEYDIR/stranger_key" key init --name stranger 2> /dev/null > /dev/null
set +e
KPY -m kilo.cli --key "$KEYDIR/stranger_key" --host "$H" status > "$OUT/cli_stranger.txt" 2>&1
s1=$?
KPY -m kilo.cli --key "$KEYDIR/no_such_key" --host "$H" status > "$OUT/cli_nokey.txt" 2>&1
s2=$?
chmod 644 "$KEYDIR/stranger_key"
KPY -m kilo.cli --key "$KEYDIR/stranger_key" --host "$H" status > "$OUT/cli_unsafe.txt" 2>&1
s3=$?
set -e
sed 's/^/   /' "$OUT/cli_stranger.txt" "$OUT/cli_nokey.txt" "$OUT/cli_unsafe.txt"
[ "$s1" -eq 1 ] && [ "$s2" -eq 1 ] && [ "$s3" -eq 1 ] || { echo "FAILED: refusals exit $s1 $s2 $s3, want 1"; exit 1; }
expect "$OUT/cli_stranger.txt" "the mind does not know this laptop's key \([0-9a-f]{16}, .*stranger_key\)\. Add .kilo key show.'s line to kernel/keys/authorized_keys" \
    "an unknown key: what to do"
expect "$OUT/cli_nokey.txt" "no host key at .*no_such_key: create one with .kilo key init." "no key file: kilo key init"
expect "$OUT/cli_unsafe.txt" "can be read by others \(mode 0644\).*chmod 600" "a key file others can read"
expect "$OUT/host_listen.log" "host: client [0-9]+: key [0-9a-f]{16} is not authorized" "the mind's refusal"
KPY "$HERE/tests/plaintext_client.py" "$H" | tee "$OUT/plaintext.txt"
expect "$OUT/plaintext.txt" "the connection closed, nothing answered" "a plaintext frame: closed, unanswered"
expect "$OUT/host_listen.log" "closed: no host channel hello \(a plaintext or version 1 client\?\), unanswered" \
    "the mind's log of the plaintext frame"

KPY -m kilo.cli --host "$H" monitor --json --rate 10 --duration 2 > "$OUT/cli_monitor.jsonl"
n="$(grep -c '"mode": "' "$OUT/cli_monitor.jsonl" || true)"
echo "   monitor: $n telemetry records in 2 s at 10 Hz"
[ "$n" -ge 15 ] || { echo "FAILED: monitor over --host: $n records"; cat "$OUT/cli_monitor.jsonl"; exit 1; }

KPY -m kilo.cli --host "$H" posture stand --for 1 > "$OUT/cli_posture.txt"
expect "$OUT/cli_posture.txt" "holding stand" "posture stand over --host"
expect "$OUT/cli_posture.txt" "released" "posture: control released at the end"
expect "$OUT/host_listen.log" "host: client [0-9]+: posture 3" "the mind latched STAND"

KPY -m kilo.cli log --host "$H" --lines 30 --duration 1 > "$OUT/cli_log.txt"
expect "$OUT/cli_log.txt" "[0-9]+\.[0-9]{3} I host: client [0-9]+ gave up control \(released\): REST latched" \
    "log tail over --host: the history"
expect "$OUT/cli_log.txt" "I link: telemetry [1-9]" "log tail over --host: a live stats line"

# Controller loss -> REST, seen in the body's telemetry by an observer: hold LIMP
# (mode LIMP), kill -9 the holder (its socket closes), and the mind latches REST
# (mode IDLE, servos powered).
KPY -m kilo.cli --host "$H" monitor --json --rate 20 --duration 4 > "$OUT/cli_watch.jsonl" &
WATCH_PID=$!
sleep 0.3
# (env, not KPY: $! must be python itself, not a subshell running it)
env PYTHONPATH="$REPO/mind" "$PYTHON" -m kilo.cli --host "$H" posture limp > "$OUT/cli_limp.txt" 2>&1 &
LIMP_PID=$!
sleep 1.5
kill -9 "$LIMP_PID"
wait "$LIMP_PID" 2>/dev/null || true
wait "$WATCH_PID"
KPY - "$OUT/cli_watch.jsonl" <<'EOF'
import json, sys
modes = [json.loads(x)["mode"] for x in open(sys.argv[1])]
first_limp = modes.index("LIMP") if "LIMP" in modes else None
assert first_limp is not None, f"never LIMP: {modes}"
after = modes[first_limp:]
assert "IDLE" in after, f"no REST (IDLE) after the controller died: {modes}"
print(f"   controller killed: modes {modes[0]} -> LIMP x{after.count('LIMP')} -> IDLE (REST)")
EOF
expect "$OUT/host_listen.log" "host: client [0-9]+ gave up control \(closed\): REST latched" "REST on the controller's loss"

echo "== integration: the raw bridge (kilo.link through BridgeTransport)"
KPY "$HERE/tests/bridge_client.py" "$H" | tee "$OUT/bridge.txt"
expect "$OUT/bridge.txt" "bridge: HELLO proto 2\.[0-9]+ build 45524f43" "HELLO through the bridge"
expect "$OUT/bridge.txt" "bridge: STATS uptime" "STATS through the bridge"
expect "$OUT/bridge.txt" "bridge closed: the mind's link has the body again" "the link resumed"
expect "$OUT/host_listen.log" "host: bridge closed \(BRIDGE_CLOSE\); link resumed" "the mind's bridge log"

kill -INT "$MIND_PID"
wait "$MIND_PID" || { echo "FAILED: kilo-mind-host exit status $?"; cat "$OUT/host_listen.log"; exit 1; }
stop_body
trap - EXIT
if grep -Eq "decode errors [1-9]" "$OUT/host_listen.log"; then echo "FAILED: decode errors"; exit 1; fi

# REBOOT: kilo reboot --host takes control, the mind answers OK, latches REST
# and kilo-mind-host exits with status 3 (the Circle kernel reboots the Pi there).
echo "== integration: kilo reboot --host (kilo-mind-host exits 3)"
start_body "$OUT/vbody_reboot.log" env PYTHONPATH="$REPO/mind" "$PYTHON" -u -m kilo.vbody.core
"$HOST" --port "$PTY" --authorized-keys "$KEYS" --listen 127.0.0.1:0 2> "$OUT/host_reboot.log" &
MIND_PID=$!
trap stop_all EXIT
HPORT=""
for _ in $(seq 1 50); do
    HPORT="$(grep -o 'listening on port [0-9]*' "$OUT/host_reboot.log" | grep -o '[0-9]*$' || true)"
    [ -n "$HPORT" ] && break
    sleep 0.1
done
[ -n "$HPORT" ] || { echo "FAILED: kilo-mind-host did not listen"; cat "$OUT/host_reboot.log"; exit 1; }
sleep 1  # HELLO
KPY -m kilo.cli reboot --host "127.0.0.1:$HPORT" > "$OUT/cli_reboot.txt" 2>&1 \
    || { echo "FAILED: kilo reboot --host exit $?"; cat "$OUT/cli_reboot.txt" "$OUT/host_reboot.log"; exit 1; }
cat "$OUT/cli_reboot.txt"
set +e
wait "$MIND_PID"
status=$?
set -e
stop_body
trap - EXIT
[ "$status" -eq 3 ] || { echo "FAILED: kilo-mind-host exit status $status, not 3"; cat "$OUT/host_reboot.log"; exit 1; }
echo "   kilo-mind-host exit status $status"
expect "$OUT/cli_reboot.txt" "rebooting" "the CLI: OK, then the connection closed"
expect "$OUT/host_reboot.log" "host: client [0-9]+ asked for a reboot \(REST latched\)" "REST before the reboot"
expect "$OUT/host_reboot.log" "REBOOT from the laptop: exiting with status 3" "the host program's line"

# Kernel update: kilo update --host writes a fake kernel (with the marker) into
# kilo-mind-host --card DIR, which stands for the SD card. The mind installs it
# (kernel7.img the new one, the old one kept as kernel7.old) and exits 3 as for
# REBOOT; the CLI waits for the mind to come back (restarted here on the same
# port, without --card) and prints its build. Before that, an image without
# the marker is refused by the CLI and by the mind, and the card is unchanged.
echo "== integration: kilo update --host into kilo-mind-host --card (exits 3, back on the same port)"
UP="$OUT/update"
rm -rf "$UP"
mkdir -p "$UP/card"
printf 'OLD KERNEL\n' > "$UP/card/kernel7.img"
printf 'STALE\n' > "$UP/card/kernel7.old"
cp "$REPO/kernel/boot/config.txt" "$UP/card/config.txt"
KPY - "$UP" <<'EOF'
import sys
from kilo import hostkey
from kilo.protocol import kilo_protocol as P
pattern = bytes((i * 131 + 7) % 256 for i in range(200000))
marker = P.HOST_UPDATE_MARKER.encode()
pub = hostkey.load().public  # built in, as kernel/keys/gen_keys.py puts the keys into a real kernel
img = pattern[:150000] + marker + pattern[150000 + len(marker):]
open(f"{sys.argv[1]}/kernel7.img", "wb").write(img[:100000] + pub + img[100000 + len(pub):])
open(f"{sys.argv[1]}/nokey.img", "wb").write(img)
open(f"{sys.argv[1]}/loader.img", "wb").write(pattern[:60000])
EOF
start_body "$OUT/vbody_update.log" env PYTHONPATH="$REPO/mind" "$PYTHON" -u -m kilo.vbody.core
"$HOST" --port "$PTY" --authorized-keys "$KEYS" --listen 127.0.0.1:0 --card "$UP/card" 2> "$UP/mind.log" &
MIND_PID=$!
trap stop_all EXIT
HPORT=""
for _ in $(seq 1 50); do
    HPORT="$(grep -o 'listening on port [0-9]*' "$UP/mind.log" | grep -o '[0-9]*$' || true)"
    [ -n "$HPORT" ] && break
    sleep 0.1
done
[ -n "$HPORT" ] || { echo "FAILED: kilo-mind-host did not listen"; cat "$UP/mind.log"; exit 1; }
H="127.0.0.1:$HPORT"
sleep 1  # HELLO
set +e
KPY -m kilo.cli update --host "$H" --image "$UP/loader.img" > "$UP/cli_loader.txt" 2>&1
s1=$?
set -e
[ "$s1" -eq 1 ] || { echo "FAILED: kilo update of an image without the marker: exit $s1"; cat "$UP/cli_loader.txt"; exit 1; }
expect "$UP/cli_loader.txt" "has no KILO-MIND-KERNEL-1: it is not the mind kernel" "the CLI's own marker check"
set +e
KPY -m kilo.cli update --host "$H" --image "$UP/nokey.img" > "$UP/cli_nokey.txt" 2>&1
s1=$?
set -e
[ "$s1" -eq 1 ] || { echo "FAILED: kilo update of a kernel without this laptop's key: exit $s1"; cat "$UP/cli_nokey.txt"; exit 1; }
expect "$UP/cli_nokey.txt" "does not have this laptop's key \([0-9a-f]{16}\) built in" "the CLI's key check"
KPY - "$H" "$UP/loader.img" <<'EOF' | tee "$UP/loader_mind.txt"
import sys
from kilo.hostlink import HostLink, HostRefused
from kilo.protocol import kilo_protocol as P
seen = []
with HostLink.connect(sys.argv[1]) as hl:
    try:
        hl.update(open(sys.argv[2], "rb").read(), progress=seen.append)
        raise SystemExit("the mind installed an image without the marker")
    except HostRefused as e:
        assert e.code == P.HostResultCode.IMAGE_BAD and "NOT_A_KERNEL" in e.text, e
        why = str(e)
    assert seen and seen[-1].phase == P.UpdatePhase.FAILED and seen[-1].error == P.UpdateError.NOT_A_KERNEL, seen
    hl.release_control()
print(f"   the mind refused it: {why}")
EOF
[ "$(cat "$UP/card/kernel7.img")" = "OLD KERNEL" ] && [ ! -e "$UP/card/kernel7.new" ] \
    || { echo "FAILED: a refused image changed the card"; ls -l "$UP/card"; exit 1; }
KPY -m kilo.cli update --host "$H" --image "$UP/kernel7.img" --wait 20 > "$UP/cli_update.txt" 2>&1 &
CLI_PID=$!
set +e
wait "$MIND_PID"
status=$?
set -e
[ "$status" -eq 3 ] || { echo "FAILED: kilo-mind-host exit status $status, not 3"; cat "$UP/mind.log"; exit 1; }
echo "   kilo-mind-host exit status $status after the update; restarting it on port $HPORT"
"$HOST" --port "$PTY" --authorized-keys "$KEYS" --listen "$H" 2> "$UP/mind2.log" &
MIND_PID=$!
set +e
wait "$CLI_PID"
s2=$?
set -e
cat "$UP/cli_update.txt"
[ "$s2" -eq 0 ] || { echo "FAILED: kilo update --host exit $s2"; cat "$UP/mind.log" "$UP/mind2.log"; exit 1; }
KPY - "$H" <<'EOF' | tee "$UP/nocard.txt"
import sys
from kilo.hostlink import HostLink, HostRefused
with HostLink.connect(sys.argv[1]) as hl:
    try:
        hl.update(b"x" * 100, force=True)
        raise SystemExit("kilo-mind-host without --card took an update")
    except HostRefused as e:
        assert e.code_name == "UNSUPPORTED" and "--card" in e.text, e
        print(f"   without --card: {e}")
EOF
kill -INT "$MIND_PID"
wait "$MIND_PID" || true
stop_body
trap - EXIT
cmp "$UP/kernel7.img" "$UP/card/kernel7.img" || { echo "FAILED: kernel7.img is not the image sent"; exit 1; }
[ "$(cat "$UP/card/kernel7.old")" = "OLD KERNEL" ] || { echo "FAILED: kernel7.old is not the previous kernel"; exit 1; }
[ ! -e "$UP/card/kernel7.new" ] || { echo "FAILED: kernel7.new left on the card"; exit 1; }
cmp "$REPO/kernel/boot/config.txt" "$UP/card/config.txt" || { echo "FAILED: config.txt changed"; exit 1; }
echo "   the card: kernel7.img the new kernel, kernel7.old the previous one, no kernel7.new"
expect "$UP/cli_update.txt" "update: writing kernel7.new: [0-9]+ of 200000 bytes" "progress: writing"
expect "$UP/cli_update.txt" "update: kernel7.new written \(200000 bytes\); reading it back" "progress: written"
expect "$UP/cli_update.txt" "update: kernel7.new read back: 200000 bytes, crc32 [0-9a-f]{8}; installing" "progress: read back"
expect "$UP/cli_update.txt" "update: DONE: kernel7.img on the SD card is the new kernel \(200000 bytes" "DONE"
expect "$UP/cli_update.txt" "update: the mind is rebooting into it" "the reboot announced"
expect "$UP/cli_update.txt" "update: the mind is back after [0-9.]+ s: mind build [0-9a-f]{8}" "the mind's build after"
expect "$UP/mind.log" "update: kernel7.new read back: 200000 bytes" "the mind's read-back"
expect "$UP/mind.log" "update: kernel7.img -> kernel7.old" "the first rename"
expect "$UP/mind.log" "update: installed: kernel7.img is the new kernel" "installed"
expect "$UP/mind.log" "kernel updated on the SD card: rebooting into it \(REST latched\)" "REST before the reboot"
expect "$UP/mind.log" "kernel update installed in --card: exiting with status 3" "the host program's line"

echo "== integration: walk-demo trots the virtual body for 3 s and stops"
start_body "$OUT/vbody_walk.log" "$PYTHON" -u -m kilo.vbody.core
trap stop_body EXIT
set +e
"$HERE/build/walk-demo" --port "$PTY" --seconds 3 2> "$OUT/walk.log"
status=$?
set -e
stop_body
trap - EXIT
cat "$OUT/walk.log"
[ "$status" -eq 0 ] || { echo "FAILED: walk-demo exit status $status"; exit 1; }
expect "$OUT/walk.log" "walk trot forward at 144\.0 mm/s" "the walk started"
expect "$OUT/walk.log" "walked: .* ended: stopped$" "the planned stop"

# ---- body flashing (docs/fast-boot.md P4): kilo flash --host / --sd against kilo-mind-host --reset-fifo ----
# The body is kernel/host/tests/fake_optiboot.py: the virtual body until RESET,
# then optiboot's STK500v1, then the virtual body again reporting --new-build.
FL="$OUT/flash"
rm -rf "$FL"; mkdir -p "$FL"
FLASH_BUILD=3d441ec0
KPY - "$FL" "$FLASH_BUILD" <<'EOF'
import random, sys
from pathlib import Path
from kilo.tools import body_image as BI
out, build = Path(sys.argv[1]), int(sys.argv[2], 16)
data = random.Random(4).randbytes(31234)        # 245 pages, the last one partial
m = BI.make_manifest(data, build)
BI.write_image(out, data, m)
bad = BI.Manifest(build=build, size=m.size, crc32=m.crc32 ^ 0x80)
(out / "bad").mkdir()
BI.write_image(out / "bad", data, bad)
print(f"   image: {m.size} bytes, build {m.build:08x}, crc32 {m.crc32:08x}")
EOF

# Start the fake body (sets BODY_PID, PTY) and a mind on it with --listen and --reset-fifo (MIND_PID, H).
start_flash_rig() {  # start_flash_rig <name> <fake_optiboot args...>
    local name="$1"; shift
    start_body "$FL/$name.body.log" env PYTHONPATH="$REPO/mind" "$PYTHON" -u "$HERE/tests/fake_optiboot.py" \
        --reset-fifo "$FL/$name.fifo" "$@"
    "$HOST" --port "$PTY" --authorized-keys "$KEYS" --listen 127.0.0.1:0 --reset-fifo "$FL/$name.fifo" 2> "$FL/$name.mind.log" &
    MIND_PID=$!
    HPORT=""
    for _ in $(seq 1 50); do
        HPORT="$(grep -o 'listening on port [0-9]*' "$FL/$name.mind.log" | grep -o '[0-9]*$' || true)"
        [ -n "$HPORT" ] && break
        sleep 0.1
    done
    [ -n "$HPORT" ] || { echo "FAILED: kilo-mind-host did not listen"; cat "$FL/$name.mind.log"; exit 1; }
    H="127.0.0.1:$HPORT"
    sleep 1  # HELLO
}

echo "== integration: kilo flash --host (STK500 through the mind to a fake optiboot)"
start_flash_rig ok --new-build "$FLASH_BUILD" --image-out "$FL/programmed.bin"
trap stop_all EXIT
KPY -m kilo.cli --host "$H" status > "$FL/status_before.txt"
expect "$FL/status_before.txt" "firmware +proto 2\.[0-9]+ +build 56424f44" "the virtual body's build before"
set +e
KPY -m kilo.cli flash --host "$H" --image "$FL/body.bin" > "$FL/flash_ok.txt" 2>&1
status=$?
set -e
cat "$FL/flash_ok.txt"
[ "$status" -eq 0 ] || { echo "FAILED: kilo flash --host exit $status"; cat "$FL/ok.mind.log" "$FL/ok.body.log"; exit 1; }
expect "$FL/flash_ok.txt" "flash: programming done, 245 pages" "progress: every page programmed"
expect "$FL/flash_ok.txt" "flash: verifying done, 245 pages" "progress: every page verified"
expect "$FL/flash_ok.txt" "flash: done; HELLO build $FLASH_BUILD reset FLASHED" "FLASH_STATUS DONE"
expect "$FL/flash_ok.txt" "verified: HELLO build $FLASH_BUILD reset FLASHED" "the end check: HELLO.fw_build"
expect "$FL/ok.body.log" "optiboot: LEAVE_PROGMODE, 245 pages programmed" "the fake was programmed"
expect "$FL/ok.mind.log" "flash: done in [0-9]+ ms: HELLO build $FLASH_BUILD reset FLASHED" "the mind's log"
KPY - "$FL/body.bin" "$FL/programmed.bin" <<'EOF'
import sys
img, prog = open(sys.argv[1], "rb").read(), open(sys.argv[2], "rb").read()
assert prog[:len(img)] == img and set(prog[len(img):]) <= {0xFF} and len(prog) % 128 == 0, "flash != body.bin"
print(f"   the fake's flash holds body.bin ({len(img)} bytes, padded to {len(prog)})")
EOF
KPY -m kilo.cli --host "$H" status > "$FL/status_after.txt"
expect "$FL/status_after.txt" "build $FLASH_BUILD +reset FLASHED" "status after: the new firmware"
expect "$FL/status_after.txt" "link +connected=True alive=True" "status after: the link alive"

echo "== integration: a CRC that does not match is refused (locally, and by the mind)"
set +e
KPY -m kilo.cli flash --host "$H" --image "$FL/bad/body.bin" > "$FL/flash_bad.txt" 2>&1
status=$?
set -e
cat "$FL/flash_bad.txt"
[ "$status" -eq 1 ] || { echo "FAILED: a bad CRC: exit $status, want 1"; exit 1; }
expect "$FL/flash_bad.txt" "crc32 mismatch" "the CLI's own check"
KPY - "$H" "$FL/body.bin" "$FLASH_BUILD" <<'EOF' | tee "$FL/flash_bad_mind.txt"
import sys, zlib
from kilo.hostlink import HostLink, HostRefused
from kilo.protocol import kilo_protocol as P
data = open(sys.argv[2], "rb").read()
seen = []
with HostLink.connect(sys.argv[1]) as hl:
    try:
        hl.flash(data, int(sys.argv[3], 16), zlib.crc32(data) ^ 1, on_status=seen.append)
        raise SystemExit("the mind flashed an image with a wrong CRC")
    except HostRefused as e:
        print(f"   refused: {e}")
        assert e.code == P.HostResultCode.IMAGE_BAD, e
    assert seen and seen[-1].phase == P.FlashPhase.FAILED and seen[-1].error == P.FlashError.CRC_MISMATCH, seen
    print(f"   FLASH_STATUS FAILED {P.FlashError(seen[-1].error).name}, {seen[-1].received} bytes received")
    st = hl.status()
    assert not st.state & P.HostState.FLASHING and st.state & P.HostState.BODY_ALIVE, st.state
    hl.release_control()
EOF
expect "$FL/flash_bad_mind.txt" "refused: FLASH_COMMIT: IMAGE_BAD \(CRC_MISMATCH" "RESULT IMAGE_BAD"
expect "$FL/ok.mind.log" "body image refused: CRC_MISMATCH" "the mind's log"
kill -INT "$MIND_PID"
wait "$MIND_PID" || { echo "FAILED: kilo-mind-host exit status $?"; cat "$FL/ok.mind.log"; exit 1; }
stop_body
trap - EXIT

echo "== integration: no bootloader answers: SYNC_TIMEOUT, and the link works again"
start_flash_rig nosync --no-bootloader
trap stop_all EXIT
set +e
KPY -m kilo.cli flash --host "$H" --image "$FL/body.bin" > "$FL/flash_nosync.txt" 2>&1
status=$?
set -e
cat "$FL/flash_nosync.txt"
[ "$status" -eq 1 ] || { echo "FAILED: no bootloader: exit $status, want 1"; cat "$FL/nosync.mind.log"; exit 1; }
expect "$FL/flash_nosync.txt" "FAILED: failed \(attempt 2\): SYNC_TIMEOUT \(STK500 command 30\)" "SYNC_TIMEOUT after 2 attempts"
expect "$FL/nosync.mind.log" "flash: FAILED: SYNC_TIMEOUT after 2 attempt\(s\)" "the mind's log"
sleep 1
KPY -m kilo.cli --host "$H" status > "$FL/status_nosync.txt"
cat "$FL/status_nosync.txt"
expect "$FL/status_nosync.txt" "build 56424f44 +reset WARM_UNEXPECTED" "the old firmware after the resets"
expect "$FL/status_nosync.txt" "link +connected=True alive=True" "the link works again"
KPY -m kilo.cli --host "$H" posture stand --for 0.5 > "$FL/posture_nosync.txt"
expect "$FL/posture_nosync.txt" "holding stand" "the body can be driven again"
kill -INT "$MIND_PID"
wait "$MIND_PID" || { echo "FAILED: kilo-mind-host exit status $?"; cat "$FL/nosync.mind.log"; exit 1; }
stop_body
trap - EXIT
if grep -Eq "decode errors [1-9]" "$FL/ok.mind.log" "$FL/nosync.mind.log"; then echo "FAILED: decode errors"; exit 1; fi

echo "== kilo flash --sd: onto a directory that looks like the boot card (--force-dir, as make-sd.sh)"
mkdir -p "$FL/card" "$FL/notcard"
touch "$FL/card/kernel7.img" "$FL/card/config.txt" "$FL/notcard/config.txt"
KPY -m kilo.cli flash --sd "$FL/card" --force-dir --image "$FL/body.bin" | tee "$FL/sd.txt"
cmp "$FL/body.bin" "$FL/card/body.bin" && cmp "$FL/body.json" "$FL/card/body.json" \
    || { echo "FAILED: the card's files differ"; exit 1; }
expect "$FL/sd.txt" "build $FLASH_BUILD" "--sd: the manifest's build"
set +e
KPY -m kilo.cli flash --sd "$FL/notcard" --force-dir --image "$FL/body.bin" > "$FL/sd_refused.txt" 2>&1
s1=$?
KPY -m kilo.cli flash --sd "$FL/card" --image "$FL/body.bin" > "$FL/sd_refused_mount.txt" 2>&1
s2=$?
set -e
[ "$s1" -eq 1 ] && [ "$s2" -eq 1 ] || { echo "FAILED: --sd refusals: exit $s1, $s2"; exit 1; }
expect "$FL/sd_refused.txt" "has no kernel7.img: not a Kilo boot card" "--sd: not our card"
expect "$FL/sd_refused_mount.txt" "is not a mount point" "--sd: not a mount"
[ ! -e "$FL/notcard/body.bin" ] || { echo "FAILED: wrote to a card it refused"; exit 1; }
echo "   --sd: wrote the card; refused a directory without kernel7.img and an unmounted one"

# ---- P3 (docs/fast-boot.md): tricks, perception, behaviour, remote and the Mind ----------------
echo "== P3: generated tables up to date (tricks, personality)"
"$PYTHON" "$REPO/kernel/mind/tricks/gen_library.py" --check  # (each generator imports this checkout's mind/)
"$PYTHON" "$REPO/kernel/mind/behaviour/gen_personality.py" --check
echo "== P3: golden vectors up to date with the Python mind (perception, behaviour)"
for v in perception behaviour; do
    "$PYTHON" "$HERE/tests/gen_${v}_vectors.py" | cmp -s - "$HERE/tests/${v}_vectors.txt" ||
        { echo "FAILED: stale; run: $PYTHON kernel/host/tests/gen_${v}_vectors.py > kernel/host/tests/${v}_vectors.txt"; exit 1; }
done

# The virtual body from this checkout's mind/ with its control FIFO (CoreBody.serve_control):
# `ctl ir 0x10` sends EVENT IR_KEY, `ctl att 40 0` tilts the scripted IMU.
FIFO="$OUT/control.fifo"
start_ctl_body() {  # start_ctl_body <log>
    rm -f "$FIFO"
    start_body "$1" env PYTHONPATH="$REPO/mind" "$PYTHON" -u -m kilo.vbody.core --control "$FIFO"
    for _ in $(seq 1 50); do [ -p "$FIFO" ] && return 0; sleep 0.1; done
    echo "FAILED: no control FIFO"; cat "$1"; exit 1
}
ctl() { timeout 5 bash -c "echo '$*' > '$FIFO'" || { echo "FAILED: the control FIFO did not take '$*'"; exit 1; }; }
run_mind() {  # run_mind <log> <args...>: kilo-mind-host in the background, MIND_PID
    local log="$1"; shift
    "$HOST" --port "$PTY" "$@" 2> "$log" &
    MIND_PID=$!
}
wait_mind() {  # wait_mind <log>: it must exit 0
    wait "$MIND_PID" || { echo "FAILED: kilo-mind-host exit status $?"; cat "$1"; exit 1; }
}

echo "== P3 integration: IR keys from the virtual body -> stand, trot, faster, slower, turn, stand, freeze, rest, calibration posture, behaviour (--remote)"
start_ctl_body "$OUT/vbody_ir.log"
trap stop_body EXIT
run_mind "$OUT/host_ir.log" --remote --seed 3 --duration 21
sleep 1
ctl ir 0x02                   # B6: stand (from LIMP)
sleep 1
ctl ir 0x68                   # A4: trot forward at 1x
sleep 2
ctl ir 0x62                   # B7: faster (1.4x)
sleep 0.5
ctl ir 0xA8                   # B5: slower (1x)
sleep 0.5
ctl ir 0x22                   # A6: turn left
sleep 1
ctl ir 0x18                   # B3: row 3, nothing
ctl ir 0x38                   # B2: nothing either (the rear-up key is off without --rear-up)
ctl ir 0x02                   # B6: stand (the planned stop, then the posture)
sleep 4
ctl ir 0x68                   # A4: trot forward again, at 1x
sleep 2
ctl ir 0xE2                   # C7: freeze (the walk aborted, the body settles in STANCE)
sleep 1
ctl ir 0x62                   # B7: ignored while frozen
sleep 0.3
ctl ir 0xA2                   # A7: rest
sleep 1.5
ctl ir 0x90                   # C5: the calibration posture
sleep 1.5
ctl ir 0xE0                   # A5: the behaviour runner
sleep 1.5
ctl ir 0x02                   # B6: stops the runner, stands
wait_mind "$OUT/host_ir.log"
stop_body
trap - EXIT
grep -E "remote|trick|walk|behaviour" "$OUT/host_ir.log"
expect "$OUT/host_ir.log" "remote B6 \(0x02, stand\): posture stand" "B6 stood"
expect "$OUT/host_ir.log" "remote A4 \(0x68, trot\): walk forward \(trot\) 144\.0 mm/s" "A4 walked"
expect "$OUT/host_ir.log" "walk trot forward at 144\.0 mm/s" "the trot started"
expect "$OUT/host_ir.log" "remote B7 \(0x62, faster\): speed 1\.15x \(trot 165\.6 mm/s forward\)" "B7 sped up"
expect "$OUT/host_ir.log" "remote B5 \(0xA8, slower\): speed 1x \(trot 144\.0 mm/s forward\)" "B5 slowed"
expect "$OUT/host_ir.log" "remote A6 \(0x22, turn left\): walk left \(trot\) 144\.0 mm/s" "A6 turned"
expect "$OUT/host_ir.log" "remote B3 \(0x18, none\): ignored \(no action on this key\)" "B3 did nothing"
expect "$OUT/host_ir.log" "remote B2 \(0x38, none\): ignored \(no action on this key\)" "B2 did nothing (rear-up key off)"
expect "$OUT/host_ir.log" "walk ended: stopped" "the planned stop"
expect "$OUT/host_ir.log" "remote C7 \(0xE2, freeze\): frozen \(walk aborted\): only B6 or A7 act now" "C7 froze"
expect "$OUT/host_ir.log" "I walk ended: aborted \\(C7 freeze\\)" "the freeze aborted the walk"
expect "$OUT/host_ir.log" "remote B7 \(0x62, faster\): ignored \(frozen: B6 or A7 to go on\)" "B7 ignored while frozen"
expect "$OUT/host_ir.log" "remote A7 \(0xA2, sleep\): unfrozen, posture rest" "A7 rested"
expect "$OUT/host_ir.log" "remote C5 \(0x90, calibration posture\): posture calib" "C5 took the calibration posture"
expect "$OUT/host_ir.log" "remote A5 \(0xE0, behaviour\): behaviour runner started" "A5 started the runner"
expect "$OUT/host_ir.log" "behaviour: idle \(free\)" "the runner ran"
expect "$OUT/host_ir.log" "remote B6 \(0x02, stand\): behaviour runner stopped, posture stand" "B6 stopped the runner"
if grep -Eq "decode errors [1-9]|remote .*failed|no longer stamping" "$OUT/host_ir.log"; then echo "FAILED: errors"; exit 1; fi
grep -q "event GAIT_STOPPED arg=1" "$OUT/host_ir.log" &&
    echo "   the freeze: the body settled after the FEET underrun (GAIT_STOPPED arg 1)"

echo "== P3 integration: the behaviour runner idles for 5 s (--behaviour)"
start_body "$OUT/vbody_bh.log" env PYTHONPATH="$REPO/mind" "$PYTHON" -u -m kilo.vbody.core
trap stop_body EXIT
"$HOST" --port "$PTY" --behaviour --seed 3 --duration 5 2> "$OUT/host_bh.log" ||
    { echo "FAILED: kilo-mind-host exit status $?"; cat "$OUT/host_bh.log"; exit 1; }
stop_body
trap - EXIT
grep -E "behaviour" "$OUT/host_bh.log"
expect "$OUT/host_bh.log" "behaviour runner: seed 3, personality kilo" "the runner started"
expect "$OUT/host_bh.log" "behaviour: idle \(free\)" "idle chosen"
expect "$OUT/host_bh.log" "behaviour: posture sit" "idle sat down through the link"
ticks="$(grep -o 'behaviour: [0-9]* ticks' "$OUT/host_bh.log" | grep -o '[0-9]*')"
[ -n "$ticks" ] && [ "$ticks" -ge 45 ] && [ "$ticks" -le 52 ] || { echo "FAILED: $ticks ticks in 5 s, want 10 Hz"; exit 1; }
if grep -Eq "fallen|low_battery|wind_down|refused|no longer stamping" "$OUT/host_bh.log"; then
    echo "FAILED: the idle run did not stay idle"; exit 1
fi

echo "== P3 integration: perception and the behaviour runner react to the virtual body's IMU (tilt, fall)"
start_ctl_body "$OUT/vbody_pc.log"
trap stop_body EXIT
run_mind "$OUT/host_pc.log" --behaviour --seed 3 --duration 9
sleep 2
ctl att 40 0                  # nose down 40 deg: tilted (> 25 for 0.5 s)
sleep 1.5
ctl att 0 0                   # level again
sleep 2
ctl att 85 0                  # past the body's 75 deg: its fall reflex (EVENT FELL)
sleep 1.5
ctl att 0 0
wait_mind "$OUT/host_pc.log"
stop_body
trap - EXIT
grep -E "percept|FELL|behaviour: [a-z_]+ \(" "$OUT/host_pc.log"
expect "$OUT/host_pc.log" "percept tilted nose_down" "the tilt seen"
expect "$OUT/host_pc.log" "percept level" "level again"
expect "$OUT/host_pc.log" "event FELL arg=2" "the body's fall reflex"
expect "$OUT/host_pc.log" "behaviour: fallen \(safety\)" "the fallen safety branch"
# ---- end P3 ---------------------------------------------------------------------------------------

# ---- walk recordings (kernel/mind/record/): walk by the remote, fetch over --host, analyse ----------
# The virtual body's IMU is scripted: it reports a fixed tilt (ctl att 4 -3) whatever the legs do,
# so measured - predicted must be that tilt less the prediction, tick by tick, and its mean the
# offset. A second walk (trot) runs while the first is downloaded: it must lose nothing.
echo "== walk recordings: a walk (C4) by the remote, kilo runs --host (list, fetch during a trot), kilo log walk"
start_ctl_body "$OUT/vbody_walks.log"
trap stop_body EXIT
WALKS="$OUT/walks"
rm -rf "$WALKS"
run_mind "$OUT/host_walks.log" --remote --balance 0.99 --seed 3 --authorized-keys "$KEYS" --listen 127.0.0.1:0 --duration 40
trap 'kill $MIND_PID 2>/dev/null || true; stop_body' EXIT
WPORT=""
for _ in $(seq 1 50); do
    WPORT="$(grep -o 'listening on port [0-9]*' "$OUT/host_walks.log" | grep -o '[0-9]*$' || true)"
    [ -n "$WPORT" ] && break
    sleep 0.1
done
[ -n "$WPORT" ] || { echo "FAILED: kilo-mind-host did not listen"; cat "$OUT/host_walks.log"; exit 1; }
W="127.0.0.1:$WPORT"
sleep 1
ctl ir 0x02                   # B6: stand
sleep 1
ctl att 4 -3                  # the IMU reports nose down 4, left side up 3 (degrees) from now on
ctl ir 0xB0                   # C4: walk (one leg at a time: the plan leans, its prediction moves)
sleep 6
ctl ir 0x02                   # B6: the planned stop, then stand
for _ in $(seq 1 60); do grep -q "walk ended" "$OUT/host_walks.log" && break; sleep 0.25; done
sleep 2.5                     # the recording's post-roll
KPY -m kilo.cli --host "$W" runs | tee "$OUT/walks_list.txt"
expect "$OUT/walks_list.txt" "1 runs this boot \(1 walks, 0 tricks\)" "the list: one walk"
expect "$OUT/walks_list.txt" "^1 +[0-9-]+ [0-9:]+ +walk +walk forward .* s +[0-9.]+ KB +RAM +stopped" \
    "the list: run 1, walk forward, in RAM"
ctl ir 0x68                   # A4: trot, while walk 1 is downloaded
sleep 0.5
KPY -m kilo.cli --host "$W" walks --fetch all --out "$WALKS" -q 2>&1 | tee "$OUT/walks_fetch.txt"
expect "$OUT/walks_fetch.txt" "walk 1: [0-9]+ bytes in [0-9.]+ s -> .*run-[0-9-]+-walk-1\.krec \(\+ run-[0-9-]+-walk-1\.kilolog\)" \
    "fetch: walk 1 written (kilo walks: the old name of kilo runs)"
expect "$OUT/walks_fetch.txt" "run 2: still being recorded" "fetch: the trot is not fetchable yet"
sleep 1.5
ctl ir 0x02                   # B6: stop the trot
wait_mind "$OUT/host_walks.log"
stop_body
trap - EXIT
grep -E "walk recorder|walk (walk|trot) |walk ended|walk error|loop: " "$OUT/host_walks.log" | grep -v "loop: " | sed 's/^/   /'
expect "$OUT/host_walks.log" "flight recorder: [0-9]+ MB of RAM" "the flight recorder's store"
expect "$OUT/host_walks.log" "I walk ended: stopped \(sent [0-9]+ skipped 0 underruns 0\)" \
    "the trot during the download: no tick skipped, no underrun"
expect "$OUT/host_walks.log" "I walk walk forward .* holds [0-9]+ catches [0-9]+ waits [0-9]+ min margin -?[0-9.]+ mm" \
    "the walk's summary line"
expect "$OUT/host_walks.log" "I walk error: [0-9]+ samples offset pitch" "the tracker's summary line"
KWALK="$(ls "$WALKS"/run-*-walk-1.krec)"
KPY -m kilo.cli log walk "$KWALK" --csv "$OUT/walk1.csv" | tee "$OUT/walk1_summary.txt"
KPY -m kilo.cli log walk "$KWALK" --json > "$OUT/walk1.json"
KPY -m kilo.cli log summary "${KWALK%.krec}.kilolog" > "$OUT/walk1_kilolog.txt"
expect "$OUT/walk1_kilolog.txt" "gait: walking [0-9.]+s, 1 GAIT_STOPPED" "kilo log summary reads the .kilolog"
expect "$OUT/walk1_kilolog.txt" "model error: offset pitch \+[0-9]+\.[0-9]" "kilo log summary: an offset (the scripted tilt less the mean prediction)"
"$PYTHON" - "$OUT/walk1.json" "$OUT/walk1.csv" <<'EOF'
import csv, json, sys
s = json.load(open(sys.argv[1]))
e, lag, res = s["error"], s["lag"], s["result"]
rows = list(csv.DictReader(open(sys.argv[2])))
def check(cond, what):
    if not cond:
        print(f"FAILED: kilo log walk: {what}\n{json.dumps(s, indent=1)[:3000]}"); sys.exit(1)
check(e["samples"] >= 250, f"{e['samples']} GAIT ticks with a prediction")
check(abs(e["meas_mean_pitch"] - 4.0) < 1e-6 and abs(e["meas_mean_roll"] + 3.0) < 1e-6, "measured = the scripted tilt")
check(abs(e["offset_pitch"] - (4.0 - e["pred_mean_pitch"])) <= 0.011 and
      abs(e["offset_roll"] - (-3.0 - e["pred_mean_roll"])) <= 0.011, "offset = measured - predicted, on average")
check(e["rms_pitch"] >= abs(e["offset_pitch"]) - 0.01 and e["rms_roll"] >= abs(e["offset_roll"]) - 0.01, "rms >= |offset|")
check({"RF+LB", "LF+RB"} <= set(s["by_phase"]), f"the walk's diagonal couplets: {sorted(s['by_phase'])}")
check(s["ticks_sent"] == int(res["sent"]) and int(res["skipped"]) == 0 and int(res["underruns"]) == 0, "the stream's counters")
check(lag["model_ms"] == 50.0 and float(s["model"]["servo_tau_ms"]) == 70.0, "the model's servo response recorded")
check("none" in s["by_phase"] and len({"LF", "RF", "RB", "LB"} & set(s["by_phase"])) >= 3,
      f"the error by phase (a leg alone before and after each couplet): {sorted(s['by_phase'])}")
check(s["margin"]["min_one_leg_up_mm"] is not None and
      abs(s["margin"]["min_one_leg_up_mm"] - s["margin"]["planner_min_mm"]) <= 0.11, "the margin: per tick = the planner's")
check(s["balance_active_fraction"] > 0.9 and s["correction"]["max_pitch"] is not None, "the body's balance and its correction")
check(40 <= s["lead_ms"]["min"] <= s["lead_ms"]["max"] <= 200, f"FEET ahead of their tick: {s['lead_ms']}")
check(s["config"]["margin_mm"] and s["masses"]["battery_g"] and s["stream"]["pattern"] == "walk", "the parameters in use")
gait = [r for r in rows if r["mode"] == "GAIT" and r["err_pitch"]]
check(len(gait) == e["samples"] and all(abs(float(r["meas_pitch"]) - float(r["pred_pitch"]) - float(r["err_pitch"])) < 1e-3
                                        for r in gait), "the CSV: err = meas - pred on every row")
print(f"   kilo log walk: {e['samples']} ticks, offset pitch {e['offset_pitch']:+.2f} roll {e['offset_roll']:+.2f} "
      f"(the tilt less the mean prediction {e['pred_mean_pitch']:+.2f} / {e['pred_mean_roll']:+.2f}), "
      f"rms {e['rms_pitch']:.2f} / {e['rms_roll']:.2f}, margin {s['margin']['min_one_leg_up_mm']} mm, "
      f"lead {s['lead_ms']['min']}-{s['lead_ms']['max']} ms, {len(rows)} CSV rows")
EOF
grep "loop: " "$OUT/host_walks.log" | awk '{for(i=1;i<=NF;i++) if($i=="max") m=($(i+1)>m?$(i+1):m)} END {print "   mind.poll max over the run: " m " us"}'

# ---- the flight recorder: a trick by the remote, every record, the card, the next boot ----------------
# The rear-up key on (--rear-up): B2 plays rear_up on the virtual body (which plays it tick-exact but
# does not tip: its IMU stays level, so the analysis sees the design's pitch never reached). Then the
# recorder's status, a time range fetched and decoded, the metrics, the log's whole history, the card
# saver on (kilo-mind-host's --card DIR stands for the SD card), and a second boot on the same card
# that lists the first boot's run and shows its log (kilo log --previous).
echo "== flight recorder: a trick (B2), kilo runs/record/metrics/log --host, the card, the next boot"
REC="$OUT/flight"
rm -rf "$REC"
mkdir -p "$REC/card"
start_ctl_body "$OUT/vbody_flight.log"
trap stop_body EXIT
run_mind "$OUT/host_flight.log" --remote --rear-up --authorized-keys "$KEYS" --listen 127.0.0.1:0 --card "$REC/card" \
    --duration 40
trap 'kill $MIND_PID 2>/dev/null || true; stop_body' EXIT
FPORT=""
for _ in $(seq 1 50); do
    FPORT="$(grep -o 'listening on port [0-9]*' "$OUT/host_flight.log" | grep -o '[0-9]*$' || true)"
    [ -n "$FPORT" ] && break
    sleep 0.1
done
[ -n "$FPORT" ] || { echo "FAILED: kilo-mind-host did not listen"; cat "$OUT/host_flight.log"; exit 1; }
F="127.0.0.1:$FPORT"
sleep 1
ctl ir 0x02                   # B6: stand
sleep 1
ctl ir 0x38                   # B2: rear_up (stage 1)
for _ in $(seq 1 120); do grep -q "TRICK_DONE" "$OUT/host_flight.log" && break; sleep 0.25; done
expect "$OUT/host_flight.log" "trick rear_up: TRICK_DONE DONE" "the trick played to its end"
sleep 2.5                     # the run's post-roll
KPY -m kilo.cli --host "$F" runs | tee "$OUT/flight_runs.txt"
expect "$OUT/flight_runs.txt" "1 runs this boot \(0 walks, 1 tricks\)" "the list: one trick"
expect "$OUT/flight_runs.txt" "^1 +[0-9-]+ [0-9:]+ +trick +rear_up stage 1 .* DONE" "the list: run 1, rear_up stage 1, DONE"
KPY -m kilo.cli --host "$F" runs --fetch last --out "$REC/runs" | tee "$OUT/flight_fetch.txt"
expect "$OUT/flight_fetch.txt" "trick 1: [0-9]+ bytes in [0-9.]+ s -> .*run-[0-9-]+-trick-1\.krec" "fetch: the trick run written"
expect "$OUT/flight_fetch.txt" "trick run 1: rear_up \(stage 1\)" "the trick analysis"
expect "$OUT/flight_fetch.txt" "end +DONE" "the analysis: how it ended"
expect "$OUT/flight_fetch.txt" "^ +5  up " "the analysis: keyframe 5, up"
KREC="$(ls "$REC"/runs/run-*-trick-1.krec)"
KPY -m kilo.cli log trick "$KREC" --csv "$OUT/trick1.csv" --json > "$OUT/trick1.json"
KPY -m kilo.cli --host "$F" record status | tee "$OUT/flight_status.txt"
expect "$OUT/flight_status.txt" "flight recorder +boot" "record status"
expect "$OUT/flight_status.txt" "a power cut now loses [0-9]+ KB" "record status: what a power cut loses"
KPY -m kilo.cli --host "$F" record fetch --last 1m --out "$REC/range" | tee "$OUT/flight_range.txt"
expect "$OUT/flight_range.txt" "[0-9]+ bytes in [0-9.]+ s -> .*rec-[0-9-]+\.krec" "record fetch --last 1m"
expect "$OUT/flight_range.txt" "run +1 trick" "the range holds the trick run"
KRANGE="$(ls "$REC"/range/rec-*.krec)"
KPY -m kilo.cli record decode "$KRANGE" --csv "$OUT/flight_range.csv" > /dev/null
KPY -m kilo.cli record decode "$KRANGE" --log > "$OUT/flight_range_log.txt"
expect "$OUT/flight_range_log.txt" "remote B2 \(0x38, rear up\)" "the decoded range: the remote's line for B2"
KPY -m kilo.cli --host "$F" metrics | tee "$OUT/flight_metrics.txt"
expect "$OUT/flight_metrics.txt" "^[a-z_]+: " "kilo metrics: group lines"
KPY -m kilo.cli log --host "$F" --no-follow --save "$REC/logs" > "$OUT/flight_log.txt"
expect "$OUT/flight_log.txt" "I remote B2 \(0x38, rear up\)" "kilo log --host: the session's key in the history"
expect "$OUT/flight_log.txt" "I trick rear_up: TRICK_DONE DONE" "kilo log --host: the trick's end in the history"
ls "$REC"/logs/kilo-*.log > /dev/null || { echo "FAILED: kilo log --save wrote nothing"; exit 1; }
KPY -m kilo.cli --host "$F" record save on | tee "$OUT/flight_save.txt"
KPY -m kilo.cli --host "$F" record save now > /dev/null
for _ in $(seq 1 60); do
    ls "$REC"/card/kilorec/*.krc > /dev/null 2>&1 && break
    sleep 0.25
done
ls "$REC"/card/kilorec/*.krc > /dev/null 2>&1 || { echo "FAILED: nothing saved to the card"; ls -lR "$REC/card"; exit 1; }
"$PYTHON" - "$OUT/trick1.json" "$OUT/trick1.csv" "$OUT/flight_range.csv" <<'EOF'
import csv, json, sys
a = json.load(open(sys.argv[1]))
rows = list(csv.DictReader(open(sys.argv[2])))
rng = list(csv.DictReader(open(sys.argv[3])))
def check(cond, what):
    if not cond:
        print(f"FAILED: the trick run: {what}\n{json.dumps(a, indent=1)[:3000]}"); sys.exit(1)
check(a["trick"] == "rear_up" and a["stage"] == "1" and a["played"] and a["end"]["reason"] == "DONE", "rear_up, played, DONE")
check(a["has_prediction"] and len(a["keyframes"]) == 12, "12 keyframes with a prediction")
check(all(k["reached"] for k in a["keyframes"]), "every keyframe seen in the telemetry")
check(sum(1 for r in rows if r["mode"] == "TRICK") > 100, "TRICK telemetry in the CSV")
check(any(r["body_keyframe"] for r in rows), "the body's keyframe in the telemetry")
check(any(r["trick"] == "rear_up" for r in rng) and any(r["key"] for r in rng), "the decoded range: the trick, the key")
print(f"   kilo log trick: {len(a['keyframes'])} keyframes, {a['telemetry']} telemetry ticks, end {a['end']['reason']}, "
      f"error mean {a['error']['mean_deg']:+.1f} deg (the virtual body does not tip); range: {len(rng)} rows")
EOF
wait_mind "$OUT/host_flight.log"
stop_body
trap - EXIT
# The next boot on the same card: the first boot's run and log are there.
start_ctl_body "$OUT/vbody_flight2.log"
trap stop_body EXIT
run_mind "$OUT/host_flight2.log" --remote --authorized-keys "$KEYS" --listen 127.0.0.1:0 --card "$REC/card" --duration 10
trap 'kill $MIND_PID 2>/dev/null || true; stop_body' EXIT
FPORT=""
for _ in $(seq 1 50); do
    FPORT="$(grep -o 'listening on port [0-9]*' "$OUT/host_flight2.log" | grep -o '[0-9]*$' || true)"
    [ -n "$FPORT" ] && break
    sleep 0.1
done
[ -n "$FPORT" ] || { echo "FAILED: kilo-mind-host did not listen"; cat "$OUT/host_flight2.log"; exit 1; }
F="127.0.0.1:$FPORT"
sleep 2
KPY -m kilo.cli --host "$F" runs | tee "$OUT/flight2_runs.txt"
expect "$OUT/flight2_runs.txt" "1 from earlier boots" "the next boot lists the first boot's run"
expect "$OUT/flight2_runs.txt" "^b1:1 .*trick +rear_up stage 1 .*card +DONE" "the first boot's trick, on the card"
KPY -m kilo.cli log --host "$F" --previous > "$OUT/flight2_prev.txt"
head -3 "$OUT/flight2_prev.txt"
expect "$OUT/flight2_prev.txt" "boot 1's log as saved on the SD card" "kilo log --previous"
expect "$OUT/flight2_prev.txt" "remote B2 \(0x38, rear up\)" "the previous boot's log: the key"
wait_mind "$OUT/host_flight2.log"
stop_body
trap - EXIT
echo "== finding Kilo by name: kilo find --ask against the mind's mDNS responder (build/mdns-host, loopback)"
MDNS_OUT="$OUT/mdns_host.out"
"$HERE/build/mdns-host" --name kilo --address 192.168.88.9 --build 2d7789e1 --duration 20 > "$MDNS_OUT" 2> "$OUT/mdns_host.log" &
MDNS_PID=$!
for _ in $(seq 1 100); do grep -q "^mdns-host port" "$MDNS_OUT" && break; sleep 0.05; done
MDNS_PORT="$(awk '/^mdns-host port/ {print $3}' "$MDNS_OUT")"
[ -n "$MDNS_PORT" ] || { echo "FAILED: mdns-host did not claim its name"; kill "$MDNS_PID"; exit 1; }
KPY -m kilo.cli find --ask "127.0.0.1:$MDNS_PORT" --json > "$OUT/find.json"
kill "$MDNS_PID"; wait "$MDNS_PID" 2>/dev/null || true
"$PYTHON" - "$OUT/find.json" <<'EOF'
import json, sys
f = json.load(open(sys.argv[1]))
assert [(x["name"], x["address"], x["port"], x["build"]) for x in f] == [("kilo.local", "192.168.88.9", 5300, "2d7789e1")], f
print(f"   kilo find: {f[0]['name']} {f[0]['address']} port {f[0]['port']} build {f[0]['build']} ({f[0]['via']})")
EOF

echo "== all kernel host tests passed"
