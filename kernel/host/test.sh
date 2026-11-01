#!/usr/bin/env bash
# Every check of the mind core on the Linux host:
#   1. the C++ unit tests (sanitizers): formatter, framing vectors, names, link state machine,
#      planner, walker, and the planner against the Python planner's vectors (float and double);
#   2. the planner's cost per tick on this machine;
#   3. the freestanding build (no libc or libstdc++ beyond memcpy/memset/memcmp, libm's
#      float sinf cosf sincosf tanf atanf atan2f);
#   4. kilo-mind-host against the virtual body (the real body core on a pty), and
#      against a silent pty and a protocol-1 fake body; walk-demo trotting the virtual body;
#   5. body flashing (P4): kilo flash --host through kilo-mind-host to a fake optiboot
#      (tests/fake_optiboot.py), a refused CRC, a sync timeout, and kilo flash --sd.
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
start_body "$OUT/vbody_host.log" env PYTHONPATH="$REPO/mind" "$PYTHON" -u -m kilo.vbody.core
"$HOST" --port "$PTY" --listen 127.0.0.1:0 2> "$OUT/host_listen.log" &
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
expect "$OUT/walk.log" "walk trot forward at 55\.6 mm/s" "the walk started"
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
    "$HOST" --port "$PTY" --listen 127.0.0.1:0 --reset-fifo "$FL/$name.fifo" 2> "$FL/$name.mind.log" &
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

echo "== P3 integration: IR keys from the virtual body -> trick, trot, turn, stand, rest (--remote)"
start_ctl_body "$OUT/vbody_ir.log"
trap stop_body EXIT
run_mind "$OUT/host_ir.log" --remote --duration 14
sleep 1
ctl ir 0x10                   # A2: hi (from LIMP: STAND, 1 s, then the trick)
sleep 3.5
ctl ir 0x30; ctl ir 0x62      # A3: trot; B7: forward
sleep 2
ctl ir 0x22                   # A6: turn left
sleep 1
ctl ir 0x02                   # B6: stand (the planned stop, then the posture)
sleep 4
ctl ir 0xA2                   # A7: rest
wait_mind "$OUT/host_ir.log"
stop_body
trap - EXIT
grep -E "remote|trick|walk" "$OUT/host_ir.log"
expect "$OUT/host_ir.log" "trick hi: uploaded \(id [0-9]+, 6 frames" "the trick uploaded"
expect "$OUT/host_ir.log" "remote A2 \(0x10, hi\): trick hi" "A2 started hi"
expect "$OUT/host_ir.log" "trick hi: TRICK_DONE DONE after [0-9]+ ms \(expected 111 ticks\)" "hi played to TRICK_DONE"
expect "$OUT/host_ir.log" "remote A3 \(0x30, trot\): pattern trot" "A3 selected trot"
expect "$OUT/host_ir.log" "remote B7 \(0x62, forward\): walk forward \(trot\)" "B7 walked"
expect "$OUT/host_ir.log" "walk trot forward at 55\.6 mm/s" "the trot started"
expect "$OUT/host_ir.log" "remote A6 \(0x22, left\): walk left \(trot\)" "A6 turned"
expect "$OUT/host_ir.log" "walk ended: stopped" "the planned stop"
expect "$OUT/host_ir.log" "remote B6 \(0x02, stand\): posture stand" "B6 stood after the stop"
expect "$OUT/host_ir.log" "remote A7 \(0xA2, rest\): posture rest" "A7 rested"
if grep -Eq "decode errors [1-9]|remote .*failed" "$OUT/host_ir.log"; then echo "FAILED: errors"; exit 1; fi

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

echo "== all kernel host tests passed"
