#!/usr/bin/env bash
# Every check of the mind core on the Linux host:
#   1. the C++ unit tests (sanitizers): formatter, framing vectors, names, link state machine,
#      planner, walker, and the planner against the Python planner's vectors (float and double);
#   2. the planner's cost per tick on this machine;
#   3. the freestanding build (no libc or libstdc++ beyond memcpy/memset/memcmp, libm's
#      float sinf cosf sincosf tanf atanf atan2f);
#   4. kilo-mind-host against the virtual body (the real body core on a pty), and
#      against a silent pty and a protocol-1 fake body; walk-demo trotting the virtual body.
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

echo "== all kernel host tests passed"
