#!/usr/bin/env bash
# Every check of the mind core on the Linux host:
#   1. the C++ unit tests (sanitizers): formatter, framing vectors, names, link state machine;
#   2. the freestanding build (no libc, libm or libstdc++ beyond memcpy/memset/memcmp);
#   3. kilo-mind-host against the virtual body (the real body core on a pty), and
#      against a silent pty and a protocol-1 fake body.
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

echo "== all kernel host tests passed"
