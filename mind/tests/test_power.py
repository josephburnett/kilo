"""kilo power (kilo.cli_power): the mind's power metrics lines, and the battery's
trend per stretch of a recording."""

from __future__ import annotations

from kilo import cli, cli_power as C

LINES = [
    "session: uptime_s=400 mind_build=00000000 boot=1",
    "power: state=asleep for_s=212 light_sleep=on sleeps=2 wakes=1 last_wake=key:A1 awake_s=120 drowsy_s=60 "
    "asleep_s=220",
    "power_hw: net=save net_device=yes net_saves=2 net_looks=21 net_errors=0 card_flushes=2 arm_mhz=600 temp_c=41 "
    "throttled=0x50005",
    "battery: mv=7600 min_mv=7598 mv_per_min_1m=? 5m=-8.4 15m=-12.0 30m=? this_state=-7.9 samples=40",
    "power_segments: asleep:200s:7690-7610mV:-8.0 awake:90s:7700-7690mV:?",
]


def test_parse_metrics() -> None:
    d = C.parse_metrics(LINES)
    assert set(d) == {"power", "power_hw", "battery", "power_segments"}
    assert d["power"]["state"] == "asleep" and d["power"]["asleep_s"] == 220 and d["power"]["last_wake"] == "key:A1"
    assert d["battery"]["5m"] == -8.4 and d["battery"]["mv_per_min_1m"] is None and d["battery"]["mv"] == 7600
    assert d["power_hw"]["arm_mhz"] == 600 and C._undervolt(d["power_hw"]["throttled"])
    assert d["power_segments"] == [
        {"state": "asleep", "s": 200, "first_mv": 7690, "last_mv": 7610, "mv_per_min": -8.0},
        {"state": "awake", "s": 90, "first_mv": 7700, "last_mv": 7690, "mv_per_min": None}]


def test_print_power(capsys) -> None:
    C.print_power(C.parse_metrics(LINES))
    out = capsys.readouterr().out
    assert "state       asleep for 0:03:32" in out and "-8.4 mV/min 5 min" in out and "under-voltage seen" in out
    assert "stretch     asleep  0:03:20  7690 -> 7610 mV  -8.0 mV/min" in out


def test_stretches() -> None:
    # 3 min idle at -60 mV/min, 2 min walking, then light sleep for 5 min at -6 mV/min after a 50 mV jump up
    samples, logs = [], []
    t, mv = 0.0, 8000.0
    while t < 180:
        samples.append((t, "IDLE", True, int(mv)))
        t += 0.04
        mv -= 0.04
    while t < 300:
        samples.append((t, "GAIT", True, int(mv)))
        t += 0.04
        mv -= 0.05
    logs.append((t, "power: light sleep (30 s after A7 with nothing else): servos off (LIMP)"))
    mv += 50
    t0 = t
    while t < t0 + 300:
        samples.append((t + 0.001, "LIMP", False, int(mv)))
        t += 0.04
        mv -= 0.004
    logs.append((t, "power: awake (key:B6) after 5 min 0 s asleep"))
    samples.append((t + 0.01, "IDLE", True, int(mv)))
    st = C.stretches(samples, logs)
    assert [g.kind for g in st] == ["idle", "walking", "asleep", "idle"]
    assert abs(st[0].mv_per_min - (-60)) < 2 and abs(st[1].mv_per_min - (-75)) < 3
    assert abs(st[2].mv_per_min - (-6)) < 1 and st[3].mv_per_min is None
    summ = C.summary(st)
    assert summ["asleep"]["mv_per_min"] == st[2].mv_per_min and summ["idle"]["s"] > 179
    # LIMP without the sleep lines is limp; no battery (0 mV) is skipped
    st = C.stretches([(0.0, "LIMP", False, 7000), (1.0, "IDLE", False, 0), (2.0, "IDLE", True, 7000)], [])
    assert [g.kind for g in st] == ["limp", "idle"]


def test_cli_wiring() -> None:
    a = cli.parse_args(["power", "--host", "10.0.0.2"])
    assert a.cmd == "power" and a.file is None
    from kilo.cli_host import HOST_COMMANDS, wants_host

    assert wants_host(a) and HOST_COMMANDS["power"] is C.cmd_power
    a = cli.parse_args(["power", "x.krec"])
    assert not wants_host(a) and a.file == "x.krec"
    assert cli.LOCAL_COMMANDS["power"](cli.parse_args(["power", "/nonexistent.krec"])) == C.EXIT_ERR
