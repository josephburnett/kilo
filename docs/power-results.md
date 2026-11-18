# Power results

The tests of [power.md](power.md) "How to measure it". One section per
run. Paste the output of `kilo power --host` and `kilo power FILE.krec`
under each.

## Test A: awake, standing (B6)

- Date:
- Kernel build (`kilo status --host`, `mind_build`):
- Battery at the start (mV):
- Left for (minutes):
- Cable in during the test: no

`kilo power --host kilo.local`:

```
```

`kilo power ~/kilo-logs/rec-....krec`:

```
```

- The `idle` stretch's slope (mV/min):
- `throttled`, SoC temperature:
- Anything seen or heard (a warm servo, buzzing):

## Test B: light sleep (A7)

- Date:
- Kernel build:
- Battery at the start (mV):
- Did the LED go out and flash every 5 s after 30 s? (yes / no):
- Left for (minutes):
- Did plugging the cable in and running `kilo power --host` work first time? How long until it answered?:

`kilo power --host kilo.local`:

```
```

`kilo power ~/kilo-logs/rec-....krec`:

```
```

- The `asleep` stretch's slope (mV/min):
- `net`, `net_saves`, `net_looks`, `net_errors`:
- `throttled`, SoC temperature:
- From the log (`kilo log --host kilo.local`): the `light sleep: N wfi wakeups/s` line, and
  `power: Ethernet PHY register 17 .... -> ....`:
- B6 after the test: did he stand up normally?

## Test C (optional): lying in REST, awake (KILO_LIGHT_SLEEP=0)

- Date:
- Battery at the start (mV):
- Left for (minutes):

`kilo power ~/kilo-logs/rec-....krec`:

```
```

- The `idle` stretch's slope (mV/min):

## Summary

| Test | Slope (mV/min) | Minutes from 7.6 V to 6.6 V at that slope |
|---|---|---|
| A awake, standing | | |
| B light sleep | | |
| C awake, lying (REST) | | |
