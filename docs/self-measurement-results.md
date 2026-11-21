# Self-measurement: results

What Kilo measured about himself with B1 (docs/self-measurement.md): the
date, the floor, what you saw, and the table `kilo measure fetch` printed.

## 2026-11-21, first run: fits, but the centre of mass's height is not measured

Build: mind 0a4c7465 (boot 13 of the card). Floor: flat (owner). What the
owner saw: "saw it move around"; nothing else reported, so whether a foot
left the floor in each test is not known. Read from the card on the laptop
(`kilo card /mnt/e --fetch`, then `kilo measure fit`), not over the network.
Recording: `~/kilo-logs/run-b13-145s-trick-4.krec`, log
`~/kilo-logs/kilo-card-b13.log`.

The log: a first B1 at 140.1 s was stopped by a second B1 after 2 s (78
ticks, 0 tips). The third B1 at 144.6 s ran to the end: `measure: done
(1759 ticks, 35 s, 6 tips)`, battery 7591 mV at the start, 7436-7525 mV
over the run. It came straight after two long-trot walks. One
`FEET_UNDERRUN` (the body ran out of queued steps) 40 ms before the end.

| parameter | unit | model | fitted | +- | significant | what |
|---|---|---|---|---|---|---|
| imu_pitch | deg | 0.00 | 2.40 | 0.08 | yes | IMU pitch offset (+ floor slope, + front/back legs) |
| imu_roll | deg | 0.00 | 0.52 | 0.08 | yes | IMU roll offset (+ floor slope, + left/right legs) |
| com_x | mm | 7.43 | 4.35 | 0.31 | yes | centre of mass, ahead of the shoulders' middle |
| com_y | mm | 0.00 | -0.73 | 0.30 | yes | centre of mass, to the left |
| com_z | mm | -2.83 | -40.00 | 7.45 | yes | centre of mass, above the shoulder axles |
| sag | deg/kg cm | 0.00 | 1.11 | 0.70 | no | leg servos' give under load |
| twist | mm | 0.00 | -0.10 | 0.04 | yes | LF+RB legs longer than RF+LB (each pair by half) |
| lag_ms | ms | 50.00 | 70.55 | 11.59 | no | servo dead time (with the IMU's own delay) |
| tau_ms | ms | 70.00 | 68.57 | 15.20 | no | servo time constant |
| tip_ms | ms | 85.76 | 508.08 | 112.81 | yes | a tip's delay: the step over the edge to 40 % of the tip |

Tick residuals: 1283 ticks, rms 0.405 deg (the routine's own rigid
prediction: 1.99 deg).

| edge test | measured crossing | model | fitted |
|---|---|---|---|
| edge_lf | none: a slow lean, not a tip | +8.1 mm | |
| edge_rf | +6.9 mm | +8.1 | +6.8 |
| edge_lb | +5.1 mm | +8.3 | +5.7 |
| edge_rb | none: a slow lean, not a tip | +8.3 mm | |
| edge_rf_roll | -3.0 mm | -11.9 | -3.9 |
| edge_rb_roll | none: a slow lean, not a tip | -6.8 mm | |

Tip rate: 1 tip timed, e-folding time 112 ms (a point mass 123 mm up; at
the fitted height it would be 65.5 ms).

What to make of it:

- **com_z is not a measurement.** -40.00 mm is the fit's lower limit
  (`mind/kilo/measure/fit.py`, -40 to 40), so the fit ran into its stop;
  the one timed tip points the other way (higher, not lower). The height
  of the centre of mass is unknown from this run.
- Half of the edge tests gave no crossing (a slow lean) and only one of the
  six tips the routine counted was timed, so the tip numbers rest on little.
  tip_ms at 508 ms against the model's 86 ms says the same thing: he leans
  over slowly where the model expects him to fall over an edge.
- The steadier numbers: the IMU reads 2.4 deg nose-down and 0.5 deg of roll
  standing on this floor, and the centre of mass sits about 3 mm further
  back than the model's (4.3 against 7.4 mm). Servo timing and sag are
  within their error of the model.
- The fit's own verdict: "Nothing here changes the walks' model."
