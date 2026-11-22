# Intents: what state Kilo is in, and "be standing"

Two new things in Kilo's mind (the program on the Pi), from 2026-11-21:

1. **The state estimate. Always on.** The mind keeps an opinion of where
   Kilo is: standing, sitting, resting, on his left or right side, on his
   back, on his nose or tail, moving, or "don't know". It only watches: it
   changes nothing he does. Every time the opinion changes it writes a line
   in the log, so every run on Kilo now tells us how good it is.
2. **Intents. Off unless built in.** With the switch on, the stand key (B6),
   the rest key (A7) and the behaviour's stand, sit and rest are no longer
   "send the body this posture". They are "be standing", "be resting", "be
   sitting": the mind looks where Kilo is, picks the next move toward that,
   plays it, looks again where he ended up, and goes on from there.

Remote keys are named by their place on the remote: columns A B C from the
left, rows 1 to 7 from the bottom.

## Words used here

- **State**: one of STANDING, SITTING, RESTING (lying on his belly, legs
  folded, or limp there), ON_LEFT_SIDE, ON_RIGHT_SIDE, ON_BACK, ON_END (nose
  or tail down, near vertical), MOVING (not settled, in a hand, or a walk, a
  trick or a move under way), UNKNOWN (settled, but in no pose the model
  knows: leaning on something, held at an angle).
- **The model**: the one description of Kilo's body in `protocol/spec.toml`
  (docs/architecture.md "One model"): his masses, legs, servos.
- **IMU**: the motion sensor on his board. It says how he is tilted: pitch
  (nose down or up) and roll (leaning left or right), and how fast he turns.
- **Asked**: the posture the mind last asked the body for. The estimate
  uses it as a hint, never as the answer: told to stand and lying on his
  side, the estimate says "on his side".
- **Move**: a short sequence of joint angles that takes him from one
  posture to another, played by the same player as the tricks.
- **Fall reflex**: the body's own getting up after a fall (the board does
  it by itself, without the Pi).

## How to see what he thinks

In the log (`kilo log --host kilo.local`, or from the card: `kilo card
/mnt/e --fetch`, then the `kilo-card-bN.log` file in `~/kilo-logs/`), lines
like these (the numbers are examples):

```
   0.937 I state: MOVING -> RESTING (pitch +0.0 roll -1.7, asked none, confidence 53)
   5.898 I state: MOVING -> STANDING (pitch +2.4 roll +0.2, asked stand, confidence 90)
  19.340 I state: STANDING -> ON_LEFT_SIDE (pitch +0.0 roll +88.0, asked none, confidence 90)
```

Each line: the state before, the state now, what the IMU read (degrees;
pitch + is nose down, roll + is his right side up), what had been asked, and
a confidence from 0 to 100 (100: exactly what the model predicts for what
was asked; about 50: it fits, but it is not what was asked, or nothing was).

The session's counters (`kilo metrics --host kilo.local`, and at the end of
every saved log) have two lines:

```
state: now=STANDING confidence=90 changes=14 MOVING=7 STANDING=4 RESTING=2 SITTING=1
intents: on=0 asked=0 reached=0 gave_up=0 cancelled=0 moves=0 body_postures=0 reflex_waits=0 failed_steps=0
```

The flight recorder gets a mark for each change too, so a recording can be
checked afterwards: `python -m kilo.intent.replay FILE.krec` replays the
estimate over a recording and prints how often it agreed with what Kilo had
been told to hold.

## How it decides

- **Moving**: he is walking or playing a trick, or he has not been still
  (turning slower than 20 degrees a second) for 0.4 seconds.
- **Fallen**, from the tilt alone: rolled 60 to 120 degrees is on a side
  (which one from the sign), past 120 on his back, pitched within 30 degrees
  of vertical on his nose or tail. (The body's own fall reflex trips at 75
  degrees.)
- **Otherwise upright**, and the model says which: for stand, sit and rest
  the model computes how he comes to rest in that posture and what the IMU
  should read:

  | State | The model's IMU reading | Kilo's own, in the recordings | It fits within |
  |---|---|---|---|
  | STANDING | level (pitch 0, roll 0) | 2.4 nose down, 0.2 | 11.9 degrees |
  | SITTING | 28.4 nose up, roll 0 | 25.7 nose up, 2.1 | 11.9 degrees |
  | RESTING | pitch 0, rolled 8.9 (right side down) | 0.7, 1.7 | 11.9 degrees |

  The posture that was asked wins if the reading fits it. If it does not
  fit but another state's does, he is in that other state. If nothing fits:
  UNKNOWN.
- A new answer has to hold for 0.3 seconds before it counts.

**Two limits you should know.**

- **Standing and resting look the same to the IMU.** Both are level. The
  estimate tells them apart only by what was asked (and at power-on by the
  servos being off: then he must be lying). A move that was meant to stand
  him up and left him lying flat would not be noticed.
- **The band is wide (11.9 degrees)** because the model is still off by up
  to 8 degrees for a posture: the model has him resting crooked (its left
  front knee stops short of its angle) where Kilo lies nearly level, and it
  has no IMU offset yet. Once a self-measurement (C5 twice) has been put into the
  model (`kilo model apply`), the band can come down
  (`MODEL_ERROR_DEG` in `mind/kilo/intent/estimate.py`).

The table is made from the model's own numbers, not from the tricks' fitted
ones (the "tricks" view of docs/architecture.md): the estimate must not
depend on a fit that a self-measurement may replace. The difference is
under 3 degrees.

On every recording we have (2026-11-17 to 21: 37,000 readings with a
posture asked and the body at rest), the estimate named the posture he had
been told to hold every time but three moments of under half a second each,
when he was being handled (`python -m kilo.intent.replay`). The recordings
have him on his side only while you held him there in the calibration
posture; a real fall has not been recorded with the estimate running.

## What the switch changes (KILO_INTENT=1)

Build with it and write the card:

```
kilo card /mnt/e KILO_INTENT=1
```

The log's first lines then say `intents ON`. Without it (the default):
`intents off`, and everything below moves exactly as before.

| Who asks | Without the switch | With KILO_INTENT=1 |
|---|---|---|
| **B6** (stand) | the body turns every joint straight to STAND | "be STANDING": see below |
| **A7** (rest, then sleep) | the body turns every joint straight to REST | "be RESTING"; the servos still go off a few seconds after he lies still |
| the behaviour runner (A5) asking for stand, sit or rest | the body's own change | the same intents |
| the distance guard sitting him down | the body's own change to SIT | "be SITTING" |
| **C5** (calibration posture; twice: the self-measurement), walking keys, the rear-up (A1) | unchanged | unchanged; they drop an intent that is still on its way |

There is no sit key on the remote; sitting comes from the behaviour runner
and the guard.

**What "be STANDING" does.**

1. It waits until he is settled (up to 3 seconds) and reads the state.
2. Already standing: it latches STAND as before. Done.
3. Otherwise it takes the first step toward standing:

   | From | Step |
   |---|---|
   | RESTING | the move "rest to stand": the hind legs first, then the front ones |
   | SITTING | the move "sit to stand" |
   | servos off (after A7's sleep, or at power-on) | REST first (the body refuses a move with the servos off), 1.5 seconds, then as from RESTING |
   | on a side, on his back, on end | wait for the body's fall reflex to right him (up to 15 seconds). With the righting experiment built in (`KILO_RIGHTING=1`, docs/righting.md): the mind's own righting moves instead, from a side or his back; on end, nothing |
   | UNKNOWN | the body's own change to STAND, as before |

4. When the step is over and he is settled, it reads the state again.
   Standing: done (`intent: STANDING reached (1 step, 2403 ms)`). Somewhere
   else: that step failed; it goes on from where he is.
5. After three failed steps (or six steps in all) it gives up, says why in
   the log, and if he is upright leaves him in REST.

"Be RESTING" and "be SITTING" work the same way. From REST to SIT and back
he goes through standing (two moves): no direct move passed the check.

Any key stops a walk as before. A walking key, the rear-up or the self-measurement while a
move is playing stops the move and drops the intent.

## What is planned from the model today, and what is not

**Planned and checked from the model** (`python -m kilo.intent.moves`):
the four moves between the postures. For each, the model tries moving all
leg joints at once (what the body does today) and every way of moving them
in two groups, and keeps the one where his centre of mass stays furthest
inside his feet all the way:

| Move | The plan | Centre of mass inside the feet, at the worst moment | The body's all-at-once way |
|---|---|---|---|
| stand to sit | all joints together | 26.9 mm | the same |
| sit to stand | all joints together | 26.9 mm | the same |
| rest to stand | hind legs first, then front | 10.3 mm | 7.1 mm |
| stand to rest | front legs first, then hind | 10.3 mm | 7.1 mm |
| rest to sit, sit to rest | not shipped: 5.0 mm at best (needs 10) | | he goes through standing |

Each shipped move was then played in the simulator that the rear-up is
checked in (soft legs, real contact with the floor), 21 times with the
model's uncertain numbers pushed to their edges (centre of mass, how much
the legs give, friction, slow servos): he never fell, never rolled past 9
degrees (the mind stops a move at 25), no servo carried more than 0.95 kg cm
(limit 2.4), and he ended where the estimate reads the target state.

Getting up from rest hind legs first means his rear comes up before his
front: the nose points about 20 degrees down for a moment, like a cat's
stretch. That is what to watch on the floor.

**Still preprogrammed:**

- **The end postures themselves.** STAND, SIT and REST are still the
  body's joint-angle tables. The moves end exactly in them.
- **The 13 stock tricks** (hi, wash face, ...) and the sitting fidgets.
- **Getting up after a fall.** Still the body's fall reflex, unless the
  kernel is built with the righting experiment (`KILO_RIGHTING=1`,
  docs/righting.md): then B1 gets him up by moves reasoned on the model,
  and the body's own getting-up is switched off. An experiment: nothing of
  it has been tried on Kilo.

## First floor try

Nothing here is timed. Do each step when you are ready; stop any time with
B6 (stand) or A7 (rest), or pick him up.

**Part 1: the estimate only. Any build from 2026-11-21 on, no switch.**

1. Flat floor, battery charged. Power on. Leave him lying a few seconds.
2. B6 (stand). Wait until he stands still. A7 (rest). B6 again.
3. With him standing: tip him gently onto his left side with your hand and
   hold him there for two or three seconds. He will try to get up by himself
   (the body's fall reflex): let him, or set him back on his feet.
4. If you like: A5 (the behaviour runner) for a minute, so he sits; B6 to
   stop it.
5. Power off, card into the laptop, and:

   ```
   make mount
   kilo card /mnt/e --fetch
   ```

   Then tell me. I read the `state:` lines in the newest
   `~/kilo-logs/kilo-card-bN.log` against what you did, and write it up in
   `docs/intent-results.md`.

What to look for yourself (answers like "don't know" are fine): nothing.
The estimate changes nothing he does; this part is only for the log.

**Part 2: intents. A build with the switch.**

```
kilo card /mnt/e KILO_INTENT=1
```

1. Flat floor, about 30 cm clear around him. Power on; he lies limp.
2. **B6.** He should first tighten into his rest pose (the servos come on),
   pause for a second or two, then get up: **rear first, then the front**.
   Look: does the nose dip as the rear comes up? Does any foot slide or
   leave the floor? Does he end standing as he usually does?
3. **A7.** He should lie down front first, then the rear. A few seconds
   after he lies still the servos go off, as always.
4. **B6** again (now from limp once more), then **A5** to let the behaviour
   runner sit him down and stand him up for a minute; **B6** stops it.
   Look: does sitting down and standing up look as before? (Those two moves
   follow the same path as the body's own change, over 0.8 seconds.)
5. If you want to try the fall: standing, tip him onto a side by hand.
   Press **B6** while he lies there. Nothing new should happen: the body's
   reflex gets him up as before, and the log should say the mind saw him on
   his side and waited.
6. Card into the laptop, `make mount`, `kilo card /mnt/e --fetch`.

If something looks wrong: A7, or pick him up. If he ends up in a heap, the
log's `intent:` lines say what the mind thought was happening.

To go back: `kilo card /mnt/e` (no switch) writes a kernel that changes
postures as before.

## Where the code is

- `mind/kilo/intent/`: the Python reference. `estimate.py` (the estimate),
  `moves.py` (the plan and the check; `moves/*.json` are its results),
  `graph.py` (which step leads where), `replay.py` (a recording through the
  estimate).
- `kernel/mind/intent/`: the mind's own (C++): `estimate.cpp`, `intent.cpp`,
  and `intent.gen.*`, generated from the Python by `gen_intent.py`.
- Tests: `mind/tests/test_intent.py`, `kernel/host/tests/test_intent.cpp`
  (the two estimates agree on every sample of `intent_vectors.txt`), and
  `kernel/host/test.sh` runs the mind with intents on against the virtual
  body: stand from limp, rest, stand, a fall onto his side.
- After a change to the model or the moves: `python -m kilo.intent.moves
  --write` (two minutes), then `kernel/mind/intent/gen_intent.py`, then the
  vectors (`kernel/host/tests/gen_intent_vectors.py`).
- The righting experiment's moves: `mind/kilo/intent/righting.py` (and
  `righting/*.json`, its results; `python -m kilo.intent.righting --write`,
  a few minutes), wired in at one place: `RIGHTING` in
  `mind/kilo/intent/graph.py`.
