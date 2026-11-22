# Getting up after a fall: measurements, recorded falls, and what the simulator reproduces

For docs/righting.md. Newest first. "Don't know" is a fine answer.

## Measurements (millimetres, Kilo switched off)

Date: ...

| What | Measured |
|---|---|
| Width across the front shoulder servos (outer faces) | |
| Width across the back shoulder servos | |
| Width across the knees, legs in the standing pose | |
| Lying on his side: what touches at the top | |
| ...that point above the line through the shoulder axles | |
| ...and from his middle (left to right) | |
| Front shoulder axles to where the neck turns | |
| Neck to the tip of the nose | |
| Width of the head | |
| Ear tips above the shoulder axles, head straight | |
| A positive head turn looks to his... (left / right) | |

Photos (from above, the side, the front, a ruler in each): ...

## Tries of the righting experiment (B1)

Copy the block for each session.

### YYYY-MM-DD

Build: ...   Floor: ...   Battery at the start: ... mV   Recording saved to the card: yes / no

| # | Laid on (left side / right side / back) | Try 1 | Try 2 | Try 3 | Ended (standing / on his belly / limp after red flashes) | Anything strained? |
|---|---|---|---|---|---|---|
| 1 | | | | | | |
| 2 | | | | | | |
| 3 | | | | | | |

## 2026-11-22, boot 16: the first tries on Kilo (from the recording)

Build 7e92703c, kernel with `KILO_RIGHTING=1` (FALL_RC 0). What the owner
saw has not been written down yet: this is what the IMU recorded
(`~/kilo-logs/rec-card-b16.krec`, the log `kilo-card-b16.log`). Times are
seconds after the Pi started. Roll as in docs/righting.md: 0 upright, +90
on his left side, -90 on his right, 180 on his back. [measured] is the
recording, [inferred] is a reading of it.

| # | Lying on | Try 1 | Try 2 | Try 3 | Ended | Anything strained? |
|---|---|---|---|---|---|---|
| 1 (B1 at 133.9 s) | his back, roll +177 | turned 32° toward his right side, then rolled back | 33°, back | 33°, back | limp on his back; red flashes, gave up | not recorded |
| 2 (B1 at 147.0 s) | his back, roll +177 | 31°, back | 31°, back | 33°, back | limp on his back; red flashes, gave up | not recorded |

**How he came to be on his back** [measured]:

1. 108.6 s: standing still (B6), roll -0.6°, nothing moving.
2. 109.9 to 110.4 s: he turned onto his **left side**: roll 0 to 81° in
   about 0.5 s, fastest 234°/s at 43°, then slower (42°/s) from 64 to 81°,
   and he settled at 94 to 97°. The body's fall check fired (FELL at
   110.5 s; with FALL_RC 0 it then laid the legs to rest and let them go
   limp at 114.1 s). The state estimate said ON_LEFT_SIDE (111.6 s):
   right. Nothing had been asked of him for 1.5 s, and the roll slowed
   down before he reached the floor; that reads more like being laid down
   by hand than a fall [inferred; the owner had been asked to lay him on
   his side].
3. 117.7 to 118.3 s: limp, after 3.6 s lying still on his side, he turned
   onto his **back**: 97 to 177° in 0.6 s, fastest 194°/s. The estimate
   said ON_BACK (119.1 s): right. Lying still and limp, a turn like that
   needs a push [inferred: a hand, again]. B1 came 15 s later, so both
   presses started from his back, never from a side.

**Each try, from his back** [measured; the six are alike within 2°]:

| part of the move | Kilo | the simulator, his fitted outline |
|---|---|---|
| start (after "REST first") | 2 to 3° toward his left side | 3° toward his left (+177) |
| wind-up (1.1 s) | 4° toward his right side | about 0° |
| kick (0.56 s) | 31 to 33° toward his right side, fastest roll 273 to 307°/s | 63° (roll -117) |
| rest pose (1 s) | rolled back onto his back, 3 to 4° toward his left | stays on his right side at -124 |
| settled | on his back | on his right side (what the move is for) |

His nose stayed within 9° of level throughout (pitch -171 to -179°, which
is 1 to 9° from level when upside down).

**Why it failed, against the simulator** [measured, then inferred]:

- The kick turned him half as far as the simulator says: 32° instead of
  63°. That is short of where the right legs (in the standing pose) can
  hold him on his side, so as soon as the last frame folded those legs
  into the rest pose he rolled back down. The simulator's own variant with
  the top edges of his back a quarter lower (`top_z` x0.75) is the one
  that ends like Kilo did: the kick turns him, the rest pose lets him roll
  back onto his back. That is the same direction as the 2026-11-21 replay
  of rc (the top corners of his back lower than guessed) [inferred].
- The variant that is said to "roll him as on Kilo" (the outline fitted
  to rc's first pose) ends on his right side (-71): it is wrong for this
  move.
- The feet may also push less than modelled (slip, or not reaching the
  floor beyond his back); the recording cannot tell the two apart.

**The repeats.** The mind played the identical move three times per
press, and each try ended exactly as the one before (within 2°). A
closed-loop version would, from the IMU it already reads:

1. After the kick, look at the roll before the next frame: if he is not
   past the angle where the legs underneath can hold him, do not fold
   those legs into the rest pose (which is what let him roll back); hold,
   or try something else.
2. Not repeat a move whose last try ended within a few degrees of the one
   before: change it (a bigger wind-up, the mirrored move toward his left
   side, which he already leaned toward, or a second kick timed to the
   roll back, rocking him like a swing), or stop after the first identical
   failure and ask for a hand.
3. Write the peak angle of each try to the log (here 31 to 33°), so that
   each try is also a measurement of how far the kick turns him.

Not changed today: the move and the retry rule are as built.

**The fall counter.** The session summary says `falls=10 recovered=0`.
One was the turn onto his left side at 110.5 s. The other nine were the
body's fall check finding him already on his back each time the legs were
given a pose again: after each B1's "REST first" (2), after each of the six
moves (6), and after A7 (1). That is the design (docs/righting.md, "No
long stall"), but it means `falls` counts "found lying" as well as falls.
Each of those nine also started a new goal epoch and cleared the mind's
posture goal; the next move still started on time (1.1 to 1.4 s after).

**After the second give-up.** A7 at 160.1 s; from 162.3 to 164.7 s he
went from his back over his left side onto his belly (roll -0.7°), limp,
in 2.4 s: turned over by hand [inferred].

## 2026-11-22: the simulator against rc, frame by frame (simulation only)

No new recordings; the same six tries on the floor as on 2026-11-21. Added
to the lying model: the head's weight (25 g, an estimate) moving with the
head; the legs', head's and tail's own swing turning the body; the tail's
direction as an unknown; an optional higher middle of the back. Then rc's
three poses were looked at one at a time
(`python -m kilo.tricks.righting --phases`): where he is after each.

Roll angle after each pose (0 upright, -90 right side, +-180 back):

| Try | rc | Kilo after pose 1, 2, 3 | Model after pose 1, 2, 3 |
|---|---|---|---|
| 17 Nov, first | as written | back, -89, up | back, +173, +115 |
| 17 Nov, second | as written | back, -95, up | back, +173, +115 |
| boot 8, 114 s | as written | back, -72, up | -124, -49, -51 |
| boot 8, 120 s, try 1 | mirrored | back, +110, +103 | +124, back, -82 |
| boot 8, 120 s, try 2 | mirrored | back, +101, +122 | back, -115, -82 |
| boot 8, 120 s, try 3 | mirrored | back, +90, up | +124, back, -82 |

Still **0 of the 4** getting-ups. Each pose of rc goes wrong for its own
reason. 195 versions of the unmeasured sizes and the two unknown directions
(head turn, tail) were tried (a grid over the back's height and width, the
ear height, the shoulder servos' size):

- **Pose 1** (from a side, Kilo rolled onto his back in all 3 tries,
  within a quarter of a second): the model rolls him onto his back only
  when the top edges of his back are lower than guessed (30 mm above the
  shoulder axles instead of 36), with any width. That is what this pose
  says about him.
- **Pose 2** (from his back, Kilo rolled over the side whose legs it
  folds): with flat top edges the model always rolls him the other way:
  the folded legs' feet reach past his back and push that side up. It goes
  his way only with a higher, rounded middle of the back (a ridge 42 mm up)
  that the feet cannot reach past, and then only for rc as written, not
  mirrored.
- **Pose 3** (from the side up onto his feet): falls short in nearly
  every version.
- No version does all three. The friction, the servos' speed and the
  legs' give change little; the outline of his back and head decides it,
  and it has never been measured.

What the data does pin down, then: the top corners of his back sit lower
than the Pi's highest point, and the middle of the back is higher than its
corners (a rounded back). Nothing pins the head or the tail.

The righting moves of docs/righting.md were checked in the same simulator,
across the same kinds of unknowns: side move onto his belly in 89 of 95
runs, back move onto his right side in 68 of 76, leg servos at most 1.1 kg
cm. Simulation only, in a simulator that gets rc wrong.

## Falls with hands off

Copy the block for each session.

### YYYY-MM-DD

Build: ...   Floor: ...   Battery at the start: ... mV   Recording saved to the card: yes / no

| # | Laid on (left side / right side / back) | Hands off before the chirp? | Try 1 | Try 2 | Try 3 | Ended (standing / lying / limp) | Touched him during it? |
|---|---|---|---|---|---|---|---|
| 1 | | | | | | | |

## 2026-11-21: what the recordings already on the laptop show

No new falls were recorded for this; it is the search of the existing
recordings (`python -m kilo.tricks.righting --falls`). Nine falls, six
tries of rc on the floor: four ended upright, two did not. Whether a hand
held him is not recorded; four of the nine falls are read as "in a hand"
from the Pi's picked-up detector and from tries that stayed level where a
free-standing Kilo could not. The model's replay of the six floor tries
(`python -m kilo.tricks.righting --bands`): upright in none of the four
that worked. Not trusted for designing a getting-up.
