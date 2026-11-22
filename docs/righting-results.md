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

| # | Laid on (left side / right side / back) | Try 1 | Try 2 | Try 3 | Ended (standing / on his belly / limp, meowed) | Anything strained? |
|---|---|---|---|---|---|---|
| 1 | | | | | | |
| 2 | | | | | | |
| 3 | | | | | | |

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
