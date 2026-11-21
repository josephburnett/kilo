# Getting up after a fall: measurements and recorded falls

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

## Falls with hands off

Copy the block for each session.

### YYYY-MM-DD

Build: ...   Floor: ...   Battery at the start: ... mV   Recording saved to the card: yes / no

| # | Laid on (left side / right side / back) | Hands off before the chirp? | Try 1 | Try 2 | Try 3 | Ended (standing / lying / limp) | Touched him during it? |
|---|---|---|---|---|---|---|---|
| 1 | | | | | | | |
| 2 | | | | | | | |
| 3 | | | | | | | |

Anything else you saw (a leg caught, a servo buzzing, the head pushing):

## 2026-11-21: what the recordings already on the laptop show

No new falls were recorded for this; it is the search of the existing
recordings (docs/righting.md "What the recordings show";
`python -m kilo.tricks.righting --falls`). Nine falls, six tries of rc on
the floor: four ended upright, two did not. Whether a hand held him is not
recorded; four of the nine falls are read as "in a hand" from the Pi's
picked-up detector and from tries that stayed level where a free-standing
Kilo could not. The model's replay of the six floor tries
(`python -m kilo.tricks.righting --bands`): upright in none of the four
that worked. Not trusted for designing a getting-up.
