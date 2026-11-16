# Rear-up trick: results

Runs of docs/rear-up-trick.md. Newest first. Copy the block below for each
run and fill in what you saw; "don't know" is a fine answer.

## YYYY-MM-DD, run N

Build (mind kernel, with `KILO_REAR_UP=1`, `2` or `3`): ...   Battery at the start: ... mV
Floor: ...   Paper under the tail tip (stages 2, 3): yes / no
Foam or cardboard over the Pi: yes / no   Tail firmly on its servo, straight back in stand: yes / no

### Checks and measurements (section 2)

| | Answer | In the model |
|---|---|---|
| T1 from stand to sit, the tail tip swings to one side and rises | | yes (76 mm aside, 28 mm up at -60) |
| T2 in sit (tail -60), the tip is on Kilo's left / right | | not known |
| T3 frame's bottom back corner: behind the hind axles / below them, mm | | 27 / 24 (estimate) |
| T4 Pi: back edge behind (+) or ahead (-) of the hind axles / highest point above them, mm | | -6 / 36 (estimate) |
| T5 in sit, what touches the floor | | the four feet (rear end 3 mm up) |
| T6 tail weight, g | | 6 (estimate) |

### Stage 1 (`KILO_REAR_UP=1`, 28 degrees; section 5)

| Step | Tries | What you saw (tail tip, tail pushed aside?, rear end, sliding, leaning, rocking back, front feet light?) | Stopped because |
|---|---|---|---|
| 1 onto the tail | | | |
| 2 the step | | | |
| 3 paws up: on the tripod / paws stayed down / came back down | | | |
| 4 all the way | | top pitch from `monitor`: ... (model: -25 to -31) | |
| 5 toggle: B2 / B6 / C7 at the top | | | |
| 6 hands further away | | | |

### Stage 2 (`KILO_REAR_UP=2`, 40 degrees)

| Step | Tries | What you saw (tail tip sliding smoothly / sticking, tail pushed aside?, hind feet, knees, tipping forward?) | Stopped because |
|---|---|---|---|
| 7 the rise | | | |
| 8 all the way, then the toggle | | top pitch from `monitor`: ... (model: -37 to -43) | |

### Stage 3 (`KILO_REAR_UP=3`, 54 degrees)

| Step | Tries | What you saw (as stage 2; does it look like the photo? forward creep?) | Stopped because |
|---|---|---|---|
| 9 the rise to the top | | how far the tail tip slid: ... cm (model: about 6) | |
| 10 all the way, the toggle and C7 at the top | | top pitch from `monitor`: ... (model: -51 to -57) | |
| 11 hands further away | | | |

Hind shoulder servos after the highest stage tried: cool / warm / too hot to hold
Anything else:
