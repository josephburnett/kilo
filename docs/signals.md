# What Kilo shows you: his eyes

Kilo's "eyes" are the six small coloured lights around the distance
sensor on his head. They are how he tells you what is going on: that he
is ready, busy, done, or that something did not work. This page is the
one place where every colour and flash is listed; the other documents
point here.

**Ready is the one to know: after switching him on his eyes stay dark,
and when they turn steady green he is ready.** Ready means his board
answers the Pi, any reprogramming of his board from the SD card is over,
and the remote's keys are acted on.

He can also play a sound for most of these (a mew, a trill, a hiss), but
**the speaker is not wired up** (the owner's report, 2026-11-22), so the
sounds are only heard once a speaker is connected (`docs/sound.md`). Read
the eyes, not your ears.

## The eye language

There are four kinds of signal, and they are easy to tell apart:

- **dark** after switching on: not ready yet; **steady green**: ready;
- **steady** colour: how he is right now;
- **white blinking slowly**: he is busy with a job; keep your hands off and wait;
- **three quick flashes** (about one second in all): an answer. The colour
  says which: green good, purple refused, red failed. Then the eyes go back
  to their steady colour.

| The eyes | What it means | When you see it |
|---|---|---|
| dark | not ready yet | from switching on until he is ready: a few seconds, or 10 to 20 more while his board is reprogrammed from the SD card (steady white then, below). Always dark if his board has the old firmware from before 2026-11-17, which has no eyes |
| steady **green** | ready; awake, nothing in front of him | when he is ready after switching on, then most of the time |
| steady **orange** (dim) | tired: the battery is low | the battery at rest under about 5 % (7000 mV), or his board's own low-battery mode; until it is back at 10 % (`docs/distance-guard.md`) |
| steady **blue** | something close in front of him | the distance guard (`docs/distance-guard.md`): about 12 cm standing or sitting, about 20 cm walking forward |
| steady **red** | something very close in front of him | the distance guard: about 8 cm standing or sitting, 15 cm walking forward |
| dark, a short dim green blink every 5 seconds | light sleep | 30 seconds after A7 with nothing happening (`docs/power.md`); any key wakes him, and the eyes are green again |
| **white, blinking slowly** (half a second on, half a second off) | busy: hands off, wait | after the second C5: while he stands still before the self-measurement and all through it (`docs/self-measurement.md`); while the righting experiment (B1) is trying to get him up (`docs/righting.md`) |
| steady **white** | his board is being reprogrammed from the SD card | at switch-on, when the card has a new body firmware (`docs/laptop-setup.md`); 10 to 20 seconds, then steady green when it worked (ready). The Pi's green light gives two short flashes a second meanwhile |
| **3 quick green** flashes | done | the self-measurement finished; the righting experiment got him on his feet |
| **3 quick purple** flashes | refused: it did not start | a key he cannot do now (the rear-up on a low battery, the self-measurement while he is tilted or on a low battery, an experiment the kernel was built without, a walk the planner refused); a forward key while something is ahead of him; at switch-on, the board's new firmware refused (battery too low, or the files on the card damaged): then dark, then green when he is ready |
| **3 quick red** flashes | failed, or gave up | at switch-on, reprogramming the board failed: then dark, then green if the board still answers (he is usable, with his old firmware); the self-measurement stopped by itself (tipped too far, a fall, the link dropped); the righting experiment failed or gave up (when it gives up he is left limp on the floor) |
| **1 long white** flash (one second) | a recording is safe on the SD card | after a walk, a trick or the self-measurement, a few seconds after it ends, once it is saved. Only when saving to the card is on (`kilo record save on --host`, or a kernel built with `KILO_RECORD_SAVE=1`). After it, switching him off loses nothing of that run |

Why a key was refused or what failed is always in his log
(`kilo log --host kilo.local`, or on the card: `kilo card CARD --status`):
every signal is a line such as `signal refused why=key refused`, next to
the remote's own line for that key, which says why (`remote A1 ...:
rear_up refused: battery ...`).

### Which one wins

1. **Steady white** while his board is being reprogrammed: nothing else
   can show then anyway (below).
2. Once he is ready: **blue and red** from the distance guard. What is in
   front of his face always shows, also while he walks and in the middle
   of an answer.
3. An **answer** (the quick flashes, the long white flash). One at a time;
   if another comes while one shows, it follows it (and if a third comes
   meanwhile, it replaces the second). An answer that comes while the
   eyes are blue or red waits until they are green again, but at most 3
   seconds; after that it is dropped (the log still has it).
4. **Dark** until he is ready.
5. **Light sleep**.
6. **Busy** (white blinking).
7. The **steady colour**: green, or orange when tired.

## What the eyes cannot show

His board drives the lights, but only with what the Pi sends it. On its
own it does one thing: when it starts (switched on, or restarted after
being reprogrammed) it turns them dark. Otherwise they keep the last
colour they were sent until the next one comes. So:

- **While his board is being reprogrammed** it cannot drive its lights at
  all. The Pi sends steady white just before it starts (a sixth of a
  second before), and the lights keep that white until the new program
  starts; it turns them dark, and a moment later the Pi makes them green
  (ready). That they really stay white while the board is written is
  expected from how these lights work, but has not yet been seen on Kilo.
- **If reprogramming failed and the board has no working program left**,
  it never answers: the eyes stay white, and the Pi's green light blinks
  slowly (one second on, one second off: "no body"). Then the red flashes
  cannot come. Switch off, then `kilo card CARD --status` on the laptop
  says what happened.
- **If the Pi stops** (it crashed, it is restarting after a kernel update,
  or it lost its power) the eyes stay frozen at their last colour, for
  example green, although nobody is driving him. When the Pi starts again
  they go dark as soon as it reaches his board, then green when he is
  ready. His board then lies down
  by itself after half a second (its watchdog). Only the Pi's green light
  tells you: steady or dark instead of blinking.
- **If the link to his board is lost** (the board restarted or stopped
  answering), the Pi cannot reach the lights either: they keep their last
  colour, and the Pi's green light blinks slowly.
- **Switched off**, nothing shows.

### The Pi's green light (ACT)

The small green light on the Pi board, next to the red power light. The
Pi drives it itself, so it works when the eyes cannot.

| The green light | What it means |
|---|---|
| steady on | the Pi is starting up (or has stopped: steady for more than 20 seconds is a Pi that did not start) |
| slow blink: 1 s on, 1 s off | the Pi runs but hears nothing from his board |
| 2 blinks a second | normal: the Pi and his board are talking |
| 5 blinks a second | his board speaks another protocol version: flash its firmware |
| two short flashes a second | reprogramming his board, or writing a new kernel to the card: do not switch off |
| a short flash every 5 s | light sleep |

(`kernel/src/kernel.cpp`, `UpdateLED`; `docs/fast-boot-first-boot.md` has
the same table for the first power-on.)

## The sounds that go with them

If a speaker is connected, each answer also plays a sound: ready (and
waking from light sleep) a `mew`, done a `trill`, refused and failed a `hiss`, the righting experiment
giving up a `meow`. The distance guard hisses when it pulls him back from
something very close (its red shows on the eyes already). The steady
colours, busy, light sleep and the saved recording have no sound.
`docs/sound.md` lists every sound.

## How it works

`kernel/mind/signal/signal.h` is the eye language and its only
implementation. Everything that tells the owner something calls one
function, `Mind::signal()`, with the signal's name (ready, done, refused,
failed, gave up, saved, flashing, guard red); it writes the log line,
plays the sound, and hands the signal to the eyes. Ready is said once
(`Mind::ready_step()`, the log line `signal ready why=ready`): on the
first pass with his board answering (its HELLO, the right protocol, its
TELEMETRY coming) once the SD card's flash check is over (the kernel holds
it until then, `Mind::hold_ready`). The remote is listened to from the
start, so from then on keys are acted on. Every pass the Mind
works out what the eyes should show (the list above, highest first) and
sends it to his board as the `SET_EYES` goal. The distance guard only
names its colour; the Mind sends it.

His board shows a new colour as soon as it arrives, except while it is
timing an echo of the distance sensor (up to 25 ms in every 80 ms: the
lights' signal would disturb the timing). So each change is sent three
times, 40 ms apart, and a quick flash is never lost or stretched. On top
of that the link sends the colour again five times a second, as every
goal.

His board's only part in this is the dark at its start (`body/src/hal/eyes.cpp`
`eyes_init`): a few bytes of program, no memory.

## What was tested, and what must be checked on Kilo

Tested on the laptop only (`kernel/host/tests/test_signal.cpp`, with the
distance guard's own tests in `test_guard.cpp` and the boot flash's in
`test_flash.cpp`): every pattern and its timing, the order of priority,
an answer waiting for green and being dropped, the colours sent to a
simulated board (each change three times, then the 5 Hz refresh), and in
the whole mind on a simulated board: dark then green at switch-on (and
dark for as long as the card's flash check holds ready), white then green
after a flash from the card (`kernel/host/test.sh`), a refused key's purple, the busy blink after C5 twice, the red flashes when
the self-measurement stops itself, and light sleep's dark eyes.

Not yet seen on Kilo:

1. That the flashes are easy to see and to tell apart (purple from blue
   and red, in daylight), at the guard's brightness (`0x30` of 255).
2. That the steady white stays on while his board is reprogrammed, and
   that the lights are dark at switch-on before the Pi reaches the board.
3. That no flash is lost or visibly stretched while the distance sensor
   measures.
