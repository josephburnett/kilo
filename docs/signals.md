# What Kilo shows you: his eyes

Kilo's "eyes" are the six small coloured lights around the distance
sensor on his head. They are how he tells you what is going on: that he
is ready, busy, done, or that something did not work. This page is the one
place where every colour and flash is listed; the other documents point
here.

He can also play a sound for most of these (a mew, a trill, a hiss), but
**the speaker is not wired up** (the owner's report, 2026-11-22), so the
sounds are only heard once a speaker is connected (`docs/sound.md`). Read
the eyes, not your ears.

## The eye language

There are three kinds of signal, and they are easy to tell apart:

- **steady** colour: how he is right now;
- **white blinking slowly**: he is busy with a job; keep your hands off and wait;
- **three quick flashes** (about one second in all): an answer. The colour
  says which: green good, purple refused, red failed. Then the eyes go back
  to their steady colour.

| The eyes | What it means | When you see it |
|---|---|---|
| dark | the Pi has not reached his board yet | the first two or three seconds after switching on (and always, if his board has the old firmware from before 2026-11-17, which has no eyes) |
| steady **green** | awake, nothing in front of him | most of the time |
| steady **orange** (dim) | tired: the battery is low | the battery at rest under about 5 % (7000 mV), or his board's own low-battery mode; until it is back at 10 % (`docs/distance-guard.md`) |
| steady **blue** | something close in front of him | the distance guard (`docs/distance-guard.md`): about 12 cm standing or sitting, about 20 cm walking forward |
| steady **red** | something very close in front of him | the distance guard: about 8 cm standing or sitting, 15 cm walking forward |
| dark, a short dim green blink every 5 seconds | light sleep | 30 seconds after A7 with nothing happening (`docs/power.md`); any key wakes him |
| **white, blinking slowly** (half a second on, half a second off) | busy: hands off, wait | after the second C5: while he stands still before the self-measurement and all through it (`docs/self-measurement.md`); while the righting experiment (B1) is trying to get him up (`docs/righting.md`) |
| steady **white** | his board is being reprogrammed from the SD card | at switch-on, when the card has a new body firmware (`docs/laptop-setup.md`); 10 to 20 seconds. The Pi's green light gives two short flashes a second meanwhile |
| **3 quick green** flashes | ready, or done | he is switched on and ready (his board answered the Pi); he woke from light sleep; the board was reprogrammed and works; the self-measurement finished; the righting experiment got him on his feet |
| **3 quick purple** flashes | refused: it did not start | a key he cannot do now (the rear-up on a low battery, the self-measurement while he is tilted or on a low battery, an experiment the kernel was built without, a walk the planner refused); a forward key while something is ahead of him; the board's new firmware refused (battery too low, or the files on the card damaged) |
| **3 quick red** flashes | failed, or gave up | reprogramming the board failed (and the board still answers); the self-measurement stopped by itself (tipped too far, a fall, the link dropped); the righting experiment failed or gave up (when it gives up he is left limp on the floor) |
| **1 long white** flash (one second) | a recording is safe on the SD card | after a walk, a trick or the self-measurement, a few seconds after it ends, once it is saved. Only when saving to the card is on (`kilo record save on --host`, or a kernel built with `KILO_RECORD_SAVE=1`). After it, switching him off loses nothing of that run |

Why a key was refused or what failed is always in his log
(`kilo log --host kilo.local`, or on the card: `kilo card CARD --status`):
every signal is a line such as `signal refused why=key refused`, next to
the remote's own line for that key, which says why (`remote A1 ...:
rear_up refused: battery ...`).

### Which one wins

1. **Steady white** while his board is being reprogrammed: nothing else
   can show then anyway (below).
2. **Blue and red** from the distance guard: what is in front of his face
   always shows, also while he walks and in the middle of an answer.
3. An **answer** (the quick flashes, the long white flash). One at a time;
   if another comes while one shows, it follows it (and if a third comes
   meanwhile, it replaces the second). An answer that comes while the
   eyes are blue or red waits until they are green again, but at most 3
   seconds; after that it is dropped (the log still has it).
4. **Light sleep**.
5. **Busy** (white blinking).
6. The **steady colour**: green, or orange when tired.

## What the eyes cannot show

His board drives the lights, but only with what the Pi sends it. It does
not change them on its own: they keep the last colour they were sent until
the next one comes. So:

- **While his board is being reprogrammed** it cannot drive its lights at
  all. The Pi sends steady white just before it starts (a sixth of a
  second before), and the lights keep that white until the board runs
  again. That they really stay white through the board's restart is
  expected from how these lights work, but has not yet been seen on Kilo.
- **If reprogramming failed and the board has no working program left**,
  it never answers: the eyes stay white, and the Pi's green light blinks
  slowly (one second on, one second off: "no body"). Then the red flashes
  cannot come. Switch off, then `kilo card CARD --status` on the laptop
  says what happened.
- **If the Pi stops** (it crashed, it is restarting after a kernel update,
  or it lost its power) the eyes stay frozen at their last colour, for
  example green, although nobody is driving him. His board then lies down
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

If a speaker is connected, each answer also plays a sound: ready a `mew`,
done a `trill`, refused and failed a `hiss`, the righting experiment
giving up a `meow`. The distance guard hisses when it pulls him back from
something very close (its red shows on the eyes already). The steady
colours, busy, light sleep and the saved recording have no sound.
`docs/sound.md` lists every sound.

## How it works

`kernel/mind/signal/signal.h` is the eye language and its only
implementation. Everything that tells the owner something calls one
function, `Mind::signal()`, with the signal's name (ready, done, refused,
failed, gave up, saved, flashing, guard red); it writes the log line,
plays the sound, and hands the signal to the eyes. Every pass the Mind
works out what the eyes should show (the list above, highest first) and
sends it to his board as the `SET_EYES` goal. The distance guard only
names its colour; the Mind sends it.

His board shows a new colour as soon as it arrives, except while it is
timing an echo of the distance sensor (up to 25 ms in every 80 ms: the
lights' signal would disturb the timing). So each change is sent three
times, 40 ms apart, and a quick flash is never lost or stretched. On top
of that the link sends the colour again five times a second, as every
goal.

Nothing here changed his board's program; the eyes cost the board nothing
new.

## What was tested, and what must be checked on Kilo

Tested on the laptop only (`kernel/host/tests/test_signal.cpp`, with the
distance guard's own tests in `test_guard.cpp` and the boot flash's in
`test_flash.cpp`): every pattern and its timing, the order of priority,
an answer waiting for green and being dropped, the colours sent to a
simulated board (each change three times, then the 5 Hz refresh), and in
the whole mind on a simulated board: the ready flashes after switch-on, a
refused key's purple, the busy blink after C5 twice, the red flashes when
the self-measurement stops itself, and light sleep's dark eyes.

Not yet seen on Kilo:

1. That the flashes are easy to see and to tell apart (purple from blue
   and red, in daylight), at the guard's brightness (`0x30` of 255).
2. That the steady white stays on while his board is reprogrammed.
3. That no flash is lost or visibly stretched while the distance sensor
   measures.
