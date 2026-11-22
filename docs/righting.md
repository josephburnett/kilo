# Getting up after a fall: the righting experiment

**Short answer (2026-11-22).** There is now a getting-up that the mind
works out from the body model instead of the fixed sequence in Kilo's
body. It is an **experiment**, off unless you build it in, and nothing of
it has been tried on Kilo. It was checked only in the simulator, and that
simulator still cannot reproduce the getting-ups recorded on Kilo (see
"How much to trust it" below). So the first try is the real test: on a
soft floor, with you watching.

Without the experiment Kilo gets up as before: the body's own fixed move.
You reported on 2026-11-22 that this built-in getting-up "didn't work 100%
of the time" (your words; no count was given).

## Words used here

- **The fall reflex**: when Kilo is tilted more than 75 degrees for a tenth
  of a second, his body (the NyBoard) takes the legs over, chirps, waits
  until he lies still, and then (as before) plays **rc**.
- **rc**: the body's fixed getting-up move ("recover"), three poses in 1.8
  seconds, copied from the original Petoi firmware. Not planned from
  anything.
- **The model / the simulator**: the program on the laptop that plays a
  move on a made-up Kilo (his weights, his legs, his outline, the floor's
  friction, servos that lag and give) and says what would happen
  (`mind/kilo/tricks/dynamics.py`, lying down: `mind/kilo/tricks/righting.py`).
- **Roll angle**: how far he is turned about his long axis: 0 upright on
  his feet, -90 on his right side, +90 on his left, 180 on his back. In the
  recordings he never lies flat on a side: he lies 26 to 40 degrees further
  over toward his back, propped on the shoulder servos and his back.
- **The state estimate**: the mind's opinion of where he is (standing, on
  his left side, on his back, ...: docs/intent.md).
- **FALL_RC**: a new switch in the body. 1 (always, unless the mind says
  otherwise): the fall reflex plays rc. 0: it never plays rc; it lies him
  down and lets go of the legs, and takes the mind's righting move instead.
  It is never saved: the body starts with 1 every time it is switched on.
- **Stall**: a servo pushing against something it cannot move. It heats up
  and draws a lot of current. That is the risk of a getting-up.

## How the move was worked out

All from the body model (`mind/kilo/intent/righting.py`, which says the
same at more length).

1. **He rolls about his long axis.** In every recorded fall and every try
   of rc his nose stayed within about 10 degrees of level. So the question
   is simple: for a pose of the joints, how high is his centre of mass at
   each roll angle? He rolls downhill.
2. **No pose on its own can do it.** His legs swing only forward and back
   (there is no sideways joint). Lying on a side, the legs underneath can
   push only toward his back or toward his belly. The model tried 3,675
   poses of the legs and head, each in 19 versions of the uncertain
   numbers: in none (at best 1 of the 19 versions) does he roll from where
   he lies on a side down onto his belly. The legs underneath can lift him
   only to about 15 degrees past his side, and there his centre of mass is
   still on the back side of the shoulder servo he pivots on.
3. **So it has to be a throw**, as rc is. The move, for his right side down
   (mirrored for the left side):
   1. **Wind up** (slowly, 0.6 s, then a 0.5 s pause): the legs underneath
      go forward and back along his body with the knees bent, feet clear
      of the floor; the legs on top reach toward his back; the head lifts.
   2. **Kick and throw** (as fast as the servos turn, about 0.2 s): the
      knees underneath straighten, so those feet hit the floor beyond his
      back and push it away, which turns him toward his belly; at the same
      moment the legs on top swing over to the standing pose (their weight
      goes toward his belly) and the head tucks.
   3. **Fold**: the legs underneath straight along his body, out of the way
      of the roll.
   4. **Rest**: the lying-down pose. From there the model-planned "rest to
      stand" of docs/intent.md stands him up.

   In the simulator the kick alone gets him over in 27 of 95 runs, the throw
   alone in 12, both together in **89 of 95**.
4. **From his back**: the left legs wind up the same way and kick at the
   floor beyond his back, lifting his left side; the right legs stay as for
   standing, so he stops on his right side. Then the side move follows. In
   the simulator he ends on his right side in **68 of 76 runs**.
5. **Left alone**: the head's turn and the tail. Which way a positive angle
   turns them is not known, so the moves keep them straight.

"Runs" are each move from several lying positions (on a side 110 to 130
degrees, sitting or standing or resting legs; on his back 172 to 180), each
in 19 versions of the numbers the model is unsure of: his centre of mass 10
mm higher or lower or 3 mm to either side, softer legs, each guessed size
of his outline a quarter bigger or smaller, the outline that makes rc's
first pose behave as it did on Kilo, the tail turning the other way, a
slippery and a grippy floor, slow servos.

Where it fails in the simulator: with his centre of mass 10 mm higher than
the model has it (the kick then throws him onto his back instead, every
time), and from his back when the top of his back is a quarter higher or
lower than guessed. The leg servos carry at most 1.1 kg cm in any run; they
are thought to stall at about 3.

## How much to trust it

Not much, yet. The same simulator was asked to replay rc against the six
recorded tries on the floor. With what was added today (the head's weight
moving with the head, the legs' and head's own swing turning the body, the
tail's direction as an unknown) it still gets **none** of the four
getting-ups right. It goes wrong in three different places, one per pose of
rc, and each points at a part of Kilo that has never been measured: the
shape of his back with the Pi on it, and his head
(docs/righting-results.md has the numbers). The getting-up designed today
was checked across those unknowns, which is the best that can be done
without measuring them; but the simulator it was checked in is known to be
wrong about lying down.

What is on the safe side whatever the simulator gets wrong:

- **Bounded.** At most three tries per press of B1. Each try lasts 2.7
  seconds. After the third, or when nothing more can be done, the mind lets
  every servo go (limp), logs `righting gave up: he is left LIMP on the
  floor, he needs a hand`, and Kilo **meows** once.
- **No long stall.** After each try the body's own fall check comes back;
  if he is still down it lies him down and lets the legs go limp 2 seconds
  later, whether the Pi is there or not.
- **Loads checked**: no leg servo over 2.4 kg cm in any simulated run (it
  came out at 1.1).
- **Any other key stops it** (the move stops and lies him down). Or pick
  him up.

## How it is switched on

1. A kernel built with `KILO_RIGHTING=1`. Without it (the default) B1 does
   nothing but log that the experiment is not built in, and the body's fall
   reflex works as always.
2. The body firmware of the same build (protocol 2.1: it knows FALL_RC).
   `kilo card` writes both; the mind flashes the body at its next boot.
3. With both, at every start the mind sets FALL_RC to 0: from then on a
   fall no longer plays rc. **He does not get up by himself any more**: he
   lies down where he fell and goes limp. B1 gets him up.
4. To go back: write a card without the switch. The next start has rc
   again.

B1 acts only when the state estimate says he is on his left side, his
right side or his back. Otherwise it is refused, with a line in the log
such as `righting refused: he is STANDING: B1 rights him only from a side
or his back`. Nose or tail down: nothing (the intent gives up at once).

## First try (no step is timed)

Before: a charged battery; a rug or a folded blanket on the floor with a
hand's width of space all round.

1. Build and write the card, on the laptop, with the card in:

   ```
   make mount
   kilo card /mnt/e KILO_RIGHTING=1 KILO_RECORD_SAVE=1
   ```

   (`KILO_RECORD_SAVE=1` saves the flight recorder to the card, so the try
   can be read afterwards.)
2. Card into Kilo, switch on. The first start after a new card also
   updates the body's program: wait until he has trilled (the update is
   done) and mewed (ready).
3. Press **B6** (column B, sixth row from the bottom): he stands.
4. Lay him gently on the blanket on his **right side**, and take your hands
   away. He chirps, lies down (legs fold) and a little later goes limp.
   That is the new behaviour: no rc.
5. Press **B1** (column B, bottom row). Hands off. Watch:
   - the legs underneath go forward and back, the top legs reach back, the
     head lifts; a short pause;
   - a sudden kick: does he roll onto his belly? onto his back? not at all?
   - then the legs fold; if he is on his belly he stands up rear first.
6. If he is down again he tries again by himself, up to three times in
   all, then goes limp and meows. That is the end of it; nothing more
   happens until you press a key.
7. Stand him up by hand if needed, press B6, and do the same on his **left
   side**, then once **on his back**. As many times as you have patience
   for; two each way already tells a lot.
8. Stop at any moment by pressing B6 or A7, or by picking him up. If a leg
   is caught under him or a servo buzzes loudly, pick him up at once.

## What to tell Claude

For each try, one line: which side, what each try did (rolled onto his
belly / onto his back / did not move / something else), how it ended
(standing / on his belly / limp and meowed), anything that looked strained
(a buzzing servo, a leg bent under him). "Don't know" is fine. Then switch
him off, card into the laptop, `make mount`, `kilo card /mnt/e --fetch`, and
tell me: the log lines starting `intent:` and `righting` and the recording
show the roll angle through every try, which is what the simulator needs.

## What would make the simulator trustworthy

Measuring him (a ruler, Kilo switched off): the widths across the front and
back shoulder servos and across the knees; lying on his side, what touches
at the top (the Pi's corner? a connector?) and how high above the shoulder
axles; the head's length, width and the ear tips' height; and which way he
looks for a positive head turn. Write them in docs/righting-results.md.
Photos from above, the side and the front with a ruler in each do as well.
The first try above gives the other half: falls and getting-ups recorded
with your hands off.

## Where things are

- `mind/kilo/intent/righting.py`: the reasoning, the two moves and their
  check; `mind/kilo/intent/righting/*.json` its results.
- `mind/kilo/tricks/righting.py`: Kilo lying in the simulator, rc replayed
  against the recorded falls (`--phases`, `--bands`), `--extract` to read
  falls from a recording; `mind/tests/fixtures/falls_2026_11.json` the
  recorded falls.
- The mind: `kernel/mind/intent/` (the Right steps, at most three, limp at
  the end), B1 in `kernel/mind/remote/`, FALL_RC set in `kernel/mind/mind.cpp`.
- The body: FALL_RC in `body/src/core/body.cpp` (64 bytes of program, 2 of
  memory).
- Tests: `mind/tests/test_righting.py`, `mind/tests/test_intent.py`,
  `kernel/host/tests/test_intent.cpp`, `body/test/test_reflex.cpp`, and
  `kernel/host/test.sh` "righting" (B1 against the real body logic on the
  laptop: FALL_RC, the moves, three tries, limp).
- `docs/righting-results.md`: what the simulator reproduces, your
  measurements and tries.
