# Getting up after a fall: can it be planned from the model?

**Short answer (2026-11-21): not yet.** The model that the rear-up trick was
designed in cannot tell a getting-up that works from one that does not. It
was put to the test against every fall in Kilo's recordings and it failed
the test that was set for it beforehand. So no new getting-up move has been
designed: a move designed in a model that gets the old one wrong would be a
guess. What is missing is not cleverness but measurements: the shape of
Kilo's head and sides, which the model has never needed before, and a
handful of recorded falls where it is known that no hand touched him. Both
are listed below, as things you can do when you like.

Until then Kilo gets up the way he does today: the move stored in his body.

## Words used here

- **The fall reflex**: when Kilo is tilted more than 75 degrees for a tenth
  of a second, his body (the NyBoard) takes the legs away from whatever was
  moving them, chirps, waits until he lies still, and plays a fixed
  getting-up move. Nothing on the Pi is involved.
- **rc**: that fixed move's name in the old firmware ("recover"). Three
  poses played one after the other, 1.8 seconds in all. It is a table of
  joint angles copied from the original Petoi firmware, not something
  planned.
- **A try**: one play of rc. If Kilo is not upright a second later, the body
  waits until he is still and tries again, mirrored left to right, up to
  three times. Then it gives up, lies him down and lets the legs go limp.
- **Mirrored**: the same move with left and right swapped. The body picks
  the side from the way he lies.
- **The model / the simulator**: the program on the laptop that plays a move
  on a made-up Kilo (his weight, his legs, the floor's friction, servos that
  lag and give a little) and says what would happen. The rear-up trick was
  designed in it (`mind/kilo/tricks/dynamics.py`).
- **On his back, on his side**: said of how the tilt sensor reads. In the
  recordings Kilo never lies flat on a side: he lies on a side rolled 26 to
  40 degrees further toward his back, propped on the shoulder servos and the
  top edge of his body.

## What the recordings show

Every recording on the laptop (17 to 21 November, the flight recorder's
files from the card) was searched for falls. There are nine. The older
ones, from October, were recorded on the Pi's old system and are not on the
laptop; your one report of a getting-up you watched (11 October, "got up on
its own") is among those.

| When | What he was doing | How he lay | What rc did |
|---|---|---|---|
| 17 Nov, twice | the first rear-up design went over | on his back, nose a little up | got him up at the first try, both times |
| card boot 3, three falls | a crawl stopped with C7, then standing | on his back or his left side | **in a hand** (see below) |
| card boot 8, first | sitting, tapped | on his right side | got him up at the first try |
| card boot 8, second | standing | on his left side | first try: nearly up, fell back onto his back. Second try (mirrored the other way): ended on his left side again. Third try: got him up |
| card boot 8, third | standing | nose or tail down | no try: the body does not try from there, it gives up at once |
| 21 Nov | drowsy | upright again before rc began | **in a hand** |

`python -m kilo.tricks.righting --falls` prints the list with the numbers;
the recordings' tilt readings through each try are kept in
`mind/tests/fixtures/falls_2026_11.json`.

**In a hand.** The recordings do not say whether you were holding him. Two
things do, indirectly. First, around four of the nine falls the Pi's own
"picked up" detector fired (it watches the tilt sensor for the way a lift
feels). Second, in every try that began with Kilo upright, he stayed within
8 degrees of level all through rc, and rc's second pose lifts both legs of
one side off the floor: a Kilo standing free cannot stay level through
that. So those tries are read as "held", and they say nothing about the
floor. That is an inference from the recordings, not something you
reported.

That leaves **six tries on the floor**: four that got him up and two that
did not. Two of the six began the same way to within one degree (on his
left side, the second fall of boot 8) and ended differently. From there rc
is on a knife edge.

What rc does, read off those six: lying on a side, its first pose rolls him
onto his back; its second pose rolls him from his back over one side
(always the side whose legs it folds away); its third pose is the standing
pose, which pushes him the last 45 degrees onto his feet. Both tries that
failed were mirrored ones (he was to go over his left side); the three that
were not mirrored all worked. Six tries are too few to call that a pattern,
but it is worth knowing: Kilo is not the same on both sides (his right hind
shoulder had less travel until it was re-seated).

## The test the model was put to

Before the first run in the simulator the test was written down
(`mind/kilo/tricks/righting.py`, "THE CRITERION"). The model would be
trusted for designing a getting-up only if all of this held, with nothing
adjusted to make it come out:

1. **It lies the way Kilo lay.** Put down in each recorded position, the
   made-up Kilo stays within 15 degrees of it.
2. **It gets the outcomes right.** It gets up in every try where Kilo got
   up, and does not in the tries where he did not. And for the knife-edge
   start, its answer flips somewhere inside the range of the numbers we are
   unsure of.
3. **It moves the way Kilo moved.** In the tries that worked, it passes
   through "on his side" within a quarter of a second of when Kilo did, and
   stays within 30 degrees of his tilt on average.

One change was made to the test after the first run, and it is written
there with the reason: the tries "in a hand" were taken out, once the first
run had shown that a free-standing Kilo cannot do what those recordings
show.

To lie down at all, the simulator needed things it never had: a start in
any position, a head that the two head servos move, and how far the
shoulder and knee servos stick out sideways. **None of those sizes has been
measured.** They are estimates, and three of them (how far the shoulder
servos stick out, how high the ears are, how high the top edge is) were set
so that point 1 comes out right. So point 1 is not a test any more; points
2 and 3 are.

## The result

`python -m kilo.tricks.righting --bands` (a minute or two):

| | Needed | Got |
|---|---|---|
| 1. Lies as Kilo lay | within 15 degrees | within 11 degrees (after setting three sizes on it) |
| 2. Outcomes | up in all 4 that worked, not in the 2 that did not | **up in 0 of the 4**; not up in the 2 |
| 2b. The knife edge flips | somewhere in the range | it does: in 6 of 44 variations |
| 3. Moves as Kilo moved | all 4 | **none of the 4** |

**The model fails.** In it, rc does not get Kilo up from any recorded
start. What it gets right and wrong, try by try:

- From his right side (boot 8, the first fall) the model follows the real
  roll closely: it goes through "on his side" at 1.34 seconds, Kilo at
  1.36. Then, 45 degrees from upright, the made-up Kilo stops, and falls
  back. The real one stood up.
- From his back (17 November) the model rolls him the wrong way. On his back
  Kilo rests on his ears, and what turns him over is mostly his head
  pushing sideways on the floor. The model's head is a guessed box, and it
  is not even known which way a positive head turn goes.
- Which tries the model gets up in changes with the guesses: with the
  shoulder servos sticking out a quarter less, the top edge a quarter
  higher or wider, the head turning the other way, the centre of mass 3 mm
  to the left or 10 mm lower, two or three of the six flip. The floor's
  friction, the servos' speed and the legs' give flip none.

So the outcome hangs on numbers nobody has measured. That is why no move
was designed.

One more thing the model says, for what it is worth: the leg servos carry
about 1.1 kg cm at most during rc, well under the 3 kg cm they are thought
to stall at. The head and tail servos' loads are not worked out, and on his
back the head carries a good part of him.

## What would make it trustworthy

Two things, both yours to do when you like. Neither is urgent: he gets up
as he always has.

### 1. Measure him (a ruler; ten minutes; Kilo switched off)

Write the numbers in `docs/righting-results.md`. Millimetres; "about" is
fine; "don't know" is fine.

| What | How |
|---|---|
| Width across the front shoulder servos | outer face to outer face, the widest point |
| Width across the back shoulder servos | the same |
| Width across the knees | with the legs in the standing pose, the widest point of the knee servos |
| The top edge he lies against | lay him on his side on the table: what touches at the top (the Pi's corner? the frame?), how far is that point above the line through the shoulder axles, and how far from his middle |
| The head | from the front shoulder axles to where the neck turns; neck to the tip of the nose; the head's width; how far the ear tips are above the shoulder axles with the head straight |
| Which way the head turns | not a measurement: when he looks to his left, note it against the remote key or pose that did it (I will give you one pose to try when you are ready) |

A photo from above, one from the side and one from the front, each with the
ruler in the picture, would do as well as the table.

### 2. Record falls with your hands off (a soft floor; whenever)

The recordings above cannot say whether you were holding him. These will.

What is needed first: a card whose kernel saves the flight recorder (ask me
for it: `kilo card` with `KILO_RECORD_SAVE=1`).

1. Kilo on a rug or a folded blanket on the floor, a hand's width of space
   around him. Press **B6** (column B, sixth row from the bottom): he
   stands.
2. Tip him over gently onto his **left side** and take your hands away at
   once. He chirps. Do not touch him until he is standing again or has gone
   limp (up to three tries, about ten seconds).
3. Write one line in `docs/righting-results.md`: which side, hands off
   before the chirp (yes/no), what he did at each try (rolled onto his back?
   got up? fell back?), and where he ended.
4. If he ended limp or lying down: stand him up by hand and press B6 again.
5. Do the same onto his **right side**, and once more leaving him **on his
   back**. Then again, in any order, as many times as you have patience
   for. Five each way would settle it; even two each way would tell a lot.
6. Switch him off, put the card in the laptop, and tell me. I will read
   the falls off the card (`kilo card /mnt/e --fetch`, then
   `python -m kilo.tricks.righting --extract`).

No step is timed. If something looks wrong (a leg caught under him, a
servo buzzing loudly), pick him up: he goes on playing in your hand and
then stands; note that it happened.

## What Kilo's body would need for the Pi to do the getting-up

For when a move planned on the Pi exists. Read from the code; nothing here
is built.

**Today.** Once fallen, the body holds the legs itself until he is upright
or it has given up. It refuses every command from the Pi with "fallen",
except "go limp" (the Pi's emergency stop), which ends the reflex. A trick
cannot be started while he is fallen, nor from limp. Tricks have a flag,
"a fall is fine" (`FALL_OK`), that switches the fall reflex off while the
trick plays; nothing uses it yet.

**The smallest change** (body firmware, `body_trick_play` in
`body/src/core/body.cpp`): let the Pi start a trick that carries `FALL_OK`
while he is fallen; the reflex lets go of the legs and the trick plays.
About 20 to 40 bytes of the body's program (an estimate: 27,472 of 30,720
bytes are used) and none of its memory (which is the tight one: 18 bytes
to spare above the stack's minimum). No new message; the protocol's text
for "play a trick" changes, so its minor version goes up.

With that, a fall goes: the body reports FELL and waits until he lies
still (0.4 to 1.4 seconds in the recordings); in that time the Pi sends
its getting-up trick for the way he lies and starts it; when it ends, the
body's fall check is on again, and if he is still down it reports FELL
again and the Pi chooses again. If the Pi is not there, or too slow, the
body plays rc as today. That keeps today's behaviour until the Pi's move
has been seen to work on Kilo.

**Then**, once it has: rc can go from the body (you said he needs no
fallback: "If he's asleep or off, he's just asleep or off"). That frees
about 40 bytes of table and an estimated 150 to 250 bytes of code, and 3
bytes of memory.

**Not possible without a change**: "go limp", then a posture, then the
trick works on paper, but the fall check comes back on as soon as he leaves
limp and trips again within a tenth of a second unless the trick has
already started. It would work some of the time.

**Nose or tail down**: the body does not try at all (it lies him down and
goes limp). The one recorded case ended that way. A move for it would be
new; nothing is known about it.

## Where things are

- `mind/kilo/tricks/righting.py`: the lying-down additions to the
  simulator, rc as the body plays it, the test and its verdict, and
  `--extract` to read falls out of a recording.
- `mind/tests/fixtures/falls_2026_11.json`: the nine falls.
- `mind/tests/test_righting.py`: checks of the above (fast; the full
  replay is the `--bands` command).
- `docs/righting-results.md`: for your measurements and the hands-off
  falls.
