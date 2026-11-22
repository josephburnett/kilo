# The walk: the next floor try

What to do the next time Kilo walks (remote key C4, as before), what to
watch, how to stop him, and how to tell whether he walks straight. Write
what you saw in [walk-trial-results.md](walk-trial-results.md); "don't
know" is a fine answer. The design and its numbers are
[walking-and-balance.md](walking-and-balance.md) section 16.

**What the walk is.** The way a cat walks slowly: one leg at a time,
always in the same order (left back, left front, right back, right
front), so he always has at least three feet on the floor. You saw it on
2026-11-22 (boot 16) and liked it; the recording of that run showed him
still rocking from side to side about as far as the old walk did, because
at one moment in every step his weight was right on the edge of the three
feet holding him up.

**What changed since boot 16** (the walk's second version, same character):

- A little more time with all four feet down just before each back leg
  lifts (60 ms instead of 20), and a little less before each front leg
  lifts (20 ms). So the footfalls are no longer exactly even: 0.22 s from
  a back foot to the front foot on the same side, 0.26 s from that front
  foot to the next back foot. You may or may not hear the difference.
- The swing is a tick shorter (0.20 s instead of 0.22) at the slower
  speeds, to make room for that; at the top two speeds it is 0.22 s as
  before.
- His body sways a little in time with the steps: it leans about 1 degree
  away from the side whose legs are stepping, and moves 3 mm forward over
  his feet while a back leg steps and 3 mm back while a front leg steps.
  Both are small; they move his weight away from the edge before each
  foot lifts.
- The speeds are the same (23 to 73 mm a second, 58 by default).
- Walking backward is exactly as on boot 16.

In the computer's simulation (not tried on Kilo) this rocks him about
40 % less at the default speed and his weight stays 1 to 2 mm inside his
feet instead of on the edge. The slowest speed (23 mm a second) is not
better.

## Before you start

- The mind on Kilo must be a build with the walk's second version. Claude
  tells you when it is there.
- A charged battery.
- **A clear, flat, hard floor at least 2.5 m long, with a straight line
  on it**: the edge of a floorboard, a tile joint, or a strip of tape
  stuck to the floor. This is for judging "straight".
- A phone for a video, if you like (from behind him is the most useful
  view for straightness; from the side for the rocking).

## The try

1. Put Kilo at one end of the line, his body along it, the line running
   under the middle of his belly from tail to nose. Check from above
   that he points along it.
2. **B6**: he stands. Check again that he still points along the line;
   nudge him straight if not.
3. **C4**: he walks forward at the default speed. Do not steer.
4. Let him walk about 2 m (about 35 seconds at this speed). Watch from
   behind him, looking along the line.
5. **B6** to stop him.
6. Note how far the middle of his body is from the line, left or right
   (a ruler across the line), and which way his nose points: along the
   line, or turned left or right (roughly how much: a little, about
   10 degrees, more).
7. Carry him back to the start, line him up again, and do it once more
   at the top speed: **C4**, then **B7** twice (he is at the top after a
   second or two), 2 m, **B6**, note the same two things.
8. If you like: once more at the default speed, with one press of **C6**
   (steer right) right after C4, to see whether that makes him straight
   as it seemed to on boot 16. Note the same two things.

Take as long as you like between the steps; nothing is waiting on a timer.

## How to stop him

- **B6** stops him: his foot in the air comes down where it is, he is
  still within about a second, then he squares his legs and stands.
- **C7** (freeze) stops him the same way and then ignores everything but
  B6 and A7.
- **Picking him up is fine**, at any moment: the walk ends on its own.
- The distance guard stops him by itself if something is close ahead.
  Do not count on it for the end of the floor: stop him with B6.

## What to watch for

- **Does it still look like the walk you liked?** If it looks worse,
  jerkier, or he seems to surge forward and back, say so: that is the
  sway, and it can be made smaller.
- Does he rock from side to side less than on boot 16, the same, or more?
- Does a foot scrape or drag as it swings forward?
- **Straightness** (steps 6 and 7): how far off the line, which side, and
  where his nose points. On boot 16 his own gyro says he turned left by
  about 2 degrees a second at the default speed and 3.6 at the top; over
  2 m that would be a turn of about 60 degrees, so even a rough answer
  (a little / a lot, left / right) helps.
- Does the drift differ between the default and the top speed?

Write in the results file: the date, what you saw for each point above,
the distances from step 6, and where any video is.
