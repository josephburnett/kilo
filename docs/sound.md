# Kilo's sounds

Kilo plays recorded cat sounds: a meow when you press B2, a soft mew when
he wakes up, now and then a purr while he sits in the A5 behaviour, and a
hiss when the distance guard has to pull him back from something very
close. The sound comes out of the Raspberry Pi's 3.5 mm headphone jack,
which feeds a small amplifier board (a PAM8403) and a speaker.

The recordings live in `kernel/sounds/` as WAV files. Building the kernel
packs every one of them into the kernel image, so there are no files to
copy to the SD card: installing a new kernel (`kilo update`) brings its
sounds with it. Where each recording came from and its licence are in
`kernel/sounds/SOURCES.md`.

Only the C++ mind that runs on Kilo makes sounds. The old Python daemon
(`kilo run --remote`) has no speaker: B2 does nothing there.

## When Kilo makes a sound

| What happens | Sound | Notes |
|---|---|---|
| You press **B2** on the remote | a meow | one of `meow1`, `meow2`, `meow3`, picked at random, never the same one twice in a row. B2 moves nothing, so it works at any time, also while he walks or runs the A5 behaviour |
| Kilo is ready after switching on (the body first answers the Pi) | `mew` | once per boot |
| He wakes from light sleep (any key, or a laptop) | `mew` | |
| The A5 behaviour, sitting | now and then `purr`, more rarely `trill` or a meow | the first one 20 to 60 s after he settles, then one every 45 to 120 s at most; sometimes nothing |
| The A5 behaviour, standing or looking around | rarely `trill` or a meow | one every 90 to 240 s at most, often nothing |
| The distance guard sees something very close (red) and backs him off | `hiss` | once each time it backs off |
| The distance guard stops him (blue) | nothing | he stops and sits often while you walk him about; a sound every time would soon be too much |
| A trick plays (the rear-up) | nothing | the behaviour's purrs wait. B2 still meows: you asked for it |
| A laptop holds the body (`kilo posture --host`) | nothing from the behaviour | the remote is the laptop's then: B2 is ignored like every other key |
| Light sleep | nothing | the speaker output is switched off (see below) |

### One sound at a time

Kilo plays one sound at a time and leaves at least a second of quiet
between two. If something asks for a sound while one is playing, it
waits; if a second request comes while one is already waiting, the newer
one takes its place (the last thing that happened is what he answers). A
request that has waited more than two seconds is dropped: a meow two
seconds after the key press is no answer to it. So pressing B2 over and
over gives a meow, a pause, a meow, not a stream.

A name with a digit at the end is one sound (`meow2`). Any other name is
a family: the sounds called that name, or that name and a number (`meow`
is `meow1`, `meow2`, `meow3`).

### Volume

The volume goes from 0 to 100. Kilo starts at **70** and with sounds on.
The steps are made to sound about even (the loudness follows the square of
the number: 50 is a quarter of the amplitude of 100). The amplifier
board's own knob, if it has one, comes on top of this.

Neither the volume nor on/off is kept when Kilo is switched off or
rebooted, the same as the distance guard's on/off: he always starts on,
at 70.

## The laptop: `kilo sound`

    kilo sound --host kilo.local                 on or off, volume, how many sounds, what played last
    kilo sound list --host kilo.local            every sound in the kernel: name, length, rate
    kilo sound play meow --host kilo.local       play one (meow2) or one of a family (meow)
    kilo sound off --host kilo.local             no sounds until `kilo sound on`
    kilo sound volume 40 --host kilo.local       the volume, 0..100

Any laptop with a key may do this; it does not need control of the body.
`--host` can be left out: then the laptop looks for `kilo.local`. A play
that cannot happen says why: no such sound, sounds off, the kernel has no
sounds, or Kilo is in light sleep. `kilo status --host` has a `sound`
line, and `kilo metrics --host` a `sound:` line with the counters: sounds
played, requests, unknown names, requests dropped, late audio chunks, and
the longest time the audio interrupt took (`callback_max_us`).

Every sound Kilo starts is a line in his log and a mark in the flight
recorder, so a recording shows what he said and when:

    sound play name=meow2 asked=meow why=key:B2 volume=70 ms=1020

`why` is what asked: `key:B2`, `ready`, `wake`, `behaviour`,
`guard red`, or `host:N` (a laptop).

## Adding or changing a sound

1. Put the recording in `kernel/sounds/` as `NAME.wav`. The name is the
   file name: lower-case letters, digits and `_`, at most 15 characters.
   To add a variant to a family, give it the next number (`meow4.wav`):
   B2 then picks from four. Note where it came from in
   `kernel/sounds/SOURCES.md`.
2. `make -C kernel`. The build prints one line per sound when they have
   changed, and a total:

       SOUND  meow4     0.95 s  22050 Hz  20948 B  (from 44100 Hz stereo 16-bit, peak -6.0 dBFS, gain x1.78)
       SOUNDS 8 sounds, 10.1 s, 218 KB of samples in the image (budget 1536 KB)

3. `kilo update --host kilo.local`, then `kilo sound list --host kilo.local`
   to see it there, and `kilo sound play meow4 --host kilo.local` to hear it.

Any sound can be played by name this way; to make Kilo use a new one on
his own (a new event), the code has to name it.

What the build takes: plain (PCM) WAV files, 8, 16, 24 or 32-bit, mono or
stereo, 8000 to 48000 Hz. Compressed WAV, MP3 and the like are refused with
a message; save the file as 16-bit PCM WAV first. A file that cannot be
used stops the build and says which file and why; nothing is left out
quietly.

What the build stores (`kernel/tools/pack_sounds.py`): one channel (stereo
is averaged); at the file's own rate, or 22050 Hz if it is higher (a small
speaker plays nothing above about 11 kHz, which 22050 Hz covers); its
loudest point brought to just below full scale, so every sound plays about
as loud at the same volume (a very quiet recording is raised at most 4
times, so that its background noise does not become loud); then 8 bits a
sample in "mu-law", the telephone format, which keeps quiet sounds clean
in half the space of 16 bits. That is 22,050 bytes for each second of
sound.

### What sounds cost

The budget is 1.5 MB of sound, about 70 seconds at 22050 Hz. Over it, the
build stops and names the largest sounds; shorten them, drop some, or save
them at 11025 Hz (half the size; still fine for a purr or a hiss).

The kernel image was 689,636 bytes before sound. The sound code adds about
27 KB; each second of sound adds 22 KB. The seven sounds in
`kernel/sounds/` (11.2 seconds together, 241 KB once packed, from 494 KB of
16-bit WAV) make it about 965 KB (965,596 bytes on 2026-11-19).

`kilo update` sends the whole image over the network and reads it back
from the card, so a bigger image takes longer to install. The one timed
update so far (591,140 bytes) was back in 10.7 seconds, reboot included;
if all of that time grew with the size (it does not: the reboot and the
network coming up do not), the 241 KB of sound would add at most 4.4 seconds.
`kilo update` prints how long the mind took to come back; compare it after
the sounds go in.

### Building without sound

`make -C kernel KILO_SOUND=0` leaves the sound device and the sounds out
of the kernel. Kilo then says nothing; `kilo sound` still answers (0
sounds, and every play request says the kernel has no sounds), and the
counters count the requests.

## How it works

The Pi 2 B makes its headphone output with PWM: a pin switching on and
off very fast, the share of "on" time following the sound, smoothed into a
voltage by a filter on the board. The kernel uses Circle's PWM sound
device (`kernel/src/sound.cpp`; Circle is the library the kernel is built
on): 44,100 switchings a second, each in 5669 steps (about 12.5 bits),
fed from memory by DMA (a part of the chip that copies data on its own)
in chunks of 11.6 ms. When a chunk is used up, an interrupt asks the
kernel for the next; the kernel's renderer (`kernel/mind/sound/render.cpp`)
fills it: it decodes the stored samples, converts 22050 to 44100 Hz,
applies the volume, and writes the same value to both channels (there is
one speaker). The player (`kernel/mind/sound/player.cpp`) decides what
plays and when; the hooks above are in `kernel/mind/mind.cpp`.

**No clicks.** Every sound fades in over 5 ms and out over 15 ms, also
when it is cut short, so a recording that starts or ends abruptly does not
click.

**No pop when the output starts or stops.** Silence for the PWM output is
"on" half the time, but a stopped output is off all the time. Jumping
between the two is the "pop" many computers make. The kernel ramps
between them over 80 ms instead: slowly enough that the speaker barely
moves. Whether that is quiet enough with this amplifier has to be heard
(below).

**Stopped between sounds.** By default the output runs only while a sound
plays: started for it (the 80 ms ramp up, then the sound), the ramp down
after it, then stopped. While it is stopped there is no switching on the
pin, so no hiss from it and no interrupts. The other way is
`make -C kernel KILO_SOUND_IDLE=run`: the output starts at boot and keeps
running at silence between sounds (no ramp per sound, 86 interrupts a
second, the switching always on). That is the one to try if the ramps
still pop.

**Light sleep** (A7, then 30 s of nothing: docs/power.md) ramps the
output down and stops it in both modes, so nothing runs or hisses while
he sleeps; any sound asked for meanwhile is dropped. Waking starts it
again ("run") and plays the mew.

**The control loop is not disturbed.** The interrupt that fills a chunk
runs with other interrupts held off, the body link's serial port's
among them, so it must be short: it only runs the renderer's loop over the
512 samples (whole-number arithmetic, no logging, no memory allocated)
and times itself. The serial port's buffer lasts about 0.5 ms past the
point where it asks for attention; the fill is estimated at a few tens of
microseconds (computed, not measured: the metrics line's
`callback_max_us` will tell). A chunk asked for late (more than 1.5 chunk
times after the last) is counted as `late`: it may have clicked.

## What was computed, and what must be checked on Kilo

Nothing here has run on the Pi yet. The tests on the laptop
(`kernel/host/tests/test_sound.cpp`, `mind/tests/test_sound.py`,
`kernel/host/test.sh`) check the renderer's output sample by sample (the
ramps, the fades, the volume, the rate conversion, no step large enough to
click), the player's queue and timing, the events above in the whole mind
on a pretend body, the pack tool, and `kilo sound` against the laptop
build of the mind, whose sounds "play" silently for as long as they would.

Computed or assumed, to be checked on the real robot:

1. **That it plays at all** on the Pi 2 B's jack with Circle's PWM device
   (the pins, the DMA, the channels): `kilo sound play mew --host
   kilo.local` and listen. The boot log line `sound: PWM on the 3.5 mm
   jack, 44100 Hz, range 5669, ...` must not say "did NOT start".
2. **The control loop and the body link while sounds play**: walk Kilo
   with the remote and press B2 repeatedly. `kilo metrics --host` then:
   `sound: ... callback_max_us=` (expected under about 100), `late=0`,
   and in the `link:` line no new decode errors; `kilo log --host` shows
   no `PL011 line errors` lines. Compare `loop_max_us` with a walk without
   sounds.
3. **Pops** when a sound starts and ends (the 80 ms ramps), and when Kilo
   falls asleep and wakes. If they are loud, try `KILO_SOUND_IDLE=run`.
4. **Idle hiss**: with Kilo awake and quiet, put an ear to the speaker. In
   the default mode the output is stopped between sounds; any hiss then
   comes from the amplifier itself. With `KILO_SOUND_IDLE=run` compare
   again.
5. **A high whine** while a sound plays or (with "run") all the time: the
   PWM switches at 44.1 kHz, above hearing, but a little of it can reach
   the amplifier. People will not hear it; a real cat in the room might.
6. **Loudness**: at volume 70 through the PAM8403, whether 70 is a good
   default, and whether the sounds are about as loud as each other (the
   build evens their peaks, not how loud they seem).
7. **What each sound is like on this speaker**: a small speaker has little
   bass, so the purr may be faint; a rate of 11025 Hz may serve it as well
   in half the space.
8. **Install time** with the sounds in (`kilo update` prints when the mind
   is back).
