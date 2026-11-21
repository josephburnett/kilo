# mind/firmware

A stored firmware image: the way back to Petoi's stock firmware. No `kilo`
command uses it (`kilo flash` builds and flashes Kilo's own body firmware
through the mind).

| File | What |
|---|---|
| `opencat-legacy.hex` | The legacy Petoi OpenCat firmware for Kilo (token protocol at 115200, uses the legacy EEPROM calibration). Built from `OpenCat.ino` on `mega` at 64b9447 (32016 B flash, 1620 B RAM; md5 b3e85cc029108f7a56b75c11a7626fde, byte-identical on x86-64 and the Pi). Its sources are no longer in the tree (five headers the tests read are in `legacy/`): to rebuild, check out 64b9447, run `arduino-cli compile --fqbn arduino:avr:uno --output-dir out .` at the top and copy `out/OpenCat.ino.hex` here. |

To flash it, plug the Petoi USB adapter into the NyBoard and run avrdude by
hand on the laptop:

```sh
avrdude -c arduino -p atmega328p -P /dev/ttyUSB0 -b 115200 -D -U flash:w:mind/firmware/opencat-legacy.hex:i
```

The adapter and the Pi both drive the body's one serial RX line, so while
the Pi's mind runs, frames from each side corrupt the other's.
`-D` writes flash only, so the EEPROM and its calibration stay as they are.
The mind cannot talk to the legacy firmware (it speaks another protocol).
`kilo flash` puts Kilo's firmware back: the mind resets the body into its
bootloader over GPIO4, whatever firmware is running.
