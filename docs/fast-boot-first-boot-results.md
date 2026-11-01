# Fast boot: first boot results

Runs of docs/fast-boot-first-boot.md. Newest first. Copy the block below
for each run and fill in what you saw; "don't know" is a fine answer.

## YYYY-MM-DD, run N

| Field | What you saw |
|---|---|
| Date and time | |
| Kernel build (commit, or the number on the screen) | |
| SD card (which card; `--netboot` or `--kernel`) | |
| Ethernet cable (to the router, or other) | |
| Kilo connection (A: USB cable, or B: header wires 8/10/6) | |
| Kilo switched on before the cable went in? | |
| Did the Pi reset when the cable went in? | |
| Loader address (from the router) and name shown | |
| `netboot.sh` output (last line) | |
| ACT LED pattern, first 10 s | |
| ACT LED pattern after 1 minute | |
| HELLO seen? (even 2 Hz blink, or the screen line) | |
| Kernel's address (router name **kilo**, or 169.254.75.73 on a direct cable) | |
| `ping kilo.local`: did it answer? With which address? | |
| `kilo status --host` output (paste it; or the error) | |
| `mind build` in the status: matches the commit? | |
| `planner bench` lines from `kilo log --host` (three) | |
| One `loop` line from `kilo log --host` | |
| `kilo posture stand --for 5`: did Kilo stand, hold, then lie down? What did the laptop print? | |
| Screen used? What it showed (serial chosen, first HELLO time, telemetry/s, decode errors) | |
| Boot probe (scope on pin 40): power-on to rising edge, rising to falling edge | |
| Flash (step 11): reset wiring (socket, fourth wire to DTR, or skipped) | |
| `kilo flash --host`: the last line it printed | |
| ACT LED while flashing (two short flashes a second?) and after | |
| Log: `flash: optiboot in sync ... ms after release` line | |
| Log: `flash: programmed` / `verified` / `done in ... ms` lines | |
| Log: `body flashed from the laptop` (or `FAILED`) line | |
| `--persist`: the `SD: --persist: wrote ...` line; after a power cycle, the `boot flash check` line | |
| Anything else | |
