# kilo-hud

A Claude Code mod that shows Kilo's state beside the chat while working in
this repository: whether he answers on the network, his address, the mind
build on him against the repository's HEAD (is an update ready to install),
and his mode, battery and body firmware from the last full reading.

Load it with `claude --plugin-dir tools/kilo-hud` from the repository root.
It runs the `kilo` command from the repository's `.venv`.

- Status line: always there, for example `Kilo: on at 192.168.88.28 · 7896 mV · update ready`.
- `/kilo`: open the pane and take a full reading.
- `/kilo up`: tell Claude that Kilo is powered on with the network cable in (starts a turn).
- `/kilo down`: tell Claude that Kilo is off or unplugged.
- `/kilo refresh`, `/kilo close`.

Every 20 s it only asks for `kilo.local` by mDNS (`kilo find`), which opens
no connection and does not wake Kilo from light sleep. The full reading
(`kilo status --host`) connects, so it is taken only when Kilo appears and on
`/kilo`, `/kilo up` and `/kilo refresh`.
