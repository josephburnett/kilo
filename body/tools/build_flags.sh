# The body build's size flags (doc §3h), defined once: sourced by size_check.sh
# and stack_check.sh, read by mind/kilo/tools/flash.py (`kilo flash`, `kilo card`).
# -mcall-prologues shares the register save/restore code between functions;
# --relax turns call/jmp into rcall/rjmp where they reach (≈420 B of flash,
# RAM unchanged). Keep the NAME="value" form: flash.py parses it.
CFLAGS="-mcall-prologues"
LDFLAGS="-Wl,--relax"
