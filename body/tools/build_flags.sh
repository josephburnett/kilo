# The body build's size flags (doc §3h), defined once: sourced by size_check.sh
# and stack_check.sh, read by mind/kilo/tools/flash.py (`kilo flash`, `kilo card`).
# -mcall-prologues shares the register save/restore code between functions;
# --relax turns call/jmp into rcall/rjmp where they reach (≈420 B of flash,
# RAM unchanged). The two --defsym place the reset record (hal/sys.h) at a
# fixed address, the top 10 bytes of RAM, and start the stack below it: the
# record must be where the previous firmware left it, whatever this build's
# globals are. Keep the NAME="value" form: flash.py parses it.
CFLAGS="-mcall-prologues"
LDFLAGS="-Wl,--relax -Wl,--defsym=sys_reset_record=0x8008F6 -Wl,--defsym=__stack=0x8F5"
