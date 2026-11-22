#!/bin/bash
# Test of the reset record's fixed place (body/src/hal/sys.h), run by
# host_tests.sh: build_flags.sh puts the record where sys.h says and the
# stack just below it, and reset_record_check (size_check.sh's check of a
# built ELF) refuses the layout that broke the boot-16 card flash (the
# record in .noinit, moving with the globals) and a stack over the record,
# and accepts the fixed layout. A fake nm prints symbol lists, so no AVR
# toolchain is needed.
# Usage: body/tools/reset_record_test.sh SCRATCH_DIR   (exit 0 = pass)
set -u
BODY="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:?usage: reset_record_test.sh SCRATCH_DIR}"
mkdir -p "$OUT"
. "$BODY/tools/build_flags.sh"
. "$BODY/tools/reset_record.sh"
ok=1
addr=$(awk '$1=="#define" && $2=="SYS_RESET_RECORD_ADDR"{print $3}' "$BODY/src/hal/sys.h")
want_rec="-Wl,--defsym=sys_reset_record=0x$(printf '%X' $((addr | 0x800000)))"
want_stk="-Wl,--defsym=__stack=0x$(printf '%X' $((addr - 1)))"
case " $LDFLAGS " in *" $want_rec "*) ;; *) echo "FAIL: LDFLAGS lack $want_rec: $LDFLAGS"; ok=0 ;; esac
case " $LDFLAGS " in *" $want_stk "*) ;; *) echo "FAIL: LDFLAGS lack $want_stk: $LDFLAGS"; ok=0 ;; esac

fake="$OUT/fake_nm.sh"
printf '#!/bin/sh\ncat "$1"\n' > "$fake"
chmod +x "$fake"
# The 7e92703c layout (boot 16): the record in .noinit after .bss.
printf '008006f7 B __bss_end\n008006f9 b sys_reset_record\n000008ff W __stack\n' > "$OUT/syms_noinit"
# The record placed, but the stack still starting at the top of RAM, over it.
printf '008008f6 A sys_reset_record\n000008ff W __stack\n' > "$OUT/syms_stack"
# The fixed layout, as the linker writes it.
printf '008006f9 N _end\n000008f5 A __stack\n008008f6 A sys_reset_record\n' > "$OUT/syms_fixed"
if reset_record_check "$fake" "$OUT/syms_noinit" > /dev/null; then echo "FAIL: accepted the record in .noinit"; ok=0; fi
if reset_record_check "$fake" "$OUT/syms_stack" > /dev/null; then echo "FAIL: accepted a stack over the record"; ok=0; fi
if ! reset_record_check "$fake" "$OUT/syms_fixed" > /dev/null; then echo "FAIL: refused the fixed layout"; ok=0; fi
[ "$ABOVE_STACK" -eq 10 ] || { echo "FAIL: ABOVE_STACK $ABOVE_STACK, not 10"; ok=0; }
if [ "$ok" -eq 1 ]; then echo "reset record: ok"; exit 0; fi
exit 1
