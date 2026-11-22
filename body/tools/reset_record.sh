# The reset record's fixed place (body/src/hal/sys.h), checked on a built ELF.
# Sourced by size_check.sh and stack_check.sh.
#
# reset_record_check NM ELF: FAILS (returns 1, says why) unless
# sys_reset_record is at 0x8008F6 (RAM 0x8F6, the top 10 bytes) and the
# stack starts just below it (__stack 0x8F5). Sets ABOVE_STACK to the bytes
# between __stack and the end of RAM (0x8FF), which belong to the globals.
#
# Why: a newly flashed firmware reads the record the previous one wrote. When
# the record lived in .noinit its address moved with the size of .data +
# .bss, and on boot 16 (2026-11-22) a build with 2 bytes more .data read the
# old record 2 bytes off: the flash was reported as a cold boot.
RESET_RECORD_ADDR=0x8008f6
RESET_STACK_TOP=0x8f5
RAM_END=0x8ff

reset_record_check() {
  local nm="$1" elf="$2" rec stk
  rec=$("$nm" "$elf" | awk '$3=="sys_reset_record"{print $1}')
  stk=$("$nm" "$elf" | awk '$3=="__stack"{print $1}')
  ABOVE_STACK=0
  local hdr
  hdr=$(awk '$1=="#define" && $2=="SYS_RESET_RECORD_ADDR"{print $3}' "$(dirname "${BASH_SOURCE[0]}")/../src/hal/sys.h")
  if [ -z "$hdr" ] || [ $((hdr | 0x800000)) -ne $((RESET_RECORD_ADDR)) ]; then
    echo "FAIL: hal/sys.h SYS_RESET_RECORD_ADDR ${hdr:-missing} is not $RESET_RECORD_ADDR (keep it, build_flags.sh and this file in step)"
    return 1
  fi
  if [ -z "$rec" ] || [ $((16#$rec)) -ne $((RESET_RECORD_ADDR)) ]; then
    echo "FAIL: reset record sys_reset_record at 0x${rec:-none}, not $RESET_RECORD_ADDR (build without the --defsym of build_flags.sh?)"
    return 1
  fi
  if [ -z "$stk" ] || [ $((16#$stk & 0xffff)) -ne $((RESET_STACK_TOP)) ]; then
    echo "FAIL: __stack at 0x${stk:-none}, not $RESET_STACK_TOP: the stack would overwrite the reset record"
    return 1
  fi
  ABOVE_STACK=$((RAM_END - RESET_STACK_TOP))
  echo "reset record: at $RESET_RECORD_ADDR, the stack starts below it at $RESET_STACK_TOP ok"
  return 0
}
