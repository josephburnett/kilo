"""The body build's compile command (kilo.tools.flash)."""

from pathlib import Path

from kilo.tools import flash as fl


def test_commands():
    assert fl.compile_cmd(Path("/r/body"), Path("/o"), "abc12345") == [
        "arduino-cli", "compile", "--fqbn", "arduino:avr:uno", "--output-dir", "/o",
        "--build-property", "compiler.cpp.extra_flags=-mcall-prologues -DKILO_BUILD=0xabc12345",
        "--build-property", "compiler.c.extra_flags=-mcall-prologues",
        "--build-property", "compiler.c.elf.extra_flags=-Wl,--relax -Wl,--defsym=sys_reset_record=0x8008F6 -Wl,--defsym=__stack=0x8F5", "/r/body"]


def test_compile_cmd_carries_the_size_flags():
    """-mcall-prologues for C and C++ and -Wl,--relax for the link (doc §3h), and the reset record at
    its fixed address at the top of RAM with the stack below it (hal/sys.h: a flashed firmware must find
    the record the old one left), with or without a build hash, and the same flags size_check.sh
    expects (body/tools/build_flags.sh)."""
    for h in ("abc12345", None):
        cmd = fl.compile_cmd(Path("/r/body"), Path("/o"), h)
        props = dict(cmd[i + 1].split("=", 1) for i, a in enumerate(cmd) if a == "--build-property")
        assert "-mcall-prologues" in props["compiler.cpp.extra_flags"].split()
        assert "-mcall-prologues" in props["compiler.c.extra_flags"].split()
        ld = props["compiler.c.elf.extra_flags"].split()
        assert "-Wl,--relax" in ld
        assert "-Wl,--defsym=sys_reset_record=0x8008F6" in ld and "-Wl,--defsym=__stack=0x8F5" in ld
        assert ("-DKILO_BUILD=0xabc12345" in props["compiler.cpp.extra_flags"].split()) == (h is not None)
    assert fl.build_flags() == ("-mcall-prologues", "-Wl,--relax -Wl,--defsym=sys_reset_record=0x8008F6 -Wl,--defsym=__stack=0x8F5")
    for script in ("size_check.sh", "stack_check.sh"):  # they source the same file
        assert '. "$BODY/tools/build_flags.sh"' in (fl.REPO_DIR / "body" / "tools" / script).read_text()
