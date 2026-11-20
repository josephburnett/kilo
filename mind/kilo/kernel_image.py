"""The mind kernel's image as the laptop handles it: where `make -C kernel`
puts it, its names on the SD card, and the checks `kilo update --host` and
`kilo card` make before installing one."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from kilo.protocol import kilo_protocol as P

REPO_DIR = Path(__file__).resolve().parents[2]
KERNEL_DIR = REPO_DIR / "kernel"
# The mind kernel `make -C kernel` builds in this checkout: the default image of both.
DEFAULT_KERNEL = KERNEL_DIR / "build" / "kernel7.img"
# On the card: the kernel the Pi boots, the one being written, the one before.
KERNEL_FILE, KERNEL_NEW, KERNEL_OLD = "kernel7.img", "kernel7.new", "kernel7.old"


def has_marker(image: bytes) -> bool:
    """Whether image holds host.update_marker, the mind kernel's identity
    (kernel/src/kernel.cpp): what the mind checks before it installs an
    image without FORCE."""
    return P.HOST_UPDATE_MARKER.encode("ascii") in image


def check_kernel(data: bytes, path: Path, force: bool, key: Any, verb: str = "write") -> tuple[list[str], list[str]]:
    """(errors, warnings) for installing data (read from path) as the mind
    kernel: its size, the marker, and whether key (this laptop's
    hostkey.HostKey, None for none) is built in. The kernel has its authorized
    keys in its image as raw bytes (build/gen/authorized_keys.h): one without
    this laptop's would lock it out of the network route. force turns the
    marker and key errors into warnings; verb is what the caller does with it
    ("write", "install")."""
    from kilo import hostkey

    done = {"write": "written", "install": "installed"}.get(verb, verb + "ed")
    if not 0 < len(data) <= P.HOST_UPDATE_MAX_IMAGE:
        return [f"{path} is {len(data)} bytes; a kernel is 1..{P.HOST_UPDATE_MAX_IMAGE}"], []
    warnings: list[str] = []
    if not has_marker(data):
        if force:
            warnings.append(f"{path} is not the mind kernel (no {P.HOST_UPDATE_MARKER}); {done} anyway (--force)")
        else:
            return [f"{path} has no {P.HOST_UPDATE_MARKER}: it is not the mind kernel (the netboot loader is "
                    f"not either); to {verb} it anyway: --force"], []
    if key is not None and key.public not in data:
        if force:
            warnings.append(f"the kernel does not have this laptop's key ({key.fingerprint}): kilo --host from here "
                            "will be refused (--force)")
        else:
            return [f"{path} does not have this laptop's key ({key.fingerprint}) built in: the new kernel "
                    f"would refuse this laptop over the network. Add `kilo key show`'s line to "
                    f"{hostkey.KEYS_FILE} and rebuild (make -C kernel); to {verb} it anyway: --force"], warnings
    return [], warnings
