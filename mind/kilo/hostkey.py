"""This laptop's key for the mind's host channel (kernel/README.md "Who may
talk to Kilo: keys").

The mind (the Circle kernel, or kilo-mind-host) serves its TCP port only to
laptops whose public key is in the list built into it
(kernel/keys/authorized_keys). The laptop proves it holds the private key
in a Noise KN handshake (kilo.hostsecure). The private key lives outside the
repo, in ``~/.config/kilo/host_key`` (``$XDG_CONFIG_HOME/kilo/host_key``),
or wherever ``kilo --key PATH`` or ``$KILO_HOST_KEY`` says:

    kilo key init     make the key pair if there is none (never overwrites); print its public line
    kilo key show     print the public line (for kernel/keys/authorized_keys) and its fingerprint

The file is text, mode 0600 in a 0700 directory; like ssh, a key file that
others can read is refused::

    # Kilo host key ...
    private-key <base64 of the 32-byte X25519 private key>
    name joe@laptop

The public line is ``<base64 of the 32-byte public key> <name>``; the
fingerprint is the first 8 bytes of BLAKE2b-512 of the public key in hex,
as the mind logs it (kernel/mind/net/noise.h fingerprint()).
"""

from __future__ import annotations

import base64
import binascii
import getpass
import hashlib
import os
import socket
import stat
from dataclasses import dataclass
from pathlib import Path

ENV = "KILO_HOST_KEY"
KEYS_FILE = "kernel/keys/authorized_keys"  # where the public line goes, in the repo


class HostKeyError(Exception):
    """The key file cannot be used; the message says what to do (one line)."""


class HostKeyMissing(HostKeyError):
    pass


class HostKeyUnsafe(HostKeyError):
    pass


def default_path() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME") or str(Path.home() / ".config")
    return Path(base) / "kilo" / "host_key"


def key_path(explicit: str | os.PathLike | None = None) -> Path:
    """--key PATH, else $KILO_HOST_KEY, else ~/.config/kilo/host_key."""
    if explicit:
        return Path(explicit).expanduser()
    env = os.environ.get(ENV)
    return Path(env).expanduser() if env else default_path()


def fingerprint(public: bytes) -> str:
    return hashlib.blake2b(public).hexdigest()[:16]


def _public_of(private: bytes) -> bytes:
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric import x25519

    return x25519.X25519PrivateKey.from_private_bytes(private).public_key().public_bytes(
        serialization.Encoding.Raw, serialization.PublicFormat.Raw)


def _default_name() -> str:
    try:
        user = getpass.getuser()
    except (KeyError, OSError):
        user = "someone"
    name = f"{user}@{socket.gethostname()}"
    return "".join(ch if ch.isprintable() and not ch.isspace() else "_" for ch in name)[:39]


@dataclass(frozen=True)
class HostKey:
    private: bytes
    public: bytes
    name: str
    path: Path

    @property
    def fingerprint(self) -> str:
        return fingerprint(self.public)

    def line(self) -> str:
        """The line for kernel/keys/authorized_keys."""
        return f"{base64.b64encode(self.public).decode()} {self.name}"

    def __repr__(self) -> str:  # never the private key
        return f"HostKey({self.fingerprint} {self.name}, {self.path})"


def _check_safe(path: Path) -> None:
    st = path.stat()
    if not stat.S_ISREG(st.st_mode):
        raise HostKeyUnsafe(f"{path} is not a file")
    if hasattr(os, "getuid") and st.st_uid != os.getuid():
        raise HostKeyUnsafe(f"{path} belongs to another user: refusing to use it as this laptop's key")
    if st.st_mode & 0o077:
        raise HostKeyUnsafe(f"{path} can be read by others (mode {stat.S_IMODE(st.st_mode):04o}): it is a private "
                            f"key; make it yours alone with: chmod 600 {path}")


def load(path: str | os.PathLike | None = None) -> HostKey:
    """The key at path (key_path()'s default); HostKeyError says what is wrong."""
    p = key_path(path)
    if not p.exists():
        raise HostKeyMissing(f"no host key at {p}: create one with `kilo key init`, add the line it prints to "
                             f"{KEYS_FILE}, and rebuild and install the kernel")
    _check_safe(p)
    try:
        text = p.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as e:
        raise HostKeyError(f"cannot read {p}: {e}") from None
    fields: dict[str, str] = {}
    for line in text.splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        k, _, v = s.partition(" ")
        fields[k] = v.strip()
    try:
        private = base64.b64decode(fields.get("private-key", ""), validate=True)
    except (binascii.Error, ValueError):
        private = b""
    if len(private) != 32:
        raise HostKeyError(f"{p} is not a kilo host key (no 32-byte private-key line); move it away and run "
                           "`kilo key init` for a new one")
    return HostKey(private, _public_of(private), fields.get("name") or _default_name(), p)


def init(path: str | os.PathLike | None = None, name: str | None = None) -> tuple[HostKey, bool]:
    """Make the key pair at path if there is no file there; never overwrites.
    Returns (the key, whether it was made now)."""
    p = key_path(path)
    if p.exists() or p.is_symlink():
        return load(p), False
    from cryptography.hazmat.primitives import serialization
    from cryptography.hazmat.primitives.asymmetric import x25519

    d = p.parent
    if not d.exists():
        d.mkdir(mode=0o700, parents=True)
        os.chmod(d, 0o700)  # mkdir's mode is masked by the umask
    key = x25519.X25519PrivateKey.generate()
    private = key.private_bytes(serialization.Encoding.Raw, serialization.PrivateFormat.Raw,
                                serialization.NoEncryption())
    label = name or _default_name()
    if any(ch.isspace() for ch in label.strip()) or not label.strip():
        label = "_".join(label.split()) or _default_name()
    body = ("# Kilo host key: this laptop's private X25519 key for the mind's host channel (kilo --host).\n"
            "# Keep it to yourself (mode 0600) and out of the repo. `kilo key show` prints the public line\n"
            f"# for {KEYS_FILE}.\n"
            f"private-key {base64.b64encode(private).decode()}\n"
            f"name {label}\n")
    fd = os.open(p, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        os.write(fd, body.encode())
        os.fsync(fd)
    finally:
        os.close(fd)
    os.chmod(p, 0o600)
    return load(p), True
