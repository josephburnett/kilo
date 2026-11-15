"""kilo.hostkey and `kilo key`: the laptop's key file (where it is, its modes,
never overwritten, refused when others can read it) and the public line and
fingerprint the mind's list and log use. Every key here is a throwaway in a
tmp dir; the owner's ~/.config/kilo/host_key is never touched."""

from __future__ import annotations

import base64
import hashlib
import os
import stat

import pytest

from kilo import cli, hostkey


def mode(p) -> int:
    return stat.S_IMODE(os.stat(p).st_mode)


def test_init_makes_a_private_key_file(tmp_path):
    p = tmp_path / "cfg" / "kilo" / "host_key"
    key, made = hostkey.init(p, "joe@test")
    assert made and p.exists() and mode(p) == 0o600 and mode(p.parent) == 0o700
    assert key.path == p and key.name == "joe@test" and len(key.private) == 32 and len(key.public) == 32
    text = p.read_text()
    assert "private-key " in text and base64.b64encode(key.public).decode() not in text  # only the private key
    assert key.private.hex() not in repr(key) and base64.b64encode(key.private).decode() not in repr(key)
    again = hostkey.load(p)
    assert again == key


def test_init_never_overwrites(tmp_path):
    p = tmp_path / "host_key"
    key, made = hostkey.init(p)
    before = p.read_bytes()
    key2, made2 = hostkey.init(p, "another-name")
    assert made and not made2 and key2.public == key.public and p.read_bytes() == before


def test_public_line_and_fingerprint(tmp_path):
    key, _ = hostkey.init(tmp_path / "k", "joe@rtb")
    field, name = key.line().split(" ")
    assert len(field) == 44 and base64.b64decode(field) == key.public and name == "joe@rtb"
    assert key.fingerprint == hashlib.blake2b(key.public).hexdigest()[:16] == hostkey.fingerprint(key.public)


def test_a_key_others_can_read_is_refused(tmp_path):
    p = tmp_path / "k"
    hostkey.init(p)
    for m in (0o640, 0o604, 0o644):
        os.chmod(p, m)
        with pytest.raises(hostkey.HostKeyUnsafe, match=f"can be read by others \\(mode {m:04o}\\).*chmod 600 {p}"):
            hostkey.load(p)
    os.chmod(p, 0o400)
    hostkey.load(p)  # read-only for its owner is fine


def test_missing_and_broken_keys(tmp_path):
    with pytest.raises(hostkey.HostKeyMissing, match="no host key at .*: create one with `kilo key init`"):
        hostkey.load(tmp_path / "none")
    bad = tmp_path / "bad"
    fd = os.open(bad, os.O_WRONLY | os.O_CREAT, 0o600)
    os.write(fd, b"private-key AAAA\n")
    os.close(fd)
    with pytest.raises(hostkey.HostKeyError, match="not a kilo host key"):
        hostkey.load(bad)
    d = tmp_path / "adir"
    d.mkdir()
    with pytest.raises(hostkey.HostKeyUnsafe, match="not a file"):
        hostkey.load(d)


def test_where_the_key_is(tmp_path, monkeypatch):
    monkeypatch.delenv("KILO_HOST_KEY", raising=False)
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / "xdg"))
    assert hostkey.key_path() == tmp_path / "xdg" / "kilo" / "host_key"
    monkeypatch.delenv("XDG_CONFIG_HOME")
    monkeypatch.setenv("HOME", str(tmp_path / "home"))
    assert hostkey.key_path() == tmp_path / "home" / ".config" / "kilo" / "host_key"
    monkeypatch.setenv("KILO_HOST_KEY", str(tmp_path / "env_key"))
    assert hostkey.key_path() == tmp_path / "env_key"
    assert hostkey.key_path(tmp_path / "flag_key") == tmp_path / "flag_key"


def test_cli_key_init_and_show(tmp_path, capsys):
    p = tmp_path / "laptop" / "host_key"
    assert cli.main(["--key", str(p), "key", "init", "--name", "joe@laptop"]) == 0
    out, err = capsys.readouterr()
    key = hostkey.load(p)
    assert out == key.line() + "\n" and key.name == "joe@laptop"
    assert f"{p} made; fingerprint {key.fingerprint}" in err and "kernel/keys/authorized_keys" in err
    assert cli.main(["key", "show", "--key", str(p)]) == 0  # --key after the command too
    out, err = capsys.readouterr()
    assert out == key.line() + "\n" and key.fingerprint in err
    assert cli.main(["--key", str(p), "key", "init"]) == 0
    out, err = capsys.readouterr()
    assert out == key.line() + "\n" and "already there, not changed" in err
    assert cli.main(["--key", str(tmp_path / "none"), "key", "show"]) == 1
    assert "kilo key init" in capsys.readouterr().err


def test_cli_key_uses_the_environment(tmp_path, capsys, monkeypatch):
    p = tmp_path / "env" / "host_key"
    monkeypatch.setenv("KILO_HOST_KEY", str(p))
    assert cli.main(["key", "init"]) == 0
    assert p.exists() and capsys.readouterr().out == hostkey.load(p).line() + "\n"
