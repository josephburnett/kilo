"""The trick library, stored as JSON in tricks/library/.

The first-version tricks (§5) are generated from src/InstinctNybble.h by
`python -m kilo.tricks.import_legacy`; the authored ones (AUTHORED) by
their design modules (`python -m kilo.tricks.rear_up`). The files are
committed; edit the importer or the design module, not the JSON.
"""

from __future__ import annotations

from pathlib import Path

from .model import Trick

LIBRARY_DIR = Path(__file__).resolve().parent / "library"

# Kilo name -> legacy array name in src/InstinctNybble.h, in §5 order.
FIRST_VERSION = {
    # basics
    "sit": "sit",
    "rest": "rest",
    "stand": "up",
    "stretch": "str",
    "balance": "balance",
    # social
    "hi": "hi",
    "wash_face": "wsf",
    "check_around": "ck",
    "sniff": "snf",
    "scratch": "scrh",
    "pee": "pee",
    "push_up": "pu",
    "butt_up": "buttUp",
}
LEGACY_ALIASES = {legacy: name for name, legacy in FIRST_VERSION.items()}

# Tricks written for Kilo (no legacy skill): name -> the module that generates the JSON.
AUTHORED = {
    "rear_up": "kilo.tricks.rear_up",
}


def path_for(name: str) -> Path:
    return LIBRARY_DIR / f"{name}.json"


def resolve(name: str) -> str:
    """Kilo name for `name`, which may also be a legacy token (`up`, `wsf`)."""
    if name in FIRST_VERSION or name in AUTHORED:
        return name
    if name in LEGACY_ALIASES:
        return LEGACY_ALIASES[name]
    raise KeyError(f"no trick {name!r} in the library")


def legacy_name(name: str) -> str:
    """The legacy token of a library trick ("" for an authored one)."""
    return FIRST_VERSION.get(resolve(name), "")


def load(name: str) -> Trick:
    return Trick.from_json(path_for(resolve(name)).read_text())


def names() -> list[str]:
    """Every trick: the first version (§5 order), then the authored ones."""
    return list(FIRST_VERSION) + list(AUTHORED)


def load_all() -> dict[str, Trick]:
    return {n: load(n) for n in names()}
