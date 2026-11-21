"""Behaviour: a minimal behaviour tree under a utility drive selector (§5).

- tree.py         the engine (Sequence, Selector, Parallel, decorators, Action, Condition)
- drives.py       energy, curiosity, sociability, arousal
- selector.py     the utility root: safety branches, scores, hysteresis
- library.py      idle, explore (stub), nap, greet, startle; fallen, wind-down, low battery
- personality.py  personality.yaml loader (PyYAML optional) and validation
- runner.py       the 10 Hz asyncio ticker and the JSONL behaviour log

The reference for the C++ mind's behaviour layer (kernel/mind/behaviour/,
checked against it by kernel/host/tests/gen_behaviour_vectors.py).
"""

from .drives import DriveInputs, Drives
from .personality import PersonalityError
from .personality import load as load_personality
from .runner import Runner
from .tree import FAILURE, RUNNING, SUCCESS, Context, Status

__all__ = ["Context", "DriveInputs", "Drives", "FAILURE", "PersonalityError", "RUNNING", "Runner",
           "SUCCESS", "Status", "load_personality"]
