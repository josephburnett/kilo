"""Kilo mind: the Python side of the Kilo body/mind split (docs/architecture.md §5)."""

__version__ = "0.0.1"

# Every kilo command's exit status: done, or failed (kilo.cli adds EXIT_USAGE).
EXIT_OK, EXIT_ERR = 0, 1
