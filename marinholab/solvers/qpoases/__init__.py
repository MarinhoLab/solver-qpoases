"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv2.1 License

Public API of the `marinholab.solvers.qpoases` package.

`Solver` is a thin, numpy-friendly Python wrapper around the compiled
qpOASES solver (`qpOASES_Solver`). The `Configuration` (keyed by option name) and the
enum types are re-exported for convenience.

The enum types are pure Python (see `_options.py`) so the compiled
extension's C++ header stays free of qpOASES types.
"""
from .solver import Solver
# TODO change this mess into inheritance via trampoline class
# Interface won't change, so this will do for now
from marinholab.solvers.qpoases._core import qpOASES_Solver

# The configuration, keyed by option name (backed by the compiled extension).
Configuration = qpOASES_Solver.Configuration
# The enum types, provided in pure Python so users can still write e.g.
# ``qpoases.HessianType.HST_SEMIDEF`` (``Configuration.set`` also accepts
# these members directly).
from marinholab.solvers.qpoases._options import (
    BooleanType,
    HessianType,
    PrintLevel,
    SubjectToStatus,
)

__all__ = [
    "Solver",
    "qpOASES_Solver",
    "Configuration",
    "HessianType",
    "BooleanType",
    "PrintLevel",
    "SubjectToStatus",
]