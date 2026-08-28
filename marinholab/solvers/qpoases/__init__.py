"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv2.1 License

Public API of the `marinholab.solvers.qpoases` package.

`Solver` is a thin, numpy-friendly Python wrapper around the compiled
qpOASES solver (`qpOASES_Solver`). The configuration and enum types are
re-exported for convenience.
"""
from .solver import Solver
# TODO change this mess into inheritance via trampoline class
# Interface won't change, so this will do for now
from marinholab.solvers.qpoases._core import qpOASES_Solver

# Re-exported for convenience so users can write e.g. ``qpoases.Configuration``
# and ``qpoases.HessianType.HST_POSDEF``.
Configuration = qpOASES_Solver.Configuration
HessianType = qpOASES_Solver.HessianType
BooleanType = qpOASES_Solver.BooleanType
PrintLevel = qpOASES_Solver.PrintLevel
SubjectToStatus = qpOASES_Solver.SubjectToStatus

__all__ = [
    "Solver",
    "qpOASES_Solver",
    "Configuration",
    "HessianType",
    "BooleanType",
    "PrintLevel",
    "SubjectToStatus",
]