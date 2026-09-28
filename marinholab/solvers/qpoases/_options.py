"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv2.1 License

Pure-Python enum types that mirror the qpOASES enumerations accepted by
`Configuration.set()`.

These live in Python (not in the compiled extension) so the C++ public
header stays free of qpOASES types. Each member's *name* is the string that
the string-based `Configuration` accepts (e.g. `HessianType.HST_SEMIDEF.name`
-> `"HST_SEMIDEF"`), and its *value* matches the underlying qpOASES integer.
"""
from __future__ import annotations

from enum import IntEnum


class BooleanType(IntEnum):
    """qpOASES logical values.

    Accepted by boolean options either as the member itself or as the string
    ``"true"`` / ``"false"``.
    """

    BT_FALSE = 0
    BT_TRUE = 1


class PrintLevel(IntEnum):
    """qpOASES print levels, describing the amount of output at runtime."""

    PL_DEBUG_ITER = -2
    PL_TABULAR = -1
    PL_NONE = 0
    PL_LOW = 1
    PL_MEDIUM = 2
    PL_HIGH = 3


class HessianType(IntEnum):
    """qpOASES Hessian definiteness types (the ``hessian_type`` option)."""

    HST_ZERO = 0
    HST_IDENTITY = 1
    HST_POSDEF = 2
    HST_POSDEF_NULLSPACE = 3
    HST_SEMIDEF = 4
    HST_INDEF = 5
    HST_UNKNOWN = 6


class SubjectToStatus(IntEnum):
    """qpOASES bound/constraint statuses (the ``initialStatusBounds`` option)."""

    ST_LOWER = -1
    ST_INACTIVE = 0
    ST_UPPER = 1
    ST_INFEASIBLE_LOWER = 2
    ST_INFEASIBLE_UPPER = 3
    ST_UNDEFINED = 4
