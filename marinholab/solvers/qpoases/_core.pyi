"""Type stubs for the compiled `_core` pybind11 extension module.

The real module is built from ``src/core.cpp``; this file only exists so that
type checkers (e.g. Pyright) can understand the public surface of the
extension without having to parse C++.

The enum types (``BooleanType``, ``HessianType``, ``PrintLevel``,
``SubjectToStatus``) are provided by the pure-Python module
``_options.py`` and re-exported by the package ``__init__.py``; they are not
part of the compiled extension.
"""

from typing import overload

from collections.abc import Mapping

import numpy as np


class qpOASES_Solver:
    """High-level, reusable solver for quadratic programs (QPs) based on qpOASES."""

    class Configuration:
        """String-keyed holder of all user-configurable solver options.

        Every qpOASES ``Options`` field plus the wrapper-specific
        ``maximum_working_set_recalculations``, ``use_hotstart`` and
        ``hessian_type`` is exposed under its name via ``set()``/``get()``.
        Values are strings: booleans are ``"true"``/``"false"``, and enum
        options take the enum value name (e.g. ``"HST_SEMIDEF"``,
        ``"PL_NONE"``). Defaults match qpOASES' double-precision defaults
        except ``printLevel`` (``"PL_NONE"``). See ``keys()`` and
        ``defaults()`` for the full list.
        """

        def __init__(self) -> None: ...

        @overload
        def set(self, key: str, value: str) -> None:
            """Sets the option ``key`` to the string ``value``."""
            ...

        @overload
        def set(self, key: str, value: int | float | bool | object) -> None:
            """Convenience overload: also accepts enum members (used via their
            name), numbers, and bools, normalised to their string form.
            (``object`` here represents e.g. an ``IntEnum`` member such as
            ``HessianType.HST_SEMIDEF``.)"""
            ...

        def get(self, key: str) -> str:
            """Returns the string value of ``key``, or its default if not set."""
            ...

        def has(self, key: str) -> bool:
            """True if ``key`` has been explicitly set."""
            ...

        def keys(self) -> list[str]:
            """Sorted list of all settable option names."""
            ...

        def defaults(self) -> Mapping[str, str]:
            """Mapping of option name -> default string value."""
            ...

        def reset(self, key: str) -> None:
            """Reverts ``key`` to its default."""
            ...

        def reset_all(self) -> None:
            """Reverts every option to its default."""
            ...

    def __init__(self, configuration: Configuration | None = None) -> None:
        """Constructs a solver with the given configuration (defaults to the default configuration)."""
        ...

    def solve_quadratic_program(
        self,
        H: np.ndarray,
        f: np.ndarray,
        A: np.ndarray,
        b: np.ndarray,
        Aeq: np.ndarray,
        beq: np.ndarray,
    ) -> np.ndarray:
        """Solves ``min(x) 0.5*x'Hx + f'x s.t. Ax <= b, Aeq*x = beq``.

        Method signature is compatible with MATLAB's ``quadprog``. Returns the
        optimal ``x``.
        """
        ...

    def get_active_set(self) -> np.ndarray:
        """Returns the active set of constraints from the most recent solve.

        One entry per row of the combined constraint matrix (rows of ``A``
        followed by rows of ``Aeq``): -1 = active at its lower bound, 0 =
        inactive, +1 = active at its upper bound (equality constraints are
        always reported as +1).
        """
        ...

    def test_vectorxd(self, v: np.ndarray) -> np.ndarray:
        """Round-trips a vector to help evaluate the Eigen <-> std conversions."""
        ...

    def test_matrixxd(self, m: np.ndarray) -> np.ndarray:
        """Round-trips a matrix to help evaluate the Eigen <-> std conversions."""
        ...


__version__: str
