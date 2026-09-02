"""Type stubs for the compiled `_core` pybind11 extension module.

The real module is built from ``src/core.cpp``; this file only exists so that
type checkers (e.g. Pyright) can understand the public surface of the
extension without having to parse C++.
"""

from enum import IntEnum

import numpy as np


class BooleanType(IntEnum):
    """qpOASES logical values."""

    BT_FALSE: BooleanType
    BT_TRUE: BooleanType


class HessianType(IntEnum):
    """qpOASES Hessian definiteness types."""

    HST_ZERO: HessianType
    HST_IDENTITY: HessianType
    HST_POSDEF: HessianType
    HST_POSDEF_NULLSPACE: HessianType
    HST_SEMIDEF: HessianType
    HST_INDEF: HessianType
    HST_UNKNOWN: HessianType


class PrintLevel(IntEnum):
    """qpOASES print levels, describing the amount of output at runtime."""

    PL_DEBUG_ITER: PrintLevel
    PL_TABULAR: PrintLevel
    PL_NONE: PrintLevel
    PL_LOW: PrintLevel
    PL_MEDIUM: PrintLevel
    PL_HIGH: PrintLevel


class SubjectToStatus(IntEnum):
    """qpOASES bound/constraint statuses."""

    ST_LOWER: SubjectToStatus
    ST_INACTIVE: SubjectToStatus
    ST_UPPER: SubjectToStatus
    ST_INFEASIBLE_LOWER: SubjectToStatus
    ST_INFEASIBLE_UPPER: SubjectToStatus
    ST_UNDEFINED: SubjectToStatus


class qpOASES_Solver:
    """High-level, reusable solver for quadratic programs (QPs) based on qpOASES."""

    # Nested aliases so the enums are also reachable as
    # ``qpOASES_Solver.BooleanType`` etc. (matching the runtime layout, where
    # ``export_values()`` binds them onto the class).
    BooleanType: type[BooleanType] = BooleanType
    HessianType: type[HessianType] = HessianType
    PrintLevel: type[PrintLevel] = PrintLevel
    SubjectToStatus: type[SubjectToStatus] = SubjectToStatus

    class Configuration:
        """All user-configurable solver options.

        Members are the 1:1 mapping of qpOASES' ``Options`` fields (plus the
        wrapper-specific ``maximum_working_set_recalculations``,
        ``use_hotstart`` and ``hessian_type``). Defaults match qpOASES'
        double-precision defaults; see the C++ header (``include/qpOASES_solver.h``)
        and the qpOASES manual for the meaning of each option.
        """

        #: Maximum number of working set recalculations during the initial homotopy.
        maximum_working_set_recalculations: int
        #: Whether subsequent solves are warm-started instead of re-initialised.
        use_hotstart: bool
        #: qpOASES print level.
        print_level: PrintLevel
        #: Enables the ramping strategy.
        enable_ramping: BooleanType
        #: Enables the far bounds strategy.
        enable_far_bounds: BooleanType
        #: Enables flipping of active bounds between lower and upper values.
        enable_flipping_bounds: BooleanType
        #: Regularises the Hessian in case (semi-)definiteness is detected.
        enable_regularisation: BooleanType
        #: Uses the condition-hardened linear independence test.
        enable_full_li_tests: BooleanType
        #: Enables the nonzero curvature test.
        enable_nzc_tests: BooleanType
        #: Frequency of drift corrections (0 = off).
        enable_drift_correction: int
        #: Frequency of full Cholesky refactorisation of the projected Hessian (0 = updates only).
        enable_cholesky_refactorisation: int
        #: Treats equality constraints as always active.
        enable_equalities: BooleanType
        #: Relative termination tolerance to stop the homotopy.
        termination_tolerance: float
        #: Lower/upper (constraints') bound tolerance.
        bound_tolerance: float
        #: Offset for relaxing constraint bounds at the start of an initial homotopy.
        bound_relaxation: float
        #: Numerator tolerance for the ratio test.
        eps_num: float
        #: Denominator tolerance for the ratio test.
        eps_den: float
        #: Maximum allowed jump in primal variables during nonzero curvature tests.
        max_primal_jump: float
        #: Maximum allowed jump in dual variables during linear independence tests.
        max_dual_jump: float
        #: Start value of the ramping strategy.
        initial_ramping: float
        #: Final value of the ramping strategy.
        final_ramping: float
        #: Initial size of the far bounds.
        initial_far_bounds: float
        #: Growth factor applied to the far bounds.
        grow_far_bounds: float
        #: Status assumed for all bounds at the first iteration.
        initial_status_bounds: SubjectToStatus
        #: Tolerance of the squared Cholesky diagonal factor which triggers flipping a bound.
        eps_flipping: float
        #: Maximum number of successive regularisation steps.
        num_regularisation_steps: int
        #: Scaling factor of the identity matrix used for Hessian regularisation.
        eps_regularisation: float
        #: Maximum number of iterative refinement steps.
        num_refinement_steps: int
        #: Early termination tolerance for iterative refinement.
        eps_iter_ref: float
        #: Tolerance used by the linear independence tests.
        eps_li_tests: float
        #: Tolerance used by the nonzero curvature tests.
        eps_nzc_tests: float
        #: Minimum reciprocal condition number of the Schur complement before refactorisation is triggered.
        rcond_s_min: float
        #: Repairs the working set when negative curvature is discovered during a hotstart.
        enable_inertia_correction: BooleanType
        #: Whether infeasible constraints may be dropped.
        enable_drop_infeasibles: BooleanType
        #: Priority used when dropping bounds.
        drop_bound_priority: int
        #: Priority used when dropping equality constraints.
        drop_eq_con_priority: int
        #: Priority used when dropping inequality constraints.
        drop_ineq_con_priority: int
        #: Definiteness assumed for the Hessian matrix.
        hessian_type: HessianType

        def __init__(self) -> None: ...

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
