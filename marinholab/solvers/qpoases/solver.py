from __future__ import annotations

import numpy as np
from marinholab.solvers.qpoases._core import qpOASES_Solver


class Solver:
    """Thin, numpy-friendly wrapper around the compiled qpOASES solver.

    Accepts ``None`` for the constraint matrices ``A``/``b``/``Aeq``/``beq``
    and converts them into suitably sized zero matrices before calling the
    underlying solver. All matrices are given as dense, row-major numpy
    arrays (or anything numpy can treat as one).
    """

    def __init__(self, configuration: qpOASES_Solver.Configuration | None = None) -> None:
        self.configuration: qpOASES_Solver.Configuration = (
            configuration if configuration is not None else qpOASES_Solver.Configuration()
        )
        self.solver: qpOASES_Solver = qpOASES_Solver(self.configuration)

    def solve_quadratic_program(
        self,
        H: np.ndarray,
        f: np.ndarray,
        A: np.ndarray | None,
        b: np.ndarray | None,
        Aeq: np.ndarray | None,
        beq: np.ndarray | None,
    ) -> np.ndarray:
        """Solves ``min(x) 0.5*x'Hx + f'x`` subject to ``Ax <= b`` and ``Aeq*x = beq``.

        Any of ``A``/``b``/``Aeq``/``beq`` may be ``None`` (meaning "no such
        constraint"), but the matrix and its right-hand side must be provided
        together. Returns the optimal ``x`` as a 1-D numpy array.
        """

        if (A is None) != (b is None):
            raise ValueError(f"A={A} and b={b} must both be None or both not None.")
        if (Aeq is None) != (beq is None):
            raise ValueError(f"Aeq={Aeq} and beq={beq} must both be None or both not None.")

        # The solver requires all six matrices; replace the ``None``
        # constraints with a single trivially satisfied zero row.
        A_full = np.zeros((1, H.shape[0])) if A is None else A
        b_full = np.zeros((1,)) if b is None else b
        Aeq_full = np.zeros((1, H.shape[0])) if Aeq is None else Aeq
        beq_full = np.zeros((1,)) if beq is None else beq

        return self.solver.solve_quadratic_program(H, f, A_full, b_full, Aeq_full, beq_full)

    def get_active_set(self) -> np.ndarray:
        """
        Returns the active set of constraints obtained in the most recent call to
        solve_quadratic_program(). The returned vector has one entry for each row of the
        combined constraint matrix, i.e. the rows of A followed by the rows of Aeq, in that
        same order, with the following meaning for each entry:
            -1: the constraint is active at its lower bound;
             0: the constraint is inactive;
            +1: the constraint is active at its upper bound (this is also the value used
                for active equality constraints, as their lower and upper bounds coincide).
        """
        return self.solver.get_active_set()
