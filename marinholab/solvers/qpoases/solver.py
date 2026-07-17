import numpy as np
from marinholab.solvers.qpoases._core import qpOASES_Solver

class Solver:
    def __init__(self, configuration=qpOASES_Solver.Configuration()):
        self.configuration = configuration
        self.solver = qpOASES_Solver(configuration)

    def solve_quadratic_program(self, H, f, A, b, Aeq, beq):

        if (A is None and b is not None) or (b is None and A is not None):
            raise ValueError(f"A={A} and b={b} must both be None or both not None.")
        if (Aeq is None and beq is not None) or (beq is None and Aeq is not None):
            raise ValueError(f"Aeq={Aeq} and beq={beq} must both be None or both not None.")

        if A is None:
            A = np.zeros((1,H.shape[0]))
            b = np.zeros((1,))
        if Aeq is None:
            Aeq = np.zeros((1,H.shape[0]))
            beq = np.zeros((1,))

        return self.solver.solve_quadratic_program(H, f, A, b, Aeq, beq)

    def get_active_set(self):
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
