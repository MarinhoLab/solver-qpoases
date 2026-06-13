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
            A = np.zeros_like(f)
            b = np.zeros((1,))
        if Aeq is None:
            Aeq = np.zeros_like(f)
            beq = np.zeros((1,))

        return self.solver.solve_quadratic_program(H, f, A, b, Aeq, beq)