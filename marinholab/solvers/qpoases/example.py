"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv3 License
"""
import numpy as np
from marinholab.solvers.qpoases import qpOASES_Solver

def main():

    solver = qpOASES_Solver

    x = np.arrray([1.0, 0.0, 0.0, 0.0])
    xd = np.array([0.0, 0.0, 0.0, 1.0])

    x_tilde = x - xd

    J = np.ones(4,4)
    H = J.T @ J
    f = 1.0 * J.T @ x_tilde

    Aeq = np.array([[x[0], x[1], x[2], x[3]]])
    beq = np.array([0.0])

    u = solver.solve_quadratic_program(H, f, None, None, Aeq, beq)
    print(u)


if __name__ == "__main__":
    main()