"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv3 License
"""
import numpy as np
from marinholab.solvers.qpoases import qpOASES_Solver

def main():

    solver = qpOASES_Solver()

    x = np.array([1.0, 0.0, 0.0, 0.0])
    xd = np.array([0.0, 0.0, 0.0, 1.0])

    x_tilde = (x - xd).reshape((4, 1))

    J = np.eye(4)
    H = J.T @ J
    f = 1.0 * J.T @ x_tilde

    A = np.array([x[0], x[1], x[2], x[3]]).reshape((1, 4))
    b = np.array([0.0]).reshape((1,1))

    # No constraints
    u = solver.solve_quadratic_program(H,
                                       f,
                                       np.array([0.0,0.0,0.0,0.0]).reshape((1,4)),
                                       np.array([0.0]),
                                       np.array([0.0,0.0,0.0,0.0]).reshape((1,4)),
                                       np.array([0.0])
                                      )
    # Equality only
    u_eq = solver.solve_quadratic_program(H,
                                       f,
                                       np.array([0.0,0.0,0.0,0.0]).reshape((1,4)),
                                       np.array([0.0]),
                                       A,
                                       b
                                      )

    # Inequality only
    u_ineq = solver.solve_quadratic_program(H,
                                       f,
                                       A,
                                       b,
                                       np.array([0.0,0.0,0.0,0.0]).reshape((1,4)),
                                       np.array([0.0])
                                      )

    # Both
    u_both = solver.solve_quadratic_program(H,
                                            f,
                                            A,
                                            b,
                                            A,
                                            b
                                            )

    print(u)
    print(u_eq)
    print(u_ineq)
    print(u_both)


if __name__ == "__main__":
    main()