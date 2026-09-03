"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv3 License
"""
import numpy as np
from marinholab.solvers import qpoases

def positivedefinite() -> None:
    solver = qpoases.Solver()

    x = np.array([1.0, 0.0, 0.0, 0.0])
    xd = np.array([0.0, 0.0, 0.0, 1.0])

    x_tilde = (x - xd).reshape((4, 1))

    J = np.eye(4)
    H = J.T @ J
    f = 1.0 * J.T @ x_tilde

    A = np.array([x[0], x[1], x[2], x[3]]).reshape((1, 4))
    b = np.array([0.0]).reshape((1, 1))

    # No constraints
    u = solver.solve_quadratic_program(H,
                                       f,
                                       np.array([0.0, 0.0, 0.0, 0.0]).reshape((1, 4)),
                                       np.array([0.0]),
                                       np.array([0.0, 0.0, 0.0, 0.0]).reshape((1, 4)),
                                       np.array([0.0])
                                       )
    # Equality only
    u_eq = solver.solve_quadratic_program(H,
                                          f,
                                          np.array([0.0, 0.0, 0.0, 0.0]).reshape((1, 4)),
                                          np.array([0.0]),
                                          A,
                                          b
                                          )

    # Inequality only
    u_ineq = solver.solve_quadratic_program(H,
                                            f,
                                            A,
                                            b,
                                            np.array([0.0, 0.0, 0.0, 0.0]).reshape((1, 4)),
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

def semidefinite() -> None:
    # The Hessian here is positive semi-definite (rank deficient), so the
    # Hessian type must be set to HST_SEMIDEF; everything else stays at the
    # defaults.
    config = qpoases.Configuration()
    config.hessian_type = qpoases.HessianType.HST_SEMIDEF
    solver = qpoases.Solver(config)

    x = np.array([1.0, 0.0, 0.0, 0.0])
    xd = np.array([0.0, 0.0, 1.0, 0.0])

    x_tilde = (x - xd).reshape((4, 1))

    J = np.diag([1.0,1.0,1.0,0.0])
    H = J.T @ J
    f = 1.0 * J.T @ x_tilde

    Wl = -np.eye(4,4)
    wl = -np.ones(4,)
    Wu = np.eye(4,4)
    wu = np.ones(4,)

    W = np.vstack((Wl, Wu))
    w = np.concatenate((wl, wu))

    # No constraints
    u = solver.solve_quadratic_program(H,
                                       f,
                                       W,
                                       w,
                                       np.array([0.0, 0.0, 0.0, 0.0]).reshape((1, 4)),
                                       np.array([0.0])
                                       )
    print(u)

def termination_tolerance() -> None:
    # The termination tolerance is the relative tolerance used by qpOASES to
    # decide when the homotopy algorithm has converged. Tightening it (i.e.
    # using a smaller value) can improve solution accuracy at the cost of
    # more working set recalculations; loosening it can speed up solves at
    # the cost of accuracy.
    config = qpoases.Configuration()
    config.terminationTolerance = 1.0e-9
    solver = qpoases.Solver(config)

    x = np.array([1.0, 0.0, 0.0, 0.0])
    xd = np.array([0.0, 0.0, 0.0, 1.0])

    x_tilde = (x - xd).reshape((4, 1))

    J = np.eye(4)
    H = J.T @ J
    f = 1.0 * J.T @ x_tilde

    A = np.array([x[0], x[1], x[2], x[3]]).reshape((1, 4))
    b = np.array([0.0]).reshape((1, 1))

    u = solver.solve_quadratic_program(H,
                                       f,
                                       A,
                                       b,
                                       np.array([0.0, 0.0, 0.0, 0.0]).reshape((1, 4)),
                                       np.array([0.0])
                                       )
    print(u)

def nones() -> None:
    solver = qpoases.Solver()

    x = np.array([1.0, 0.0, 0.0, 0.0])
    xd = np.array([0.0, 0.0, 0.0, 1.0])

    x_tilde = (x - xd).reshape((4, 1))

    J = np.eye(4)
    H = J.T @ J
    f = 1.0 * J.T @ x_tilde

    A = np.array([x[0], x[1], x[2], x[3]]).reshape((1, 4))
    b = np.array([0.0]).reshape((1, 1))

    # No constraints
    u = solver.solve_quadratic_program(H,
                                       f,
                                       None,
                                       None,
                                       None,
                                       None
                                       )

    # Equality only
    u_eq = solver.solve_quadratic_program(H,
                                          f,
                                          None,
                                          None,
                                          A,
                                          b
                                          )

    # Inequality only
    u_ineq = solver.solve_quadratic_program(H,
                                            f,
                                            A,
                                            b,
                                            None,
                                            None
                                            )

def active_set() -> None:
    # get_active_set() reports, for each row of the combined constraint matrix
    # (rows of A followed by rows of Aeq), whether it is inactive (0), active at
    # its lower bound (-1), or active at its upper bound (+1). Equality
    # constraints are always reported as active (+1), since their lower and
    # upper bounds coincide.
    solver = qpoases.Solver()

    H = np.eye(2)
    f = np.array([-1.0, -1.0])  # unconstrained optimum would be x = [1, 1]

    # This inequality constraint is tight at the optimum: x[0] <= 0.2
    A = np.array([1.0, 0.0]).reshape((1, 2))
    b = np.array([0.2])

    # This inequality constraint is loose at the optimum: x[1] <= 5.0
    A = np.vstack((A, np.array([0.0, 1.0])))
    b = np.concatenate((b, np.array([5.0])))

    u = solver.solve_quadratic_program(H,
                                       f,
                                       A,
                                       b,
                                       np.array([0.0, 0.0]).reshape((1, 2)),
                                       np.array([0.0])
                                       )
    print(u)
    print(solver.get_active_set())  # expected [1, 0, 0]: first constraint active at its upper bound

def main() -> None:
    positivedefinite()
    semidefinite()
    termination_tolerance()
    nones()
    active_set()


if __name__ == "__main__":
    main()