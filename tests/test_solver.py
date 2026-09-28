"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv2.1 License

Tests for `Solver`, in particular reusing one instance for problems whose
number of variables or constraints changes between calls.
"""
from __future__ import annotations

import numpy as np
import pytest

from marinholab.solvers import qpoases

H2 = np.eye(2)
F2 = np.array([-1.0, -1.0])
A_THREE_ROWS = np.array([[1.0, 0.0], [0.0, 1.0], [1.0, 1.0]])
B_THREE_ROWS = np.array([0.2, 0.3, 10.0])
A_ONE_ROW = np.array([[1.0, 0.0]])
B_ONE_ROW = np.array([0.2])


def _configuration(use_hotstart: bool) -> qpoases.Configuration:
    configuration = qpoases.Configuration()
    configuration.set("use_hotstart", "true" if use_hotstart else "false")
    return configuration


def _random_problem(rng: np.random.Generator, n: int, m: int):
    """A strictly convex QP for which x = 0 is feasible."""
    M = rng.standard_normal((n, n))
    H = M @ M.T + np.eye(n)
    f = rng.standard_normal(n)
    A = rng.standard_normal((m, n))
    b = rng.uniform(0.1, 1.0, m)
    return H, f, A, b


def test_single_solve():
    x = qpoases.Solver().solve_quadratic_program(H2, F2, A_ONE_ROW, B_ONE_ROW, None, None)
    assert np.allclose(x, [0.2, 1.0])


@pytest.mark.parametrize("use_hotstart", [True, False])
def test_fewer_constraints_after_first_solve(use_hotstart: bool):
    solver = qpoases.Solver(_configuration(use_hotstart))
    x = solver.solve_quadratic_program(H2, F2, A_THREE_ROWS, B_THREE_ROWS, None, None)
    assert np.allclose(x, [0.2, 0.3])
    x = solver.solve_quadratic_program(H2, F2, A_ONE_ROW, B_ONE_ROW, None, None)
    assert np.allclose(x, [0.2, 1.0])


@pytest.mark.parametrize("use_hotstart", [True, False])
def test_more_constraints_after_first_solve(use_hotstart: bool):
    solver = qpoases.Solver(_configuration(use_hotstart))
    x = solver.solve_quadratic_program(H2, F2, A_ONE_ROW, B_ONE_ROW, None, None)
    assert np.allclose(x, [0.2, 1.0])
    x = solver.solve_quadratic_program(H2, F2, A_THREE_ROWS, B_THREE_ROWS, None, None)
    assert np.allclose(x, [0.2, 0.3])


@pytest.mark.parametrize("use_hotstart", [True, False])
def test_different_number_of_variables(use_hotstart: bool):
    solver = qpoases.Solver(_configuration(use_hotstart))
    x = solver.solve_quadratic_program(H2, F2, A_ONE_ROW, B_ONE_ROW, None, None)
    assert np.allclose(x, [0.2, 1.0])
    H3 = np.eye(3)
    f3 = np.array([-1.0, -1.0, -1.0])
    x = solver.solve_quadratic_program(H3, f3, np.array([[0.0, 0.0, 1.0]]), np.array([0.5]), None, None)
    assert np.allclose(x, [1.0, 1.0, 0.5])


def test_same_size_warm_start():
    """Consecutive problems of the same size, as in a control loop."""
    rng = np.random.default_rng(1)
    solver = qpoases.Solver()
    H, f, A, b = _random_problem(rng, 5, 8)
    for _ in range(50):
        f = f + 0.05 * rng.standard_normal(5)
        b = np.clip(b + 0.05 * rng.standard_normal(8), 0.1, None)
        x_reused = solver.solve_quadratic_program(H, f, A, b, None, None)
        x_fresh = qpoases.Solver().solve_quadratic_program(H, f, A, b, None, None)
        assert np.allclose(x_reused, x_fresh, atol=1e-6)


@pytest.mark.parametrize("use_hotstart", [True, False])
def test_reused_solver_matches_fresh_solver_when_sizes_change(use_hotstart: bool):
    rng = np.random.default_rng(0)
    solver = qpoases.Solver(_configuration(use_hotstart))
    for _ in range(200):
        n = int(rng.integers(2, 8))
        m = int(rng.integers(1, 12))
        H, f, A, b = _random_problem(rng, n, m)
        x_fresh = qpoases.Solver().solve_quadratic_program(H, f, A, b, None, None)
        x_reused = solver.solve_quadratic_program(H, f, A, b, None, None)
        assert np.allclose(x_reused, x_fresh, atol=1e-6)
