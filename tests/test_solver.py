"""
Copyright (C) 2025 Murilo Marques Marinho (www.murilomarinho.info)
LGPLv2.1 License

Tests for `Solver`, in particular reusing one instance for problems whose
number of variables or constraints changes between calls.
"""
from __future__ import annotations

import subprocess
import sys

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


# qpOASES' own defaults for its real-valued tolerances (double precision,
# EPS = 2.221e-16), from Options::setToDefault().
_QPOASES_TOLERANCE_DEFAULTS = {
    "terminationTolerance": 5.0e6 * 2.221e-16,
    "boundTolerance": 1.0e6 * 2.221e-16,
    "epsNum": -1.0e3 * 2.221e-16,
    "epsDen": 1.0e3 * 2.221e-16,
    "epsFlipping": 1.0e3 * 2.221e-16,
    "epsRegularisation": 1.0e3 * 2.221e-16,
    "epsIterRef": 1.0e2 * 2.221e-16,
    "epsLITests": 1.0e5 * 2.221e-16,
    "epsNZCTests": 3.0e3 * 2.221e-16,
    "rcondSMin": 1.0e-14,
}


@pytest.mark.parametrize("key,expected", sorted(_QPOASES_TOLERANCE_DEFAULTS.items()))
def test_tolerance_defaults_match_qpoases(key: str, expected: float):
    # These were formatted with std::to_string (six decimals), which turned
    # every one of them into 0 and made qpOASES fail on well-posed QPs.
    value = float(qpoases.Configuration().get(key))
    assert value == pytest.approx(expected, rel=1e-12, abs=0.0)


_H_NEARLY_PARALLEL = np.array([
    [0.25651736899745614, -2.5913434446707725e-05, -0.0626048559312173, -0.11271769882530888, 6.513887957419305e-05, -0.00015414092821932845, -8.1436538068478e-07],
    [-2.5913434446707725e-05, 0.11825599479590891, -0.11819089295419094, 0.02616913110062032, 0.00023847513301919875, 1.1421942888437196e-05, 0.0002495395558210686],
    [-0.0626048559312173, -0.11819089295419094, 0.1554834678390627, -6.940259506938462e-18, -5.255297456570405e-05, -0.00015092587165896066, 1.4854805485548448e-05],
    [-0.11271769882530888, 0.02616913110062032, -6.940259506938462e-18, 0.16219789999999978, -1.29090690005129e-17, 0.00019664251931266594, 5.169218302319338e-05],
    [6.513887957419305e-05, 0.00023847513301919875, -5.255297456570405e-05, -1.29090690005129e-17, 0.01025, 4.425505005905512e-20, 0.00023556807516247075],
    [-0.00015414092821932845, 1.1421942888437196e-05, -0.00015092587165896066, 0.00019664251931266594, 4.425505005905512e-20, 0.01025, 1.941222440883036e-20],
    [-8.1436538068478e-07, 0.0002495395558210686, 1.4854805485548448e-05, 5.169218302319338e-05, 0.00023556807516247075, 1.941222440883036e-20, 0.01025],
])
_F_NEARLY_PARALLEL = np.array([0.42952553527646126, -0.00020940312632713637, -0.10887144722472775, -0.6151113176841987, -3.6810925829469135e-05, 0.0001676549184949939, 3.2740960161923156e-05])
_A_NEARLY_PARALLEL = np.array([
    [-1.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0],
    [-0.3789752768804049, -0.020480625849327427, -8.326449931361553e-17, 5.551122648706248e-17, -0.0, -0.0, -0.0],
])
_B_NEARLY_PARALLEL = np.array([1.1935165390418951, 0.44207317007108227])


def test_two_nearly_parallel_inactive_constraints():
    """Regression: two nearly parallel constraints (cosine 0.999), neither
    active at the solution. With the tolerances zeroed, qpOASES returned
    RET_INIT_FAILED_HOTSTART (36) here; the solution is the unconstrained
    minimiser."""
    x = qpoases.Solver().solve_quadratic_program(
        _H_NEARLY_PARALLEL, _F_NEARLY_PARALLEL, _A_NEARLY_PARALLEL, _B_NEARLY_PARALLEL, None, None)
    x_unconstrained = np.linalg.solve(_H_NEARLY_PARALLEL, -_F_NEARLY_PARALLEL)
    assert np.all(_A_NEARLY_PARALLEL @ x_unconstrained < _B_NEARLY_PARALLEL)
    assert np.allclose(x, x_unconstrained, atol=1e-9)


def test_hotstart_without_far_bounds_does_not_crash():
    """Regression: with enableFarBounds off, qpOASES' hotstart() dereferenced
    the NULL variable bounds that the wrapper passed (segmentation fault on the
    second solve). Run in a subprocess so a crash fails the test instead of
    the test run."""
    code = """
import numpy as np
from marinholab.solvers import qpoases
configuration = qpoases.Configuration()
configuration.set("enableFarBounds", "false")
solver = qpoases.Solver(configuration)
H = np.eye(2); f = np.array([-1.0, -1.0]); A = np.array([[1.0, 0.0]]); b = np.array([0.2])
for _ in range(3):
    x = solver.solve_quadratic_program(H, f, A, b, None, None)
    assert np.allclose(x, [0.2, 1.0]), x
"""
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, (result.returncode, result.stderr[-2000:])
