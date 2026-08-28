# solver-qpoases

A [qpOASES](https://github.com/coin-or/qpOASES) wrapper for Python that ships
prebuilt binaries. It exposes qpOASES' online active-set solver through a thin,
numpy-friendly, MATLAB-`quadprog`-like interface.

```console
pip install marinholab-solvers-qpoases
```

## Overview

Given a symmetric matrix `H`, a vector `f`, and (optionally) inequality and
equality constraint matrices, the solver solves the quadratic program

```
min_x   0.5 * x' H x + f' x
s.t.    A x <= b
        Aeq x = beq
```

Once a problem has been solved once, subsequent solves on the same `Solver`
instance are *warm-started* by default (`Configuration.use_hotstart = True`),
which is the main performance benefit of qpOASES for repeated, related QPs.

## Quickstart

```python
import numpy as np
from marinholab.solvers import qpoases

solver = qpoases.Solver()

H = np.eye(2)                          # positive definite Hessian
f = np.array([-1.0, -1.0])             # linear term
A = np.array([[1.0, 0.0]])             # x[0] <= 0.2
b = np.array([0.2])

x = solver.solve_quadratic_program(H, f, A, b,
                                   Aeq=np.zeros((1, 2)),
                                   beq=np.zeros((1,)))
# x ≈ [0.2, 1.0]
```

### Omitting constraints

Any of the four constraint arguments (`A`, `b`, `Aeq`, `beq`) can be `None`,
meaning "no such constraint". The matrix and its right-hand side must be
omitted (or provided) together:

```python
# Unconstrained
x = solver.solve_quadratic_program(H, f, None, None, None, None)

# Equality constraints only
x = solver.solve_quadratic_program(H, f, None, None, Aeq, beq)
```

### Active set

`solver.get_active_set()` reports, for each row of the combined constraint
matrix (rows of `A` followed by rows of `Aeq`), whether it is:

* `-1` active at its **lower** bound,
* `0`  inactive,
* `+1` active at its **upper** bound (equality constraints are always `+1`,
       since their bounds coincide).

```python
solver.solve_quadratic_program(H, f, A, b, Aeq, beq)
solver.get_active_set()  # e.g. [ 1.0, 0.0, 0.0]
```

## Configuration

All of qpOASES' `Options` fields are exposed, plus a few wrapper-specific
settings. Create a `Configuration`, tweak the fields you need, and pass it to
the solver:

```python
config = qpoases.Configuration()
config.hessian_type = qpoases.HessianType.HST_SEMIDEF   # H is rank-deficient
config.termination_tolerance = 1.0e-9                    # tighter convergence
solver = qpoases.Solver(config)
```

The enum types are re-exported for convenience: `qpoases.BooleanType`,
`qpoases.HessianType`, `qpoases.PrintLevel`, and `qpoases.SubjectToStatus`.

### Wrapper-specific options

| Option | Default | Type | Description |
|---|---|---|---|
| `maximum_working_set_recalculations` | `150` | `int` | Max working-set recalculations during the initial homotopy (`nWSR` passed to `init`/`hotstart`). Increase if solves hit the maximum. |
| `use_hotstart` | `True` | `bool` | Warm-start subsequent solves with `hotstart()` instead of re-initialising with `init()`. |
| `hessian_type` | `HST_POSDEF` | `HessianType` | Definiteness assumed for `H`; given to the underlying `SQProblem`. |

### Hessian definiteness (`HessianType`)

| Value | Meaning |
|---|---|
| `HST_ZERO` | Hessian is the zero matrix (LP formulation) |
| `HST_IDENTITY` | Hessian is the identity matrix |
| `HST_POSDEF` | Hessian is (strictly) positive definite |
| `HST_POSDEF_NULLSPACE` | Positive definite on the null space of active bounds/constraints |
| `HST_SEMIDEF` | Positive semi-definite |
| `HST_INDEF` | Indefinite |
| `HST_UNKNOWN` | Unknown |

### qpOASES options

These map 1:1 onto qpOASES' `Options` fields. Defaults match qpOASES' own
defaults for a **double-precision** build (see `Options::setToDefault()`). See
the [qpOASES manual](https://www.coin-or.org/qpOASES/doc/3.0/manual.pdf) for a
full description of each option.

**Booleans** (`BooleanType`: `BT_FALSE` / `BT_TRUE`)

| Option | Default | Description |
|---|---|---|
| `enable_ramping` | `BT_TRUE` | Enables the ramping strategy. |
| `enable_far_bounds` | `BT_TRUE` | Enables the far bounds strategy. |
| `enableFlippingBounds` | `BT_TRUE` | Allows flipping active bounds between lower and upper values. |
| `enableRegularisation` | `BT_FALSE` | Regularises `H` when (semi-)definiteness is detected. |
| `enable_full_li_tests` | `BT_FALSE` | Uses the condition-hardened linear-independence (LI) test. |
| `enableNZCTests` | `BT_TRUE` | Enables the nonzero-curvature test. |
| `enable_equalities` | `BT_FALSE` | Treats equality constraints as always active. |
| `enable_inertia_correction` | `BT_TRUE` | Repairs the working set when negative curvature is found during a hotstart. |
| `enable_drop_infeasibles` | `BT_FALSE` | Whether infeasible constraints may be dropped. |

**Integers** (`int_t`)

| Option | Default | Description |
|---|---|---|
| `enable_drift_correction` | `1` | Frequency of drift corrections (`0` = off). |
| `enable_cholesky_refactorisation` | `0` | Frequency of full Cholesky refactorisation of the projected Hessian (`0` = rank updates only). |
| `num_regularisation_steps` | `0` | Max successive regularisation steps. |
| `num_refinement_steps` | `1` | Max iterative-refinement steps. |
| `drop_bound_priority` | `1` | Priority used when dropping bounds. |
| `drop_eq_con_priority` | `1` | Priority used when dropping equality constraints. |
| `drop_ineq_con_priority` | `1` | Priority used when dropping inequality constraints. |

**Reals** (`real_t`, `double`)

| Option | Default | Description |
|---|---|---|
| `termination_tolerance` | `5.0e6 * EPS` (~`1.1e-9`) | Relative tolerance that stops the homotopy. Smaller = more accurate, more work. |
| `bound_tolerance` | `1.0e6 * EPS` | Bound tolerance; a constraint whose bounds differ by less is treated as an equality. |
| `bound_relaxation` | `1.0e4` | Offset for relaxing bounds at the start of the initial homotopy (also the initial far-bound value). |
| `eps_num` | `-1.0e3 * EPS` | Numerator tolerance for the ratio test. |
| `eps_den` | `1.0e3 * EPS` | Denominator tolerance for the ratio test. |
| `max_primal_jump` | `1.0e8` | Max allowed primal jump in nonzero-curvature tests. |
| `max_dual_jump` | `1.0e8` | Max allowed dual jump in LI tests. |
| `initial_ramping` | `0.5` | Start value of the ramping strategy. |
| `final_ramping` | `1.0` | Final value of the ramping strategy. |
| `initial_far_bounds` | `1.0e6` | Initial size of the far bounds. |
| `grow_far_bounds` | `1.0e3` | Growth factor applied to the far bounds. |
| `eps_flipping` | `1.0e3 * EPS` | Tolerance of the squared Cholesky diagonal factor that triggers flipping a bound. |
| `eps_regularisation` | `1.0e3 * EPS` | Scaling factor of the identity matrix used for Hessian regularisation. |
| `eps_iter_ref` | `1.0e2 * EPS` | Early-termination tolerance for iterative refinement. |
| `eps_li_tests` | `1.0e5 * EPS` | Tolerance for the linear-independence tests. |
| `eps_nzc_tests` | `3.0e3 * EPS` | Tolerance for the nonzero-curvature tests. |
| `rcond_s_min` | `1.0e-14` | Min reciprocal condition number of the Schur complement before a refactorisation is triggered. |

**Status / print enums**

| Option | Default | Type | Description |
|---|---|---|---|
| `print_level` | `PL_MEDIUM` | `PrintLevel` | Verbosity of qpOASES output (`PL_NONE`, `PL_LOW`, `PL_MEDIUM`, `PL_HIGH`, `PL_TABULAR`, `PL_DEBUG_ITER`). |
| `initial_status_bounds` | `ST_LOWER` | `SubjectToStatus` | Status assumed for all bounds at the first iteration. |

### Print levels (`PrintLevel`)

`PL_DEBUG_ITER`, `PL_TABULAR`, `PL_NONE`, `PL_LOW`, `PL_MEDIUM`, `PL_HIGH`.

### Bound/constraint statuses (`SubjectToStatus`)

`ST_LOWER`, `ST_INACTIVE`, `ST_UPPER`, `ST_INFEASIBLE_LOWER`,
`ST_INFEASIBLE_UPPER`, `ST_UNDEFINED`.

## Examples

* `marinholab/solvers/qpoases/example.py` — positive-definite and
  semi-definite solves, the `None`-constraint path, and the active set. Run it
  with `qpoases_example` (installed as a console script).
* `marinholab/solvers/qpoases/example_kinematics.py` — an *optional* example
  showing the solver used in a hierarchical (task-priority) controller for a
  kinematically redundant robot, built on [`dqrobotics`](https://pypi.org/project/dqrobotics/).
  It requires the optional dependencies `dqrobotics` and `dqrobotics-pyplot`
  (`pip install --pre dqrobotics dqrobotics-pyplot`).

## Building from source

The package builds a C++ extension (via CMake + pybind11) and vendors
[qpOASES](https://github.com/coin-or/qpOASES) and [pybind11](https://github.com/pybind/pybind11)
as git submodules.

```console
git clone --recurse-submodules <repo>
pip install .
```

Prerequisites: a C++23 compiler, CMake, an Eigen3 installation, and Python.
On Ubuntu: `sudo apt-get install cmake libeigen3-dev`.

## Type checking

The package ships a type stub (`marinholab/solvers/qpoases/_core.pyi`) and a
`py.typed` marker, so downstream projects can be checked with
[Pyright](https://github.com/microsoft/pyright) (or Pylance) without extra
configuration. Run the project's own check with:

```console
pyright
```

## License

The qpOASES library is LGPLv2.1; the wrapper is under the terms of the
included `LICENSE` file.
