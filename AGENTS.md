# AGENTS.md

Repository conventions for the `marinholab-solvers-qpoases` project — a Python
(C++ via pybind11) wrapper around [qpOASES](https://github.com/coin-or/qpOASES),
an online active-set solver for quadratic programs.

## Project layout

```
marinholab/solvers/qpoases/
  __init__.py            Public API re-exports (Solver, Configuration, enums)
  solver.py              numpy-friendly Solver wrapper
  example.py             runnable example (console script: qpoases_example)
  example_kinematics.py  OPTIONAL example (needs dqrobotics + dqrobotics-pyplot)
  _core.pyi              type stub for the compiled _core extension (ships in the wheel)
  py.typed               PEP 561 marker so stubs are picked up by type checkers
include/qpOASES_solver.h C++ header: qpOASES_Solver + Configuration (doxygen-documented)
src/core.cpp             pybind11 module (_core): binds qpOASES_Solver + Configuration + enums
src/core_function.cpp    C++ implementation (wraps qpOASES' QPSolver)
qpOASES/                 qpOASES (git submodule)
pybind11/                pybind11 (git submodule)
setup.py                 PEP 517 build (CMake + pybind11)
```

## Build & install

Requires: a C++23 compiler (e.g. `g++`), CMake, Ninja, `eigen3` (dev headers),
and Python >= 3.9 with `numpy` + `setuptools`/`wheel`. On Ubuntu:
`sudo apt-get install cmake libeigen3-dev`.

```console
git submodule update --init --recursive   # if cloning without submodules
pip install -e .                          # editable install (builds _core in build_ext)
# or produce a wheel:
python setup.py bdist_wheel
pip install dist/marinholab_solvers_qpoases-*.whl
```

The extension name is `marinholab.solvers.qpoases._core`. Builds are slow on
first run; CMake reuses the `build/` cache across builds.

## Run the example (smoke test)

```console
qpoases_example
```

(or `python -m marinholab.solvers.qpoases.example`). Exits 0 and prints one
line of optimal `x` per sub-example (positive-definite, semi-definite,
`None`-constraints, active set).

`example_kinematics.py` additionally needs the *optional* dependencies
`dqrobotics` and `dqrobotics-pyplot` (`pip install --pre dqrobotics
dqrobotics-pyplot`); it is not required for the core package to work.

## Type checking (Pyright)

```console
pyright
```

Configuration lives in `pyrightconfig.json` (`pythonVersion` 3.9,
`typeCheckingMode` basic). It must pass with **0 errors / 0 warnings**.

- The compiled extension `_core` is typed through the `_core.pyi` stub.
  Keep the stub in sync with the pybind11 surface in `src/core.cpp`.
- `example_kinematics.py` depends on the untyped third-party `dqrobotics`
  package; its `import` lines and matplotlib 3-D axis calls carry targeted
  `# type: ignore[reportAttributeAccessIssue]` comments. Do not remove them
  (they are the documented reason those lines are ignored).
- The package ships `py.typed` + `_core.pyi` in the wheel (`package_data` in
  `setup.py`) so downstream projects can be checked against it.

## Conventions

- **Defaults match qpOASES.** `Configuration` defaults mirror qpOASES'
  own `Options::setToDefault()` for a double-precision build (see the
  defaults table in `README.md`). Do not silently override them here; if a
  particular problem needs a non-default option, set it on the
  `Configuration` in the *caller* (e.g. `example.py:semidefinite()` sets
  `hessian_type = HST_SEMIDEF` because its Hessian is rank-deficient).
  Re-validate the `example.py` and `example_kinematics.py` solves after any
  change to a default.
- **`Solver` API.** `Solver.solve_quadratic_program()` accepts `None` for
  `A`/`b`/`Aeq`/`beq` and substitutes a single trivially-satisfied zero row;
  `Solver.get_active_set()` returns one entry per combined constraint row
  (-1 lower / 0 inactive / +1 upper).
- **Style.** Match the existing style: docstrings on the public API and
  snake_case for every `Configuration` field (including the ones whose
  qpOASES `Options` names are camelCase, e.g. `enable_flipping_bounds` for
  `Options::enableFlippingBounds`).
- **Doxygen.** C++ types and members are documented with Doxygen
  (`/** ... @brief ... @see ... */` blocks). Keep that when adding fields.
- **Annotations.** All public Python API is fully type-annotated and must
  remain Pyright-clean. `requires-python` is `>= 3.9`; use
  `from __future__ import annotations` where PEP 604 `X | Y` is used so the
  module still parses on 3.9.

## CI

`.github/workflows/python-publish.yml` builds the wheel on `ubuntu-latest` and
`ubuntu-24.04-arm` for Python 3.12 and 3.13 and publishes to PyPI. Local
builds here mirror that (aarch64, Python 3.13).

## Version

The version is `0.0.1` in `pyproject.toml`; `setup.py` computes a
date+commit-derived fallback when the git tag isn't available, which is why
locally-built wheels may show a different distribution version — this is
pre-existing and unrelated to code changes.
