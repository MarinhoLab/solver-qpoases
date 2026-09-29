# AGENTS.md

Repository conventions for the `marinholab-solvers-qpoases` project — a Python
(C++ via pybind11) wrapper around [qpOASES](https://github.com/coin-or/qpOASES),
an online active-set solver for quadratic programs.

## Project layout

```
marinholab/solvers/qpoases/
  __init__.py            Public API re-exports (Solver, Configuration, enums)
  solver.py              numpy-friendly Solver wrapper
  _options.py            Pure-Python IntEnum types (BooleanType, HessianType,
                         PrintLevel, SubjectToStatus) that mirror the qpOASES
                         enums accepted by Configuration.set()
  example.py             runnable example (console script: qpoases_example)
  example_kinematics.py  OPTIONAL example (needs dqrobotics + dqrobotics-pyplot)
  _core.pyi              type stub for the compiled _core extension (ships in the wheel)
  py.typed               PEP 561 marker so stubs are picked up by type checkers
include/marinholab/solvers/qpoases.h C++ header (Solver + Configuration) — qpOASES-free (pimpl Solver; Configuration keyed by name with typed `OptionValue`s)
src/core.cpp             pybind11 module (_core): binds `Solver` + `Configuration` (set/get/has/keys/defaults/reset) with native Python values
src/core_function.cpp    C++ implementation (wraps qpOASES' QPSolver via pimpl); option name->string map; compiled into the `marinholab_qpoases` library (shared by default, `-DBUILD_SHARED_LIBS=OFF` for static)
CMakeLists.txt           CMake build: `marinholab_qpoases` lib (shared by default), `_core` pybind11 module (`-DBUILD_PYTHON=ON`), optional C++ example (`-DBUILD_EXAMPLES=ON`). The vendored qpOASES is always built **static**, linked **PRIVATE** into `marinholab_qpoases`, and its archive is bundled/installed with the package — consumers only link `marinholab::solvers::qpoases`.
example/example.cpp      standalone C++ usage example (target: `example_qpoases`)
tests/test_solver.py     pytest tests for `Solver` (see "Tests")
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

The standalone C++ example (`example/example.cpp`, target `example_qpoases`)
is built only when `BUILD_EXAMPLES=ON`; it is *not* built by `pip install .`:

```console
cmake -B build -GNinja -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=ON
cmake --build build
./build/example_qpoases   # prints the optimal x and the active set
```

## Tests

```console
pip install pytest
cd tests && python -m pytest
```

Run pytest from inside `tests/`, not from the repository root. From the root,
the source `marinholab/` directory (which has no compiled `_core`) shadows the
installed package. CI runs the tests in the Docker job (`docker/compose.yml`)
before `qpoases_example`.

`tests/test_solver.py` checks that one `Solver` instance gives the same
answers as a fresh one when the number of variables or constraints changes
between calls, with `use_hotstart` on and off.

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
  defaults table in `README.md`), with one documented exception:
  `printLevel` defaults to `PL_NONE` (the least verbose level) so the
  solver is quiet by default (qpOASES' own default is `PL_MEDIUM`). Do not
  silently override them here; if a
  particular problem needs a non-default option, set it on the
  `Configuration` in the *caller* (e.g. `example.py:semidefinite()` does
  `config.set("hessian_type", qpoases.HessianType.HST_SEMIDEF)` because its
  Hessian is rank-deficient). Re-validate the `example.py` and
  `example_kinematics.py` solves after any change to a default.
- **Option values are typed, never round-tripped through text.**
  `Configuration` stores `OptionValue = std::variant<bool, long long, double,
  std::string>`, converted to the option's kind in `set()`
  (`detail::normalize`). Defaults come straight from qpOASES' `Options`
  (`detail::default_values`), and `options_from()` starts from qpOASES'
  `Options`, applies the quieter `printLevel`, then only the options that were
  set. Storing values as strings once turned every small qpOASES tolerance
  (`terminationTolerance`, `epsNum`, `epsLITests`, ...) into 0 via
  `std::to_string`, and qpOASES then failed on well-posed QPs (errors 36/68);
  `test_tolerance_defaults_match_qpoases` checks them.
- **Variable bounds are passed explicitly** as -INFTY/+INFTY arrays, never
  `NULL`: with `enableFarBounds` off, qpOASES' `hotstart()` dereferences them
  without a NULL check.
- **`Configuration` keyed by name.** Options are set by name
  (`set(key, value)`); the C++ public header exposes no qpOASES types
  (qpOASES is reached only through the pimpl in `src/core_function.cpp`), so
  enumeration values are held by name. The qpOASES option keys keep the library's
  native camelCase spelling (e.g. `enableFlippingBounds`,
  `terminationTolerance`); only the wrapper-specific keys that have no
  qpOASES counterpart are snake_case (`maximum_working_set_recalculations`,
  `use_hotstart`, `hessian_type`). Enum options take the enum value name
  (e.g. `HST_SEMIDEF`, `PL_NONE`) or, in Python, an enum member; `set()` also
  converts strings for the other kinds (`"1e-9"`, `"false"`). The
  option name->kind map (`detail::option_kinds`) and the pure-Python enums
  in `_options.py` must stay in sync with the `Configuration` options.
- **`Solver` API.** `Solver.solve_quadratic_program()` accepts `None` for
  `A`/`b`/`Aeq`/`beq` and substitutes a single trivially-satisfied zero row;
  `Solver.get_active_set()` returns one entry per combined constraint row
  (-1 lower / 0 inactive / +1 upper).
- **Style.** Match the existing style: docstrings on the public API.
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

## Debian build (libmarinholab-solver-qpoases)

The `.deb` is **not** built by this repo's CI — it is built by the
SmartArmStack `build_ros2.sh` (SmartArmStack/smart_arm_stack_ROS2), which
clones `main` with submodules, runs `bash tools/bump-changelog.sh` and
`dpkg-buildpackage -us -uc -b` on this `debian/` directory, then `dpkg -i`
installs it. `build_ros2.sh`'s matrix is `ubuntu-24.04` (amd64) +
`ubuntu-24.04-arm` (arm64); the `debian/` packaging is shared, so it must
build on **both** architectures.

```console
bash docker/build-deb.sh            # build the .deb -> ./.deb-out/
bash docker/build-deb.sh --test     # + install it and build/run a
                                    # find_package consumer inside the container
```

`debian/rules` appends `-fno-lto` after the `dpkg-buildflags` defaults:
Ubuntu noble enables `-flto=auto`, and LTO hits a GCC 13 linker internal
compiler error on x86-64 when linking the vendored static qpOASES archive
(`lto1: return_token, opts-common.cc:2137`). Do not re-enable LTO.

Verified (2026-09-28, commit `fix-amd64-lto-debian-build` / PR #14): on both
amd64 and arm64 — `dpkg-buildpackage` succeeds, the `.deb` installs, and a
`find_package(marinholab_solver_qpoases)` consumer linking
`marinholab::solvers::qpoases` builds and solves. The `.deb` carries the
static `libqpOASES.a` (bundled) plus the headers; `Depends` is only
libc6/libgcc-s1/libstdc++6, i.e. qpOASES is fully private.

## Version

The version is `0.0.1` in `pyproject.toml`; `setup.py` computes a
date+commit-derived fallback when the git tag isn't available, which is why
locally-built wheels may show a different distribution version — this is
pre-existing and unrelated to code changes.
