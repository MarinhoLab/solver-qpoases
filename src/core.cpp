/**
(C) Copyright 2025-26 Murilo Marinho (murilomarinho@ieee.org)

pybind11 bindings for marinholab::solvers::qpoases::Solver.

The `Configuration` is exposed with string-based accessors (`set`, `get`,
`has`, `keys`, `defaults`, `reset`), backed by a map of option name ->
string value, so the Python surface is free of qpOASES enum types. For
convenience, `set()` also accepts Python enum members (e.g. a
`HessianType`) or plain numbers/booleans and normalises them to the string
form expected by the C++ core.
*/

#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <marinholab/solvers/qpoases.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;
using namespace marinholab::solvers::qpoases;

namespace
{

/**
 * @brief Normalises an arbitrary Python value into the string form the C++
 * `Configuration::set()` expects.
 *
 * Accepts:
 *  - `str`              -> unchanged
 *  - an enum member     -> its `.name` (e.g. `HessianType.HST_SEMIDEF` ->
 *                          `"HST_SEMIDEF"`)
 *  - `bool`             -> `"true"` / `"false"`
 *  - `int`              -> its decimal representation
 *  - `float`            -> its repr
 */
std::string normalize_option_value(py::handle value)
{
    if(py::isinstance<py::str>(value))
        return value.cast<std::string>();

    if(py::hasattr(value, "name"))
        return py::str(value.attr("name")).cast<std::string>();

    if(py::isinstance<py::bool_>(value))
        return py::cast<bool>(value) ? "true" : "false";

    if(py::isinstance<py::int_>(value))
        return std::to_string(py::cast<long long>(value));

    if(py::isinstance<py::float_>(value))
        return py::str(value).cast<std::string>();

    throw py::type_error(
        "Option value must be a string, an enum member, a number, or a bool; "
        "got " + py::str(value.get_type()).cast<std::string>());
}

} // namespace

PYBIND11_MODULE(_core, m) {

    m.doc() = "Python bindings for a qpOASES-based quadratic program solver.";

    py::class_<Solver> qpoases_solver(m, "qpOASES_Solver",
        "High-level, reusable solver for quadratic programs (QPs) based on qpOASES.\n\n"
        "Solves the following problem:\n\n"
        "    min(x)  0.5*x'Hx + f'x\n"
        "    s.t.    Ax <= b\n"
        "            Aeq*x = beq\n\n"
        "Method signature is compatible with MATLAB's `quadprog`. Once the\n"
        "problem has been solved once, subsequent solves are warm-started by\n"
        "default (see the `use_hotstart` option on the Configuration).");

    py::class_<Configuration> configuration(qpoases_solver, "Configuration",
        "String-keyed holder of all user-configurable solver options.\n\n"
        "Every qpOASES `Options` field plus the wrapper-specific settings\n"
        "(`maximum_working_set_recalculations`, `use_hotstart`,\n"
        "`hessian_type`) is exposed under its name via `set()`/`get()`.\n"
        "Values are strings; booleans are \"true\"/\"false\", and enum\n"
        "options take the enum value name (e.g. \"HST_SEMIDEF\", \"PL_NONE\").\n"
        "Defaults match qpOASES' double-precision defaults except\n"
        "`printLevel`, which defaults to \"PL_NONE\". See `keys()` and\n"
        "`defaults()` for the full list.");

    configuration.def(py::init<>());
    configuration.def("set",
        [](Configuration& self, const std::string& key, const std::string& value) {
            self.set(key, value);
        },
        py::arg("key"), py::arg("value"),
        "Sets the option `key` to the string `value`. Raises ValueError for an "
        "unknown key or a value that does not parse for that option's type.");
    configuration.def("set",
        [](Configuration& self, const std::string& key, py::handle value) {
            self.set(key, normalize_option_value(value));
        },
        py::arg("key"), py::arg("value"),
        "Convenience overload: also accepts enum members, numbers, and bools "
        "(normalised to their string form).");
    configuration.def("get",
        &Configuration::get,
        py::arg("key"),
        "Returns the string value of `key`, or its default if not set.");
    configuration.def("has",
        &Configuration::has,
        py::arg("key"),
        "True if `key` has been explicitly set.");
    configuration.def("keys",
        &Configuration::keys,
        "Sorted list of all settable option names.");
    configuration.def("defaults",
        &Configuration::defaults,
        "Mapping of option name -> default string value.");
    configuration.def("reset",
        &Configuration::reset,
        py::arg("key"),
        "Reverts `key` to its default.");
    configuration.def("reset_all",
        &Configuration::reset_all,
        "Reverts every option to its default.");

    qpoases_solver.def(py::init<const Configuration&>(),
                       py::arg("configuration") = Configuration(),
                       "Constructs a solver with the given configuration (defaults to the default configuration).");
    qpoases_solver.def("solve_quadratic_program",
                       &Solver::solve_quadratic_program,
                       py::arg("H"), py::arg("f"), py::arg("A"), py::arg("b"), py::arg("Aeq"), py::arg("beq"),
                       "Solves min(x) 0.5*x'Hx + f'x s.t. Ax <= b and Aeq*x = beq (MATLAB `quadprog`-like signature). Returns the optimal x.");
    qpoases_solver.def("get_active_set",
                       &Solver::get_active_set,
                       "Returns the active set of constraints obtained in the most recent call to solve_quadratic_program(). One entry per row of the combined constraint matrix (rows of A followed by rows of Aeq): -1 = active at its lower bound, 0 = inactive, +1 = active at its upper bound (equality constraints are always reported as +1).");
    qpoases_solver.def("test_vectorxd",
                       &Solver::test_vectorxd,
                       py::arg("v"),
                       "Round-trips a vector to help evaluate the Eigen <-> std conversions used across the wrapper.");
    qpoases_solver.def("test_matrixxd",
                       &Solver::test_matrixxd,
                       py::arg("m"),
                       "Round-trips a matrix to help evaluate the Eigen <-> std conversions used across the wrapper.");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
