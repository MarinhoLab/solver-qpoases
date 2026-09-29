/**
(C) Copyright 2025-26 Murilo Marinho (murilomarinho@ieee.org)

pybind11 bindings for marinholab::solvers::qpoases::Solver.

The `Configuration` is exposed with accessors keyed by option name (`set`,
`get`, `has`, `keys`, `defaults`, `reset`). Values are native Python bool,
int, float, or str (enumeration values by name), so the Python surface is
free of qpOASES enum types. `set()` also accepts Python enum members (e.g. a
`HessianType`), which are passed by name.
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
 * @brief Converts a Python value into an `OptionValue` for
 * `Configuration::set()`, which then converts it to the option's kind.
 *
 * Accepts:
 *  - `bool`             -> bool (checked before int, since bool is an int)
 *  - an enum member     -> its `.name` (e.g. `HessianType.HST_SEMIDEF` ->
 *                          `"HST_SEMIDEF"`; checked before int, since an
 *                          `IntEnum` is an int)
 *  - `int`, or anything with `__index__` (e.g. numpy integers) -> long long
 *  - `float`, or anything with `__float__` (e.g. numpy floats) -> double
 *  - `str`              -> str
 */
OptionValue to_option_value(py::handle value)
{
    if(py::isinstance<py::bool_>(value))
        return value.cast<bool>();

    // Leaked on purpose: a static py::object would be destroyed after the
    // interpreter has shut down.
    static const py::object* enum_type = new py::object(py::module_::import("enum").attr("Enum"));
    if(py::isinstance(value, *enum_type))
        return py::str(value.attr("name")).cast<std::string>();

    if(py::isinstance<py::str>(value))
        return value.cast<std::string>();

    if(py::isinstance<py::int_>(value) || py::hasattr(value, "__index__"))
        return py::int_(py::reinterpret_borrow<py::object>(value)).cast<long long>();

    if(py::isinstance<py::float_>(value) || py::hasattr(value, "__float__"))
        return py::float_(py::reinterpret_borrow<py::object>(value)).cast<double>();

    throw py::type_error(
        "Option value must be a bool, an int, a float, a string, or an enum member; "
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
        "Holder of all user-configurable solver options, keyed by name.\n\n"
        "Every qpOASES `Options` field plus the wrapper-specific settings\n"
        "(`maximum_working_set_recalculations`, `use_hotstart`,\n"
        "`hessian_type`) is exposed under its name via `set()`/`get()`.\n"
        "Values are bool, int, float, or str: enum options take the enum\n"
        "value name (e.g. \"HST_SEMIDEF\", \"PL_NONE\") or an enum member.\n"
        "`set()` also converts strings such as \"1e-9\" or \"false\".\n"
        "Unset options use qpOASES' double-precision defaults except\n"
        "`printLevel`, which defaults to \"PL_NONE\". See `keys()` and\n"
        "`defaults()` for the full list.");

    configuration.def(py::init<>());
    configuration.def("set",
        [](Configuration& self, const std::string& key, py::handle value) {
            self.set(key, to_option_value(value));
        },
        py::arg("key"), py::arg("value"),
        "Sets the option `key` to `value` (bool, int, float, str, or an enum "
        "member), converted to the option's kind. Raises ValueError for an "
        "unknown key or a value that does not convert to that kind.");
    configuration.def("get",
        &Configuration::get,
        py::arg("key"),
        "Returns the value of `key` (bool, int, float, or str), or its default "
        "if not set.");
    configuration.def("has",
        &Configuration::has,
        py::arg("key"),
        "True if `key` has been explicitly set.");
    configuration.def("keys",
        &Configuration::keys,
        "Sorted list of all settable option names.");
    configuration.def("defaults",
        &Configuration::defaults,
        "Mapping of option name -> default value.");
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
