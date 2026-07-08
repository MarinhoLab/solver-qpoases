/**
(C) Copyright 2025-26 Murilo Marinho (murilomarinho@ieee.org)
*/

#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <qpOASES_solver.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;
using namespace M3;

PYBIND11_MODULE(_core, m) {

    py::class_<qpOASES_Solver> qpoases_solver(m, "qpOASES_Solver");

    py::enum_<BooleanType>(qpoases_solver, "BooleanType")
    .value("BT_FALSE", BooleanType::BT_FALSE)
    .value("BT_TRUE", BooleanType::BT_TRUE)
    .export_values();

    py::enum_<HessianType>(qpoases_solver, "HessianType")
    .value("HST_POSDEF", HessianType::HST_POSDEF)
    .value("HST_SEMIDEF", HessianType::HST_SEMIDEF)
    .value("HST_IDENTITY", HessianType::HST_IDENTITY)
    .export_values();

    py::class_<qpOASES_Solver::Configuration> qpoases_configuration(qpoases_solver, "Configuration");
    qpoases_configuration.def(py::init<>());
    qpoases_configuration.def_readwrite("maximum_working_set_recalculations", &qpOASES_Solver::Configuration::maximum_working_set_recalculations);
    qpoases_configuration.def_readwrite("use_hotstart", &qpOASES_Solver::Configuration::use_hotstart);
    qpoases_configuration.def_readwrite("hessian_type", &qpOASES_Solver::Configuration::hessian_type);
    qpoases_configuration.def_readwrite("enableRegularisation", &qpOASES_Solver::Configuration::enableRegularisation);
    qpoases_configuration.def_readwrite("enableNZCTests", &qpOASES_Solver::Configuration::enableNZCTests);
    qpoases_configuration.def_readwrite("enableFlippingBounds", &qpOASES_Solver::Configuration::enableFlippingBounds);
    qpoases_configuration.def_readwrite("termination_tolerance", &qpOASES_Solver::Configuration::termination_tolerance);

    qpoases_solver.def(py::init<const qpOASES_Solver::Configuration&>(),
                       py::arg("configuration") = qpOASES_Solver::Configuration());
    qpoases_solver.def("solve_quadratic_program",&qpOASES_Solver::solve_quadratic_program,".");

    // Helps evaluating the wrapper when versions show any issues
    qpoases_solver.def("test_vectorxd",&qpOASES_Solver::test_vectorxd,".");
    qpoases_solver.def("test_matrixxd",&qpOASES_Solver::test_matrixxd,".");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
