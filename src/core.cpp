/**
(C) Copyright 2025-26 Murilo Marinho (murilomarinho@ieee.org)
*/

#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <qpOASES_solver.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {

    py::class_<qpOASES_Solver> qpoases_solver(m, "qpOASES_Solver");

    qpoases_solver.def(py::init<>());
    qpoases_solver.def("solve_quadratic_program",&qpOASES_Solver::solve_quadratic_program,".");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
