/**
(C) Copyright 2025-26 Murilo Marinho (murilomarinho@ieee.org)

pybind11 bindings for marinholab::solvers::qpoases::Solver.
*/

#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include <qpOASES_solver.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;
using namespace marinholab::solvers::qpoases;

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
        "default (see `Configuration.use_hotstart`).");

    py::enum_<BooleanType>(qpoases_solver, "BooleanType",
        "qpOASES logical values.")
    .value("BT_FALSE", BooleanType::BT_FALSE)
    .value("BT_TRUE", BooleanType::BT_TRUE)
    .export_values();

    py::enum_<HessianType>(qpoases_solver, "HessianType",
        "qpOASES Hessian definiteness types.")
    .value("HST_ZERO", HessianType::HST_ZERO)
    .value("HST_IDENTITY", HessianType::HST_IDENTITY)
    .value("HST_POSDEF", HessianType::HST_POSDEF)
    .value("HST_POSDEF_NULLSPACE", HessianType::HST_POSDEF_NULLSPACE)
    .value("HST_SEMIDEF", HessianType::HST_SEMIDEF)
    .value("HST_INDEF", HessianType::HST_INDEF)
    .value("HST_UNKNOWN", HessianType::HST_UNKNOWN)
    .export_values();

    py::enum_<PrintLevel>(qpoases_solver, "PrintLevel",
        "qpOASES print levels, describing the desired amount of output at runtime.")
    .value("PL_DEBUG_ITER", PrintLevel::PL_DEBUG_ITER)
    .value("PL_TABULAR", PrintLevel::PL_TABULAR)
    .value("PL_NONE", PrintLevel::PL_NONE)
    .value("PL_LOW", PrintLevel::PL_LOW)
    .value("PL_MEDIUM", PrintLevel::PL_MEDIUM)
    .value("PL_HIGH", PrintLevel::PL_HIGH)
    .export_values();

    py::enum_<SubjectToStatus>(qpoases_solver, "SubjectToStatus",
        "qpOASES bound/constraint statuses.")
    .value("ST_LOWER", SubjectToStatus::ST_LOWER)
    .value("ST_INACTIVE", SubjectToStatus::ST_INACTIVE)
    .value("ST_UPPER", SubjectToStatus::ST_UPPER)
    .value("ST_INFEASIBLE_LOWER", SubjectToStatus::ST_INFEASIBLE_LOWER)
    .value("ST_INFEASIBLE_UPPER", SubjectToStatus::ST_INFEASIBLE_UPPER)
    .value("ST_UNDEFINED", SubjectToStatus::ST_UNDEFINED)
    .export_values();

    py::class_<Configuration> qpoases_configuration(qpoases_solver, "Configuration",
        "All user-configurable solver options.\n\n"
        "Members are the 1:1 mapping of qpOASES' `Options` fields (plus the\n"
        "wrapper-specific `maximum_working_set_recalculations`, `use_hotstart`\n"
        "and `hessian_type`). See the qpOASES manual for a full description\n"
        "of each option: https://www.coin-or.org/qpOASES/doc/3.0/manual.pdf");

    qpoases_configuration.def(py::init<>());
    // Wrapper-specific options
    qpoases_configuration.def_readwrite("maximum_working_set_recalculations", &Configuration::maximum_working_set_recalculations, "Maximum number of working set recalculations during the initial homotopy.");
    qpoases_configuration.def_readwrite("use_hotstart", &Configuration::use_hotstart, "Whether subsequent solves are warm-started instead of re-initialised.");
    // qpOASES `Options` fields
    qpoases_configuration.def_readwrite("printLevel", &Configuration::printLevel, "qpOASES print level (default PL_NONE, the least verbose).");
    qpoases_configuration.def_readwrite("enableRamping", &Configuration::enableRamping, "Enables the ramping strategy.");
    qpoases_configuration.def_readwrite("enableFarBounds", &Configuration::enableFarBounds, "Enables the far bounds strategy.");
    qpoases_configuration.def_readwrite("enableFlippingBounds", &Configuration::enableFlippingBounds, "Enables flipping of active bounds between lower and upper values.");
    qpoases_configuration.def_readwrite("enableRegularisation", &Configuration::enableRegularisation, "Regularises the Hessian in case (semi-)definiteness is detected.");
    qpoases_configuration.def_readwrite("enableFullLITests", &Configuration::enableFullLITests, "Uses the condition-hardened linear independence test.");
    qpoases_configuration.def_readwrite("enableNZCTests", &Configuration::enableNZCTests, "Enables the nonzero curvature test.");
    qpoases_configuration.def_readwrite("enableDriftCorrection", &Configuration::enableDriftCorrection, "Frequency of drift corrections (0 = off).");
    qpoases_configuration.def_readwrite("enableCholeskyRefactorisation", &Configuration::enableCholeskyRefactorisation, "Frequency of full Cholesky refactorisation of the projected Hessian (0 = updates only).");
    qpoases_configuration.def_readwrite("enableEqualities", &Configuration::enableEqualities, "Treats equality constraints as always active.");
    qpoases_configuration.def_readwrite("terminationTolerance", &Configuration::terminationTolerance, "Relative termination tolerance to stop the homotopy.");
    qpoases_configuration.def_readwrite("boundTolerance", &Configuration::boundTolerance, "Lower/upper (constraints') bound tolerance.");
    qpoases_configuration.def_readwrite("boundRelaxation", &Configuration::boundRelaxation, "Offset for relaxing constraint bounds at the start of an initial homotopy.");
    qpoases_configuration.def_readwrite("epsNum", &Configuration::epsNum, "Numerator tolerance for the ratio test.");
    qpoases_configuration.def_readwrite("epsDen", &Configuration::epsDen, "Denominator tolerance for the ratio test.");
    qpoases_configuration.def_readwrite("maxPrimalJump", &Configuration::maxPrimalJump, "Maximum allowed jump in primal variables during nonzero curvature tests.");
    qpoases_configuration.def_readwrite("maxDualJump", &Configuration::maxDualJump, "Maximum allowed jump in dual variables during linear independence tests.");
    qpoases_configuration.def_readwrite("initialRamping", &Configuration::initialRamping, "Start value of the ramping strategy.");
    qpoases_configuration.def_readwrite("finalRamping", &Configuration::finalRamping, "Final value of the ramping strategy.");
    qpoases_configuration.def_readwrite("initialFarBounds", &Configuration::initialFarBounds, "Initial size of the far bounds.");
    qpoases_configuration.def_readwrite("growFarBounds", &Configuration::growFarBounds, "Growth factor applied to the far bounds.");
    qpoases_configuration.def_readwrite("initialStatusBounds", &Configuration::initialStatusBounds, "Status assumed for all bounds at the first iteration.");
    qpoases_configuration.def_readwrite("epsFlipping", &Configuration::epsFlipping, "Tolerance of the squared Cholesky diagonal factor which triggers flipping a bound.");
    qpoases_configuration.def_readwrite("numRegularisationSteps", &Configuration::numRegularisationSteps, "Maximum number of successive regularisation steps.");
    qpoases_configuration.def_readwrite("epsRegularisation", &Configuration::epsRegularisation, "Scaling factor of the identity matrix used for Hessian regularisation.");
    qpoases_configuration.def_readwrite("numRefinementSteps", &Configuration::numRefinementSteps, "Maximum number of iterative refinement steps.");
    qpoases_configuration.def_readwrite("epsIterRef", &Configuration::epsIterRef, "Early termination tolerance for iterative refinement.");
    qpoases_configuration.def_readwrite("epsLITests", &Configuration::epsLITests, "Tolerance used by the linear independence tests.");
    qpoases_configuration.def_readwrite("epsNZCTests", &Configuration::epsNZCTests, "Tolerance used by the nonzero curvature tests.");
    qpoases_configuration.def_readwrite("rcondSMin", &Configuration::rcondSMin, "Minimum reciprocal condition number of the Schur complement before refactorisation is triggered.");
    qpoases_configuration.def_readwrite("enableInertiaCorrection", &Configuration::enableInertiaCorrection, "Repairs the working set when negative curvature is discovered during a hotstart.");
    qpoases_configuration.def_readwrite("enableDropInfeasibles", &Configuration::enableDropInfeasibles, "Whether infeasible constraints may be dropped.");
    qpoases_configuration.def_readwrite("dropBoundPriority", &Configuration::dropBoundPriority, "Priority used when dropping bounds.");
    qpoases_configuration.def_readwrite("dropEqConPriority", &Configuration::dropEqConPriority, "Priority used when dropping equality constraints.");
    qpoases_configuration.def_readwrite("dropIneqConPriority", &Configuration::dropIneqConPriority, "Priority used when dropping inequality constraints.");
    qpoases_configuration.def_readwrite("hessian_type", &Configuration::hessian_type, "Definiteness assumed for the Hessian matrix.");

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
