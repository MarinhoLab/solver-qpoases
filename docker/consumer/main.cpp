/**
 * Downstream CMake project that links the *installed*
 * `marinholab::solvers::qpoases` package, i.e. what a user gets from
 * `apt-get install libmarinholab-solver-qpoases` (the PPA) or from
 * `cmake --install`.
 *
 * It uses nothing but `find_package()` and the imported target -- no paths
 * into the source tree -- so it is the regression test for the packaging and
 * for `marinholab_solver_qpoases-config.cmake`. Run it with
 * `docker/test-ppa.sh` (package from the repository) or `docker/build-deb.sh
 * --test` (locally built package).
 */
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <marinholab/solvers/qpoases.h>

namespace qpoases = marinholab::solvers::qpoases;

namespace
{

int failures = 0;

void check(const std::string& what, const Eigen::VectorXd& got, const std::vector<double>& expected, const double tolerance = 1e-6)
{
    bool ok = (got.size() == static_cast<Eigen::Index>(expected.size()));
    for (Eigen::Index i = 0; ok && i < got.size(); ++i)
    {
        ok = std::abs(got[i] - expected[i]) < tolerance;
    }

    const Eigen::Map<const Eigen::VectorXd> want(expected.data(), static_cast<Eigen::Index>(expected.size()));
    std::cout << (ok ? "ok   " : "FAIL ") << what << ": got [" << got.transpose()
              << "], expected [" << want.transpose() << "]\n";
    if (!ok)
    {
        ++failures;
    }
}

void check_true(const std::string& what, const bool ok)
{
    std::cout << (ok ? "ok   " : "FAIL ") << what << "\n";
    if (!ok)
    {
        ++failures;
    }
}

/** min 0.5*x'x - [1,1]'x  s.t. x0 <= 0.2 (plus an inactive equality) -> [0.2, 1]. */
void inequality_constraint()
{
    qpoases::Configuration config;
    config.set("terminationTolerance", 1.0e-9);
    qpoases::Solver solver(config);

    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd f(2);
    f << -1.0, -1.0;
    Eigen::MatrixXd A(1, 2);
    A << 1.0, 0.0;
    Eigen::VectorXd b(1);
    b << 0.2;
    Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(1, 2);
    Eigen::VectorXd beq = Eigen::VectorXd::Zero(1);

    check("inequality-constrained x", solver.solve_quadratic_program(H, f, A, b, Aeq, beq), {0.2, 1.0});
    // Row 0 (x0 <= 0.2) is active at its upper bound, the trivial equality is not.
    check("active set", solver.get_active_set(), {1.0, 0.0});
}

/**
 * min 0.5*x'x - [1,1]'x with no real constraints. The C++ API takes matrices
 * rather than `None`, so a single trivially-satisfied zero row stands for "no
 * such constraint" -- the same substitution the Python wrapper makes.
 */
void unconstrained()
{
    qpoases::Solver solver;

    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd f(2);
    f << -1.0, -1.0;
    Eigen::MatrixXd none = Eigen::MatrixXd::Zero(1, 2);
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(1);

    check("unconstrained x", solver.solve_quadratic_program(H, f, none, zero, none, zero), {1.0, 1.0});
}

/** min 0.5*x'x - [1,1]'x  s.t. x0 + x1 = 1  ->  [0.5, 0.5]. */
void equality_constraint()
{
    qpoases::Solver solver;

    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd f(2);
    f << -1.0, -1.0;
    Eigen::MatrixXd Aeq(1, 2);
    Aeq << 1.0, 1.0;
    Eigen::VectorXd beq(1);
    beq << 1.0;
    Eigen::MatrixXd none = Eigen::MatrixXd::Zero(1, 2);
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(1);

    check("equality-constrained x", solver.solve_quadratic_program(H, f, none, zero, Aeq, beq), {0.5, 0.5});
}

/**
 * Rank-deficient Hessian: min 0.5*x1^2 - 2*x1 + 0.5*x2 s.t. -1 <= x <= 1.
 * Needs the `hessian_type` option set to HST_SEMIDEF -> [1, -1].
 */
void semidefinite()
{
    qpoases::Configuration config;
    config.set("hessian_type", "HST_SEMIDEF");
    qpoases::Solver solver(config);

    Eigen::MatrixXd H(2, 2);
    H << 1.0, 0.0, 0.0, 0.0;
    Eigen::VectorXd f(2);
    f << -2.0, 0.5;
    Eigen::MatrixXd A(4, 2);
    A << -Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd b = Eigen::VectorXd::Ones(4);
    Eigen::MatrixXd none = Eigen::MatrixXd::Zero(1, 2);
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(1);

    check("semi-definite x", solver.solve_quadratic_program(H, f, A, b, none, zero), {1.0, -1.0});
}

/** Repeated solves on one instance must stay correct (the hotstart path). */
void warm_start()
{
    qpoases::Solver solver;

    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd f(2);
    f << -1.0, -1.0;
    Eigen::MatrixXd none = Eigen::MatrixXd::Zero(1, 2);
    Eigen::VectorXd zero = Eigen::VectorXd::Zero(1);

    for (int i = 0; i < 5; ++i)
    {
        Eigen::MatrixXd A(1, 2);
        A << 1.0, 0.0;
        Eigen::VectorXd b(1);
        b << 0.1 * i;
        check("warm-started solve " + std::to_string(i), solver.solve_quadratic_program(H, f, A, b, none, zero), {0.1 * i, 1.0});
    }
}

/** The option accessors work through the installed package. */
void configuration()
{
    qpoases::Configuration config;
    config.set("terminationTolerance", 1.0e-9);

    check_true("Configuration accessors",
               config.has("terminationTolerance") && !config.has("boundTolerance")
                   && std::get<double>(config.get("terminationTolerance")) == 1.0e-9
                   && !config.keys().empty() && !config.defaults().empty());
}

/**
 * The real-valued defaults must reach the package as doubles. Serialising the
 * `Configuration` through text once turned every qpOASES tolerance into 0
 * (`std::to_string` truncates), and qpOASES then failed on well-posed QPs.
 */
void tolerance_defaults()
{
    const auto defaults = qpoases::Configuration().defaults();

    for (const char* key : {"terminationTolerance", "boundTolerance", "epsNum", "epsLITests", "epsRegularisation"})
    {
        const auto it = defaults.find(key);
        const double value = it == defaults.end() ? 0.0 : std::get<double>(it->second);
        // epsNum is negative by design, so only its magnitude is checked.
        std::cout << (std::abs(value) > 0.0 ? "ok   " : "FAIL ") << "default " << key << " = " << value << " (must be non-zero)\n";
        if (!(std::abs(value) > 0.0))
        {
            ++failures;
        }
    }
}

}  // namespace

int main()
{
    inequality_constraint();
    unconstrained();
    equality_constraint();
    semidefinite();
    warm_start();
    configuration();
    tolerance_defaults();

    if (failures != 0)
    {
        std::cout << failures << " check(s) FAILED\n";
        return 1;
    }
    std::cout << "All checks passed.\n";
    return 0;
}
