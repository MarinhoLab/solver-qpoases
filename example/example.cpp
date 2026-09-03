/**
 * Example usage of the C++ API of `marinholab-solvers-qpoases`.
 *
 * Builds a small quadratic program and solves it with
 * `marinholab::solvers::qpoases::Solver`, mirroring the Python quickstart in
 * the README. Build and run it with the `BUILD_EXAMPLES` option (OFF by
 * default so the normal `pip install .` build is unaffected):
 *
 *     cmake -B build -GNinja -DBUILD_EXAMPLES=ON
 *     cmake --build build
 *     ./build/example/example_qpoases
 */
#include <iostream>
#include <string>

#include <marinholab/solvers/qpoases.h>

namespace qpoases = marinholab::solvers::qpoases;

namespace
{
    void print_vector(const std::string& label, const Eigen::VectorXd& v)
    {
        std::cout << label << " = [";
        for (Eigen::Index i = 0; i < v.size(); ++i)
        {
            if (i != 0)
                std::cout << ' ';
            std::cout << v[i];
        }
        std::cout << "]\n";
    }
} // namespace

int main()
{
    // 1. Configure the solver. Only a couple of fields are set here; the rest
    //    keep their defaults (which mirror qpOASES' own defaults for a
    //    double-precision build).
    qpoases::Configuration config;
    config.terminationTolerance = 1.0e-9;  // tighter convergence

    qpoases::Solver solver(config);

    // 2. The problem:
    //
    //      min_x  0.5 * x' H x + f' x
    //      s.t.   A x <= b
    //             Aeq x = beq
    //
    //    H = I, f = [-1, -1], x0 <= 0.2, plus one trivially-satisfied equality.
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd f(2);
    f << -1.0, -1.0;

    Eigen::MatrixXd A(1, 2);
    A << 1.0, 0.0;
    Eigen::VectorXd b(1);
    b << 0.2;

    Eigen::MatrixXd Aeq = Eigen::MatrixXd::Zero(1, 2);
    Eigen::VectorXd beq = Eigen::VectorXd::Zero(1);

    // 3. Solve. The Solver keeps the underlying qpOASES problem, so repeated
    //    calls on the same instance are warm-started by default
    //    (`Configuration.use_hotstart = true`).
    Eigen::VectorXd x = solver.solve_quadratic_program(H, f, A, b, Aeq, beq);

    // 4. Inspect the result and the active set.
    print_vector("x", x);
    print_vector("active set", solver.get_active_set());

    return 0;
}
