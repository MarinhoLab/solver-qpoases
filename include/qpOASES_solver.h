#pragma once

#include <vector>
#include <Eigen/Dense>
using namespace Eigen;

#include <qpOASES.hpp>
USING_NAMESPACE_QPOASES

namespace M3
{

class qpOASES_Solver
{
    public:
        struct Configuration
        {
            //The integer argument nWSR specifies the maximum number of working set recalculations to be performed during the initial homotopy (on output it contains the number
            //of working set recalculations actually performed!)
            //Page 14 of https://www.coin-or.org/qpOASES/doc/3.0/manual.pdf
            int_t maximum_working_set_recalculations = 150;
            bool use_hotstart = true; //Use hotstart for subsequent calls.
            HessianType hessian_type = HST_POSDEF; //Hessian definiteness. Page 22.
            BooleanType enableRegularisation = BT_TRUE; //Regularisation. Page 26.
            BooleanType enableNZCTests = BT_FALSE; //Nonzero curvature test. Page 22.
            BooleanType enableFlippingBounds = BT_FALSE; //Flipping bounds. Page 22.
            real_t termination_tolerance = 5.0e6 * EPS; //Relative termination tolerance to stop homotopy.
            Configuration(); //https://stackoverflow.com/questions/53408962/try-to-understand-compiler-error-message-default-member-initializer-required-be
        };
    protected:
        bool qpoases_solve_first_time_;
        SQProblem qpoases_problem_;
        Configuration configuration_;

        //https://github.com/SmartArmStack/sas_conversions/blob/master/src/eigen3_std_conversions.cpp
        //A copy from sas
        std::vector<double> _vectorxd_to_std_vector_double(const VectorXd& vectorxd);

        //Another copy from sas
        VectorXd _std_vector_double_to_vectorxd(std::vector<double> std_vector_double);


    public:
        qpOASES_Solver(const Configuration& configuration = qpOASES_Solver::Configuration());
        ~qpOASES_Solver()=default;

        /**
         * @brief
         *   Solves the following quadratic program
         *   min(x)  0.5*x'Hx + f'x
         *   s.t.    Ax <= b
         *           Aeqx = beq.
         * Method signature is compatible with MATLAB's 'quadprog'.
         * @param H the n x n matrix of the quadratic coefficients of the decision variables.
         * @param f the n x 1 vector of the linear coefficients of the decision variables.
         * @param A the m x n matrix of inequality constraints.
         * @param b the m x 1 value for the inequality constraints.
         * @param Aeq the m x n matrix of equality constraints.
         * @param beq the m x 1 value for the inequality constraints.
         * @return the optimal x
         */
        VectorXd solve_quadratic_program(const MatrixXd& H, const VectorXd& f, const MatrixXd& A, const VectorXd& b, const MatrixXd& Aeq, const VectorXd& beq);

        /**
         * @brief
         *   Returns the active set of constraints obtained in the most recent call to
         *   solve_quadratic_program(). The returned vector has one entry for each row of
         *   the combined constraint matrix, i.e. the rows of A followed by the rows of Aeq,
         *   in that same order, with the following meaning for each entry:
         *     -1: the constraint is active at its lower bound;
         *      0: the constraint is inactive;
         *     +1: the constraint is active at its upper bound (this is also the value used
         *         for active equality constraints, as their lower and upper bounds coincide).
         * @return the active set, as described above.
         */
        VectorXd get_active_set();

        VectorXd test_vectorxd(const VectorXd& v);
        MatrixXd test_matrixxd(const MatrixXd& m);

};

}