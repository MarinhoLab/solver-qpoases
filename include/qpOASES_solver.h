#pragma once

#include <vector>
#include <Eigen/Dense>
using namespace Eigen;

#include <qpOASES.hpp>
USING_NAMESPACE_QPOASES

class qpOASES_Solver
{
    protected:
        bool qpoases_solve_first_time_;
        SQProblem qpoases_problem_;
        //The integer argument nWSR specifies the maximum number of working set recalculations to be performed during the initial homotopy (on output it contains the number
        //of working set recalculations actually performed!)
        //Page 14 of https://www.coin-or.org/qpOASES/doc/3.0/manual.pdf
        int_t maximum_working_set_recalculations_;

        // Equality constraints are handled by means of appended inequality constraints within an equality tolerance threshold.
        double_t equality_constraints_tolerance_ = 1e-12;

        //Overload this method in a child class to change the configuration.
        void _config_solver();

        //https://github.com/SmartArmStack/sas_conversions/blob/master/src/eigen3_std_conversions.cpp
        //A copy from sas
        std::vector<double> _vectorxd_to_std_vector_double(const VectorXd& vectorxd);

        //Another copy from sas
        VectorXd _std_vector_double_to_vectorxd(std::vector<double> std_vector_double);


    public:
        qpOASES_Solver();
        ~qpOASES_Solver()=default;

        void set_maximum_working_set_recalculations(const int& maximum_working_set_recalculations);

        /**
         * @brief
         *  Sets the tolerance for the equality constraints.
         * @param equality_constraints_tolerance the tolerance for the equality constraints.
         */
        void set_equality_constraints_tolerance(const double &equality_constraints_tolerance);

        /**
         * @brief
         *  Gets the tolerance for the equality constraints.
         * @return the tolerance for the equality constraints.
         */
        double get_equality_constraints_tolerance();

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

        VectorXd test_vectorxd(const VectorXd& v);
        MatrixXd test_matrixxd(const MatrixXd& m);

};