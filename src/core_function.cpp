/**
Based on https://github.com/dqrobotics/cpp-interface-qpoases
Originally by Murilo M. Marinho
*/
#include <qpOASES_solver.h>

qpOASES_Solver::qpOASES_Solver():
    qpoases_solve_first_time_(true),
    maximum_working_set_recalculations_(500)
{

}

std::vector<double> qpOASES_Solver::_vectorxd_to_std_vector_double(const VectorXd& vectorxd)
{
    std::vector<double> vec(vectorxd.data(), vectorxd.data() + vectorxd.rows() * vectorxd.cols());
    return vec;
}

VectorXd qpOASES_Solver::_std_vector_double_to_vectorxd(std::vector<double> std_vector_double)
{
    double* ptr = &std_vector_double[0];
    Eigen::Map<Eigen::VectorXd> vec(ptr,std_vector_double.size());
    return vec;
}

void qpOASES_Solver::set_maximum_working_set_recalculations(const int& maximum_working_set_recalculations)
{
    maximum_working_set_recalculations_ = maximum_working_set_recalculations;
}

void qpOASES_Solver::set_equality_constraints_tolerance(const double &equality_constraints_tolerance) {
    equality_constraints_tolerance_ = equality_constraints_tolerance;
}

double qpOASES_Solver::get_equality_constraints_tolerance() {
    return equality_constraints_tolerance_;
}

VectorXd qpOASES_Solver::solve_quadratic_program(const MatrixXd& H, const VectorXd& f, const MatrixXd& A, const VectorXd& b, const MatrixXd& Aeq, const VectorXd& beq)
{
    const int PROBLEM_SIZE = H.rows();
    const int INEQUALITY_CONSTRAINT_SIZE = b.size();
    const int EQUALITY_CONSTRAINT_SIZE = beq.size();

    ///Check sizes
    //Objective function
    if(H.rows()!=H.cols())
        throw std::runtime_error("qpOASES_Solver::solve_quadratic_program(): H must be symmetric. H.rows()="+std::to_string(H.rows())+" but H.cols()="+std::to_string(H.cols())+".");
    if(f.size()!=H.rows())
        throw std::runtime_error("qpOASES_Solver::solve_quadratic_program(): f must be compatible with H. H.rows()=H.cols()="+std::to_string(H.rows())+" but f.size()="+std::to_string(f.size())+".");

    //Inequality constraints
    if(b.size()!=A.rows())
        throw std::runtime_error("qpOASES_Solver::solve_quadratic_program(): size of b="+std::to_string(b.size())+" should be compatible with rows of A="+std::to_string(A.rows())+".");

    //Equality constraints
    if(beq.size()!=Aeq.rows())
        throw std::runtime_error("qpOASES_Solver::solve_quadratic_program(): size of beq="+std::to_string(beq.size())+" should be compatible with rows of Aeq="+std::to_string(Aeq.rows())+".");

    //Append equality constraints to inequality constraints
    MatrixXd A_extended = A;
    VectorXd ub_extended = b;
    VectorXd lb_extended;
    if(EQUALITY_CONSTRAINT_SIZE!=0 && INEQUALITY_CONSTRAINT_SIZE!=0)
    {
        A_extended.resize(INEQUALITY_CONSTRAINT_SIZE + EQUALITY_CONSTRAINT_SIZE, PROBLEM_SIZE);
        A_extended << A, Aeq;
        ub_extended.resize(INEQUALITY_CONSTRAINT_SIZE + EQUALITY_CONSTRAINT_SIZE);
        ub_extended << b, beq;
        lb_extended.resize(INEQUALITY_CONSTRAINT_SIZE + EQUALITY_CONSTRAINT_SIZE);
        lb_extended << -VectorXd::Ones(b.size()) * INFTY, beq;
    } else if(EQUALITY_CONSTRAINT_SIZE!=0)
    {
        A_extended.resize(EQUALITY_CONSTRAINT_SIZE, PROBLEM_SIZE);
        A_extended << Aeq;
        ub_extended.resize(EQUALITY_CONSTRAINT_SIZE);
        ub_extended << beq;
        lb_extended << beq;
    }

    std::vector<double> H_std_vec(H.data(), H.data() + H.rows() * H.cols());
    real_t* H_vec = &H_std_vec[0];

    auto g_std_vec = _vectorxd_to_std_vector_double(f);
    real_t* g_vec = &g_std_vec[0];

    real_t* A_vec = nullptr;   // Default for unconstrained cases
    real_t* ubA_vec = nullptr; // Default for unconstrained cases
    real_t* lbA_vec = nullptr;

    std::vector<double> A_std_vec;
    std::vector<double> ub_std_vec;
    std::vector<double> lb_std_vec;
    if (EQUALITY_CONSTRAINT_SIZE + INEQUALITY_CONSTRAINT_SIZE > 0)
    {
        // For constrained cases, we update A_vec and ubA_vec accordingly
        MatrixXd AT = A_extended.transpose();
        A_std_vec = std::vector<double>(AT.data(), AT.data() + AT.rows() * AT.cols());
        A_vec = &A_std_vec[0];

        ub_std_vec = _vectorxd_to_std_vector_double(ub_extended);
        ubA_vec = &ub_std_vec[0];

        if(lb_extended.size() > 0)
        {
            lb_std_vec = _vectorxd_to_std_vector_double(lb_extended);
            lbA_vec = &lb_std_vec[0];
        }
    }

    if(qpoases_solve_first_time_)
    {
        qpoases_problem_ = SQProblem(PROBLEM_SIZE, INEQUALITY_CONSTRAINT_SIZE + EQUALITY_CONSTRAINT_SIZE, HST_POSDEF);
        Options options;
        options.printLevel = qpOASES::PrintLevel::PL_NONE;
        qpoases_problem_.setOptions( options );
        auto maximum_working_set_recalculations_local = maximum_working_set_recalculations_; //qpOASES changes the value, so we make a local copy
        auto problem_init_return = qpoases_problem_.init(H_vec,g_vec,A_vec,NULL,NULL,lbA_vec,ubA_vec,maximum_working_set_recalculations_local);
        if(problem_init_return != SUCCESSFUL_RETURN)
            throw std::runtime_error("qpOASES_Solver::solve_quadratic_program(): Unable to solve quadratic program.");
        qpoases_solve_first_time_ = false;
    }
    else
    {
        auto maximum_working_set_recalculations_local = maximum_working_set_recalculations_; //qpOASES changes the value, so we make a local copy
        auto problem_init_return = qpoases_problem_.hotstart(H_vec,g_vec,A_vec,NULL,NULL,lbA_vec,ubA_vec,maximum_working_set_recalculations_local);
        if(problem_init_return != SUCCESSFUL_RETURN)
            throw std::runtime_error("qpOASES_Solver::solve_quadratic_program(): Unable to solve quadratic program.");
    }

    real_t xOpt[PROBLEM_SIZE];
    qpoases_problem_.getPrimalSolution( xOpt );

    std::vector<double> return_value_std(xOpt, xOpt + PROBLEM_SIZE);

    return _std_vector_double_to_vectorxd(return_value_std);
}

// Helper functions to help evaluate the wrapper when needed.
VectorXd qpOASES_Solver::test_vectorxd(const VectorXd& v)
{
    return v;
}

MatrixXd qpOASES_Solver::test_matrixxd(const MatrixXd& m)
{
    return m;
}
