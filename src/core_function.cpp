/**
Based on https://github.com/dqrobotics/cpp-interface-qpoases
Originally by Murilo M. Marinho
*/
#include <qpOASES_solver.h>
//#include <print>

namespace M3
{

// https://stackoverflow.com/questions/53408962/try-to-understand-compiler-error-message-default-member-initializer-required-be
qpOASES_Solver::Configuration::Configuration() = default;

qpOASES_Solver::qpOASES_Solver(const Configuration& configuration):
    qpoases_solve_first_time_(true),
    configuration_(configuration)
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

void evaluate_problem_return_value(returnValue problem_return_value)
{
    if(problem_return_value != SUCCESSFUL_RETURN)
    {
        if(problem_return_value == RET_MAX_NWSR_REACHED)
            throw std::runtime_error("qpOASES_Solver::solve_quadratic_program(): Maximum number of working set recalculations reached. Consider increasing the 'maximum_working_set_recalculations' parameter in the configuration.");
        else if( problem_return_value == RET_INIT_FAILED)
            throw std::runtime_error("qpOASES_Solver::solve_quadratic_program(): Initialization failed. Check if the problem is well defined and if the parameters are valid.");
        else
        {
            throw std::runtime_error("qpOASES_Solver::solve_quadratic_program(): Unable to solve quadratic program. qpOASES returned error code: " + std::to_string(problem_return_value) + std::string(" ") + std::to_string(getSimpleStatus(problem_return_value)));
        }
    }
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
        qpoases_problem_ = SQProblem(PROBLEM_SIZE, INEQUALITY_CONSTRAINT_SIZE + EQUALITY_CONSTRAINT_SIZE, configuration_.hessian_type);
        Options options;
        options.printLevel = qpOASES::PrintLevel::PL_LOW;
        options.enableNZCTests = configuration_.enableNZCTests; //Nonzero curvature test
        options.enableFlippingBounds = configuration_.enableFlippingBounds; //Flipping bounds
        options.terminationTolerance = configuration_.termination_tolerance; //Relative termination tolerance to stop homotopy
        qpoases_problem_.setOptions( options );
        auto maximum_working_set_recalculations_local = configuration_.maximum_working_set_recalculations; //qpOASES changes the value, so we make a local copy
        auto problem_init_return = qpoases_problem_.init(H_vec,g_vec,A_vec,NULL,NULL,lbA_vec,ubA_vec,maximum_working_set_recalculations_local);

        evaluate_problem_return_value(problem_init_return);

        qpoases_solve_first_time_ = false;
    }
    else
    {
        auto maximum_working_set_recalculations_local = configuration_.maximum_working_set_recalculations; //qpOASES changes the value, so we make a local copy


        returnValue problem_return_value;
        if(configuration_.use_hotstart)
            problem_return_value = qpoases_problem_.hotstart(H_vec,g_vec,A_vec,NULL,NULL,lbA_vec,ubA_vec,maximum_working_set_recalculations_local);
        else
            problem_return_value = qpoases_problem_.init(H_vec,g_vec,A_vec,NULL,NULL,lbA_vec,ubA_vec,maximum_working_set_recalculations_local);
        evaluate_problem_return_value(problem_return_value);
    }

    // Use a std::vector instead of a variable-length array so that the code
    // compiles with MSVC (VLAs are a non-standard extension rejected by it).
    std::vector<real_t> xOpt(PROBLEM_SIZE);
    qpoases_problem_.getPrimalSolution( xOpt.data() );

    return _std_vector_double_to_vectorxd(std::vector<double>(xOpt.begin(), xOpt.end()));
}

VectorXd qpOASES_Solver::get_active_set()
{
    if(qpoases_solve_first_time_)
        throw std::runtime_error("qpOASES_Solver::get_active_set(): solve_quadratic_program() must be called at least once before the active set can be retrieved.");

    const int_t NC = qpoases_problem_.getNC();
    if(NC == 0)
        return VectorXd(0);

    std::vector<double> active_set_std(NC, 0.0);
    evaluate_problem_return_value(qpoases_problem_.getWorkingSetConstraints(&active_set_std[0]));

    return _std_vector_double_to_vectorxd(active_set_std);
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

} // namespace M3