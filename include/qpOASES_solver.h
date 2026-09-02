#pragma once

#include <vector>
#include <Eigen/Dense>

#include <qpOASES.hpp>

namespace M3
{
// Keep the using-directives scoped to this namespace rather than global
// scope: a global `using namespace Eigen;` would be active while
// <qpOASES.hpp> is parsed, making `friend class SparseMatrix;` in
// qpOASES/Indexlist.hpp ambiguous with the Eigen::SparseMatrix template
// (MSVC rejects this with error C2990).
using namespace Eigen;
using namespace qpOASES;

/**
 * @brief High-level, reusable solver for quadratic programs (QPs) based on
 *        qpOASES.
 *
 * qpOASES_Solver exposes qpOASES' online active set solver through a
 * MATLAB/`quadprog`-like, matrix-based interface. Internally it keeps an
 * SQProblem object so that, once initialised, subsequent calls are warm
 * started (see use_hotstart).
 *
 * The solver is configured through a Configuration structure whose members
 * map directly onto qpOASES' `Options` class. In addition to the standard
 * qpOASES options, the configuration carries a few wrapper-specific
 * settings (hotstart, maximum working-set recalculations).
 *
 * @note The class is not thread-safe: a single instance owns one
 *       underlying qpOASES problem and its state changes across calls.
 */
class qpOASES_Solver
{
    public:
        /**
         * @brief Holds all user-configurable solver options.
         *
         * Every qpOASES `Options` field is exposed here. Defaults match
         * qpOASES' own defaults for a double-precision build (see
         * `Options::setToDefault()`).
         *
         * @note qpOASES applies `Options::ensureConsistency()` when it sets
         *       its options, which will silently adjust any value that falls
         *       outside its allowed range (e.g. negative tolerances) to a
         *       valid one.
         */
        struct Configuration
        {
            // ------------------------------------------------------------------
            // Wrapper-specific options (not part of qpOASES' `Options`).
            // ------------------------------------------------------------------

            /**
             * @brief Maximum number of working set recalculations performed
             *        during the initial homotopy.
             *
             * This is the nWSR argument passed to `init()`/`hotstart()`.
             * On qpOASES' side it is overwritten with the number of
             * recalculations actually performed. See page 14 of
             * https://www.coin-or.org/qpOASES/doc/3.0/manual.pdf
             */
            int_t maximum_working_set_recalculations = 150;

            /**
             * @brief Whether subsequent solves warm-start the problem
             *        (`hotstart()`) instead of re-initialising it
             *        (`init()`).
             */
            bool use_hotstart = true;

            // ------------------------------------------------------------------
            // qpOASES `Options` fields (1:1 mapping, snake_case names).
            // ------------------------------------------------------------------

            /**
             * @brief Verbose-ness of qpOASES output.
             * @see `Options::printLevel`
             */
            PrintLevel print_level = PL_MEDIUM;

            /**
             * @brief Whether the ramping strategy shall be used.
             * @see `Options::enableRamping`
             */
            BooleanType enable_ramping = BT_TRUE;

            /**
             * @brief Whether far bounds shall be used.
             * @see `Options::enableFarBounds`
             */
            BooleanType enable_far_bounds = BT_TRUE;

            /**
             * @brief Whether active bounds may flip between lower and upper
             *        values.
             * @see `Options::enableFlippingBounds`. Page 22 of the manual.
             */
            BooleanType enable_flipping_bounds = BT_TRUE;

            /**
             * @brief Whether the Hessian shall be regularised in case
             *        (semi-)definiteness is detected.
             * @see `Options::enableRegularisation`. Page 26 of the manual.
             */
            BooleanType enable_regularisation = BT_FALSE;

            /**
             * @brief Whether the condition-hardened linear independence
             *        (LI) test shall be used.
             * @see `Options::enableFullLITests`
             */
            BooleanType enable_full_li_tests = BT_FALSE;

            /**
             * @brief Whether nonzero curvature tests shall be used.
             * @see `Options::enableNZCTests`. Page 22 of the manual.
             */
            BooleanType enable_nzc_tests = BT_TRUE;

            /**
             * @brief Frequency of drift corrections (0 = off).
             * @see `Options::enableDriftCorrection`
             */
            int_t enable_drift_correction = 1;

            /**
             * @brief Frequency of full Cholesky refactorisation of the
             *        projected Hessian (0 = use rank updates only).
             * @see `Options::enableCholeskyRefactorisation`
             */
            int_t enable_cholesky_refactorisation = 0;

            /**
             * @brief Whether equality constraints shall always be treated
             *        as active.
             * @see `Options::enableEqualities`
             */
            BooleanType enable_equalities = BT_FALSE;

            /**
             * @brief Relative termination tolerance to stop the homotopy.
             * @see `Options::terminationTolerance`
             */
            real_t termination_tolerance = 5.0e6 * EPS;

            /**
             * @brief Lower/upper (constraints') bound tolerance; a
             *        constraint whose bounds differ by less is regarded as
             *        an equality constraint.
             * @see `Options::boundTolerance`
             */
            real_t bound_tolerance = 1.0e6 * EPS;

            /**
             * @brief Offset for relaxing constraint bounds at the start of
             *        an initial homotopy; also used as the initial far-bound
             *        value.
             * @see `Options::boundRelaxation`
             */
            real_t bound_relaxation = 1.0e4;

            /**
             * @brief Numerator tolerance for the ratio test.
             * @see `Options::epsNum`
             */
            real_t eps_num = -1.0e3 * EPS;

            /**
             * @brief Denominator tolerance for the ratio test.
             * @see `Options::epsDen`
             */
            real_t eps_den = 1.0e3 * EPS;

            /**
             * @brief Maximum allowed jump in primal variables during nonzero
             *        curvature tests.
             * @see `Options::maxPrimalJump`
             */
            real_t max_primal_jump = 1.0e8;

            /**
             * @brief Maximum allowed jump in dual variables during linear
             *        independence tests.
             * @see `Options::maxDualJump`
             */
            real_t max_dual_jump = 1.0e8;

            /**
             * @brief Start value of the ramping strategy.
             * @see `Options::initialRamping`
             */
            real_t initial_ramping = 0.5;

            /**
             * @brief Final value of the ramping strategy.
             * @see `Options::finalRamping`
             */
            real_t final_ramping = 1.0;

            /**
             * @brief Initial size of the far bounds.
             * @see `Options::initialFarBounds`
             */
            real_t initial_far_bounds = 1.0e6;

            /**
             * @brief Growth factor applied to the far bounds.
             * @see `Options::growFarBounds`
             */
            real_t grow_far_bounds = 1.0e3;

            /**
             * @brief Status assumed for all bounds at the first iteration.
             * @see `Options::initialStatusBounds`
             */
            SubjectToStatus initial_status_bounds = ST_LOWER;

            /**
             * @brief Tolerance of the squared Cholesky diagonal factor which
             *        triggers flipping of a bound.
             * @see `Options::epsFlipping`
             */
            real_t eps_flipping = 1.0e3 * EPS;

            /**
             * @brief Maximum number of successive regularisation steps.
             * @see `Options::numRegularisationSteps`
             */
            int_t num_regularisation_steps = 0;

            /**
             * @brief Scaling factor of the identity matrix used for Hessian
             *        regularisation.
             * @see `Options::epsRegularisation`
             */
            real_t eps_regularisation = 1.0e3 * EPS;

            /**
             * @brief Maximum number of iterative refinement steps.
             * @see `Options::numRefinementSteps`
             */
            int_t num_refinement_steps = 1;

            /**
             * @brief Early termination tolerance for iterative refinement.
             * @see `Options::epsIterRef`
             */
            real_t eps_iter_ref = 1.0e2 * EPS;

            /**
             * @brief Tolerance used by the linear independence tests.
             * @see `Options::epsLITests`
             */
            real_t eps_li_tests = 1.0e5 * EPS;

            /**
             * @brief Tolerance used by the nonzero curvature tests.
             * @see `Options::epsNZCTests`
             */
            real_t eps_nzc_tests = 3.0e3 * EPS;

            /**
             * @brief Minimum reciprocal condition number of the Schur
             *        complement matrix S below which a refactorisation is
             *        triggered.
             * @see `Options::rcondSMin`
             */
            real_t rcond_s_min = 1.0e-14;

            /**
             * @brief Whether the working set shall be repaired when negative
             *        curvature is discovered during a hotstart.
             * @see `Options::enableInertiaCorrection`
             */
            BooleanType enable_inertia_correction = BT_TRUE;

            /**
             * @brief Whether infeasible constraints may be dropped.
             * @see `Options::enableDropInfeasibles`
             */
            BooleanType enable_drop_infeasibles = BT_FALSE;

            /**
             * @brief Priority used when dropping bounds.
             * @see `Options::dropBoundPriority`
             */
            int_t drop_bound_priority = 1;

            /**
             * @brief Priority used when dropping equality constraints.
             * @see `Options::dropEqConPriority`
             */
            int_t drop_eq_con_priority = 1;

            /**
             * @brief Priority used when dropping inequality constraints.
             * @see `Options::dropIneqConPriority`
             */
            int_t drop_ineq_con_priority = 1;

            /**
             * @brief Definiteness assumed for the Hessian matrix.
             *
             * Not an `Options` field: this is the HessianType handed to the
             * underlying SQProblem. Page 22 of the manual.
             */
            HessianType hessian_type = HST_POSDEF;

            /**
             * @brief Default constructor.
             *
             * Initialises every option to the defaults documented above.
             *
             * @note Declared here and defined out of line so that the
             *       in-class member initializers are used as expected. See
             *       https://stackoverflow.com/questions/53408962
             */
            Configuration();
        };

    protected:
        /** @brief True until the first solve has initialised the problem. */
        bool qpoases_solve_first_time_;
        /** @brief The underlying qpOASES problem. */
        SQProblem qpoases_problem_;
        /** @brief Active configuration, copied at construction. */
        Configuration configuration_;

        /**
         * @brief Copies an Eigen vector into a std::vector<double>.
         * @param vectorxd Source vector.
         * @return A copy of the vector's data.
         *
         * Copied from SmartArmStack's sas_conversions
         * (https://github.com/SmartArmStack/sas_conversions/blob/master/src/eigen3_std_conversions.cpp).
         */
        std::vector<double> _vectorxd_to_std_vector_double(const VectorXd& vectorxd);

        /**
         * @brief Maps a std::vector<double> into an Eigen vector.
         * @param std_vector_double Source vector.
         * @return An Eigen vector wrapping the source data.
         */
        VectorXd _std_vector_double_to_vectorxd(std::vector<double> std_vector_double);

        /**
         * @brief Translates the user configuration into a qpOASES `Options`
         *        object.
         * @return An `Options` with all qpOASES fields filled from
         *         `configuration_`.
         */
        Options _to_qpoases_options() const;

    public:
        /**
         * @brief Constructs a solver.
         * @param configuration Options to use; defaults to the default
         *                       configuration.
         */
        qpOASES_Solver(const Configuration& configuration = qpOASES_Solver::Configuration());

        /** @brief Default destructor. */
        ~qpOASES_Solver()=default;

        /**
         * @brief Solves the following quadratic program:
         *
         *   min(x)  0.5*x'Hx + f'x
         *   s.t.    Ax <= b
         *           Aeq*x = beq.
         *
         * Method signature is compatible with MATLAB's `quadprog`.
         *
         * @param H the n x n matrix of the quadratic coefficients of the
         *        decision variables.
         * @param f the n x 1 vector of the linear coefficients of the
         *        decision variables.
         * @param A the m x n matrix of inequality constraints.
         * @param b the m x 1 value for the inequality constraints.
         * @param Aeq the k x n matrix of equality constraints.
         * @param beq the k x 1 value for the equality constraints.
         * @return the optimal x.
         * @throws std::runtime_error if any matrix is size-incompatible or
         *         qpOASES fails to solve the problem.
         */
        VectorXd solve_quadratic_program(const MatrixXd& H, const VectorXd& f, const MatrixXd& A, const VectorXd& b, const MatrixXd& Aeq, const VectorXd& beq);

        /**
         * @brief Returns the active set of constraints obtained in the most
         *        recent call to solve_quadratic_program(). The returned
         *        vector has one entry for each row of the combined
         *        constraint matrix, i.e. the rows of A followed by the rows
         *        of Aeq, in that same order, with the following meaning for
         *        each entry:
         *
         *          - -1: the constraint is active at its lower bound;
         *          -  0: the constraint is inactive;
         *          - +1: the constraint is active at its upper bound (this is
         *                also the value used for active equality
         *                constraints, as their lower and upper bounds
         *                coincide).
         *
         * @return the active set, as described above.
         * @throws std::runtime_error if solve_quadratic_program() has not
         *         been called at least once.
         */
        VectorXd get_active_set();

        /**
         * @brief Round-trips a vector to help evaluate the Eigen <-> std
         *        conversions used across the wrapper.
         * @param v The vector to test.
         * @return The same vector.
         */
        VectorXd test_vectorxd(const VectorXd& v);

        /**
         * @brief Round-trips a matrix to help evaluate the Eigen <-> std
         *        conversions used across the wrapper.
         * @param m The matrix to test.
         * @return The same matrix.
         */
        MatrixXd test_matrixxd(const MatrixXd& m);
};

} // namespace M3
