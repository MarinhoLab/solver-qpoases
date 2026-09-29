/**
Based on https://github.com/dqrobotics/cpp-interface-qpoases
Originally by Murilo M. Marinho
*/
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

#include <marinholab/solvers/qpoases.h>

#include <qpOASES.hpp>

using namespace qpOASES;

namespace marinholab
{

namespace solvers
{

namespace qpoases
{

// ---------------------------------------------------------------------------
// Option metadata
// ---------------------------------------------------------------------------

enum class OptionKind
{
    Boolean,
    Integer,
    Real,
    PrintLevel,
    HessianType,
    SubjectToStatus,
};

namespace detail
{

inline const std::map<std::string, OptionKind>& option_kinds()
{
    static const std::map<std::string, OptionKind> kinds = {
        // Wrapper-specific options.
        {"maximum_working_set_recalculations", OptionKind::Integer},
        {"use_hotstart", OptionKind::Boolean},
        {"hessian_type", OptionKind::HessianType},
        // qpOASES `Options` fields (1:1 mapping).
        {"printLevel", OptionKind::PrintLevel},
        {"enableRamping", OptionKind::Boolean},
        {"enableFarBounds", OptionKind::Boolean},
        {"enableFlippingBounds", OptionKind::Boolean},
        {"enableRegularisation", OptionKind::Boolean},
        {"enableFullLITests", OptionKind::Boolean},
        {"enableNZCTests", OptionKind::Boolean},
        {"enableDriftCorrection", OptionKind::Integer},
        {"enableCholeskyRefactorisation", OptionKind::Integer},
        {"enableEqualities", OptionKind::Boolean},
        {"terminationTolerance", OptionKind::Real},
        {"boundTolerance", OptionKind::Real},
        {"boundRelaxation", OptionKind::Real},
        {"epsNum", OptionKind::Real},
        {"epsDen", OptionKind::Real},
        {"maxPrimalJump", OptionKind::Real},
        {"maxDualJump", OptionKind::Real},
        {"initialRamping", OptionKind::Real},
        {"finalRamping", OptionKind::Real},
        {"initialFarBounds", OptionKind::Real},
        {"growFarBounds", OptionKind::Real},
        {"initialStatusBounds", OptionKind::SubjectToStatus},
        {"epsFlipping", OptionKind::Real},
        {"numRegularisationSteps", OptionKind::Integer},
        {"epsRegularisation", OptionKind::Real},
        {"numRefinementSteps", OptionKind::Integer},
        {"epsIterRef", OptionKind::Real},
        {"epsLITests", OptionKind::Real},
        {"epsNZCTests", OptionKind::Real},
        {"rcondSMin", OptionKind::Real},
        {"enableInertiaCorrection", OptionKind::Boolean},
        {"enableDropInfeasibles", OptionKind::Boolean},
        {"dropBoundPriority", OptionKind::Integer},
        {"dropEqConPriority", OptionKind::Integer},
        {"dropIneqConPriority", OptionKind::Integer},
    };
    return kinds;
}

inline bool is_known_option(const std::string& key)
{
    return option_kinds().find(key) != option_kinds().end();
}

inline std::string to_lower(const std::string& s)
{
    std::string out(s);
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return out;
}

/**
 * @brief Parses a boolean string, returning true when the value is true.
 */
inline bool parse_bool(const std::string& value)
{
    const std::string v = to_lower(value);
    if(v == "true" || v == "1" || v == "on")
        return true;
    if(v == "false" || v == "0" || v == "off")
        return false;
    throw std::invalid_argument("Invalid boolean value '" + value + "' (expected true/false).");
}

/**
 * @brief Parses an integer string.
 */
inline int parse_int(const std::string& value)
{
    const char* begin = value.c_str();
    char* end = nullptr;
    const long parsed = std::strtol(begin, &end, 10);
    if(begin == end || *end != '\0')
        throw std::invalid_argument("Invalid integer value '" + value + "'.");
    return static_cast<int>(parsed);
}

/**
 * @brief Parses a floating-point string.
 */
inline double parse_real(const std::string& value)
{
    const char* begin = value.c_str();
    char* end = nullptr;
    const double parsed = std::strtod(begin, &end);
    if(begin == end || *end != '\0')
        throw std::invalid_argument("Invalid number value '" + value + "'.");
    return parsed;
}

/**
 * @brief Parses an enumeration value by name, e.g. "HST_SEMIDEF".
 */
template <typename T>
inline T parse_enum(const std::string& key, const std::string& value,
                    const std::map<std::string, T>& allowed)
{
    const auto it = allowed.find(value);
    if(it == allowed.end())
    {
        std::string options;
        for(const auto& pair : allowed)
            options += (options.empty() ? "" : ", ") + pair.first;
        throw std::invalid_argument("Invalid value '" + value + "' for option '" + key + "'. Allowed values: " + options + ".");
    }
    return it->second;
}

/**
 * @brief Formats a real so that it reads back exactly.
 *
 * std::to_string prints six decimals, which turns qpOASES' small tolerances
 * (e.g. epsNum = -1e3 * EPS) into zero. 15 significant digits are tried
 * first for a short string, then 17, which always round-trips a double.
 * (snprintf is avoided: qpOASES defines it as a macro for _snprintf on MSVC.)
 */
inline std::string real_to_string(double value)
{
    std::string text;
    for (int precision : {15, 17})
    {
        std::ostringstream stream;
        stream.imbue(std::locale::classic());
        stream << std::setprecision(precision) << value;
        text = stream.str();
        std::istringstream back(text);
        back.imbue(std::locale::classic());
        double parsed = 0.0;
        back >> parsed;
        if (parsed == value)
            break;
    }
    return text;
}

/**
 * @brief The default value of `key`, as a string, from qpOASES itself.
 */
inline std::string default_for(const std::string& key)
{
    static const std::map<std::string, std::string> defaults = [] {
        Options options;  // default-constructed = setToDefault()
        options.printLevel = PL_NONE;  // quiet by default (see Configuration)
        std::map<std::string, std::string> d;
        d["maximum_working_set_recalculations"] = "150";
        d["use_hotstart"] = "true";
        d["hessian_type"] = "HST_POSDEF";
        d["printLevel"] = [&] {
            const char* name = "PL_NONE";
            switch(options.printLevel) {
                case PL_DEBUG_ITER: name = "PL_DEBUG_ITER"; break;
                case PL_TABULAR: name = "PL_TABULAR"; break;
                case PL_NONE: name = "PL_NONE"; break;
                case PL_LOW: name = "PL_LOW"; break;
                case PL_MEDIUM: name = "PL_MEDIUM"; break;
                case PL_HIGH: name = "PL_HIGH"; break;
            }
            return std::string(name);
        }();
        d["enableRamping"] = options.enableRamping == BT_TRUE ? "true" : "false";
        d["enableFarBounds"] = options.enableFarBounds == BT_TRUE ? "true" : "false";
        d["enableFlippingBounds"] = options.enableFlippingBounds == BT_TRUE ? "true" : "false";
        d["enableRegularisation"] = options.enableRegularisation == BT_TRUE ? "true" : "false";
        d["enableFullLITests"] = options.enableFullLITests == BT_TRUE ? "true" : "false";
        d["enableNZCTests"] = options.enableNZCTests == BT_TRUE ? "true" : "false";
        d["enableDriftCorrection"] = std::to_string(options.enableDriftCorrection);
        d["enableCholeskyRefactorisation"] = std::to_string(options.enableCholeskyRefactorisation);
        d["enableEqualities"] = options.enableEqualities == BT_TRUE ? "true" : "false";
        d["terminationTolerance"] = real_to_string(options.terminationTolerance);
        d["boundTolerance"] = real_to_string(options.boundTolerance);
        d["boundRelaxation"] = real_to_string(options.boundRelaxation);
        d["epsNum"] = real_to_string(options.epsNum);
        d["epsDen"] = real_to_string(options.epsDen);
        d["maxPrimalJump"] = real_to_string(options.maxPrimalJump);
        d["maxDualJump"] = real_to_string(options.maxDualJump);
        d["initialRamping"] = real_to_string(options.initialRamping);
        d["finalRamping"] = real_to_string(options.finalRamping);
        d["initialFarBounds"] = real_to_string(options.initialFarBounds);
        d["growFarBounds"] = real_to_string(options.growFarBounds);
        d["initialStatusBounds"] = [&] {
            const char* name = "ST_LOWER";
            switch(options.initialStatusBounds) {
                case ST_LOWER: name = "ST_LOWER"; break;
                case ST_INACTIVE: name = "ST_INACTIVE"; break;
                case ST_UPPER: name = "ST_UPPER"; break;
                case ST_INFEASIBLE_LOWER: name = "ST_INFEASIBLE_LOWER"; break;
                case ST_INFEASIBLE_UPPER: name = "ST_INFEASIBLE_UPPER"; break;
                case ST_UNDEFINED: name = "ST_UNDEFINED"; break;
            }
            return std::string(name);
        }();
        d["epsFlipping"] = real_to_string(options.epsFlipping);
        d["numRegularisationSteps"] = std::to_string(options.numRegularisationSteps);
        d["epsRegularisation"] = real_to_string(options.epsRegularisation);
        d["numRefinementSteps"] = std::to_string(options.numRefinementSteps);
        d["epsIterRef"] = real_to_string(options.epsIterRef);
        d["epsLITests"] = real_to_string(options.epsLITests);
        d["epsNZCTests"] = real_to_string(options.epsNZCTests);
        d["rcondSMin"] = real_to_string(options.rcondSMin);
        d["enableInertiaCorrection"] = options.enableInertiaCorrection == BT_TRUE ? "true" : "false";
        d["enableDropInfeasibles"] = options.enableDropInfeasibles == BT_TRUE ? "true" : "false";
        d["dropBoundPriority"] = std::to_string(options.dropBoundPriority);
        d["dropEqConPriority"] = std::to_string(options.dropEqConPriority);
        d["dropIneqConPriority"] = std::to_string(options.dropIneqConPriority);
        return d;
    }();
    return defaults.at(key);
}

} // namespace detail

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

Configuration::Configuration() = default;
Configuration::Configuration(const Configuration& other) = default;
Configuration& Configuration::operator=(const Configuration& other) = default;
Configuration::~Configuration() = default;

void Configuration::set(const std::string& key, const std::string& value)
{
    if(!detail::is_known_option(key))
        throw std::invalid_argument("Unknown option '" + key + "'. Use keys() for the valid option names.");
    // Validate the value up-front so type errors surface at set-time.
    switch(detail::option_kinds().at(key)) {
        case OptionKind::Boolean:
            detail::parse_bool(value);
            break;
        case OptionKind::Integer:
            detail::parse_int(value);
            break;
        case OptionKind::Real:
            detail::parse_real(value);
            break;
        case OptionKind::PrintLevel:
            detail::parse_enum<PrintLevel>(key, value, {
                {"PL_DEBUG_ITER", PL_DEBUG_ITER},
                {"PL_TABULAR", PL_TABULAR},
                {"PL_NONE", PL_NONE},
                {"PL_LOW", PL_LOW},
                {"PL_MEDIUM", PL_MEDIUM},
                {"PL_HIGH", PL_HIGH},
            });
            break;
        case OptionKind::HessianType:
            detail::parse_enum<HessianType>(key, value, {
                {"HST_ZERO", HST_ZERO},
                {"HST_IDENTITY", HST_IDENTITY},
                {"HST_POSDEF", HST_POSDEF},
                {"HST_POSDEF_NULLSPACE", HST_POSDEF_NULLSPACE},
                {"HST_SEMIDEF", HST_SEMIDEF},
                {"HST_INDEF", HST_INDEF},
                {"HST_UNKNOWN", HST_UNKNOWN},
            });
            break;
        case OptionKind::SubjectToStatus:
            detail::parse_enum<SubjectToStatus>(key, value, {
                {"ST_LOWER", ST_LOWER},
                {"ST_INACTIVE", ST_INACTIVE},
                {"ST_UPPER", ST_UPPER},
                {"ST_INFEASIBLE_LOWER", ST_INFEASIBLE_LOWER},
                {"ST_INFEASIBLE_UPPER", ST_INFEASIBLE_UPPER},
                {"ST_UNDEFINED", ST_UNDEFINED},
            });
            break;
    }
    options_[key] = value;
}

std::string Configuration::get(const std::string& key) const
{
    if(!detail::is_known_option(key))
        throw std::invalid_argument("Unknown option '" + key + "'.");
    const auto it = options_.find(key);
    if(it != options_.end())
        return it->second;
    return detail::default_for(key);
}

bool Configuration::has(const std::string& key) const
{
    if(!detail::is_known_option(key))
        throw std::invalid_argument("Unknown option '" + key + "'. Use keys() for the valid option names.");
    return options_.find(key) != options_.end();
}

std::vector<std::string> Configuration::keys() const
{
    const auto& kinds = detail::option_kinds();
    std::vector<std::string> out;
    out.reserve(kinds.size());
    for(const auto& pair : kinds)
        out.push_back(pair.first);
    return out;
}

void Configuration::reset(const std::string& key)
{
    if(!detail::is_known_option(key))
        throw std::invalid_argument("Unknown option '" + key + "'. Use keys() for the valid option names.");
    options_.erase(key);
}

void Configuration::reset_all()
{
    options_.clear();
}

std::map<std::string, std::string> Configuration::defaults() const
{
    const auto& kinds = detail::option_kinds();
    std::map<std::string, std::string> out;
    for(const auto& pair : kinds)
        out.emplace(pair.first, detail::default_for(pair.first));
    return out;
}

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

struct Solver::Impl
{
    /** @brief True until the first solve has initialised the problem. */
    bool qpoases_solve_first_time_;
    /** @brief The underlying qpOASES problem. */
    qpOASES::SQProblem qpoases_problem_;
    /** @brief Active configuration, copied at construction. */
    Configuration configuration_;

    Impl(const Configuration& configuration)
        : qpoases_solve_first_time_(true)
        , qpoases_problem_()
        , configuration_(configuration)
    {
    }
};

Solver::Solver(const Configuration& configuration)
    : impl_(std::make_unique<Impl>(configuration))
{
}

Solver::Solver(Solver&& other) noexcept = default;

Solver& Solver::operator=(Solver&& other) noexcept
{
    if(this != &other)
    {
        impl_ = std::move(other.impl_);
    }
    return *this;
}

Solver::~Solver() = default;

std::vector<double> to_std_vector_double(const Eigen::VectorXd& vectorxd)
{
    return std::vector<double>(vectorxd.data(), vectorxd.data() + vectorxd.rows() * vectorxd.cols());
}

Eigen::VectorXd from_std_vector_double(const std::vector<double>& std_vector_double)
{
    // Map requires a non-const pointer; we only read through it (the VectorXd
    // copy is made before the map is used), so const_cast is safe.
    double* ptr = const_cast<double*>(std_vector_double.data());
    Eigen::Map<Eigen::VectorXd> vec(ptr, std_vector_double.size());
    return vec;
}

namespace detail
{

/**
 * @brief Maps every option of the wrapper configuration onto a qpOASES
 * `Options` object.
 */
inline Options options_from(const Configuration& configuration)
{
    Options options;
    const auto get_int = [&configuration](const std::string& key) {
        return parse_int(configuration.get(key));
    };
    const auto get_real = [&configuration](const std::string& key) {
        return static_cast<real_t>(parse_real(configuration.get(key)));
    };
    const auto get_bool = [&configuration](const std::string& key) {
        return parse_bool(configuration.get(key)) ? BT_TRUE : BT_FALSE;
    };
    options.printLevel = parse_enum<PrintLevel>("printLevel", configuration.get("printLevel"), {
        {"PL_DEBUG_ITER", PL_DEBUG_ITER},
        {"PL_TABULAR", PL_TABULAR},
        {"PL_NONE", PL_NONE},
        {"PL_LOW", PL_LOW},
        {"PL_MEDIUM", PL_MEDIUM},
        {"PL_HIGH", PL_HIGH},
    });
    options.enableRamping = get_bool("enableRamping");
    options.enableFarBounds = get_bool("enableFarBounds");
    options.enableFlippingBounds = get_bool("enableFlippingBounds");
    options.enableRegularisation = get_bool("enableRegularisation");
    options.enableFullLITests = get_bool("enableFullLITests");
    options.enableNZCTests = get_bool("enableNZCTests");
    options.enableDriftCorrection = get_int("enableDriftCorrection");
    options.enableCholeskyRefactorisation = get_int("enableCholeskyRefactorisation");
    options.enableEqualities = get_bool("enableEqualities");
    options.terminationTolerance = get_real("terminationTolerance");
    options.boundTolerance = get_real("boundTolerance");
    options.boundRelaxation = get_real("boundRelaxation");
    options.epsNum = get_real("epsNum");
    options.epsDen = get_real("epsDen");
    options.maxPrimalJump = get_real("maxPrimalJump");
    options.maxDualJump = get_real("maxDualJump");
    options.initialRamping = get_real("initialRamping");
    options.finalRamping = get_real("finalRamping");
    options.initialFarBounds = get_real("initialFarBounds");
    options.growFarBounds = get_real("growFarBounds");
    options.initialStatusBounds = parse_enum<SubjectToStatus>("initialStatusBounds", configuration.get("initialStatusBounds"), {
        {"ST_LOWER", ST_LOWER},
        {"ST_INACTIVE", ST_INACTIVE},
        {"ST_UPPER", ST_UPPER},
        {"ST_INFEASIBLE_LOWER", ST_INFEASIBLE_LOWER},
        {"ST_INFEASIBLE_UPPER", ST_INFEASIBLE_UPPER},
        {"ST_UNDEFINED", ST_UNDEFINED},
    });
    options.epsFlipping = get_real("epsFlipping");
    options.numRegularisationSteps = get_int("numRegularisationSteps");
    options.epsRegularisation = get_real("epsRegularisation");
    options.numRefinementSteps = get_int("numRefinementSteps");
    options.epsIterRef = get_real("epsIterRef");
    options.epsLITests = get_real("epsLITests");
    options.epsNZCTests = get_real("epsNZCTests");
    options.rcondSMin = get_real("rcondSMin");
    options.enableInertiaCorrection = get_bool("enableInertiaCorrection");
    options.enableDropInfeasibles = get_bool("enableDropInfeasibles");
    options.dropBoundPriority = get_int("dropBoundPriority");
    options.dropEqConPriority = get_int("dropEqConPriority");
    options.dropIneqConPriority = get_int("dropIneqConPriority");
    return options;
}

inline HessianType hessian_type_from(const Configuration& configuration)
{
    return parse_enum<HessianType>("hessian_type", configuration.get("hessian_type"), {
        {"HST_ZERO", HST_ZERO},
        {"HST_IDENTITY", HST_IDENTITY},
        {"HST_POSDEF", HST_POSDEF},
        {"HST_POSDEF_NULLSPACE", HST_POSDEF_NULLSPACE},
        {"HST_SEMIDEF", HST_SEMIDEF},
        {"HST_INDEF", HST_INDEF},
        {"HST_UNKNOWN", HST_UNKNOWN},
    });
}

inline int maximum_working_set_recalculations_from(const Configuration& configuration)
{
    return parse_int(configuration.get("maximum_working_set_recalculations"));
}

inline bool use_hotstart_from(const Configuration& configuration)
{
    return parse_bool(configuration.get("use_hotstart"));
}

inline void evaluate_problem_return_value(returnValue problem_return_value)
{
    if(problem_return_value != SUCCESSFUL_RETURN)
    {
        if(problem_return_value == RET_MAX_NWSR_REACHED)
            throw std::runtime_error("Solver::solve_quadratic_program(): Maximum number of working set recalculations reached. Consider increasing the 'maximum_working_set_recalculations' option in the configuration.");
        else if(problem_return_value == RET_INIT_FAILED)
            throw std::runtime_error("Solver::solve_quadratic_program(): Initialization failed. Check if the problem is well defined and if the parameters are valid.");
        else
        {
            throw std::runtime_error("Solver::solve_quadratic_program(): Unable to solve quadratic program. qpOASES returned error code: " + std::to_string(problem_return_value) + std::string(" ") + std::to_string(getSimpleStatus(problem_return_value)));
        }
    }
}

} // namespace detail

Eigen::VectorXd Solver::solve_quadratic_program(const Eigen::MatrixXd& H, const Eigen::VectorXd& f, const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const Eigen::MatrixXd& Aeq, const Eigen::VectorXd& beq)
{
    const int PROBLEM_SIZE = H.rows();
    const int INEQUALITY_CONSTRAINT_SIZE = b.size();
    const int EQUALITY_CONSTRAINT_SIZE = beq.size();

    ///Check sizes
    //Objective function
    if(H.rows()!=H.cols())
        throw std::runtime_error("Solver::solve_quadratic_program(): H must be symmetric. H.rows()="+std::to_string(H.rows())+" but H.cols()="+std::to_string(H.cols())+".");
    if(f.size()!=H.rows())
        throw std::runtime_error("Solver::solve_quadratic_program(): f must be compatible with H. H.rows()=H.cols()="+std::to_string(H.rows())+" but f.size()="+std::to_string(f.size())+".");

    //Inequality constraints
    if(b.size()!=A.rows())
        throw std::runtime_error("Solver::solve_quadratic_program(): size of b="+std::to_string(b.size())+" should be compatible with rows of A="+std::to_string(A.rows())+".");

    //Equality constraints
    if(beq.size()!=Aeq.rows())
        throw std::runtime_error("Solver::solve_quadratic_program(): size of beq="+std::to_string(beq.size())+" should be compatible with rows of Aeq="+std::to_string(Aeq.rows())+".");

    //Append equality constraints to inequality constraints
    Eigen::MatrixXd A_extended = A;
    Eigen::VectorXd ub_extended = b;
    Eigen::VectorXd lb_extended;
    if(EQUALITY_CONSTRAINT_SIZE!=0 && INEQUALITY_CONSTRAINT_SIZE!=0)
    {
        A_extended.resize(INEQUALITY_CONSTRAINT_SIZE + EQUALITY_CONSTRAINT_SIZE, PROBLEM_SIZE);
        A_extended << A, Aeq;
        ub_extended.resize(INEQUALITY_CONSTRAINT_SIZE + EQUALITY_CONSTRAINT_SIZE);
        ub_extended << b, beq;
        lb_extended.resize(INEQUALITY_CONSTRAINT_SIZE + EQUALITY_CONSTRAINT_SIZE);
        lb_extended << -Eigen::VectorXd::Ones(b.size()) * INFTY, beq;
    } else if(EQUALITY_CONSTRAINT_SIZE!=0)
    {
        A_extended.resize(EQUALITY_CONSTRAINT_SIZE, PROBLEM_SIZE);
        A_extended << Aeq;
        ub_extended.resize(EQUALITY_CONSTRAINT_SIZE);
        ub_extended << beq;
        lb_extended.resize(EQUALITY_CONSTRAINT_SIZE);
        lb_extended << beq;
    }

    std::vector<double> H_std_vec(H.data(), H.data() + H.rows() * H.cols());
    real_t* H_vec = &H_std_vec[0];

    auto g_std_vec = to_std_vector_double(f);
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
        Eigen::MatrixXd AT = A_extended.transpose();
        A_std_vec = std::vector<double>(AT.data(), AT.data() + AT.rows() * AT.cols());
        A_vec = &A_std_vec[0];

        ub_std_vec = to_std_vector_double(ub_extended);
        ubA_vec = &ub_std_vec[0];

        if(lb_extended.size() > 0)
        {
            lb_std_vec = to_std_vector_double(lb_extended);
            lbA_vec = &lb_std_vec[0];
        }
    }

    // No variable bounds: pass -INFTY/+INFTY explicitly. qpOASES accepts NULL
    // for "no bounds", but with enableFarBounds off its hotstart() reads
    // lb_new[i] without checking for NULL (QProblem::updateActivitiesForHotstart).
    std::vector<real_t> lb_vec(PROBLEM_SIZE, -INFTY);
    std::vector<real_t> ub_vec(PROBLEM_SIZE, INFTY);

    auto& problem = impl_->qpoases_problem_;
    auto& configuration = impl_->configuration_;

    // qpOASES fixes the number of variables and constraints when the problem
    // is constructed. Both hotstart() and a repeated init() read the new data
    // with those sizes, so a problem with different sizes must be constructed
    // again instead of reusing the previous one.
    if(!impl_->qpoases_solve_first_time_
       && (problem.getNV() != PROBLEM_SIZE
           || problem.getNC() != INEQUALITY_CONSTRAINT_SIZE + EQUALITY_CONSTRAINT_SIZE))
        impl_->qpoases_solve_first_time_ = true;

    if(impl_->qpoases_solve_first_time_)
    {
        problem = qpOASES::SQProblem(PROBLEM_SIZE, INEQUALITY_CONSTRAINT_SIZE + EQUALITY_CONSTRAINT_SIZE, detail::hessian_type_from(configuration));
        problem.setOptions(detail::options_from(configuration));
        auto maximum_working_set_recalculations_local = detail::maximum_working_set_recalculations_from(configuration); //qpOASES changes the value, so we make a local copy
        auto problem_init_return = problem.init(H_vec,g_vec,A_vec,lb_vec.data(),ub_vec.data(),lbA_vec,ubA_vec,maximum_working_set_recalculations_local);

        detail::evaluate_problem_return_value(problem_init_return);

        impl_->qpoases_solve_first_time_ = false;
    }
    else
    {
        auto maximum_working_set_recalculations_local = detail::maximum_working_set_recalculations_from(configuration); //qpOASES changes the value, so we make a local copy

        returnValue problem_return_value;
        if(detail::use_hotstart_from(configuration))
            problem_return_value = problem.hotstart(H_vec,g_vec,A_vec,lb_vec.data(),ub_vec.data(),lbA_vec,ubA_vec,maximum_working_set_recalculations_local);
        else
            problem_return_value = problem.init(H_vec,g_vec,A_vec,lb_vec.data(),ub_vec.data(),lbA_vec,ubA_vec,maximum_working_set_recalculations_local);
        detail::evaluate_problem_return_value(problem_return_value);
    }

    // Use a std::vector instead of a variable-length array so that the code
    // compiles with MSVC (VLAs are a non-standard extension rejected by it).
    std::vector<real_t> xOpt(PROBLEM_SIZE);
    problem.getPrimalSolution( xOpt.data() );

    return from_std_vector_double(std::vector<double>(xOpt.begin(), xOpt.end()));
}

Eigen::VectorXd Solver::get_active_set()
{
    if(impl_->qpoases_solve_first_time_)
        throw std::runtime_error("Solver::get_active_set(): solve_quadratic_program() must be called at least once before the active set can be retrieved.");

    const int_t NC = impl_->qpoases_problem_.getNC();
    if(NC == 0)
        return Eigen::VectorXd(0);

    std::vector<double> active_set_std(NC, 0.0);
    detail::evaluate_problem_return_value(impl_->qpoases_problem_.getWorkingSetConstraints(&active_set_std[0]));

    return from_std_vector_double(active_set_std);
}

// Helper functions to help evaluate the wrapper when needed.
Eigen::VectorXd Solver::test_vectorxd(const Eigen::VectorXd& v)
{
    return v;
}

Eigen::MatrixXd Solver::test_matrixxd(const Eigen::MatrixXd& m)
{
    return m;
}

} // namespace qpoases

} // namespace solvers

} // namespace marinholab
