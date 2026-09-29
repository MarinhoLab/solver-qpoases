/**
Based on https://github.com/dqrobotics/cpp-interface-qpoases
Originally by Murilo M. Marinho
*/
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <stdexcept>
#include <variant>

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
inline long long parse_int(const std::string& value)
{
    const char* begin = value.c_str();
    char* end = nullptr;
    const long long parsed = std::strtoll(begin, &end, 10);
    if(begin == end || *end != '\0')
        throw std::invalid_argument("Invalid integer value '" + value + "'.");
    return parsed;
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

/** @brief Names of the qpOASES `PrintLevel` values. */
inline const std::map<std::string, PrintLevel>& print_levels()
{
    static const std::map<std::string, PrintLevel> values = {
        {"PL_DEBUG_ITER", PL_DEBUG_ITER},
        {"PL_TABULAR", PL_TABULAR},
        {"PL_NONE", PL_NONE},
        {"PL_LOW", PL_LOW},
        {"PL_MEDIUM", PL_MEDIUM},
        {"PL_HIGH", PL_HIGH},
    };
    return values;
}

/** @brief Names of the qpOASES `HessianType` values. */
inline const std::map<std::string, HessianType>& hessian_types()
{
    static const std::map<std::string, HessianType> values = {
        {"HST_ZERO", HST_ZERO},
        {"HST_IDENTITY", HST_IDENTITY},
        {"HST_POSDEF", HST_POSDEF},
        {"HST_POSDEF_NULLSPACE", HST_POSDEF_NULLSPACE},
        {"HST_SEMIDEF", HST_SEMIDEF},
        {"HST_INDEF", HST_INDEF},
        {"HST_UNKNOWN", HST_UNKNOWN},
    };
    return values;
}

/** @brief Names of the qpOASES `SubjectToStatus` values. */
inline const std::map<std::string, SubjectToStatus>& subject_to_statuses()
{
    static const std::map<std::string, SubjectToStatus> values = {
        {"ST_LOWER", ST_LOWER},
        {"ST_INACTIVE", ST_INACTIVE},
        {"ST_UPPER", ST_UPPER},
        {"ST_INFEASIBLE_LOWER", ST_INFEASIBLE_LOWER},
        {"ST_INFEASIBLE_UPPER", ST_INFEASIBLE_UPPER},
        {"ST_UNDEFINED", ST_UNDEFINED},
    };
    return values;
}

/**
 * @brief The name of an enumeration value, from one of the tables above.
 */
template <typename T>
inline std::string name_of(const std::map<std::string, T>& names, T value)
{
    for(const auto& pair : names)
        if(pair.second == value)
            return pair.first;
    throw std::logic_error("Enumeration value without a name.");
}

/**
 * @brief Converts `value` to the kind of option `key`.
 *
 * Booleans accept `bool` or "true"/"false"; integers accept `long long` or
 * an integer literal; reals accept `double`, `long long` or a
 * floating-point literal; enumerations accept a value name.
 */
inline OptionValue normalize(const std::string& key, const OptionValue& value)
{
    const auto wrong_type = [&key](const std::string& expected) {
        return std::invalid_argument("Invalid value for option '" + key + "': expected " + expected + ".");
    };
    const auto* text = std::get_if<std::string>(&value);
    switch(option_kinds().at(key)) {
        case OptionKind::Boolean:
            if(const auto* b = std::get_if<bool>(&value))
                return *b;
            if(text)
                return parse_bool(*text);
            throw wrong_type("a bool");
        case OptionKind::Integer:
            if(const auto* i = std::get_if<long long>(&value))
                return *i;
            if(text)
                return parse_int(*text);
            throw wrong_type("an integer");
        case OptionKind::Real:
            if(const auto* d = std::get_if<double>(&value))
                return *d;
            if(const auto* i = std::get_if<long long>(&value))
                return static_cast<double>(*i);
            if(text)
                return parse_real(*text);
            throw wrong_type("a real number");
        case OptionKind::PrintLevel:
            if(text)
                return name_of(print_levels(), parse_enum(key, *text, print_levels()));
            throw wrong_type("a PrintLevel name, e.g. \"PL_NONE\"");
        case OptionKind::HessianType:
            if(text)
                return name_of(hessian_types(), parse_enum(key, *text, hessian_types()));
            throw wrong_type("a HessianType name, e.g. \"HST_POSDEF\"");
        case OptionKind::SubjectToStatus:
            if(text)
                return name_of(subject_to_statuses(), parse_enum(key, *text, subject_to_statuses()));
            throw wrong_type("a SubjectToStatus name, e.g. \"ST_LOWER\"");
    }
    throw std::logic_error("Unhandled option kind.");
}

/**
 * @brief Every option's default value, taken directly from qpOASES.
 */
inline const std::map<std::string, OptionValue>& default_values()
{
    static const std::map<std::string, OptionValue> defaults = [] {
        Options options;  // default-constructed = setToDefault()
        const auto boolean = [](BooleanType value) { return value == BT_TRUE; };
        const auto integer = [](int_t value) { return static_cast<long long>(value); };
        const auto real = [](real_t value) { return static_cast<double>(value); };
        std::map<std::string, OptionValue> d;
        d["maximum_working_set_recalculations"] = 150LL;
        d["use_hotstart"] = true;
        d["hessian_type"] = std::string("HST_POSDEF");
        d["printLevel"] = std::string("PL_NONE");  // quiet by default (see Configuration)
        d["enableRamping"] = boolean(options.enableRamping);
        d["enableFarBounds"] = boolean(options.enableFarBounds);
        d["enableFlippingBounds"] = boolean(options.enableFlippingBounds);
        d["enableRegularisation"] = boolean(options.enableRegularisation);
        d["enableFullLITests"] = boolean(options.enableFullLITests);
        d["enableNZCTests"] = boolean(options.enableNZCTests);
        d["enableDriftCorrection"] = integer(options.enableDriftCorrection);
        d["enableCholeskyRefactorisation"] = integer(options.enableCholeskyRefactorisation);
        d["enableEqualities"] = boolean(options.enableEqualities);
        d["terminationTolerance"] = real(options.terminationTolerance);
        d["boundTolerance"] = real(options.boundTolerance);
        d["boundRelaxation"] = real(options.boundRelaxation);
        d["epsNum"] = real(options.epsNum);
        d["epsDen"] = real(options.epsDen);
        d["maxPrimalJump"] = real(options.maxPrimalJump);
        d["maxDualJump"] = real(options.maxDualJump);
        d["initialRamping"] = real(options.initialRamping);
        d["finalRamping"] = real(options.finalRamping);
        d["initialFarBounds"] = real(options.initialFarBounds);
        d["growFarBounds"] = real(options.growFarBounds);
        d["initialStatusBounds"] = name_of(subject_to_statuses(), options.initialStatusBounds);
        d["epsFlipping"] = real(options.epsFlipping);
        d["numRegularisationSteps"] = integer(options.numRegularisationSteps);
        d["epsRegularisation"] = real(options.epsRegularisation);
        d["numRefinementSteps"] = integer(options.numRefinementSteps);
        d["epsIterRef"] = real(options.epsIterRef);
        d["epsLITests"] = real(options.epsLITests);
        d["epsNZCTests"] = real(options.epsNZCTests);
        d["rcondSMin"] = real(options.rcondSMin);
        d["enableInertiaCorrection"] = boolean(options.enableInertiaCorrection);
        d["enableDropInfeasibles"] = boolean(options.enableDropInfeasibles);
        d["dropBoundPriority"] = integer(options.dropBoundPriority);
        d["dropEqConPriority"] = integer(options.dropEqConPriority);
        d["dropIneqConPriority"] = integer(options.dropIneqConPriority);
        return d;
    }();
    return defaults;
}

} // namespace detail

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

Configuration::Configuration() = default;
Configuration::Configuration(const Configuration& other) = default;
Configuration& Configuration::operator=(const Configuration& other) = default;
Configuration::~Configuration() = default;

void Configuration::set(const std::string& key, const OptionValue& value)
{
    if(!detail::is_known_option(key))
        throw std::invalid_argument("Unknown option '" + key + "'. Use keys() for the valid option names.");
    // Validate and convert up-front so type errors surface at set-time.
    options_[key] = detail::normalize(key, value);
}

void Configuration::set(const std::string& key, const char* value)
{
    set(key, OptionValue(std::string(value)));
}

OptionValue Configuration::get(const std::string& key) const
{
    if(!detail::is_known_option(key))
        throw std::invalid_argument("Unknown option '" + key + "'.");
    const auto it = options_.find(key);
    if(it != options_.end())
        return it->second;
    return detail::default_values().at(key);
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

std::map<std::string, OptionValue> Configuration::defaults() const
{
    return detail::default_values();
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
 * @brief Builds the qpOASES `Options`: qpOASES' own defaults, the wrapper's
 * quieter `printLevel`, and then every option that was explicitly set.
 */
inline Options options_from(const Configuration& configuration)
{
    Options options;  // qpOASES' defaults
    options.printLevel = PL_NONE;  // the wrapper's default (see Configuration)
    const auto real = [&configuration](const std::string& key, real_t& field) {
        if(configuration.has(key))
            field = static_cast<real_t>(std::get<double>(configuration.get(key)));
    };
    const auto integer = [&configuration](const std::string& key, int_t& field) {
        if(configuration.has(key))
            field = static_cast<int_t>(std::get<long long>(configuration.get(key)));
    };
    const auto boolean = [&configuration](const std::string& key, BooleanType& field) {
        if(configuration.has(key))
            field = std::get<bool>(configuration.get(key)) ? BT_TRUE : BT_FALSE;
    };
    if(configuration.has("printLevel"))
        options.printLevel = print_levels().at(std::get<std::string>(configuration.get("printLevel")));
    boolean("enableRamping", options.enableRamping);
    boolean("enableFarBounds", options.enableFarBounds);
    boolean("enableFlippingBounds", options.enableFlippingBounds);
    boolean("enableRegularisation", options.enableRegularisation);
    boolean("enableFullLITests", options.enableFullLITests);
    boolean("enableNZCTests", options.enableNZCTests);
    integer("enableDriftCorrection", options.enableDriftCorrection);
    integer("enableCholeskyRefactorisation", options.enableCholeskyRefactorisation);
    boolean("enableEqualities", options.enableEqualities);
    real("terminationTolerance", options.terminationTolerance);
    real("boundTolerance", options.boundTolerance);
    real("boundRelaxation", options.boundRelaxation);
    real("epsNum", options.epsNum);
    real("epsDen", options.epsDen);
    real("maxPrimalJump", options.maxPrimalJump);
    real("maxDualJump", options.maxDualJump);
    real("initialRamping", options.initialRamping);
    real("finalRamping", options.finalRamping);
    real("initialFarBounds", options.initialFarBounds);
    real("growFarBounds", options.growFarBounds);
    if(configuration.has("initialStatusBounds"))
        options.initialStatusBounds = subject_to_statuses().at(
            std::get<std::string>(configuration.get("initialStatusBounds")));
    real("epsFlipping", options.epsFlipping);
    integer("numRegularisationSteps", options.numRegularisationSteps);
    real("epsRegularisation", options.epsRegularisation);
    integer("numRefinementSteps", options.numRefinementSteps);
    real("epsIterRef", options.epsIterRef);
    real("epsLITests", options.epsLITests);
    real("epsNZCTests", options.epsNZCTests);
    real("rcondSMin", options.rcondSMin);
    boolean("enableInertiaCorrection", options.enableInertiaCorrection);
    boolean("enableDropInfeasibles", options.enableDropInfeasibles);
    integer("dropBoundPriority", options.dropBoundPriority);
    integer("dropEqConPriority", options.dropEqConPriority);
    integer("dropIneqConPriority", options.dropIneqConPriority);
    return options;
}

inline HessianType hessian_type_from(const Configuration& configuration)
{
    return hessian_types().at(std::get<std::string>(configuration.get("hessian_type")));
}

inline int maximum_working_set_recalculations_from(const Configuration& configuration)
{
    return static_cast<int>(std::get<long long>(configuration.get("maximum_working_set_recalculations")));
}

inline bool use_hotstart_from(const Configuration& configuration)
{
    return std::get<bool>(configuration.get("use_hotstart"));
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
