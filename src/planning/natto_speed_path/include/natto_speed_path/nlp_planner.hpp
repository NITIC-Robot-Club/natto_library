#ifndef __NLP_PLANNER_HPP_
#define __NLP_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <natto_msgs/msg/speed_path.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <vector>

namespace nlp_planner {

enum class ADOp {
    Constant,
    Variable,
    Add,
    Sub,
    Mul,
    Div,
    Neg,
    Square,
    Sqrt,
    Sin,
    Cos,
    Abs
};

struct ADNode {
    ADOp                     op;
    double                   value;
    double                   adjoint;
    std::vector<std::size_t> in;
};

class ADGraph {
   public:
    std::vector<ADNode> nodes;
    std::size_t         n_var;

    std::size_t constant (double);
    std::size_t variable (double);

    std::size_t add (std::size_t, std::size_t);
    std::size_t sub (std::size_t, std::size_t);
    std::size_t mul (std::size_t, std::size_t);
    std::size_t div (std::size_t, std::size_t);
    std::size_t neg (std::size_t);

    std::size_t square (std::size_t);
    std::size_t sqrt (std::size_t);
    std::size_t sin (std::size_t);
    std::size_t cos (std::size_t);
    std::size_t abs (std::size_t);

    void forward ();
    void reverse (std::size_t);

    void gradient (std::size_t, std::vector<double> &) const;
    void jacobian (const std::vector<std::size_t> &, std::vector<double> &, std::vector<std::size_t> &, std::vector<std::size_t> &) const;

    void hessian (std::size_t, std::vector<double> &, std::vector<std::size_t> &, std::vector<std::size_t> &) const;
};

struct NLPStructure {
    std::size_t nz;
    std::size_t neq;
    std::size_t nineq;
};

struct NLPBounds {
    std::vector<double> zL, zU;
    std::vector<double> gL, gU;
};

struct PrimalDualState {
    std::vector<double> z;
    std::vector<double> s;
    std::vector<double> lambda;
    std::vector<double> nu;
    double              mu;
};

struct Residuals {
    std::vector<double> r_dual;
    std::vector<double> r_primal;
    std::vector<double> r_comp;
    double              infeas;
    double              obj;
};

struct SparseSymmetricMatrix {
    std::vector<std::size_t> row, col;
    std::vector<double>      val;
    std::size_t              n;
};

class SparseLDLSolver {
   public:
    virtual ~SparseLDLSolver ()                            = default;
    virtual bool factorize (const SparseSymmetricMatrix &) = 0;
    virtual bool solve (std::vector<double> &) const       = 0;
};

class KKTAssembler {
   public:
    void assemble (const ADGraph &, const PrimalDualState &, const Residuals &, SparseSymmetricMatrix &, std::vector<double> &) const;
};

class InteriorPointOptimizer {
   public:
    void initialize (PrimalDualState &) const;
    void compute_residuals (const ADGraph &, const NLPStructure &, const PrimalDualState &, Residuals &) const;
    bool step (const ADGraph &, const NLPStructure &, PrimalDualState &, SparseLDLSolver &) const;
    bool converged (const Residuals &) const;
};

class nlp_planner : public rclcpp::Node {
   public:
    explicit nlp_planner (const rclcpp::NodeOptions &);

   private:
    std::size_t N;
    double      r_max;
    double      v_wheel_max;
    double      max_w;
    double      max_a;
    double      max_aw;

    ADGraph         graph;
    NLPStructure    structure;
    NLPBounds       bounds;
    PrimalDualState state;

    void build_problem ();
    void solve ();

    natto_msgs::msg::SpeedPath build_msg () const;

    rclcpp::Publisher<natto_msgs::msg::SpeedPath>::SharedPtr pub;
};

}  // namespace nlp_planner

#endif
