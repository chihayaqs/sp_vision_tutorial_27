#pragma once

#include <Eigen/Dense>

namespace sp_controller_server::predictive
{

class ConstrainedQp
{
public:
  // 求解0.5×xᵀHx + gradientᵀx，满足 lower ≤ A×x ≤ upper
  void configure(const Eigen::MatrixXd & H, const Eigen::MatrixXd & A, double rho = 1.0)
  {
    A_ = A;
    rho_ = rho;
    // 缓存ADMM更新和无约束极小点所需的矩阵分解
    factor_.compute(H + rho_ * A_.transpose() * A_);
    unconstrained_.compute(H);
  }

  Eigen::VectorXd solve(
    const Eigen::VectorXd & gradient, const Eigen::VectorXd & lower,
    const Eigen::VectorXd & upper, const Eigen::VectorXd & initial,
    int iterations = 100, double tolerance = 1e-5)
  {
    // 无约束极小点若已满足所有限制，也是带约束问题的最优点
    Eigen::VectorXd optimum = unconstrained_.solve(-gradient);
    const Eigen::VectorXd projected = A_ * optimum;
    if ((projected.array() >= lower.array()).all() &&
      (projected.array() <= upper.array()).all())
    {
      return optimum;
    }
    // x从上一轮指令开始；z 是约束区间内的投影，dual 是缩放后的乘子
    Eigen::VectorXd x = initial;
    Eigen::VectorXd z = (A_ * x).cwiseMax(lower).cwiseMin(upper);
    Eigen::VectorXd dual = Eigen::VectorXd::Zero(z.size());
    // 投影落在线性约束区间内，乘子推动预测轨迹满足同一组约束
    for (int iteration = 0; iteration < iterations; ++iteration) {
      x = factor_.solve(-gradient + rho_ * A_.transpose() * (z - dual));
      const Eigen::VectorXd value = A_ * x;
      const Eigen::VectorXd previous_z = z;
      z = (value + dual).cwiseMax(lower).cwiseMin(upper);
      dual += value - z;
      // 原始残差检查Ax与z的一致性，对偶残差检查投影变化；两者均达标才提前结束
      const double primal_residual = (value - z).lpNorm<Eigen::Infinity>();
      const double dual_residual =
        (rho_ * A_.transpose() * (z - previous_z)).lpNorm<Eigen::Infinity>();
      if (primal_residual < tolerance && dual_residual < tolerance) {
        break;
      }
    }
    return x;
  }

private:
  Eigen::MatrixXd A_;
  Eigen::LDLT<Eigen::MatrixXd> factor_;
  Eigen::LDLT<Eigen::MatrixXd> unconstrained_;
  double rho_;
};

}
