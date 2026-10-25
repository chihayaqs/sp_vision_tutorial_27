#pragma once

#include <cmath>
#include <Eigen/Dense>

namespace sp_controller_server::predictive
{

struct DelayedVelocityModel
{
  // 状态顺序为位置、实际速度、滤波速度、延迟指令队列；A推进状态，B注入新指令
  Eigen::MatrixXd A;
  Eigen::VectorXd B;
  double dt;

  void configure(double timestep, double delay, double filter_tau, double velocity_tau)
  {
    dt = timestep;
    // delay_steps将指令延迟向上取整为队列长度
    const int delay_steps = static_cast<int>(std::ceil(delay / dt));
    const int size = 3 + delay_steps;
    A = Eigen::MatrixXd::Zero(size, size);
    B = Eigen::VectorXd::Zero(size);
    // alpha、beta是滤波和速度惯性每个周期保留的比例
    const double alpha = std::exp(-dt / filter_tau);
    const double beta = std::exp(-dt / velocity_tau);
    // 位置、实际速度、滤波速度与时延队列共同构成预测状态
    A(2, 2) = alpha;
    A(2, 3) = 1.0 - alpha;
    A(1, 1) = beta;
    A.row(1) += (1.0 - beta) * A.row(2);
    A(0, 0) = 1.0;
    A.row(0) += dt * A.row(1);
    // 旧指令依次前移，新指令由B放入队尾，经过延迟后影响滤波速度
    for (int i = 3; i + 1 < size; ++i) {
      A(i, i + 1) = 1.0;
    }
    B(size - 1) = 1.0;
  }

  int stateSize() const {return static_cast<int>(B.size());}

  Eigen::VectorXd advance(const Eigen::VectorXd & state, double input) const
  {
    return A * state + B * input;
  }
};

struct Prediction
{
  // Fp/Fv/Fa 对应当前状态贡献，Gp/Gv/Ga 对应未来指令贡献；p/v/a 表示位置/速度/加速度
  Eigen::MatrixXd Fp, Fv, Fa;
  Eigen::MatrixXd Gp, Gv, Ga;
};

inline Prediction buildPrediction(
  const DelayedVelocityModel & model, int horizon_steps, int move_steps)
{
  // 每move_steps步共用一个指令；80步预测得到每轴20个待求指令
  const int moves = (horizon_steps + move_steps - 1) / move_steps;
  const int size = model.stateSize();
  Prediction prediction;
  prediction.Fp.resize(horizon_steps, size);
  prediction.Fv.resize(horizon_steps, size);
  prediction.Fa.resize(horizon_steps, size);
  prediction.Gp.resize(horizon_steps, moves);
  prediction.Gv.resize(horizon_steps, moves);
  prediction.Ga.resize(horizon_steps, moves);
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(size, size);
  Eigen::MatrixXd G = Eigen::MatrixXd::Zero(size, moves);
  for (int k = 0; k < horizon_steps; ++k) {
    const Eigen::RowVectorXd previous_Fv = F.row(1);
    const Eigen::RowVectorXd previous_Gv = G.row(1);
    F = (model.A * F).eval();
    G = (model.A * G).eval();
    // 同一控制块累计同一个指令的影响，状态仍逐个dt推进
    G.col(k / move_steps) += model.B;
    prediction.Fp.row(k) = F.row(0);
    prediction.Fv.row(k) = F.row(1);
    // 相邻预测速度之差除以dt，得到实际加速度的预测矩阵
    prediction.Fa.row(k) = (F.row(1) - previous_Fv) / model.dt;
    prediction.Gp.row(k) = G.row(0);
    prediction.Gv.row(k) = G.row(1);
    prediction.Ga.row(k) = (G.row(1) - previous_Gv) / model.dt;
  }
  return prediction;
}

}
