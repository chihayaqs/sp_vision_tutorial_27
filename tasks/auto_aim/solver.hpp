#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <list>
#include <opencv2/core/eigen.hpp>
#include <optional>

#include "armor.hpp"

namespace auto_aim
{

class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q); // 根据姿态信息更新坐标转换关系

  void solve(Armor & armor) const;// 计算装甲板的空间位姿；引用参数，直接修改传入的armor对象，解算结果填入对应字段

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

  std::optional<cv::Point2f> world2pixel(const Eigen::Vector3d & world_point) const;

private:
  Eigen::Vector3d t_yaw2pitch_;
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  void optimize_yaw(Armor & armor) const;

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP