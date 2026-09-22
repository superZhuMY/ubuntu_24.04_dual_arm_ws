// Copyright 2026 zmy

#include "double_arm_harvest_execution/pose_utils.hpp"

#include <cmath>

namespace double_arm_harvest_execution
{

bool is_finite_pose(const geometry_msgs::msg::Pose & pose)
{
  const auto & p = pose.position;
  const auto & q = pose.orientation;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
         std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w);
}

bool normalize_quaternion(geometry_msgs::msg::Quaternion & q, double tolerance)
{
  if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
    !std::isfinite(q.w)) {
    return false;
  }
  const double norm =
    std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < 1e-9 || std::abs(norm - 1.0) > tolerance) {
    return false;
  }
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  return true;
}

geometry_msgs::msg::Point apply_tool_offset(
  const geometry_msgs::msg::Pose & target_pose,
  const std::array<double, 3> & tool_offset)
{
  // Rotate v by the unit quaternion q = (w, xyz):
  //   t  = 2 * cross(q.xyz, v)
  //   v' = v + w * t + cross(q.xyz, t)
  const auto & q = target_pose.orientation;
  const double vx = tool_offset[0];
  const double vy = tool_offset[1];
  const double vz = tool_offset[2];

  const double tx = 2.0 * (q.y * vz - q.z * vy);
  const double ty = 2.0 * (q.z * vx - q.x * vz);
  const double tz = 2.0 * (q.x * vy - q.y * vx);

  geometry_msgs::msg::Point result;
  result.x = target_pose.position.x + vx + q.w * tx + (q.y * tz - q.z * ty);
  result.y = target_pose.position.y + vy + q.w * ty + (q.z * tx - q.x * tz);
  result.z = target_pose.position.z + vz + q.w * tz + (q.x * ty - q.y * tx);
  return result;
}

}  // namespace double_arm_harvest_execution
