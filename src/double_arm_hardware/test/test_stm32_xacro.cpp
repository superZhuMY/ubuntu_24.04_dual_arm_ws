/// Static offline checks for the Xacro backend selection and the F.2
/// read-only launch (review task §4). These inspect the actual files on
/// disk — no build products, no /dev, no ROS runtime.

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

namespace
{

std::string read_file(const std::string & path)
{
  std::ifstream in(path);
  if (!in) return "";
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

const std::string kUrdf =
  std::string("/home/zmy/tomato/test1/src/double_arm_robot_moveit_config/") +
  "config/double_arm_robot.urdf.xacro";
const std::string kRos2Ctrl =
  std::string("/home/zmy/tomato/test1/src/double_arm_robot_moveit_config/") +
  "config/double_arm_robot.ros2_control.xacro";
const std::string kF2Launch =
  std::string("/home/zmy/tomato/test1/src/double_arm_robot_moveit_config/") +
  "launch/f2_stm32_readonly.launch.py";
const std::string kF2Yaml =
  std::string("/home/zmy/tomato/test1/src/double_arm_robot_moveit_config/") +
  "config/ros2_controllers_f2_stm32_readonly.yaml";

}  // namespace

// ══════════════════════════════════════════════════════════════════
//  transport_type is an independent selection parameter
// ══════════════════════════════════════════════════════════════════

TEST(Stm32Xacro, TransportTypeIndependentParam)
{
  const std::string urdf = read_file(kUrdf);
  ASSERT_FALSE(urdf.empty());
  EXPECT_NE(urdf.find("transport_type"), std::string::npos);
  EXPECT_NE(urdf.find("default=\"direct_motor\""), std::string::npos)
    << "transport_type must be its own argument, not folded into hardware_mode";
}

TEST(Stm32Xacro, DirectMotorKeepsTwoArmPlugins)
{
  const std::string rc = read_file(kRos2Ctrl);
  ASSERT_FALSE(rc.empty());
  // The direct-motor branch instantiates ArmSystemHardware twice via the
  // arm_hw macro (the <plugin> tag itself appears once inside the macro).
  int n = 0;
  size_t pos = 0;
  while ((pos = rc.find("xacro:arm_hw", pos)) != std::string::npos) {
    ++n;
    pos += 1;
  }
  EXPECT_EQ(n, 2) << "ArmSystemHardware macro must be invoked exactly twice";
  EXPECT_NE(rc.find("transport_type == 'direct_motor'"), std::string::npos);
}

TEST(Stm32Xacro, Stm32BranchSingleInstanceTwelveJoints)
{
  const std::string rc = read_file(kRos2Ctrl);
  ASSERT_FALSE(rc.empty());
  int n = 0;
  size_t pos = 0;
  while ((pos = rc.find("double_arm_hardware/Stm32SystemHardware", pos)) !=
         std::string::npos) {
    ++n;
    pos += 1;
  }
  EXPECT_EQ(n, 1) << "Stm32SystemHardware must appear exactly once";
  EXPECT_NE(rc.find("transport_type == 'stm32'"), std::string::npos);

  // The STM32 branch must invoke the stm32_j macro exactly once per joint.
  const std::string stm32_branch = rc.substr(rc.find("transport_type == 'stm32'"));
  for (int i = 1; i <= 6; ++i) {
    for (const char * side : {"L", "R"}) {
      const std::string jn = std::string(side) + "_Joint_" + std::to_string(i);
      size_t c = 0, p = 0;
      while ((p = stm32_branch.find("stm32_j j=\"" + jn + "\"", p)) !=
             std::string::npos) {
        ++c;
        p += 1;
      }
      EXPECT_EQ(c, 1u) << "Joint '" << jn << "' must be invoked exactly once";
    }
  }
}

TEST(Stm32Xacro, FakeModeUnchanged)
{
  const std::string rc = read_file(kRos2Ctrl);
  ASSERT_FALSE(rc.empty());
  EXPECT_NE(rc.find("mock_components/GenericSystem"), std::string::npos);
}

TEST(Stm32Xacro, Stm32BranchPassesGateParams)
{
  const std::string rc = read_file(kRos2Ctrl);
  ASSERT_FALSE(rc.empty());
  const std::string stm32 = rc.substr(rc.find("transport_type == 'stm32'"));
  EXPECT_NE(stm32.find("stm32_read_only"), std::string::npos);
  EXPECT_NE(stm32.find("allow_motor_enable"), std::string::npos);
  EXPECT_NE(stm32.find("enable_hardware"), std::string::npos);
  EXPECT_NE(stm32.find("allow_hardware_io"), std::string::npos);
  EXPECT_NE(stm32.find("stm32_control_ack_timeout_ms"), std::string::npos);
  EXPECT_NE(stm32.find("stm32_zero_offsets"), std::string::npos);
}

// ══════════════════════════════════════════════════════════════════
//  F.2 read-only launch
// ══════════════════════════════════════════════════════════════════

TEST(Stm32F2Launch, NoTrajectoryControllersNoMoveIt)
{
  const std::string launch = read_file(kF2Launch);
  ASSERT_FALSE(launch.empty());
  EXPECT_NE(launch.find("stm32_read_only"), std::string::npos);
  EXPECT_NE(launch.find("allow_motor_enable"), std::string::npos);
  EXPECT_EQ(launch.find("move_group"), std::string::npos)
    << "F.2 launch must not start MoveIt";
  EXPECT_EQ(launch.find("follow_joint_trajectory"), std::string::npos)
    << "F.2 launch must not reference trajectory controllers";
}

TEST(Stm32F2Launch, F2YamlOnlyJointStateBroadcaster)
{
  const std::string yaml = read_file(kF2Yaml);
  ASSERT_FALSE(yaml.empty());
  EXPECT_NE(yaml.find("joint_state_broadcaster"), std::string::npos);
  EXPECT_EQ(yaml.find("joint_trajectory_controller"), std::string::npos)
    << "F.2 controller set must not contain trajectory controllers";
  EXPECT_EQ(yaml.find("l_arm"), std::string::npos);
  EXPECT_EQ(yaml.find("r_arm"), std::string::npos);
}

TEST(Stm32F2Launch, LaunchStartsStateChain)
{
  const std::string launch = read_file(kF2Launch);
  ASSERT_FALSE(launch.empty());
  EXPECT_NE(launch.find("robot_state_publisher"), std::string::npos);
  EXPECT_NE(launch.find("ros2_control_node"), std::string::npos);
  EXPECT_NE(launch.find("joint_state_broadcaster"), std::string::npos);
}
