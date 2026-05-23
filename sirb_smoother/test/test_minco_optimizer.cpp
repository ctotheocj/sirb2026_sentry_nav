#include <gtest/gtest.h>
#include <cmath>
#include "sirb_smoother/minco_optimizer.hpp"

using namespace sirb_smoother;

static nav_msgs::msg::Path makePath(const std::vector<std::pair<double, double>> & pts)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & [x, y] : pts) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    path.poses.push_back(ps);
  }
  return path;
}

static MincoOptimizer makeOptimizer(bool use_lbfgs = true)
{
  MincoOptimizer::Options opt;
  opt.enabled = true;
  opt.use_lbfgs = use_lbfgs;
  opt.max_pieces = 20;
  opt.pre_max_iterations = 20;
  opt.fine_max_iterations = 30;
  opt.w_obstacle_traj = 0.0;
  opt.w_velocity = 0.0;
  opt.w_acceleration = 0.0;
  opt.dynamic_obstacle_enabled = false;
  return MincoOptimizer(opt);
}

static void checkResult(
  const MincoOptimizer::Result & res,
  const nav_msgs::msg::Path & output,
  const nav_msgs::msg::Path & input)
{
  ASSERT_TRUE(res.success) << "reason: " << res.reason;
  ASSERT_GE(output.poses.size(), 2u);

  // endpoints preserved within tolerance
  const auto & ip = input.poses.front().pose.position;
  const auto & op = output.poses.front().pose.position;
  EXPECT_NEAR(op.x, ip.x, 0.05);
  EXPECT_NEAR(op.y, ip.y, 0.05);
  const auto & it = input.poses.back().pose.position;
  const auto & ot = output.poses.back().pose.position;
  EXPECT_NEAR(ot.x, it.x, 0.05);
  EXPECT_NEAR(ot.y, it.y, 0.05);

  // all positions finite
  for (const auto & pose : output.poses) {
    EXPECT_TRUE(std::isfinite(pose.pose.position.x));
    EXPECT_TRUE(std::isfinite(pose.pose.position.y));
  }

  // path length > 0
  double len = 0.0;
  for (size_t i = 1; i < output.poses.size(); ++i) {
    const double dx = output.poses[i].pose.position.x - output.poses[i-1].pose.position.x;
    const double dy = output.poses[i].pose.position.y - output.poses[i-1].pose.position.y;
    len += std::hypot(dx, dy);
  }
  EXPECT_GT(len, 0.0);
}

TEST(MincoOptimizer, StraightLine)
{
  auto opt = makeOptimizer();
  auto input = makePath({{0,0},{1,0},{2,0},{3,0},{4,0},{5,0}});
  nav_msgs::msg::Path output;
  auto res = opt.smooth(input, output);
  checkResult(res, output, input);
}

TEST(MincoOptimizer, LShape)
{
  auto opt = makeOptimizer();
  auto input = makePath({{0,0},{1,0},{2,0},{2,1},{2,2},{2,3}});
  nav_msgs::msg::Path output;
  auto res = opt.smooth(input, output);
  checkResult(res, output, input);
}

TEST(MincoOptimizer, LShapeUsesRoundedCornerGuide)
{
  auto opt = makeOptimizer();
  auto input = makePath({{0,0},{1,0},{2,0},{2,1},{2,2},{2,3}});
  nav_msgs::msg::Path output;
  auto res = opt.smooth(input, output);
  checkResult(res, output, input);

  bool has_rounded_sample = false;
  for (const auto & pose : output.poses) {
    const auto & p = pose.pose.position;
    if (p.x > 1.65 && p.x < 2.0 && p.y > 0.0 && p.y < 0.35) {
      has_rounded_sample = true;
      break;
    }
  }
  EXPECT_TRUE(has_rounded_sample);
}

TEST(MincoOptimizer, SShape)
{
  auto opt = makeOptimizer();
  auto input = makePath({{0,0},{1,0.5},{2,0},{3,-0.5},{4,0},{5,0.5},{6,0}});
  nav_msgs::msg::Path output;
  auto res = opt.smooth(input, output);
  checkResult(res, output, input);
}

TEST(MincoOptimizer, TooShort)
{
  auto opt = makeOptimizer();
  auto input = makePath({{0,0}});
  nav_msgs::msg::Path output;
  auto res = opt.smooth(input, output);
  EXPECT_FALSE(res.success);
}

TEST(MincoOptimizer, TwoPointPath)
{
  auto opt = makeOptimizer();
  auto input = makePath({{0,0},{1,0}});
  nav_msgs::msg::Path output;
  auto res = opt.smooth(input, output);
  checkResult(res, output, input);
}

TEST(MincoOptimizer, DuplicatePoints)
{
  auto opt = makeOptimizer();
  auto input = makePath({{0,0},{0,0},{1,0},{1,0},{2,0},{2,0},{3,0}});
  nav_msgs::msg::Path output;
  auto res = opt.smooth(input, output);
  // should either succeed or fail gracefully (no crash, no inf)
  if (res.success) {
    for (const auto & pose : output.poses) {
      EXPECT_TRUE(std::isfinite(pose.pose.position.x));
      EXPECT_TRUE(std::isfinite(pose.pose.position.y));
    }
  }
}

TEST(MincoOptimizer, NoLbfgs)
{
  auto opt = makeOptimizer(false);
  auto input = makePath({{0,0},{1,0},{2,1},{3,0},{4,0}});
  nav_msgs::msg::Path output;
  auto res = opt.smooth(input, output);
  checkResult(res, output, input);
}

TEST(MincoOptimizer, ReferenceTrajectoryUsesNonzeroInitialVelocity)
{
  MincoOptimizer::Options options;
  options.enabled = true;
  options.use_lbfgs = false;
  options.v_ref = 1.8;
  options.v_max = 2.2;
  options.a_max = 3.4;
  options.min_segment_time = 0.10;
  options.max_segment_time = 4.0;
  options.w_velocity = 1.0;
  options.w_acceleration = 0.5;
  MincoOptimizer opt(options);

  auto input = makePath({{0,0},{1,0},{2,0},{3,0},{4,0},{5,0}});
  nav_msgs::msg::Path output;
  MincoOptimizer::Result result;
  Eigen::Vector3d initial_velocity(1.6, 0.0, 0.0);
  Eigen::Vector3d initial_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d terminal_velocity(1.8, 0.0, 0.0);
  Eigen::Vector3d terminal_acceleration = Eigen::Vector3d::Zero();

  ASSERT_TRUE(opt.buildReferenceTrajectory(
    input, result, output, nullptr,
    &initial_velocity, &initial_acceleration,
    &terminal_velocity, &terminal_acceleration));
  checkResult(result, output, input);
  EXPECT_NEAR(result.initial_velocity.x(), 1.6, 0.05);
  EXPECT_NEAR(result.terminal_velocity.x(), 1.8, 0.05);
  EXPECT_LT(result.traj_duration, 4.0);
  EXPECT_GT(result.max_velocity, 1.4);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
