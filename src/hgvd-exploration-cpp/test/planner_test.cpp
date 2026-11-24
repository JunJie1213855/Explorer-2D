#include "planner.h"

#include <cmath>
#include <gtest/gtest.h>

namespace
{
using hgvd::Cell;
using hgvd::Planner;

TEST(Planner, ContinuesExploringWithoutVoronoiSamples)
{
  constexpr int width = 11, height = 6;
  std::vector<int8_t> grid(width * height, -1);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (x < 4 || x >= 8) {grid[y * width + x] = 0;}
    }
  }
  Planner planner({});
  planner.update_map(width, height, 0.2, grid);
  const Cell robot{1, 2};
  ASSERT_TRUE(planner.update(robot));
  EXPECT_TRUE(planner.samples().empty());
  EXPECT_FALSE(planner.reachable({8, 2}));
  EXPECT_TRUE(planner.has_frontiers());
  auto targets = planner.viewpoints(robot, 1.0, 2.0, 0.35, {});
  ASSERT_FALSE(targets.empty());
  for (const auto & [target, score] : planner.scores(robot, 0, targets)) {
    EXPECT_EQ(target.x, 3);
    EXPECT_TRUE(std::isfinite(score));
  }
  EXPECT_TRUE(planner.viewpoints(robot, 1.0, 2.0, 100.0, {}).empty());
  EXPECT_TRUE(planner.has_frontiers());
}

TEST(Planner, SamplesAndRebuildsAgainstChangedRectangularMap)
{
  constexpr int width = 24, height = 11;
  std::vector<int8_t> grid(width * height, 0);
  for (int x = 0; x < width; ++x) {
    grid[x] = grid[(height - 1) * width + x] = 100;
  }
  Planner planner({});
  planner.update_map(width, height, 0.2, grid);
  ASSERT_TRUE(planner.update({2, 5}));
  ASSERT_FALSE(planner.samples().empty());
  const auto before = planner.topology_revision();
  ASSERT_TRUE(planner.update({3, 5}));
  EXPECT_EQ(planner.topology_revision(), before);
  ASSERT_TRUE(planner.update({21, 5}));
  EXPECT_GT(planner.topology_revision(), before);
  for (const auto & sample : planner.samples()) {
    EXPECT_TRUE(planner.reachable(sample.cell));
    EXPECT_GE(sample.radius, 2.0);
  }
  for (int y = 0; y < height; ++y) {grid[y * width + 16] = 100;}
  planner.update_map(width, height, 0.2, grid);
  ASSERT_TRUE(planner.update({21, 5}));
  EXPECT_FALSE(planner.reachable({2, 5}));
  for (const auto & sample : planner.samples()) {EXPECT_GT(sample.cell.x, 16);}
}

TEST(Planner, FindsAReachableRouteAroundAnOccludingWall)
{
  constexpr int width = 15, height = 11;
  std::vector<int8_t> grid(width * height, 0);
  for (int y = 0; y < height - 1; ++y) {grid[y * width + 7] = 100;}
  Planner planner({});
  planner.update_map(width, height, 1.0, grid);
  const Cell robot{2, 2}, target{12, 2};
  ASSERT_TRUE(planner.update(robot));
  EXPECT_TRUE(planner.reachable(target));
  EXPECT_GT(planner.scores(robot, 0, {target}).at(target), 10);
  grid[(height - 1) * width + 7] = 100;
  planner.update_map(width, height, 1.0, grid);
  ASSERT_TRUE(planner.update(robot));
  EXPECT_FALSE(planner.reachable(target));
  EXPECT_EQ(planner.scores(robot, 0, {target}).at(target), INFINITY);
}

TEST(Planner, RecoversAnUnknownRobotCellWithinRadiusWithoutCrossingWalls)
{
  Planner planner({});
  std::vector<int8_t> grid{0, 100, -1, -1, 0, 0};
  planner.update_map(6, 1, 0.05, grid);
  const Cell pose{2, 0}, expected{4, 0};
  EXPECT_FALSE(planner.start_cell(pose, 2, 0, 0.09));
  EXPECT_FALSE(planner.start_cell({1, 0}, 1, 0, 0.25));
  EXPECT_FALSE(planner.start_cell({-1, 0}, -1, 0, 0.25));
  const auto anchor = planner.start_cell(pose, 2, 0, 0.25);
  ASSERT_TRUE(anchor);
  EXPECT_EQ(*anchor, expected);
  ASSERT_TRUE(planner.update(*anchor));
  EXPECT_FALSE(planner.reachable({0, 0}));
  EXPECT_FALSE(planner.reachable(pose));
}
}  // namespace
