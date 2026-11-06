#include <gtest/gtest.h>
#include "cleannav_simulation/laser_self_filter.hpp"

TEST(LaserSelfFilter, OwnHitIsInvalidAndExternalNearObstacleIsPreserved)
{
  std::vector<float> ranges{0.21f, 0.15f, 3.2f};
  const auto masked = cleannav_simulation::mask_self_returns(
    ranges, {"vehicle::base::collision", "wall::link::collision", "cafe::link::collision"},
    "vehicle");
  ASSERT_EQ(masked, std::vector<size_t>{0});
  EXPECT_TRUE(std::isnan(ranges[0]));
  EXPECT_FALSE(std::isinf(ranges[0]));
  EXPECT_FLOAT_EQ(ranges[1], 0.15f);
  EXPECT_FLOAT_EQ(ranges[2], 3.2f);
}

TEST(LaserSelfFilter, NamesRequireExactModelBoundary)
{
  std::vector<float> ranges{0.21f, 0.21f};
  EXPECT_TRUE(cleannav_simulation::mask_self_returns(
    ranges, {"vehicle_neighbor::base::collision", "other::vehicle::collision"},
    "vehicle").empty());
  EXPECT_FLOAT_EQ(ranges[0], 0.21f);
  EXPECT_FLOAT_EQ(ranges[1], 0.21f);
}

TEST(LaserSelfFilter, InvalidAndNoReturnMeasurementsRemainUnchanged)
{
  std::vector<float> ranges{NAN, INFINITY, -INFINITY};
  EXPECT_TRUE(cleannav_simulation::mask_self_returns(
    ranges, {"vehicle::base", "vehicle::base", "vehicle::base"}, "vehicle").empty());
  EXPECT_TRUE(std::isnan(ranges[0]));
  EXPECT_EQ(ranges[1], INFINITY);
  EXPECT_EQ(ranges[2], -INFINITY);
}

TEST(LaserSelfFilter, StaleEntityNameWithNoReturnDoesNotMakeClearingRayInvalid)
{
  std::vector<float> ranges{INFINITY};
  EXPECT_TRUE(cleannav_simulation::mask_self_returns(
    ranges, {"vehicle::base::collision"}, "vehicle").empty());
  EXPECT_EQ(ranges[0], INFINITY);
}

TEST(LaserSelfFilter, MissingEntityMetadataFailsExplicitly)
{
  std::vector<float> ranges{0.21f};
  EXPECT_THROW(cleannav_simulation::mask_self_returns(ranges, {}, "vehicle"),
    std::invalid_argument);
  EXPECT_THROW(cleannav_simulation::mask_self_returns(ranges, {"wall::link"}, ""),
    std::invalid_argument);
  EXPECT_FLOAT_EQ(ranges[0], 0.21f);
}
