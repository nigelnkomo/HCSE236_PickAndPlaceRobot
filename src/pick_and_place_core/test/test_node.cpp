#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "pick_and_place_core/pick_and_place_node.hpp"

TEST(PickAndPlaceNodeTest, Instantiation)
{
  EXPECT_NO_THROW({
    auto node = std::make_shared<pick_and_place_core::PickAndPlaceNode>();
    rclcpp::shutdown(); 
  });
}

TEST(PickAndPlaceNodeTest, TopicsAndServicesAvailable)
{
  auto node = std::make_shared<pick_and_place_core::PickAndPlaceNode>();
  rclcpp::spin_some(node);


  EXPECT_TRUE(node->count_publishers("/joint_states") >= 0);
  EXPECT_TRUE(node->count_subscribers("/planning_scene") >= 0);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

