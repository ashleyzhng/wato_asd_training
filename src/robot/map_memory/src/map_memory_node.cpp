#include "map_memory_node.hpp"
#include <cmath>
#include <chrono>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Subscribe to robot odometry updates
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  // Subscribe to local costmap updates
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap",
      10,
      std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );
  
  // Check every second whether the global map should be updated
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&MapMemoryNode::updateMap, this)
  );
}

// Handles new costmap data
void MapMemoryNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // Store the latest costmap
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

// Handles new odometry data and tracks robot movement
void MapMemoryNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Store the latest odometry data
    latest_odom_ = *msg;

    // Get the robot's current position
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    // Calculate the distance from the last map update position
    double distance = std::sqrt(
        std::pow(current_x - last_update_x_, 2) +
        std::pow(current_y - last_update_y_, 2)
    );

    // Check if the robot has moved at least 5 meters
    if (distance >= 5.0) {
        last_update_x_ = current_x;
        last_update_y_ = current_y;
        should_update_map_ = true;
    }
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
