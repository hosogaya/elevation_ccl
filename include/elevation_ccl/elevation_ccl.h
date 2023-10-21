#pragma once

#include <rclcpp/rclcpp.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <elevation_ccl/ccl_solver.h>

namespace elevation_ccl {

using Layers = std::pair<int, grid_map::Matrix>;
class ElevationCCL: public rclcpp::Node
{
public:
    ElevationCCL(const rclcpp::NodeOptions options = rclcpp::NodeOptions().use_intra_process_comms(true));
    ~ElevationCCL();
private:
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_grid_map_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_;
    CclSolver ccl_solver_;

    void callbackGridMap(const grid_map_msgs::msg::GridMap::UniquePtr msg);
    void visualize(grid_map::GridMap& map, const ccl::LabelMatrix& label);

    double roughness_thres_ = 0.1;
    double slope_thres_ = 0.1;
    int cell_num_thres_ = 10;
};
}
