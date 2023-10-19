#include <elevation_ccl/elevation_ccl.h>

namespace ccl
{
ElevationCCL::ElevationCCL(const rclcpp::NodeOptions options) : rclcpp::Node("ccl", options)
{
    roughness_thres_ = declare_parameter("roughness_threshold", 0.1);
    slope_thres_ = declare_parameter("slope_threshold", 0.1);
    //
    double distance_threshold = declare_parameter("distance_threshold", 0.1);
    ccl_solver_.setDistanceThres(distance_threshold);

    sub_grid_map_ = create_subscription<grid_map_msgs::msg::GridMap>(
        "input/grid_map", 1, std::bind(&ElevationCCL::callbackGridMap, this, std::placeholders::_1)
    );

    pub_grid_map_ = create_publisher<grid_map_msgs::msg::GridMap>(
        "output/grid_map", 1
    );
}

ElevationCCL::~ElevationCCL() {}

void ElevationCCL::callbackGridMap(const grid_map_msgs::msg::GridMap::UniquePtr msg)
{
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map);

    // set state matrix
    ccl::StateMatrix states(map.getSize()[0]);
    for (int i=0; i<map.getSize()[0]; ++i) {
        states[i].resize(map.getSize()[1]);
        for (int j=0; j<map.getSize()[1]; ++j)
        {
            states[i][j].resize(3);
        }
    }
    
    const Eigen::MatrixXf& normal_x = map.get("normal_x");
    const Eigen::MatrixXf& normal_y = map.get("normal_y");
    const Eigen::MatrixXf& normal_z = map.get("normal_z");

    for (int i=0; i<map.getSize()[0]; ++i)
    {
        for (int j=0; j<map.getSize()[1]; ++j)
        {
            states[i][j].x() = normal_x(i, j);
            states[i][j].y() = normal_y(i, j);
            states[i][j].z() = normal_z(i, j);
        }
    }

    // set score matrix
    const grid_map::Matrix& roughness = map.get("roughness"); 
    const grid_map::Matrix& slope = map.get("slope"); 
    ScoreMatrix scores;
    scores.emplace_back(std::make_pair(roughness, roughness_thres_));
    scores.emplace_back(std::make_pair(slope, slope_thres_));

    // solve ccl
    ccl::Matrix label(slope.rows(), slope.cols());
    ccl_solver_.initialize(states, scores, label);
    ccl_solver_.firstScan();
    while (1)
    {
        if (ccl_solver_.backwardScan()) break;
        if (ccl_solver_.forwardScan()) break;
    }

    map.add("label", label);
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ccl::ElevationCCL)