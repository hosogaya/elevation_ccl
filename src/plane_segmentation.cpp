#include <plane_segmentation/plane_segmentation.h>

namespace plane_segmentation
{
PlaneSegmentation::PlaneSegmentation(const rclcpp::NodeOptions options) : rclcpp::Node("ccl", options)
{
    roughness_thres_ = declare_parameter("roughness_threshold", 0.1);
    slope_thres_ = declare_parameter("slope_threshold", 0.1);
    cell_num_thres_ = declare_parameter("cell_num_threshold", 10);
    //
    double distance_threshold = declare_parameter("distance_threshold", 0.1);
    ccl_solver_.setDistanceThres(distance_threshold);

    sub_grid_map_ = create_subscription<grid_map_msgs::msg::GridMap>(
        "input/grid_map", 1, std::bind(&PlaneSegmentation::callbackGridMap, this, std::placeholders::_1)
    );

    pub_grid_map_ = create_publisher<grid_map_msgs::msg::GridMap>(
        "output/grid_map", 1
    );
}

PlaneSegmentation::~PlaneSegmentation() {}

void PlaneSegmentation::callbackGridMap(const grid_map_msgs::msg::GridMap::UniquePtr msg)
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
    
    const Eigen::MatrixXf& normal_x = map.get("normal_vectors_x");
    const Eigen::MatrixXf& normal_y = map.get("normal_vectors_y");
    const Eigen::MatrixXf& normal_z = map.get("normal_vectors_z");

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
    ccl::ScoreMatrix scores;
    scores.emplace_back(std::make_pair(roughness, roughness_thres_));
    scores.emplace_back(std::make_pair(slope, slope_thres_));

    // solve ccl
    ccl::LabelMatrix label(slope.rows(), slope.cols());
    ccl_solver_.initialize(states, scores, label);
    ccl_solver_.firstScan();
    while (1)
    {
        if (!ccl_solver_.backwardScan()) break;
        if (!ccl_solver_.forwardScan()) break;
    }
    std::cout << "finish ccl" << std::endl;

    map.add("labels", label.cast<float>());
    map.add("labeled_ground_shape", map.get("elevation_smooth"));
    // visualize(map, label);
    for (int i=0; i<label.rows(); ++i)
        for (int j=0; j<label.cols(); ++j)
            if (label(i,j)==0) map.get("labeled_ground_shape").coeffRef(i,j) = NAN;

    grid_map_msgs::msg::GridMap::UniquePtr pub_msg = grid_map::GridMapRosConverter::toMessage(map);
    pub_grid_map_->publish(std::move(pub_msg));
}

void PlaneSegmentation::visualize(grid_map::GridMap& map, const ccl::LabelMatrix& label)
{
    std::vector<Layers> mat;
    for (int i=0; i<label.rows(); ++i)
    {
        for (int j=0; j<label.cols(); ++j)
        {
            if (label(i,j) == 0) continue;
            auto itr = std::find_if(mat.begin(), mat.end(), [i,j,label](const Layers& m){return m.first == label(i,j);});
            if (itr == mat.end())
            {
                if(ccl_solver_.getRegionRef(label(i,j)).component_num_ < cell_num_thres_) continue;
                mat.emplace_back(std::make_pair(label(i,j), grid_map::Matrix(map.getSize()[0], map.getSize()[1])));
                int ind = mat.size() -1;
                mat[ind].first = label(i,j);
                mat[ind].second.fill(NAN);
                mat[ind].second(i,j) = 1.0;
            }
            else 
            {
                itr->second(i, j) = 1.0;
            }
        }
    }
    RCLCPP_INFO(get_logger(), "vaild region num: %ld", mat.size());
    std::string layer;
    std::sort(mat.begin(), mat.end(), [](const Layers& m1, const Layers& m2){return m1.first < m2.first;});
    for (size_t i=0; i< 3; ++i)
    {
        if (mat.size() <=i) continue;
        layer = "label"+mat[i].first;
        map.add(layer, mat[i].second);
    }
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(plane_segmentation::PlaneSegmentation)