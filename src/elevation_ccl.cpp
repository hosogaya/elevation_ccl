#include <elevation_ccl/elevation_ccl.h>

namespace elevation_ccl
{
ElevationCCL::ElevationCCL(const rclcpp::NodeOptions options) : rclcpp::Node("ccl", options)
{
    roughness_thres_ = declare_parameter("roughness_threshold", 0.1);
    slope_thres_ = declare_parameter("slope_threshold", 0.1);
    cell_num_thres_ = declare_parameter("cell_num_threshold", 10);
    //
    double distance_threshold = declare_parameter("distance_threshold", 0.1);
    ccl_solver_.setDistanceThres(distance_threshold);

    sub_grid_map_ = create_subscription<grid_map_msgs::msg::GridMap>(
        "elevation_ccl/input/grid_map", 1, std::bind(&ElevationCCL::callbackGridMap, this, std::placeholders::_1)
    );

    pub_grid_map_ = create_publisher<grid_map_msgs::msg::GridMap>(
        "elevation_ccl/output/grid_map", 1
    );
}

ElevationCCL::~ElevationCCL() {}

void ElevationCCL::callbackGridMap(const grid_map_msgs::msg::GridMap::UniquePtr msg)
{
    RCLCPP_INFO(get_logger(), "subscribe map address: 0x%x", &(msg->data));
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

    map.add("labels", label.cast<float>());
    int valid_label_num = 0;
    for (int i=1; i<=ccl_solver_.getRegionNum(); ++i)
    {
        if (ccl_solver_.getRegion(i).component_num_ >= cell_num_thres_) valid_label_num++;
    }

    // RCLCPP_INFO(get_logger(), "valid label num: %d", valid_label_num);
    // RCLCPP_INFO(get_logger(), "create valid_labels layer");
    map.add("valid_labels", NAN);
    map.add("valid_ground_shape", NAN);
    // if ( (label.rows() != map.getSize()[0]) || (label.cols() != map.getSize()[1]) )
    //     RCLCPP_ERROR(get_logger(), "matrix sizes are different");

    int valid_region_num = 0;
    for (const Region& r: ccl_solver_.getRegions())
    {
        if (r.component_num_ >= cell_num_thres_ && r.is_root_)
        {
            valid_region_num++;
            int label = r.label_;
            for (grid_map::GridMapIterator iter(map); !iter.isPastEnd(); ++iter)
            {
                if (map.at("labels", *iter) == label)
                {
                    map.at("valid_labels", *iter) = label;
                    map.at("valid_ground_shape", *iter) = map.at("elevation_smooth", *iter);
                }
            }
        }
    }
    
    // for (const Region& r: ccl_solver_.getRegions())
    // {
    //     if (r.is_root_)
    //     {
    //         std::cout << "label: " << r.label_ << " num: " << r.component_num_ << std::endl;
    //     }
    // }
    // RCLCPP_INFO(get_logger(), "valid region num: %d", valid_region_num);

    grid_map_msgs::msg::GridMap::UniquePtr pub_msg = grid_map::GridMapRosConverter::toMessage(map);
    RCLCPP_INFO(get_logger(), "publish map address: 0x%x", &(pub_msg->data));
    pub_grid_map_->publish(std::move(pub_msg));
}

void ElevationCCL::visualize(grid_map::GridMap& map, const ccl::LabelMatrix& label)
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
RCLCPP_COMPONENTS_REGISTER_NODE(elevation_ccl::ElevationCCL)