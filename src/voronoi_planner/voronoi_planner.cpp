/**
 * VoronoiPlanner node initialization and parameters routines.
 *
 * Lorenzo Bianchi <lnz.bnc@gmail.com>
 *
 * January 13, 2024
 */

/**
 * This is free software.
 * You can redistribute it and/or modify this file under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 3 of the License, or (at your option) any later
 * version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this file; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <voronoi_planner/voronoi_planner.hpp>

namespace VoronoiPlanner
{

double Line::point_distance = 0.0;
double Triangle::distance_thresh = 0.0;
float GeneralizedVoronoi::rdp_epsilon = 0.0;

/**
 * @brief VoronoiPlanner node constructor.
 *
 * @param node_opts Options for the base node.
 */
VoronoiPlannerNode::VoronoiPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("voronoi_planner", node_options)
{
  // Initialize parameters
  init_parameters();

  // Initialize topic publishers
  init_publishers();

  // Initialize timers
  init_timers();

  // Initialize TOPP-RA variables
  init_toppra();

  // Initialize static variables
  init_static_vars();

  //////////////////////////////////////////
  // Create and populate occupancy grid
  std::vector<int> grid_size = {int(field_size_[1] / grid_resolution_ + 1),
                                int(field_size_[0] / grid_resolution_ + 1),
                                int(field_size_[2] / grid_resolution_)};

  for (int i = 0; i < grid_size[2]; i++)
  {
    OccupancyGrid2D grid2D = OccupancyGrid2D(grid_size[0], grid_size[1]);
    grid2D.setZero();
    grid3D.push_back(grid2D);
  }

  std::mt19937 gen(seed_);
  std::uniform_int_distribution<int> rand_x(10, grid_size[1] - 10);
  std::uniform_int_distribution<int> rand_y(0, grid_size[0] - 1);
  std::uniform_int_distribution<int> rand_width(2, grid_size[1] / 8);
  std::uniform_int_distribution<int> rand_height(2, grid_size[0] / 4);
  std::uniform_int_distribution<int> rand_depth(2, grid_size[2]);

  int num_parallelepipeds = 30;
  for (int i = 0; i < num_parallelepipeds; i++)
  {
    int x = rand_x(gen);
    int y = rand_y(gen);
    int z = 0;
    int width = rand_width(gen);
    int height = rand_height(gen);
    int depth = rand_depth(gen);

    // insert parallelepiped in grid3D
    for (int k = z; k < z + depth; k++)
    {
      for (int i = y; i < std::min(y + height, grid_size[0]-1); i++)
      {
        for (int j = x; j < std::min(x + width, grid_size[1]-10); j++)
          grid3D[k](i, j) = true;
      }
    }
  }

  // delete all previous markers
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(marker);
  marker_pub_->publish(marker_array);

  ///////////////////////////////////////////////////////////
  compute_path();
  ///////////////////////////////////////////////////////////

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

/**
 * @brief VoronoiPlanner node destructor.
 */
VoronoiPlannerNode::~VoronoiPlannerNode()
{
  RCLCPP_INFO(this->get_logger(), "Node destroyed");
}

/**
 * @brief Routine to initialize parameters.
 */
void VoronoiPlannerNode::init_parameters()
{
  // Declare parameters
  this->declare_parameter("distance_thresh", rclcpp::ParameterValue(0.1));
  this->declare_parameter("distance_thresh_max_fails", rclcpp::ParameterValue(1));
  this->declare_parameter("field_size", rclcpp::ParameterValue(std::vector<double>{20.0, 10.0, 2.0}));
  this->declare_parameter("grid_resolution", rclcpp::ParameterValue(0.1));
  this->declare_parameter("layers_above", rclcpp::ParameterValue(1));
  this->declare_parameter("layers_graph_3d", rclcpp::ParameterValue(1));
  this->declare_parameter("layers_lower", rclcpp::ParameterValue(1));
  this->declare_parameter("layers_threshold", rclcpp::ParameterValue(0.1));
  this->declare_parameter("line_increase", rclcpp::ParameterValue(0.1));
  this->declare_parameter("max_acc", rclcpp::ParameterValue(1.0));
  this->declare_parameter("max_vel", rclcpp::ParameterValue(1.0));
  this->declare_parameter("move_coefficient", rclcpp::ParameterValue(0.1));
  this->declare_parameter("plot_voronoi", rclcpp::ParameterValue(false));
  this->declare_parameter("plot_size", rclcpp::ParameterValue(std::vector<int64_t>{1, 1}));
  this->declare_parameter("point_distance", rclcpp::ParameterValue(0.1));
  this->declare_parameter("points_thresh", rclcpp::ParameterValue(0.1));
  this->declare_parameter("rdp_epsilon_astar", rclcpp::ParameterValue(0.1));
  this->declare_parameter("rdp_epsilon_voronoi", rclcpp::ParameterValue(0.1));
  this->declare_parameter("robot_goal", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));
  this->declare_parameter("robot_start", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));
  this->declare_parameter("sample_points", rclcpp::ParameterValue(1));
  this->declare_parameter("save_yaml", rclcpp::ParameterValue(false));
  this->declare_parameter("save_yaml_path", rclcpp::ParameterValue(std::string("")));
  this->declare_parameter("save_log", rclcpp::ParameterValue(false));
  this->declare_parameter("seed", rclcpp::ParameterValue(0));
  this->declare_parameter("spline_bc_order", rclcpp::ParameterValue(1));
  this->declare_parameter("spline_bc_values", rclcpp::ParameterValue(std::vector<double>{0.0, 0.0, 0.0}));

  // Get parameters
  distance_thresh_ = this->get_parameter("distance_thresh").as_double();
  distance_thresh_max_fails_ = this->get_parameter("distance_thresh_max_fails").as_int();
  field_size_ = this->get_parameter("field_size").as_double_array();
  grid_resolution_ = this->get_parameter("grid_resolution").as_double();
  layers_above_ = this->get_parameter("layers_above").as_int();
  layers_graph_3d_ = this->get_parameter("layers_graph_3d").as_int();
  layers_lower_ = this->get_parameter("layers_lower").as_int();
  layers_threshold_ = this->get_parameter("layers_threshold").as_double();
  line_increase_ = this->get_parameter("line_increase").as_double();
  max_acc_ = this->get_parameter("max_acc").as_double();
  max_vel_ = this->get_parameter("max_vel").as_double();
  move_coefficient_ = this->get_parameter("move_coefficient").as_double();
  plot_voronoi_ = this->get_parameter("plot_voronoi").as_bool();
  plot_size_ = this->get_parameter("plot_size").as_integer_array();
  point_distance_ = this->get_parameter("point_distance").as_double();
  points_thresh_ = this->get_parameter("points_thresh").as_double();
  rdp_epsilon_astar_ = this->get_parameter("rdp_epsilon_astar").as_double();
  rdp_epsilon_voronoi_ = this->get_parameter("rdp_epsilon_voronoi").as_double();
  robot_goal_ = this->get_parameter("robot_goal").as_double_array();
  robot_start_ = this->get_parameter("robot_start").as_double_array();
  sample_points_ = this->get_parameter("sample_points").as_int();
  save_yaml_ = this->get_parameter("save_yaml").as_bool();
  save_yaml_path_ = this->get_parameter("save_yaml_path").as_string();
  save_log_ = this->get_parameter("save_log").as_bool();
  seed_ = this->get_parameter("seed").as_int();
  spline_bc_order_ = this->get_parameter("spline_bc_order").as_int();
  spline_bc_values_ = this->get_parameter("spline_bc_values").as_double_array();

  // Print parameters
  RCLCPP_INFO(this->get_logger(), "distance_thresh: %f", distance_thresh_);
  RCLCPP_INFO(this->get_logger(), "distance_thresh_max_fails: %ld", distance_thresh_max_fails_);
  RCLCPP_INFO(this->get_logger(), "field_size: [%f, %f, %f]", field_size_[0], field_size_[1], field_size_[2]);
  RCLCPP_INFO(this->get_logger(), "grid_resolution: %f", grid_resolution_);
  RCLCPP_INFO(this->get_logger(), "layers_above: %ld", layers_above_);
  RCLCPP_INFO(this->get_logger(), "layers_graph_3d: %ld", layers_graph_3d_);
  RCLCPP_INFO(this->get_logger(), "layers_lower: %ld", layers_lower_);
  RCLCPP_INFO(this->get_logger(), "layers_threshold: %f", layers_threshold_);
  RCLCPP_INFO(this->get_logger(), "line_increase: %f", line_increase_);
  RCLCPP_INFO(this->get_logger(), "max_acc: %f", max_acc_);
  RCLCPP_INFO(this->get_logger(), "max_vel: %f", max_vel_);
  RCLCPP_INFO(this->get_logger(), "move_coefficient: %f", move_coefficient_);
  RCLCPP_INFO(this->get_logger(), "plot_voronoi: %d", plot_voronoi_);
  RCLCPP_INFO(this->get_logger(), "plot_size: [%ld, %ld]", plot_size_[0], plot_size_[1]);
  RCLCPP_INFO(this->get_logger(), "point_distance: %f", point_distance_);
  RCLCPP_INFO(this->get_logger(), "points_thresh: %f", points_thresh_);
  RCLCPP_INFO(this->get_logger(), "rdp_epsilon_astar: %f", rdp_epsilon_astar_);
  RCLCPP_INFO(this->get_logger(), "rdp_epsilon_voronoi: %f", rdp_epsilon_voronoi_);
  RCLCPP_INFO(this->get_logger(), "robot_goal: [%f, %f, %f]", robot_goal_[0], robot_goal_[1], robot_goal_[2]);
  RCLCPP_INFO(this->get_logger(), "robot_start: [%f, %f, %f]", robot_start_[0], robot_start_[1], robot_start_[2]);
  RCLCPP_INFO(this->get_logger(), "sample_points: %ld", sample_points_);
  RCLCPP_INFO(this->get_logger(), "save_yaml: %d", save_yaml_);
  RCLCPP_INFO(this->get_logger(), "save_yaml_path: %s", save_yaml_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "save_log: %d", save_log_);
  RCLCPP_INFO(this->get_logger(), "seed: %ld", seed_);
  RCLCPP_INFO(this->get_logger(), "spline_bc_order: %ld", spline_bc_order_);
  RCLCPP_INFO(this->get_logger(), "spline_bc_values: [%f, %f, %f]", spline_bc_values_[0], spline_bc_values_[1], spline_bc_values_[2]);
}

/**
 * @brief Routine to initialize topic publishers.
 */
void VoronoiPlannerNode::init_publishers()
{
  // Marker array
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/test/obstacles",
    rclcpp::QoS(1));
}

/**
 * @brief Routine to initialize static variables.
 */
void VoronoiPlannerNode::init_static_vars()
{
  Line::point_distance = point_distance_;
  Triangle::distance_thresh = distance_thresh_;
  GeneralizedVoronoi::rdp_epsilon = rdp_epsilon_voronoi_;
}

/**
 * @brief Routine to initialize timers.
 */
void VoronoiPlannerNode::init_timers()
{
  visualization_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(5.0),
    std::bind(
        &VoronoiPlannerNode::visualization_timer_clbk,
        this));
}

/**
 * @brief Routine to initialize topic publishers.
 */
void VoronoiPlannerNode::init_toppra()
{
  toppra::Vector vel_limit_lower = -max_vel_ * toppra::Vector::Ones(3);
  toppra::Vector vel_limit_upper =  max_vel_ * toppra::Vector::Ones(3);
  toppra::Vector acc_limit_lower = -max_acc_ * toppra::Vector::Ones(3);
  toppra::Vector acc_limit_upper =  max_acc_ * toppra::Vector::Ones(3);

  toppra::LinearConstraintPtr ljv, lja;
  ljv = std::make_shared<toppra::constraint::LinearJointVelocity>(vel_limit_lower, vel_limit_upper);
  lja = std::make_shared<toppra::constraint::LinearJointAcceleration>(acc_limit_lower, acc_limit_upper);

  constraints = toppra::LinearConstraintPtrs{ljv, lja};
}

} // namespace VoronoiPlanner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(VoronoiPlanner::VoronoiPlannerNode)
