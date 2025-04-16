// Copyright 2020 amsl

/**
 * @file dwa_planner.h // Corrected filename typo if necessary
 * @brief C++ implementation for dwa planner
 * @author AMSL
 */

#ifndef DWA_PLANNER_DWA_PLANNER_H
#define DWA_PLANNER_DWA_PLANNER_H

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64.h> // Needed for social cost message
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <utility>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <optional> // Using C++17 optional
#include <std_msgs/Int32.h> 

#include <Eigen/Dense>

/**
 * @class DWAPlanner
 * @brief A class implementing a local planner using the Dynamic Window Approach
 */
class DWAPlanner
{
public:
  /**
   * @brief Constructor for the DWAPlanner
   */
  DWAPlanner(void);

  /**
   * @brief Execute local path planning
   */
  void process(void);

  // --- Nested Classes ---

  /**
   * @class State
   * @brief A data class for state of robot
   */
  class State
  {
  public:
    State(void);
    State(const double x, const double y, const double yaw, const double velocity, const double yawrate);
    double x_;
    double y_;
    double yaw_;
    double velocity_;
    double yawrate_;
  }; // End class State

  /**
   * @class Window
   * @brief A data class for dynamic window
   */
  class Window
  {
  public:
    Window(void);
    void show(void);
    double min_velocity_;
    double max_velocity_;
    double min_yawrate_;
    double max_yawrate_;
  }; // End class Window

  /**
   * @class Cost
   * @brief A data class for cost
   */
  class Cost
  {
  public:
    Cost(void);
    /**
     * @brief Constructor
     * @param obs_cost The cost of obstacle
     * @param to_goal_cost The cost of distance to goal
     * @param speed_cost The cost of speed
     * @param path_cost The cost of path
     * @param social_cost The social cost // MODIFIED: Added social_cost param
     */
    // CORRECTED Constructor Declaration: Takes 5 components
    Cost(
        const float obs_cost, const float to_goal_cost, const float speed_cost, const float path_cost,
        const float social_cost); // REMOVED total_cost from args

    void show(void);
    void calc_total_cost(void);

    float obs_cost_;
    float to_goal_cost_;
    float speed_cost_;
    float path_cost_;
    int social_cost_; // ADDED: social_cost_ member
    float total_cost_;
  }; // End class Cost


// Using private for internal implementation details is generally preferred
private:
  // --- Callbacks ---
  void goal_callback(const geometry_msgs::PoseStampedConstPtr &msg);
  void scan_callback(const sensor_msgs::LaserScanConstPtr &msg);
  void local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg);
  void odom_callback(const nav_msgs::OdometryConstPtr &msg);
  void target_velocity_callback(const geometry_msgs::TwistConstPtr &msg);
  void footprint_callback(const geometry_msgs::PolygonStampedPtr &msg);
  void dist_to_goal_th_callback(const std_msgs::Float64ConstPtr &msg);
  void edge_on_global_path_callback(const nav_msgs::PathConstPtr &msg);
  // ADDED: social_callback declaration
  void social_callback(const std_msgs::Int32ConstPtr &msg);

  // --- Core DWA Functions ---
  std::vector<State> dwa_planning(const Eigen::Vector3d &goal, std::vector<std::pair<std::vector<State>, bool>> &trajectories);
  Window calc_dynamic_window(void);
  std::vector<State> generate_trajectory(const double velocity, const double yawrate);
  std::vector<State> generate_trajectory(const double yawrate, const Eigen::Vector3d &goal);
  Cost evaluate_trajectory(const std::vector<State> &trajectory, const Eigen::Vector3d &goal);
  void normalize_costs(std::vector<Cost> &costs);

  // --- Cost Calculation Functions ---
  float calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal);
  float calc_obs_cost(const std::vector<State> &traj);
  float calc_speed_cost(const std::vector<State> &traj);
  float calc_path_cost(const std::vector<State> &traj);
  float calc_dist_to_path(const State state);
  // ADDED: calc_social_cost declaration
  int calc_social_cost(const std::vector<State>& traj);

  // --- Helper Functions ---
  void load_params(void);
  void print_params(void);
  bool can_move(void);
  geometry_msgs::Twist calc_cmd_vel(void);
  bool can_adjust_robot_direction(const Eigen::Vector3d &goal);
  bool check_collision(const std::vector<State> &traj);
  void motion(State &state, const double velocity, const double yawrate);
  void create_obs_list(const sensor_msgs::LaserScan &scan);
  void create_obs_list(const nav_msgs::OccupancyGrid &map);
  geometry_msgs::PolygonStamped move_footprint(const State &target_pose);
  bool is_inside_of_robot(const geometry_msgs::Point &obstacle, const geometry_msgs::PolygonStamped &footprint, const State &state);
  bool is_inside_of_triangle(const geometry_msgs::Point &target_point, const geometry_msgs::Polygon &triangle);
  float calc_dist_from_robot(const geometry_msgs::Point &obstacle, const State &state);
  geometry_msgs::Point calc_intersection(const geometry_msgs::Point &obstacle, const State &state, geometry_msgs::PolygonStamped footprint);

  // --- Visualization ---
  void visualize_trajectory(const std::vector<State> &trajectory, const ros::Publisher &pub);
  void visualize_trajectories(const std::vector<std::pair<std::vector<State>, bool>> &trajectories, const ros::Publisher &pub);
  void visualize_footprints(const std::vector<State> &trajectory, const ros::Publisher &pub);
  visualization_msgs::Marker create_marker_msg(const int id, const double scale, const std_msgs::ColorRGBA color, const std::vector<State> &trajectory, const geometry_msgs::PolygonStamped &footprint = geometry_msgs::PolygonStamped());


  // --- ROS Communication ---
  ros::NodeHandle nh_;
  ros::NodeHandle local_nh_;
  ros::Publisher velocity_pub_;
  ros::Publisher candidate_trajectories_pub_;
  ros::Publisher selected_trajectory_pub_;
  ros::Publisher predict_footprints_pub_;
  ros::Publisher finish_flag_pub_;
  ros::Subscriber dist_to_goal_th_sub_;
  ros::Subscriber edge_on_global_path_sub_;
  ros::Subscriber footprint_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber local_map_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber target_velocity_sub_;
  // ADDED: social_sub_ declaration
  ros::Subscriber social_sub_;
  tf::TransformListener listener_;

  // --- Parameters ---
  std::string global_frame_;
  std::string robot_frame_;
  double hz_;
  double predict_time_;
  double sim_period_;
  int sim_time_samples_;
  double target_velocity_;
  double max_velocity_;
  double min_velocity_;
  double max_yawrate_;
  double min_yawrate_;
  double max_acceleration_;
  double max_deceleration_;
  double max_d_yawrate_;
  int velocity_samples_;
  int yawrate_samples_;
  double angle_resolution_;
  double obs_range_;
  double robot_radius_;
  double footprint_padding_;
  double dist_to_goal_th_;
  double turn_direction_th_;
  double angle_to_goal_th_;
  double slow_velocity_th_;
  double min_in_place_yawrate_;
  double max_in_place_yawrate_;
  double sleep_time_after_finish_;
  double v_path_width_;
  int subscribe_count_th_;
  bool use_scan_as_input_;
  bool use_footprint_;
  bool use_path_cost_;
  bool use_speed_cost_;
  // Cost Gains
  double obs_cost_gain_;
  double to_goal_cost_gain_;
  double speed_cost_gain_;
  double path_cost_gain_;
  // ADDED: social_cost_gain_ declaration (using double to match others, ensure consistency with cpp)
  double social_cost_gain_;
  double sim_direction_;


  // --- State Variables ---
  geometry_msgs::Twist current_cmd_vel_;
  std::optional<geometry_msgs::PoseStamped> goal_msg_;
  std::optional<geometry_msgs::PolygonStamped> footprint_;
  std::optional<nav_msgs::Path> edge_points_on_path_;
  geometry_msgs::PoseArray obs_list_;
  bool odom_updated_;
  bool local_map_updated_;
  bool scan_updated_;
  bool has_reached_;
  std_msgs::Bool has_finished_;
  int odom_not_subscribe_count_;
  int local_map_not_subscribe_count_;
  int scan_not_subscribe_count_;
  // ADDED: social_ declaration (using double for consistency, ensure consistency with cpp)
  int social_;

  // Deprecated/Unused? Check if these are still needed
  // double sim_direction_; // This was mentioned in generate_trajectory but not declared before

}; // End class DWAPlanner

#endif // DWA_PLANNER_DWA_PLANNER_H