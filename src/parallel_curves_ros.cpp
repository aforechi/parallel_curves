#include <parallel_curves/parallel_curves_ros.h>
#include <pluginlib/class_list_macros.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav2_core/exceptions.hpp>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(parallel_curves::ParallelCurvesRos, nav2_core::GlobalPlanner)

namespace parallel_curves {

ParallelCurvesRos::ParallelCurvesRos()
: tf_(nullptr), costmap_(nullptr)
{
}

ParallelCurvesRos::~ParallelCurvesRos()
{
    RCLCPP_INFO(logger_, "Destroying plugin %s of type ParallelCurves", name_.c_str());

    if (collision_)
    {
        delete collision_;
        collision_ = nullptr;   
    }
}

void ParallelCurvesRos::cleanup()
{
  RCLCPP_INFO(logger_, "CleaningUp plugin %s of type parallel_curves", name_.c_str());
}

void ParallelCurvesRos::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type parallel_curves", name_.c_str());
  auto node = parent_node_.lock();
}

void ParallelCurvesRos::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type parallel_curves", name_.c_str());
}

void ParallelCurvesRos::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    tf_ = tf;
    name_ = name;
    parent_node_ = parent;
    auto node = parent_node_.lock();

    logger_ = node->get_logger();
    clock_ = node->get_clock();
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
    collision_ = new CollisionDetector(costmap_);

    // initialize other planner parameters
    node->declare_parameter(name_ + ".cell_size", costmap_->getResolution());
    node->get_parameter(name_ + ".cell_size", _cell_size);
    node->declare_parameter(name_ + ".min_dist_btw_nodes", 2 * costmap_->getResolution());
    node->get_parameter(name_ + ".min_dist_btw_nodes", _min_distance_between_nodes);
    node->declare_parameter(name_ + ".max_iter_inside_circle", 200);
    node->get_parameter(name_ + ".max_iter_inside_circle", _max_iterations_inside_the_circle);
    node->declare_parameter(name_ + ".max_range", std::hypot(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY()) );
    node->get_parameter(name_ + ".max_range", _max_range);
    node->declare_parameter(name_ + ".line_splits_", 2);
    node->get_parameter(name_ + ".line_splits_", line_splits_);


    marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 1);

    RCLCPP_INFO(logger_, "Parallel Curves Global Planner initialized successfully.");
}

bool ParallelCurvesRos::directPath(const Point& start, const Point& goal)
{
    return collision_->collisionFree(start, goal);
}

nav_msgs::msg::Path ParallelCurvesRos::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
    nav_msgs::msg::Path global_path;

    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;

    Point start2D = {start.pose.position.x, start.pose.position.y};
    Point goal2D = {goal.pose.position.x, goal.pose.position.y};

    //clear the starting cell within the costmap because we know it can't be an obstacle
    collision_->clearRobotCell(start2D);

    if (!collision_->collisionFree(start2D)) 
    {
        throw nav2_core::PlannerException("Start Coordinates of(" + std::to_string(start.pose.position.x) + ", " +
            std::to_string(start.pose.position.y) + ") was in lethal cost");
    }

    if (!collision_->collisionFree(goal2D)) 
    {
        throw nav2_core::PlannerException("Goal Coordinates of(" + std::to_string(goal.pose.position.x) + ", " +
            std::to_string(goal.pose.position.y) + ") was in lethal cost");
    }

    RCLCPP_INFO(logger_, "Parallel Curves Global Planner");
    RCLCPP_INFO(logger_, "Current Position: ( %.2lf, %.2lf)", start2D[0], start2D[1]);
    RCLCPP_INFO(logger_, "GOAL Position: ( %.2lf, %.2lf)", goal2D[0], goal2D[1]);

    auto path = this->plan(start2D, goal2D);

    if (path.size() < 2)
    {
        throw nav2_core::PlannerException("The planner failed to find a path, choose other goal position");
    }

    // Split path segments
    for (uint p=0; p < path.size()-1; p++) 
    {
        double x_increment = (path[p+1][0] - path[p][0]) / static_cast<double>(line_splits_ + 1);
        double y_increment = (path[p+1][1] - path[p][1]) / static_cast<double>(line_splits_ + 1);

        for (int i = 0; i <= line_splits_; i++) 
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = global_path.header;
            pose.pose.position.x = path[p][0] + x_increment * i;
            pose.pose.position.y = path[p][1] + y_increment * i;
            pose.pose.position.z = 0.0;
            global_path.poses.push_back(pose);
        }
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = global_path.header;
    pose.pose.position.x = path.back()[0];
    pose.pose.position.y = path.back()[1];
    pose.pose.position.z = 0.0;
    global_path.poses.push_back(pose);

    // Fill path orientation
    global_path.poses[0].pose.orientation = start.pose.orientation;
    for (uint i=1; i < global_path.poses.size()-1; i++)
    {
        tf2::Quaternion waypoint_quat;
        waypoint_quat.setRPY(0, 0, atan2(global_path.poses[i].pose.position.y - global_path.poses[i-1].pose.position.y, 
                                         global_path.poses[i].pose.position.x - global_path.poses[i-1].pose.position.x));
        global_path.poses[i].pose.orientation.x = waypoint_quat.x();
        global_path.poses[i].pose.orientation.y = waypoint_quat.y();
        global_path.poses[i].pose.orientation.z = waypoint_quat.z();
        global_path.poses[i].pose.orientation.w = waypoint_quat.w();
    }
    global_path.poses.back().pose.orientation = goal.pose.orientation;

    visualizeCurves(this->radius(), this->isforward() ? goal2D : start2D);

    return global_path;
}

void ParallelCurvesRos::visualizeCurves(const std::vector<double> &radius, const Point &target)
{
    visualization_msgs::msg::MarkerArray ma;

    for(uint i=0; i<radius.size(); i++)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = global_frame_;
        marker.header.stamp = clock_->now();
        marker.ns = "parallel_curves";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = target[0];
        marker.pose.position.y = target[1];
        marker.pose.position.z = 0.001;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2.0 * radius[i];
        marker.scale.y = 2.0 * radius[i];
        marker.scale.z = 0.01;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.1;
        marker.lifetime = rclcpp::Duration::from_seconds(0);

        ma.markers.push_back(marker);
    }
    marker_pub->publish(ma);
}

}