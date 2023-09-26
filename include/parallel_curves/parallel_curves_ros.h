#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_core/global_planner.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <angles/angles.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <parallel_curves/parallel_curves.h>
#include <parallel_curves/collision_detector.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace parallel_curves
{
/**
* @class ParallelCurves
* @brief Provides a simple yet effective global planner that will compute a valid goal point for the local planner by finding tangent segments allong the parallel curves centered at the goal point.
*/
class ParallelCurvesRos : public nav2_core::GlobalPlanner, public parallel_curves::ParallelCurves 
{
public:

    ~ParallelCurvesRos();
 
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;

    void activate() override;

    void deactivate() override;

  /**
   * @brief Given a start and goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @return plan The plan... filled by the planner
   */
    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override;
protected:
    bool directPath(const Point& start, const Point& goal);
    void visualizeCurves(const std::vector<double> &radius, const Point &target);
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Logger logger_{rclcpp::get_logger("ParallelCurvesPlanner")};
private:
    CollisionDetector* collision_{nullptr};
    nav2_costmap_2d::Costmap2D* costmap_{nullptr};
    std::string global_frame_, name_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
};

}