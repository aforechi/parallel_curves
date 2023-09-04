#pragma once

#include <string>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <parallel_curves/parallel_curves.h>
#include <parallel_curves/collision_detector.h>

namespace parallel_curves
{
/**
* @class ParallelCurves
* @brief Provides a simple yet effective global planner that will compute a valid goal point for the local planner by finding tangent segments allong the parallel curves centered at the goal point.
*/
class ParallelCurvesRos : public nav_core::BaseGlobalPlanner, public parallel_curves::ParallelCurves 
{
public:
    /**
    * @brief  Constructor
    */
    ParallelCurvesRos();
    
    /**
    * @brief  Destructor
    */
    ~ParallelCurvesRos();
 
    /**
    * @brief  Constructor for the ParallelCurves object
    * @param  name The name of this planner
    * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
    */
    ParallelCurvesRos(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
   
    /**
     * @brief  Constructor for the ParallelCurves object
     * @param  name The name of this planner
     * @param  costmap A pointer to the costmap to use
     * @param  global_frame The global frame of the costmap
     */
    ParallelCurvesRos(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

    /**
     * @brief  Initialization function for the ParallelCurves object
     * @param  name The name of this planner
     * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief  Initialization function for the ParallelCurves object
     * @param  name The name of this planner
     * @param  costmap A pointer to the costmap to use for planning
     * @param  global_frame The global frame of the costmap
     */
    void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame);

  /**
   * @brief Given a start and goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
    bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
protected:
    bool directPath(const Point& start, const Point& goal);
    void visualizeCurves(const std::vector<double> &radius, const Point &target);
private:
    CollisionDetector* collision_{nullptr};
    costmap_2d::Costmap2D* costmap_{nullptr};
    bool initialized_{false};
    std::string global_frame_;
    ros::Publisher marker_pub;
};

}