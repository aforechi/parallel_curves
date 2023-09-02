#include <parallel_curves/parallel_curves_ros.h>
#include <pluginlib/class_list_macros.hpp>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(parallel_curves::ParallelCurvesRos, nav_core::BaseGlobalPlanner)

namespace parallel_curves {

ParallelCurvesRos::~ParallelCurvesRos()
{
    if (collision_)
    {
        delete collision_;
        collision_ = nullptr;   
    }
}

ParallelCurvesRos::ParallelCurvesRos(): initialized_(false), costmap_(nullptr), collision_(nullptr),
parallel_curves::ParallelCurves()
{
}

ParallelCurvesRos::ParallelCurvesRos(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : initialized_(false), costmap_(nullptr), collision_(nullptr),
parallel_curves::ParallelCurves()
{
    initialize(name, costmap_ros);
}

ParallelCurvesRos::ParallelCurvesRos(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame) : initialized_(false), costmap_(nullptr), collision_(nullptr),
parallel_curves::ParallelCurves() 
{
    initialize(name, costmap, global_frame);
}

void ParallelCurvesRos::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
{
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void ParallelCurvesRos::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
{
    if(!initialized_)
    {
        costmap_ = costmap;
        global_frame_ = global_frame;
        collision_ = new CollisionDetector(costmap_);
        // initialize other planner parameters
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("cell_size", _cell_size, costmap_->getResolution());
        private_nh.param("min_dist_btw_nodes", _min_distance_between_nodes, 2 * costmap_->getResolution());
        private_nh.param("max_iter_inside_circle", _max_iterations_inside_the_circle, 200);
        private_nh.param("max_range", _max_range, std::hypot(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY()) );

        ROS_INFO("Parallel Curves Global Planner initialized successfully.");
        initialized_ = true;
    }
    else
    {
        ROS_WARN("This planner has already been initialized... doing nothing");
    }
}

bool ParallelCurvesRos::directPath(const Point& start, const Point& goal)
{
    return collision_->collisionFree(start, goal);
}

bool ParallelCurvesRos::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    if(!initialized_)
    {
        ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
        return false;
    }

    if(goal.header.frame_id != global_frame_)
    {
        ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
            global_frame_.c_str(), goal.header.frame_id.c_str());
        return false;
    }

    parallel_curves::ParallelCurves::Point start2D = {start.pose.position.x, start.pose.position.y};
    parallel_curves::ParallelCurves::Point goal2D = {goal.pose.position.x, goal.pose.position.y};

    //clear the starting cell within the costmap because we know it can't be an obstacle
    collision_->clearRobotCell(start2D);

    if (!collision_->collisionFree(start2D)) 
    {
        ROS_ERROR("Start point chosen is NOT in the FREE SPACE! Choose other goal!");
        return false;
    }

    if (!collision_->collisionFree(goal2D)) 
    {
        ROS_ERROR("Goal point chosen is NOT in the FREE SPACE! Choose other goal!");
        return false;
    }

    ROS_INFO("Parallel Curves Global Planner");
    ROS_INFO("Current Position: ( %.2lf, %.2lf)", start2D[0], start2D[1]);
    ROS_INFO("GOAL Position: ( %.2lf, %.2lf)", goal2D[0], goal2D[1]);

    auto path = this->plan(start2D, goal2D);

    if (path.size() < 2)
    {
        ROS_WARN("The planner failed to find a path, choose other goal position");
        return false;
    }

    plan.clear();
    for (const auto &point2D : path) 
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_;
        pose.pose.position.x = point2D[0];
        pose.pose.position.y = point2D[1];
        pose.pose.position.z = 0.0;
        plan.push_back(pose);
    }

    plan[0].pose.orientation = start.pose.orientation;
    for (int i=1; i < plan.size()-1; i++)
    {
        tf2::Quaternion waypoint_quat;
        waypoint_quat.setRPY(0, 0, atan2(plan[i].pose.position.y - plan[i-1].pose.position.y, 
                                         plan[i].pose.position.x - plan[i-1].pose.position.x));
        plan[i].pose.orientation.x = waypoint_quat.x();
        plan[i].pose.orientation.y = waypoint_quat.y();
        plan[i].pose.orientation.z = waypoint_quat.z();
        plan[i].pose.orientation.w = waypoint_quat.w();
    }
    plan[plan.size()-1].pose.orientation = goal.pose.orientation;

    return true;
}

}