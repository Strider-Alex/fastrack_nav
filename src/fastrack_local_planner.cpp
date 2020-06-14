#include <pluginlib/class_list_macros.h>
#include <fastrack_nav/fastrack_local_planner.h>
//#include <Eigen/Core>
#include <cmath>
#include <ros/console.h>
#include <pluginlib/class_list_macros.h> 
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(fastrack_local_planner::FastrackLocalPlanner, nav_core::BaseLocalPlanner)

using namespace std;

//Default Constructor
namespace fastrack_local_planner
{

    FastrackLocalPlanner::FastrackLocalPlanner()
    {
    }

    void FastrackLocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {

    }

    bool FastrackLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        return true;
    }

   bool FastrackLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
   {
       return true;
   }

   bool FastrackLocalPlanner::isGoalReached()
   {
       return true;
   }
}; // namespace fastrack_local_planner
