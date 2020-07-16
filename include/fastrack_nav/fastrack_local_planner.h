/** include the libraries you need in your planner here */
/** for local path planner interface */
#include <ros/ros.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <nav_core/base_local_planner.h>
#include <pluginlib/class_loader.h>
#include <string>
#include <vector>

#include <fastrack/value/matlab_value_function.h>
#include <fastrack/control/Q4D_control_bound.h>
#include <fastrack/state/Q4D_rel_Q2D_state.h>
#include <fastrack/dynamics/Q4D_rel_Q2D.h>
#include <fastrack/utils/types.h>

#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

using CustomValueFunction = fastrack::value::MatlabValueFunction<Vector4d, Vector2d, 
   fastrack::control::Q4DControlBoundBox, Vector2d, Vector2d, fastrack::control::Q4DControlBoundBox,
   fastrack::state::Q4DRelQ2DState, fastrack::dynamics::Q4DRelQ2D, double>; //TODO: double is for testing. Shouldn't matter what type to use.

namespace fastrack_local_planner
{

   class FastrackLocalPlanner : public nav_core::BaseLocalPlanner
   {
   public:
      FastrackLocalPlanner();

      FastrackLocalPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

      virtual ~FastrackLocalPlanner() {}

      void initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros);

      bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

      bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

      bool isGoalReached();

      bool isInitialized()
      {
         return initialized_;
      }
   private:
      bool initialized_;
      CustomValueFunction value_function_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      int last_plan_point_;
      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_;
      const double goal_tolerance_ = 0.1;
      const double intermediate_tolerance_ = 0.5;
      int nextPoint(const int start_point, const tf::Stamped<tf::Pose> &pose) const;
   };
}; // namespace fastrack_local_planner
#endif