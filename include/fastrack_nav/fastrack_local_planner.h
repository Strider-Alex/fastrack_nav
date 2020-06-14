/** include the libraries you need in your planner here */
/** for local path planner interface */
#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <pluginlib/class_loader.h>
#include <string>
#include <vector>

#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

namespace fastrack_local_planner
{

   class FastrackLocalPlanner : public nav_core::BaseLocalPlanner
   {
   public:
      FastrackLocalPlanner();

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
   };
}; // namespace fastrack_local_planner
#endif