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
        : initialized_(false), odom_helper_("odom")
    {
    }

    FastrackLocalPlanner::FastrackLocalPlanner(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            this->initialize(name, tf, costmap_ros);
        }
    }

    void FastrackLocalPlanner::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        costmap_ros_ = costmap_ros;
        last_plan_point_ = 0;

        ros::NodeHandle private_nh("~/" + name);
        if (private_nh.getParam("odom_topic", odom_topic_))
        {
            odom_helper_.setOdomTopic(odom_topic_);
        }
        
        //value_function_.LoadParameters();
        bool ret = value_function_.InitializeFromMatFile("/home/kefan/fastrack_navigation_ws/src/fastrack_nav/value_function.mat"); //TODO: Use a better way to initialize
        if (!ret)
        {
            ROS_ERROR("LocalPlanner: failed to intialize from matlab file");
            return;
        }

        initialized_ = true;
    }

    bool FastrackLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        tf::Stamped<tf::Pose> current_pose;
        costmap_ros_->getRobotPose(current_pose);
        geometry_msgs::Pose current_pose_msg;
        tf::poseTFToMsg(current_pose, current_pose_msg);

        int plan_point = nextPoint(last_plan_point_, current_pose);
        last_plan_point_ = plan_point;
        geometry_msgs::PoseStamped plan_pose = global_plan_[plan_point];

        tf::Stamped<tf::Pose> robot_vel;
        odom_helper_.getRobotVel(robot_vel);
        //double v = std::sqrt(std::pow(robot_vel.getOrigin().x(),2)+std::pow(robot_vel.getOrigin().y(),2));
        double v = robot_vel.getOrigin().x();
        Vector4d tracker_state = Vector4d(current_pose_msg.position.x, current_pose_msg.position.y,
                                          v,
                                          tf::getYaw(current_pose_msg.orientation));
        Vector2d planner_state = Vector2d(plan_pose.pose.position.x, plan_pose.pose.position.y);
        //planner_state = Vector2d(0.5, 0.5);
        Vector2d control = value_function_.OptimalControl(tracker_state, planner_state);

        //ROS_INFO_STREAM("########################tracker_state:" << tracker_state << ", planner_state:" << planner_state << std::endl);
        //ROS_INFO_STREAM("########################global plan size:" << global_plan_.size() << std::endl);
        //ROS_INFO_STREAM("########################OPTIMAL CONTROL IS: " << control << std::endl);
        cmd_vel.linear.x = v + control(0) * 0.01; // Frequency of the planner is 100 Hz
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = control(1);
        //ROS_INFO_STREAM("########################CMD: " << cmd_vel << std::endl);
        return true;
    }

    bool FastrackLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        if (!isInitialized())
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        //reset the global plan
        global_plan_.clear();
        global_plan_ = orig_global_plan;
        last_plan_point_ = 0;
        return true;
    }

    bool FastrackLocalPlanner::isGoalReached()
    {
        tf::Stamped<tf::Pose> current_pose;
        costmap_ros_->getRobotPose(current_pose);
        double dist = base_local_planner::getGoalPositionDistance(current_pose,
                                                                  global_plan_.back().pose.position.x, global_plan_.back().pose.position.y);
        return dist < goal_tolerance_;
    }

    int FastrackLocalPlanner::nextPoint(const int start_point,
                                        const tf::Stamped<tf::Pose> &pose) const
    {
        double best_metric = std::numeric_limits<double>::max();
        double dist = base_local_planner::getGoalPositionDistance(pose,
                                                                  global_plan_[start_point].pose.position.x, global_plan_[start_point].pose.position.y);
        ROS_INFO_STREAM("########################Dist: " << dist << std::endl);
        if (dist < intermediate_tolerance_ && start_point < global_plan_.size() - 1)
        {
            return start_point + 1;
        }
        return start_point;
    }

}; // namespace fastrack_local_planner
