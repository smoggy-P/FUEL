#ifndef TRAJECTORY_BRIDGE_H
#define TRAJECTORY_BRIDGE_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <agiros_msgs/Reference.h>
#include <agiros_msgs/Setpoint.h>
#include <agiros_msgs/QuadState.h>
#include <agiros_msgs/Command.h>
#include <tf/transform_datatypes.h>

#include <deque>
#include <vector>
#include <string>

namespace trajectory_bridge {

class TrajectoryBridge {
public:
    TrajectoryBridge(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~TrajectoryBridge() = default;

private:
    // ROS related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    // Publishers and subscribers
    ros::Subscriber pos_cmd_sub_;
    ros::Subscriber trajectory_sub_;
    ros::Publisher reference_pub_;
    ros::Publisher force_hover_pub_;

    // Parameters
    double trajectory_duration_;
    double sampling_rate_;
    std::string output_frame_id_;
    bool enable_jerk_snap_;
    
    // Internal state
    std::deque<quadrotor_msgs::PositionCommand> trajectory_buffer_;
    ros::Time last_trajectory_time_;
    
    // Callbacks
    void positionCommandCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    void trajectoryCallback(const nav_msgs::Path::ConstPtr& msg);
    
    // Helper functions
    agiros_msgs::Setpoint createSetpoint(const quadrotor_msgs::PositionCommand& pos_cmd, double timestamp);
    agiros_msgs::QuadState createQuadState(const quadrotor_msgs::PositionCommand& pos_cmd, double timestamp);
    agiros_msgs::Command createCommand(const quadrotor_msgs::PositionCommand& pos_cmd, double timestamp);
    geometry_msgs::Quaternion yawToQuaternion(double yaw);
    void publishReference();
    
    // Trajectory processing
    void processTrajectory();
    std::vector<agiros_msgs::Setpoint> interpolateTrajectory();
};

} // namespace trajectory_bridge

#endif // TRAJECTORY_BRIDGE_H 