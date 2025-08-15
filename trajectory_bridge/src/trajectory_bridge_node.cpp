#include "trajectory_bridge/trajectory_bridge.h"

namespace trajectory_bridge {

TrajectoryBridge::TrajectoryBridge(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
    
    // Load parameters
    nh_private_.param("trajectory_duration", trajectory_duration_, 10.0);
    nh_private_.param("sampling_rate", sampling_rate_, 1.0); // Hz
    nh_private_.param("output_frame_id", output_frame_id_, std::string("world"));
    nh_private_.param("enable_jerk_snap", enable_jerk_snap_, false);
    
    // Initialize subscribers
    pos_cmd_sub_ = nh_.subscribe("planning/pos_cmd", 100, 
                                 &TrajectoryBridge::positionCommandCallback, this);
    trajectory_sub_ = nh_.subscribe("planning/trajectory", 10, 
                                   &TrajectoryBridge::trajectoryCallback, this);
    
    // Initialize publishers
    reference_pub_ = nh_.advertise<agiros_msgs::Reference>("/kingfisher/agiros_pilot/trajectory", 10);
    force_hover_pub_ = nh_.advertise<std_msgs::Empty>("/kingfisher/agiros_pilot/force_hover", 10);
    ROS_INFO("Trajectory Bridge initialized");
    ROS_INFO("Subscribing to: planning/pos_cmd, planning/trajectory");
    ROS_INFO("Publishing to: /agilicious_trajectory");
    ROS_INFO("Trajectory duration: %.2f s", trajectory_duration_);
    ROS_INFO("Sampling rate: %.1f Hz", sampling_rate_);
}

void TrajectoryBridge::positionCommandCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
    // Store the position command in buffer
    trajectory_buffer_.push_back(*msg);
    
    // Keep only recent commands within trajectory duration
    ros::Time current_time = ros::Time::now();
    while (!trajectory_buffer_.empty() && 
           (current_time - trajectory_buffer_.front().header.stamp).toSec() > trajectory_duration_) {
        trajectory_buffer_.pop_front();
    }
    
    // Process trajectory if we have enough data
    if (trajectory_buffer_.size() > 1) {
        processTrajectory();
    }
}

void TrajectoryBridge::trajectoryCallback(const nav_msgs::Path::ConstPtr& msg) {
    // Convert path to position commands and process
    trajectory_buffer_.clear();
    
    for (const auto& pose : msg->poses) {
        quadrotor_msgs::PositionCommand pos_cmd;
        pos_cmd.header = pose.header;
        pos_cmd.position = pose.pose.position;
        pos_cmd.velocity = geometry_msgs::Vector3(); // Will be computed if needed
        pos_cmd.acceleration = geometry_msgs::Vector3(); // Will be computed if needed
        pos_cmd.yaw = tf::getYaw(pose.pose.orientation);
        pos_cmd.yaw_dot = 0.0;
        
        trajectory_buffer_.push_back(pos_cmd);
    }
    
    if (trajectory_buffer_.size() > 1) {
        processTrajectory();
    }
}

void TrajectoryBridge::processTrajectory() {
    if (trajectory_buffer_.size() < 2) {
        return;
    }
    
    // Create interpolated trajectory
    auto setpoints = interpolateTrajectory();
    
    if (setpoints.empty()) {
        return;
    }
    
    // Create and publish reference message
    agiros_msgs::Reference reference_msg;
    reference_msg.header.stamp = ros::Time::now();
    reference_msg.header.frame_id = output_frame_id_;
    reference_msg.points = setpoints;

    // Publish empty message to force_hover
    // std_msgs::Empty empty_msg;
    // force_hover_pub_.publish(empty_msg);
    
    reference_pub_.publish(reference_msg);
    
    ROS_DEBUG("Published reference trajectory with %zu setpoints", setpoints.size());
}

std::vector<agiros_msgs::Setpoint> TrajectoryBridge::interpolateTrajectory() {
    std::vector<agiros_msgs::Setpoint> setpoints;
    
    if (trajectory_buffer_.size() < 2) {
        return setpoints;
    }
    
    // Calculate time step based on sampling rate
    double dt = 1.0 / sampling_rate_;
    
    // Get trajectory start and end times
    ros::Time start_time = trajectory_buffer_.front().header.stamp;
    ros::Time end_time = trajectory_buffer_.back().header.stamp;
    double total_duration = (end_time - start_time).toSec();
    
    // Generate setpoints at regular intervals
    for (double t = 0.0; t <= total_duration; t += dt) {
        ros::Time current_time = start_time + ros::Duration(t);
        
        // Find the two closest trajectory points for interpolation
        size_t idx = 0;
        for (size_t i = 0; i < trajectory_buffer_.size() - 1; ++i) {
            if ((current_time - trajectory_buffer_[i].header.stamp).toSec() >= 0 &&
                (current_time - trajectory_buffer_[i + 1].header.stamp).toSec() <= 0) {
                idx = i;
                break;
            }
        }
        
        if (idx >= trajectory_buffer_.size() - 1) {
            idx = trajectory_buffer_.size() - 2;
        }
        
        // Interpolate between two points
        const auto& p1 = trajectory_buffer_[idx];
        const auto& p2 = trajectory_buffer_[idx + 1];
        
        double t1 = (p1.header.stamp - start_time).toSec();
        double t2 = (p2.header.stamp - start_time).toSec();
        double alpha = (t - t1) / (t2 - t1);
        alpha = std::max(0.0, std::min(1.0, alpha)); // Clamp to [0, 1]
        
        // Create interpolated position command
        quadrotor_msgs::PositionCommand interpolated_cmd;
        interpolated_cmd.header.stamp = current_time;
        interpolated_cmd.position.x = p1.position.x + alpha * (p2.position.x - p1.position.x);
        interpolated_cmd.position.y = p1.position.y + alpha * (p2.position.y - p1.position.y);
        interpolated_cmd.position.z = p1.position.z + alpha * (p2.position.z - p1.position.z);
        interpolated_cmd.velocity.x = p1.velocity.x + alpha * (p2.velocity.x - p1.velocity.x);
        interpolated_cmd.velocity.y = p1.velocity.y + alpha * (p2.velocity.y - p1.velocity.y);
        interpolated_cmd.velocity.z = p1.velocity.z + alpha * (p2.velocity.z - p1.velocity.z);
        interpolated_cmd.acceleration.x = p1.acceleration.x + alpha * (p2.acceleration.x - p1.acceleration.x);
        interpolated_cmd.acceleration.y = p1.acceleration.y + alpha * (p2.acceleration.y - p1.acceleration.y);
        interpolated_cmd.acceleration.z = p1.acceleration.z + alpha * (p2.acceleration.z - p1.acceleration.z);
        interpolated_cmd.yaw = p1.yaw + alpha * (p2.yaw - p1.yaw);
        interpolated_cmd.yaw_dot = p1.yaw_dot + alpha * (p2.yaw_dot - p1.yaw_dot);
        
        // Create setpoint
        agiros_msgs::Setpoint setpoint = createSetpoint(interpolated_cmd, t);
        setpoints.push_back(setpoint);
    }
    
    return setpoints;
}

agiros_msgs::Setpoint TrajectoryBridge::createSetpoint(const quadrotor_msgs::PositionCommand& pos_cmd, double timestamp) {
    agiros_msgs::Setpoint setpoint;
    
    // Create quad state
    setpoint.state = createQuadState(pos_cmd, timestamp);
    
    // Create command
    // setpoint.command = createCommand(pos_cmd, timestamp);
    
    return setpoint;
}

agiros_msgs::QuadState TrajectoryBridge::createQuadState(const quadrotor_msgs::PositionCommand& pos_cmd, double timestamp) {
    agiros_msgs::QuadState quad_state;
    
    quad_state.header.stamp = pos_cmd.header.stamp;
    quad_state.header.frame_id = output_frame_id_;
    quad_state.t = timestamp;
    
    // Set pose
    quad_state.pose.position = pos_cmd.position;
    quad_state.pose.orientation = yawToQuaternion(pos_cmd.yaw);
    
    // Set velocity
    quad_state.velocity.linear = pos_cmd.velocity;
    quad_state.velocity.angular.x = 0.0;
    quad_state.velocity.angular.y = 0.0;
    quad_state.velocity.angular.z = pos_cmd.yaw_dot;
    
    // Set acceleration
    quad_state.acceleration.linear = pos_cmd.acceleration;
    quad_state.acceleration.angular.x = 0.0;
    quad_state.acceleration.angular.y = 0.0;
    quad_state.acceleration.angular.z = 0.0;
    
    // Set bias (set to zero)
    quad_state.acc_bias.x = 0.0;
    quad_state.acc_bias.y = 0.0;
    quad_state.acc_bias.z = 0.0;
    quad_state.gyr_bias.x = 0.0;
    quad_state.gyr_bias.y = 0.0;
    quad_state.gyr_bias.z = 0.0;
    
    // Set jerk and snap if enabled
    if (enable_jerk_snap_) {
        // These would need to be computed from velocity/acceleration differences
        // For now, set to zero
        quad_state.jerk.x = 0.0;
        quad_state.jerk.y = 0.0;
        quad_state.jerk.z = 0.0;
        quad_state.snap.x = 0.0;
        quad_state.snap.y = 0.0;
        quad_state.snap.z = 0.0;
    } else {
        quad_state.jerk.x = 0.0;
        quad_state.jerk.y = 0.0;
        quad_state.jerk.z = 0.0;
        quad_state.snap.x = 0.0;
        quad_state.snap.y = 0.0;
        quad_state.snap.z = 0.0;
    }
    
    // Set motor speeds (not available in PositionCommand, set to default)
    quad_state.motors.push_back(0.0);
    quad_state.motors.push_back(0.0);
    quad_state.motors.push_back(0.0);
    quad_state.motors.push_back(0.0);
    
    return quad_state;
}

agiros_msgs::Command TrajectoryBridge::createCommand(const quadrotor_msgs::PositionCommand& pos_cmd, double timestamp) {
    agiros_msgs::Command command;
    
    command.header.stamp = pos_cmd.header.stamp;
    command.t = timestamp;
    
    // Use collective thrust and body rates mode
    command.is_single_rotor_thrust = false;
    
    // Calculate collective thrust from acceleration (assuming mass = 1 kg)
    double g = 9.81;
    double collective_thrust = pos_cmd.acceleration.z + g;
    command.collective_thrust = collective_thrust;
    
    // Set body rates (yaw rate from PositionCommand, roll/pitch rates set to 0)
    command.bodyrates.x = 0.0;
    command.bodyrates.y = 0.0;
    command.bodyrates.z = pos_cmd.yaw_dot;
    
    // Set single rotor thrusts to zero (not used in this mode)
    command.thrusts[0] = 0.0;
    command.thrusts[1] = 0.0;
    command.thrusts[2] = 0.0;
    command.thrusts[3] = 0.0;
    
    return command;
}

geometry_msgs::Quaternion TrajectoryBridge::yawToQuaternion(double yaw) {
    geometry_msgs::Quaternion quat;
    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = sin(yaw / 2.0);
    quat.w = cos(yaw / 2.0);
    return quat;
}

} // namespace trajectory_bridge

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_bridge");
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    trajectory_bridge::TrajectoryBridge bridge(nh, nh_private);
    
    ros::spin();
    
    return 0;
} 