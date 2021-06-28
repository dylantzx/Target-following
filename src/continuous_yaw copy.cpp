/**
 * @file continuous_yaw.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <math.h>
#include <tf/tf.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

gazebo_msgs::ModelStates current_pos;
double curr_roll, curr_pitch, curr_yaw;
void current_pos_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
    current_pos = *msg;
    
    tf::Quaternion q(
        current_pos.pose[1].orientation.x,
        current_pos.pose[1].orientation.x,
        current_pos.pose[1].orientation.z,
        current_pos.pose[1].orientation.w
    );
    tf::Matrix3x3 m(q);
    m.getRPY(curr_roll, curr_pitch, curr_yaw); 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    std::string followerNS= "uav0";
    std::string targetNS= "uav1";

    /* SUBSCRIBERS */
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            (followerNS + "/mavros/state", 10, state_cb);

    ros::Subscriber current_pos_sub = nh.subscribe<gazebo_msgs::ModelStates>
            ("gazebo/model_states", 10, current_pos_cb);

    /* PUBLISHERS */
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            (followerNS + "/mavros/setpoint_position/local", 10);

    ros::Publisher local_yaw_pub = nh.advertise<geometry_msgs::Twist>
            (followerNS + "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    /* SERVICE CLIENTS */
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (followerNS + "/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (followerNS + "/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    double f_curr_z = current_pos.pose[1].position.z;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = f_curr_z;

    geometry_msgs::Twist yaw;
    double degree_of_error = 0.1;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int count = 0;

    while(ros::ok()){

        if(count == 0){
            ROS_INFO("Continuous Yaw Starting...");
            count++;
        }

        double f_curr_x = current_pos.pose[1].position.x;
        double f_curr_y = current_pos.pose[1].position.y;
        f_curr_z = current_pos.pose[1].position.z;

        double t_curr_x = current_pos.pose[2].position.x;
        double t_curr_y = current_pos.pose[2].position.y;

        double diff_x = t_curr_x - f_curr_x;
        double diff_y = t_curr_y - f_curr_y;

        double angle_to_target = atan2 (diff_y, diff_x);

        ROS_INFO("Current yaw in radians is: %f", curr_yaw);

        double actual_angle_to_yaw = angle_to_target - curr_yaw;
        ROS_INFO("Angle to yaw in radians: %f", actual_angle_to_yaw);
        
        if (actual_angle_to_yaw > degree_of_error){
            yaw.linear.x = 0;
            yaw.angular.z = 3.14159/6;
        }
        else if (actual_angle_to_yaw < -degree_of_error){
            yaw.linear.x = 0;
            yaw.angular.z = -3.14159/6;
        }
        else{
            yaw.linear.x = 0;
            yaw.angular.z = 0;
        }

        pose.pose.position.z = f_curr_z;

        last_request = ros::Time::now();
        local_yaw_pub.publish(yaw);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
