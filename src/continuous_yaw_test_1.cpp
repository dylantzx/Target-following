/**
 * 
 * This target following code is based on a simple algorithm of calculating the angle required to yaw
 * then yaw at a constant speed. 
 * 
 * Expected to be very jerk-ish and may not accommodate to different velocities of target
 * 
 * Values are currently based off gazebo values 
 * 
 * How to use code:
 * 1) After compiling with catkin build
 * 2) source setup.bash file in catkin_ws/devel
 * 3) rosrun continuous_yaw continuous_yaw (Change file name accordingly whereever required)
 * 4) Change vehicle 1 in QGC to offboard mode
 * 
 **/

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
gazebo_msgs::ModelStates current_pos;

int iris = 0;
int typhoon = 0;

double curr_roll, curr_pitch, curr_yaw;
double degree_of_error = 0.1;

void get_model_order(){
    ROS_INFO("Model order: (1) %s (2) %s", current_pos.name[1].c_str(), current_pos.name[2].c_str());

    if (strcmp(current_pos.name[1].c_str(),"iris1") == 0){
        iris = 1;
        typhoon = 2;
    }
    else{
        iris = 2;
        typhoon = 1;
    }

    ROS_INFO("Iris: %d Typhoon: %d", iris, typhoon);
}

void quat_to_euler(gazebo_msgs::ModelStates current_pos){
    tf::Quaternion q(
        current_pos.pose[typhoon].orientation.x,
        current_pos.pose[typhoon].orientation.x,
        current_pos.pose[typhoon].orientation.z,
        current_pos.pose[typhoon].orientation.w
    );
    tf::Matrix3x3 m(q);
    m.getRPY(curr_roll, curr_pitch, curr_yaw); 
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void current_pos_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
    current_pos = *msg;
    
    // Gazebo models may not spawn in order
    get_model_order();

    // Converting quaternion values to roll, pitch and yaw and sets current yaw direction
    // Note that angles are from -180 to 180 degrees
    quat_to_euler(current_pos);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    std::string followerNS= "uav0";
    std::string targetNS= "uav1";

    // Currently not using state_sub and service clients. They are left there for now in case needed in future

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

    geometry_msgs::PoseStamped pose;
    double f_curr_z = current_pos.pose[typhoon].position.z;
    pose.pose.position.z = f_curr_z;

    geometry_msgs::Twist yaw;

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

        double f_curr_x = current_pos.pose[typhoon].position.x;
        double f_curr_y = current_pos.pose[typhoon].position.y;
        f_curr_z = current_pos.pose[typhoon].position.z;

        double t_curr_x = current_pos.pose[iris].position.x;
        double t_curr_y = current_pos.pose[iris].position.y;

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
