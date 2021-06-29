/**
 * 
 * This target following code is based on a simple algorithm of using the distance travelled by the target
 * in a second, as the predicted distance travelled in the next second.
 * The angle required to yaw is then based off cosine rule.
 * 
 * Expected to have a smooth yaw following but cannot accommodate to accelerations of target
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

int iris = 0, typhoon = 0;

double degree_of_error = 0.2;
double curr_roll = 0, curr_pitch = 0, curr_yaw = 0, yaw_rate = 0;
double last_callback_time = 0, time_elapsed = 0;
bool first_callback = true;

double target_prev_x = 0, target_prev_y = 0, target_curr_x = 0, target_curr_y = 0;
double follower_curr_x = 0, follower_curr_y = 0;
double pred_travelled_dist = 0, calc_first_target_dist = 0, calc_second_target_dist = 0;

double distance_betw_points(double first_x, double first_y, double second_x, double second_y){
   
    return sqrt(pow((first_x - second_x),2) + pow((first_y - second_y),2));

}

void calc_yaw_rate(gazebo_msgs::ModelStates current_pos){
    target_curr_x = current_pos.pose[iris].position.x;
    target_curr_y = current_pos.pose[iris].position.y;

    follower_curr_x = current_pos.pose[typhoon].position.x;
    follower_curr_y = current_pos.pose[typhoon].position.y;

    pred_travelled_dist = distance_betw_points(target_prev_x, target_prev_y, target_curr_x, target_curr_y);
    calc_first_target_dist = distance_betw_points(target_prev_x, target_prev_y, follower_curr_x, follower_curr_y);
    calc_second_target_dist = distance_betw_points(target_curr_x, target_curr_y, follower_curr_x, follower_curr_y);

    yaw_rate = acos(( pow(calc_first_target_dist,2) + pow(calc_second_target_dist,2) - pow(pred_travelled_dist,2)) / (2*calc_first_target_dist*calc_second_target_dist));

    if (target_curr_x - target_prev_x >= 0){
        // Moving to the right
        yaw_rate = -yaw_rate;
    }

    ROS_INFO("Dist travelled: %f Yaw-rate: %f", pred_travelled_dist, yaw_rate);
}

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

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void current_pos_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
    double callback_time = ros::Time::now().toSec();
    current_pos = *msg;

    if (!first_callback){
        time_elapsed = ros::Time::now().toSec() - last_callback_time;

        if (time_elapsed >= 1.0f){

            ROS_INFO("----- %f time elapsed -----", time_elapsed);
            
            calc_yaw_rate(current_pos);

            // Update current pose as the next previous pose
            target_prev_x = target_curr_x;
            target_prev_y = target_curr_y;

            last_callback_time = callback_time;
        }
    }

    else
    {
        // Gazebo models may not spawn in order
        get_model_order();

        // Initialize "first" pose of target
        target_prev_x = current_pos.pose[iris].position.x;
        target_prev_y = current_pos.pose[iris].position.y;

        first_callback = false;
    }

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
    pose.pose.position.z = current_pos.pose[typhoon].position.z;

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

        if (pred_travelled_dist >= 0){
            
            yaw.linear.x = 0;
            yaw.angular.z = yaw_rate;
        }
        else{
            yaw.linear.x = 0;
            yaw.angular.z = 0;
        }

        last_request = ros::Time::now();
        local_yaw_pub.publish(yaw);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}