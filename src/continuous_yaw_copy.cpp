#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <math.h>
#include <tf/tf.h>

mavros_msgs::State current_state;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::PositionTarget raw;
mavros_msgs::PositionTarget::_type_mask_type upper4bitsOn = 61440u;

gazebo_msgs::ModelStates current_pos;
geometry_msgs::PoseStamped pose;
geometry_msgs::Twist yaw;

ros::Subscriber state_sub;
ros::Subscriber current_pos_sub;
ros::Publisher local_pos_pub;
ros::Publisher local_yaw_pub;
ros::Publisher local_raw_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

const std::string followerNS= "uav0"; // namespace of uav0
const std::string targetNS= "uav1"; // namespace of uav1

int iris = 0, typhoon = 0;

/*PID controllers constants*/
const double Kp = 1.1, Ki = 0.5, Kd = 1.0;

const double pi = 3.14159;
const double degree_of_error = 1.0;
const double catchup_yawRate = 3.141593/3.2; // 36 degrees per second
const double callback_limit = 1.0;
const double locked_distance_btw = 15.0;
const double degree_of_error_dist = 5.0;
//const double locked_distance_btw_max = 20.0;

bool checkIfClockwise = false;
bool clockwise = true;
bool FirstCallback = true;

double proportional = 0, integrator = 0, derivative = 0;
double curr_roll = 0 , curr_pitch = 0 , curr_yaw = 0;
double last_callback_time = 0, time_elapsed = 0;

double fixed_cur_z;

double error_yaw;
double error_yaw_prev;
double linear_velocity;

double error_prev_z;
double error_curr_z;
double error_derivative_z;

double error_prev_x;
double error_curr_x;
double error_derivative_x;
// double pose_x;
// double pose_y;

/* Distance between the two vehicles */
double prev_distance_btw;
double cur_distance_btw;

/* Position of Vehicle 1 (aka Typhoon) */
double f_curr_x;
double f_curr_y;
double f_curr_z;
double f_prev_x;
double f_prev_y;
double f_prev_z;

/* Position of Vehicle 2 (aka Iris) */
double t_curr_x;
double t_curr_y;
double t_curr_z;
double t_prev_x;
double t_prev_y;
double t_prev_z;

double diff_t_x_prev;
double diff_t_y_prev;
double diff_t_x;
double diff_t_y;

/*Velocity of Vehicle 2 (aka IRIS)*/
double t_curr_vel_x;
double t_curr_vel_y;

/* Timestamps */
double cur_secs;
double prev_secs;


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double CalcDistance(double posX1, double posY1, double posZ1, double posX2, double posY2, double posZ2)
{
    double distance;
    distance = sqrt((posX1-posX2)*(posX1-posX2) + (posY1-posY2)*(posY1-posY2) + (posZ1-posZ2)*(posZ1-posZ2));
    return distance;
}

double CalcVelocity(double distance, double time)
{
    double velocity = distance/time;
    return velocity;
}

void CalcFollowingParam(gazebo_msgs::ModelStates current_pos) 
{
    ROS_INFO("\n");
    prev_distance_btw = cur_distance_btw;

    f_prev_z = f_curr_z;

    f_curr_x = current_pos.pose[typhoon].position.x;
    f_curr_y = current_pos.pose[typhoon].position.y;
    f_curr_z = current_pos.pose[typhoon].position.z;

    t_prev_x = t_curr_x;
    t_prev_y = t_curr_y;
    t_prev_z = t_curr_z;

    t_curr_x = current_pos.pose[iris].position.x;
    t_curr_y = current_pos.pose[iris].position.y;
    t_curr_z = current_pos.pose[iris].position.z;

    t_curr_vel_x = current_pos.twist[iris].linear.x;
    t_curr_vel_y = current_pos.twist[iris].linear.y;

    diff_t_x_prev = diff_t_x;
    diff_t_y_prev = diff_t_y;
    diff_t_x = t_curr_x - t_prev_x;
    diff_t_y = t_curr_y - t_prev_y;

    double dist_travelled_by_veh2 = CalcDistance(t_prev_x, t_prev_y, t_prev_z, t_curr_x, t_curr_y, t_curr_z);
    ROS_INFO("Distance travelled by vehicle 2 is: %f", dist_travelled_by_veh2);
    
    cur_distance_btw = CalcDistance(t_curr_x, t_curr_y, t_curr_z, f_curr_x, f_curr_y, f_curr_z);
    ROS_INFO("Current distance btw them is: %f", cur_distance_btw);
    ROS_INFO("Previous distance btw them is: %f", prev_distance_btw);
    ROS_INFO("diff_t_x is: %f diff_t_y is : %f", diff_t_x, diff_t_y);

    // use current distance btw them add predicted velocity to find the next distance.
    // using similar triangles, find the next pos x and pos y at that new distance
    // using similar triangle use same formula to get the new one. 
    if (cur_distance_btw >= (locked_distance_btw + degree_of_error_dist )|| cur_distance_btw <= locked_distance_btw)
    {
        error_curr_x = cur_distance_btw - locked_distance_btw;
        error_prev_x = prev_distance_btw - locked_distance_btw;
        error_derivative_x = (error_curr_x - error_prev_x)/1;
        linear_velocity = Kp*error_curr_x + Kd*error_derivative_x;
    }
    else
    {
        linear_velocity = 0;
    }

    error_yaw_prev = error_yaw;

    // only calculate angle moved per second 
    // when CV model is implemented. Error_yaw will be calculated with the center of the video as the fixed point and the distance btw the target drone to the center of the image will be the error.
    error_yaw = acos((pow(dist_travelled_by_veh2, 2) - pow(prev_distance_btw, 2) - pow(cur_distance_btw, 2))/((-2)*prev_distance_btw*cur_distance_btw));

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

/* Number 1 represent vehicle 1 and number 2 represent vehicle 2 */
void current_pos_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
    current_pos = *msg;
    double callback_time = ros::Time::now().toSec();
    if (!FirstCallback)
    {
        time_elapsed = ros::Time::now().toSec() - last_callback_time;

        if (time_elapsed >= callback_limit){
            tf::Quaternion q(
                current_pos.pose[typhoon].orientation.x,
                current_pos.pose[typhoon].orientation.x,
                current_pos.pose[typhoon].orientation.z,
                current_pos.pose[typhoon].orientation.w
            );
            tf::Matrix3x3 m(q);
            m.getRPY(curr_roll, curr_pitch, curr_yaw); 

            CalcFollowingParam(current_pos);
            last_callback_time = callback_time;
        }
    }
    else
    {
        get_model_order();
        FirstCallback = false;
    }
    
}

void yaw_control() 
{
    // this part is to solve the transition from -180 to 180 vice versa problem.
    // angle to yaw should not exceed -180 or 180 degrees
    double diff_x = t_curr_x - f_curr_x;
    double diff_y = t_curr_y - f_curr_y;
    double angle_to_targetPose = atan2(diff_y, diff_x);
    double angle_to_yaw = angle_to_targetPose - curr_yaw;
    
    //raw.IGNORE_AFX + raw.IGNORE_AFY + raw.IGNORE_AFZ + 
    
    if (angle_to_yaw > 3.14159){
        // scenario: -180 to 180 degrees
        // eg. angle_to_yaw should be negative (-0.043 rad) but received (6.24 rad)
        angle_to_yaw = angle_to_yaw - 6.28319; 
    } 
    else if (angle_to_yaw < -3.14159){
        // scenario: 180 to -180 degrees
        // eg. angle_to_yaw should be positive (0.043 rad) but received (-6.24 rad)
        angle_to_yaw = 6.28319 + angle_to_yaw;
    }

    ROS_INFO("Curr yaw: %f Angle to target: %f", curr_yaw, angle_to_yaw);

    if (angle_to_yaw > degree_of_error){
        yaw.linear.x = 0;
        yaw.angular.z = catchup_yawRate;
        ROS_INFO("Catch up + catch up");
    }
    else if (angle_to_yaw < -degree_of_error){
        yaw.linear.x = 0;
        yaw.angular.z = -catchup_yawRate;
        ROS_INFO("Catch up - catch up");
    }
    else{
        yaw.angular.z = error_yaw;
        ROS_INFO("Angle required to yaw: %f", error_yaw);
        ROS_INFO("Not Catch up");
    }
    
    local_yaw_pub.publish(yaw);
}

void yaw_control_pid()
{
    proportional = Kp * error_yaw;
    ROS_INFO("Proportional Yaw: %f", proportional);
    
    // Realise Integrator is not needed as we do not need an output to maintain the yaw (or like height or temperature)
    // Needs the derivative to ensure smooth slowing down.

    derivative = (error_yaw - error_yaw_prev)/1;
    ROS_INFO("Derivative Yaw: %f", derivative);

    double output = proportional + Kd*derivative;

    if(!checkIfClockwise || ((t_curr_vel_x < 0.01 && t_curr_vel_x > -0.01) && (t_curr_vel_y < 0.01 && t_curr_vel_y > -0.01))){
        checkIfClockwise = true;
        if (curr_yaw >= 0 && curr_yaw < pi/2){
            ROS_INFO("First Quadrant");
            if (diff_t_x >= 0 || diff_t_y < 0){
                clockwise = true;
                ROS_INFO("First Quadrant clockwise");
            }
            else 
            clockwise = false;
            
        }

        else if (curr_yaw >= pi/2 && curr_yaw < pi){
            ROS_INFO("Second Quadrant");
            if (diff_t_x >= 0 || diff_t_y >= 0){ 
                clockwise = true;
                ROS_INFO("Second Quadrant clockwise");
            }
            else
                clockwise = false;
            
        }

        else if (curr_yaw >= -pi && curr_yaw < -pi/2){
            ROS_INFO("Third Quadrant");
            if (diff_t_x < 0 || diff_t_y >= 0){
                clockwise = true;
                ROS_INFO("Third Quadrant clockwise");
            }
            else
                clockwise = false;
            
        }

        else {
            ROS_INFO("Fourth Quadrant");
            if (diff_t_x < 0){
                clockwise = true;
                ROS_INFO("Fourth Quadrant clockwise");
            }
            else
                clockwise = false;
            
        }
    }

    if (clockwise) {
        output = -output;
    }
    ROS_INFO("diff_t_x: %f diff_t_y: %f", diff_t_x, diff_t_y);
    ROS_INFO("Vel_x: %f Vel_y: %f", t_curr_vel_x, t_curr_vel_y);

    /*Account if the drone is out of the camera's view*/
    /*When incorporated with CV model, i believe it will be done differently*/
    // FIrst, we segregate the frames to check if the drone is moving left or right. 
    // Once, we determine its direction, we can determine yaw direction of the follower drone as well
    // Next, check if the target drone is within the camera's frames. If it has disappeared, check the prev frame
    // If prev frame shows that the target is at the edge of right side then yaw right by catchup_yawrate and vice versa. 
    // Below implementation is basically the prev step to account if the drone is out of camera view

    double diff_x = t_curr_x - f_curr_x;
    double diff_y = t_curr_y - f_curr_y;
    double angle_to_targetPose = atan2(diff_y, diff_x);
    double angle_to_yaw = angle_to_targetPose - curr_yaw;
    
    if (angle_to_yaw > 3.14159){
        // scenario: -180 to 180 degrees
        // eg. angle_to_yaw should be negative (-0.043 rad) but received (6.24 rad)
        angle_to_yaw = angle_to_yaw - 6.28319; 
    } 
    else if (angle_to_yaw < -3.14159){
        // scenario: 180 to -180 degrees
        // eg. angle_to_yaw should be positive (0.043 rad) but received (-6.24 rad)
        angle_to_yaw = 6.28319 + angle_to_yaw;
    }

    ROS_INFO("Curr yaw: %f Angle to target: %f", curr_yaw, angle_to_yaw);

    if (angle_to_yaw > degree_of_error){
        yaw.linear.x = 0;
        yaw.angular.z = catchup_yawRate;
        checkIfClockwise = false;
        ROS_INFO("Catch up + catch up");
    }
    else if (angle_to_yaw < -degree_of_error){
        yaw.linear.x = 0;
        yaw.angular.z = -catchup_yawRate;
        checkIfClockwise = false;
        ROS_INFO("Catch up - catch up");
    }
    else{
        yaw.linear.x = linear_velocity;
        yaw.angular.z = output;
        ROS_INFO("Angle required to yaw: %f", error_yaw);
        ROS_INFO("Angle velocity of yaw: %f", output);
        ROS_INFO("Linear velocity: %f", linear_velocity);
    }

    local_yaw_pub.publish(yaw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    //get_model_order();

    /* SUBSCRIBERS */
    state_sub = nh.subscribe<mavros_msgs::State>(followerNS + "/mavros/state", 10, state_cb);

    current_pos_sub = nh.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 10, current_pos_cb);

    /* PUBLISHERS */
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(followerNS + "/mavros/setpoint_position/local", 10);

    local_yaw_pub = nh.advertise<geometry_msgs::Twist>(followerNS + "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    local_raw_pub = nh.advertise<mavros_msgs::PositionTarget>(followerNS + "/mavros/setpoint_raw/local", 10);

    /* SERVICE CLIENTS */
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(followerNS + "/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(followerNS + "/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    //double f_curr_z = current_pos.pose[typhoon].position.z;
    fixed_cur_z = f_curr_z;
    pose.pose.position.z = fixed_cur_z;

    int count = 0;
    
    while(ros::ok()){

        if(count == 0){
            ROS_INFO("Continuous Yaw Starting!!!...");
            count++;
        }
        prev_secs = cur_secs;
        cur_secs = ros::Time::now().toSec();
        double time_taken = cur_secs - prev_secs;
        ROS_INFO("Time taken: %f", time_taken);
        
        yaw_control_pid();

        ros::spinOnce();
        ros::Duration(1.0).sleep(); // Sleep for a second
    }

    return 0;
}