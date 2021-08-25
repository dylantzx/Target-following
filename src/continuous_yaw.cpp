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
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <gazebo_msgs/ModelStates.h>
#include <mask_rcnn_ros/Bbox_values.h>
#include <iostream>
#include <math.h>
#include <tf/tf.h>

mavros_msgs::State current_state;
mavros_msgs::CommandBool arm_cmd;
mavros_msgs::SetMode offb_set_mode;

gazebo_msgs::ModelStates current_pos;
mask_rcnn_ros::Bbox_values cv;
geometry_msgs::PoseStamped pose;
geometry_msgs::Twist yaw;

ros::Subscriber state_sub;
ros::Subscriber current_pos_sub;
ros::Subscriber cv_sub;
ros::Publisher local_pos_pub;
ros::Publisher local_yaw_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

const std::string followerNS= "uav0"; // namespace of uav0
const std::string targetNS= "uav1"; // namespace of uav1
const double degree_of_error = 0.4; // 0.4 rad = error of 23 degrees 
const double callback_limit = 0.5; // take callback values every 0.5 seconds
const double catchup_yawRate = (3.141593/8)/ callback_limit; // 22.5 degrees per second
const double horizontal_fov = 1.0472; // fov in radians
const int total_hor_pix = 1280; // Based on gazebo image resolution 1280 x 720 px
const int centre_hor_pix = total_hor_pix/2; 

bool first_callback = true;

/* Initialize CV centres of target drone */
int prev_cv_centre_x = 0, prev_cv_centre_y = 0;
int curr_cv_centre_x = 0, curr_cv_centre_y = 0;

/* Initialize to get correct model based on gazebo model state */
int iris = 0, typhoon = 0; 

double curr_roll = 0 , curr_pitch = 0 , curr_yaw = 0;
double yaw_rate = 0;
double last_callback_time = 0;
double time_elapsed = 0;

/* Initialize previous and current positions of target and follower drones */
double target_prev_x = 0, target_prev_y = 0, target_prev_z = 0;
double target_curr_x = 0, target_curr_y = 0, target_curr_z = 0;
double follower_prev_x = 0, follower_prev_y = 0, follower_prev_z = 0;
double follower_curr_x = 0, follower_curr_y = 0, follower_curr_z = 0;

double target_dist = 0;
double radians_to_centre = 0;

void update_curr_pose(gazebo_msgs::ModelStates current_pos){
    target_curr_x = current_pos.pose[iris].position.x;
    target_curr_y = current_pos.pose[iris].position.y;

    follower_curr_x = current_pos.pose[typhoon].position.x;
    follower_curr_y = current_pos.pose[typhoon].position.y;
}

void update_prev_pose(){
    target_prev_x = target_curr_x;
    target_prev_y = target_curr_y;

    prev_cv_centre_x = curr_cv_centre_x;
    prev_cv_centre_y = curr_cv_centre_y;
}

double calc_dist(double p1_x, double p1_y, double p2_x, double p2_y){
   
    return sqrt(pow((p1_x - p2_x),2) + pow((p1_y - p2_y),2));

}

double calc_radians_difference(double x1, double x2){
    
    return (x1 - x2)/total_hor_pix * horizontal_fov;

}

void get_yaw_rate(){

    target_dist = calc_dist(target_curr_x, target_curr_y, follower_curr_x, follower_curr_y);

    yaw_rate = calc_radians_difference(curr_cv_centre_x, prev_cv_centre_x); // This is radians moved per callback limit 

    // Since the yaw_rate in rad/s required, we have to divide by the callback_limit
    yaw_rate = yaw_rate/callback_limit;

    yaw_rate = -yaw_rate; // Positive pixel difference means yaw clockwise (negative radians)

    ROS_INFO("target dist: %f Yaw-rate: %f", target_dist, yaw_rate);
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

void pos_control(){
    
}

void yaw_control(){

    yaw.linear.x = 0;
    
    if (radians_to_centre > degree_of_error){
        ROS_INFO("Catch up clockwise %f", radians_to_centre);
        yaw.angular.z = -catchup_yawRate;
    }
    else if (radians_to_centre < -degree_of_error){
        ROS_INFO("Catch up anti-clockwise %f", radians_to_centre);
        yaw.angular.z = catchup_yawRate;
    }
    else{
        ROS_INFO("Following target speed");
        yaw.angular.z = yaw_rate;
    }

    // pos_control();

    local_yaw_pub.publish(yaw);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void cv_cb(const mask_rcnn_ros::Bbox_values::ConstPtr& msg){

    double callback_time = ros::Time::now().toSec();
    cv = *msg;

    // Set current centre of target in pixels
    // Have to filter so that I do not set the curr cv pos as (0,0) if there are no tracked bbox
    if (cv.x != 0 && cv.y != 0){
        curr_cv_centre_x = int(cv.x + cv.w/2);
        curr_cv_centre_y = int (cv.y + cv.h/2);
    }

    radians_to_centre = calc_radians_difference(curr_cv_centre_x, centre_hor_pix);

    ROS_INFO("Radians to centre: %f", radians_to_centre);

    if (!first_callback){
        time_elapsed = ros::Time::now().toSec() - last_callback_time;

        if (time_elapsed >= callback_limit){

            ROS_INFO("--- %f time elapsed ---", time_elapsed);

            get_yaw_rate();

            ROS_INFO("Prev cv pose: (%d,%d)", prev_cv_centre_x, prev_cv_centre_y);
            ROS_INFO("Curr cv pose: (%d,%d)", curr_cv_centre_x, curr_cv_centre_y);

            // Use current pose as the next previous pose
            update_prev_pose();

            last_callback_time = callback_time;
        }
    }

    else
    {
        // Initialize "first" pose of target
        update_prev_pose();

        first_callback = false;
    }

}

void current_pos_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
    current_pos = *msg;

    // Gazebo coordinates
    update_curr_pose(current_pos);

    if (iris == 0 && typhoon == 0){
        // Gazebo models may not spawn in order
        get_model_order();
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Currently not using state_sub and service clients. They are left there for now in case needed in future

    /* SUBSCRIBERS */
    state_sub = nh.subscribe<mavros_msgs::State>(followerNS + "/mavros/state", 10, state_cb);

    current_pos_sub = nh.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 10, current_pos_cb);

    cv_sub = nh.subscribe<mask_rcnn_ros::Bbox_values>("bbox_output", 10, cv_cb);

    /* PUBLISHERS */
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(followerNS + "/mavros/setpoint_position/local", 10);

    local_yaw_pub = nh.advertise<geometry_msgs::Twist>(followerNS + "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

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

    int count = 0;

    while(ros::ok()){

        if(count == 0){
            ROS_INFO("Continuous Yaw Starting...");
            count++;
        }

        yaw_control();
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
