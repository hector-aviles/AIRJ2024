#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    current_speed = 0;
    current_steering = 0;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{
    pub_speed       = n->advertise<std_msgs::Float64>("/speed"   ,10);
    pub_steering    = n->advertise<std_msgs::Float64>("/steering",10);
    pub_start       = n->advertise<std_msgs::Empty>("/policy_started" ,10);
    pub_change_left = n->advertise<std_msgs::Bool> ("/start_change_lane_on_left",10);
    pub_cruise      = n->advertise<std_msgs::Bool> ("/cruise/enable",10);
    pub_follow      = n->advertise<std_msgs::Bool> ("/follow/enable",10);
    pub_change_right= n->advertise<std_msgs::Bool> ("/start_change_lane_on_right",10);
    pub_speed_cars_left  = n->advertise<std_msgs::Float64>("/speed_cars_left_lane", 1);
    pub_speed_cars_right = n->advertise<std_msgs::Float64>("/speed_cars_right_lane",1);
    pub_action      = n->advertise<std_msgs::String>("/action", 1);
    ros::Rate loop(30);
    
    while(ros::ok() && !this->gui_closed)
    {
        ros::spinOnce();
        emit updateGraphics();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;   
}

void QtRosNode::publish_speed(double speed)
{
    current_speed = speed;
    std_msgs::Float64 msg;
    msg.data = speed;
    pub_speed.publish(msg);
}

void QtRosNode::publish_steering(double steering)
{
    current_steering = steering;
    std_msgs::Float64 msg;
    msg.data = steering;
    pub_steering.publish(msg);
}

void QtRosNode::update_speed_and_publish(double delta_speed)
{
    current_speed += delta_speed;
    if(current_speed >  50.0) current_speed =  50.0;
    if(current_speed < -20.0) current_speed = -20.0;
    std_msgs::Float64 msg;
    msg.data = current_speed;
    pub_speed.publish(msg);
}

void QtRosNode::update_steering_and_publish(double delta_steering)
{
    current_steering += delta_steering;
    if(current_steering >  0.5) current_steering =  0.5;
    if(current_steering < -0.5) current_steering = -0.5;
    std_msgs::Float64 msg;
    msg.data = current_steering;
    pub_steering.publish(msg);
}

void QtRosNode::publish_start(double speed_cars_left, double speed_cars_right)
{
    std_msgs::Empty msg;
    std_msgs::Float64 msg_speeds_left;
    std_msgs::Float64 msg_speeds_right;
    msg_speeds_left.data = speed_cars_left;
    msg_speeds_right.data = speed_cars_right;
    pub_start.publish(msg);
    pub_speed_cars_left.publish(msg_speeds_left);
    pub_speed_cars_right.publish(msg_speeds_right);
}

void QtRosNode::publish_change_left()
{
    std_msgs::Bool msg_change_left;
    std_msgs::Bool msg_cruise;      
    std_msgs::Bool msg_follow;      
    std_msgs::Bool msg_change_right;
    msg_change_left.data = true;
    msg_cruise.data = false;
    msg_follow.data = false;
    msg_change_right.data = false;
    pub_change_left.publish(msg_change_left);
    pub_cruise.publish(msg_cruise);
    pub_follow.publish(msg_follow);
    pub_change_right.publish(msg_change_right);
    std_msgs::String msg_action;
    msg_action.data = "change_left";
    pub_action.publish(msg_action);
}

void QtRosNode::publish_cruise()
{
    std_msgs::Bool msg_change_left;
    std_msgs::Bool msg_cruise;      
    std_msgs::Bool msg_follow;      
    std_msgs::Bool msg_change_right;
    msg_change_left.data = false;
    msg_cruise.data = true;
    msg_follow.data = false;
    msg_change_right.data = false;
    pub_change_left.publish(msg_change_left);
    pub_cruise.publish(msg_cruise);
    pub_follow.publish(msg_follow);
    pub_change_right.publish(msg_change_right);
    std_msgs::String msg_action;
    msg_action.data = "cruise";
    pub_action.publish(msg_action);
}

void QtRosNode::publish_follow()
{
    std_msgs::Bool msg_change_left;
    std_msgs::Bool msg_cruise;      
    std_msgs::Bool msg_follow;      
    std_msgs::Bool msg_change_right;
    msg_change_left.data = false;
    msg_cruise.data = false;
    msg_follow.data = true;
    msg_change_right.data = false;
    pub_change_left.publish(msg_change_left);
    pub_cruise.publish(msg_cruise);
    pub_follow.publish(msg_follow);
    pub_change_right.publish(msg_change_right);
    std_msgs::String msg_action;
    msg_action.data = "follow";
    pub_action.publish(msg_action);
}

void QtRosNode::publish_change_right()
{
    std_msgs::Bool msg_change_left;
    std_msgs::Bool msg_cruise;      
    std_msgs::Bool msg_follow;      
    std_msgs::Bool msg_change_right;
    msg_change_left.data = false;
    msg_cruise.data = false;
    msg_follow.data = false;
    msg_change_right.data = true;
    pub_change_left.publish(msg_change_left);
    pub_cruise.publish(msg_cruise);
    pub_follow.publish(msg_follow);
    pub_change_right.publish(msg_change_right);
    std_msgs::String msg_action;
    msg_action.data = "change_right";
    pub_action.publish(msg_action);
}
