#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float64.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();
    
    float current_speed;    //Speed is intended to be given in m/s
    float current_steering; //Steering is intended to be given in rad
    
    ros::NodeHandle* n;
    ros::Publisher pub_speed;
    ros::Publisher pub_steering;
    ros::Publisher pub_start;
    ros::Publisher pub_change_left;
    ros::Publisher pub_cruise;
    ros::Publisher pub_follow;
    ros::Publisher pub_change_right;
    ros::Publisher pub_speed_cars_left;
    ros::Publisher pub_speed_cars_right;
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publish_speed(double speed);
    void publish_steering(double steering);
    void update_speed_and_publish(double delta_speed);
    void update_steering_and_publish(double delta_steering);

    void publish_start(double speed_cars_left, double speed_cars_right);
    void publish_change_left();
    void publish_cruise();
    void publish_follow();
    void publish_change_right();
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
