#include "ros/ros.h"
#include "turtlesim/Spawn.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <string>

ros::Publisher pub_turtle1, pub_turtle2;

void spawnTurtle(const std::string &name, float x, float y, float theta) {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");

    turtlesim::Spawn srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.theta = theta;
    srv.request.name = name;

    if (client.call(srv)) {
        ROS_INFO("Spawned turtle '%s' at (%f, %f, %f)", name.c_str(), x, y, theta);
    } else {
        ROS_ERROR("Failed to call service /spawn");
        ros::shutdown();
    }
}

void sendCommand(ros::Publisher &pub, float linear, float angular) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;

    pub.publish(cmd);
    ROS_INFO("Command sent: linear=%f, angular=%f", linear, angular);

    ros::Duration(1.0).sleep(); // Send the command for 1 second

    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    pub.publish(cmd);
    ROS_INFO("Turtle stopped.");
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;


    // Spawn a second turtle
    spawnTurtle("turtle2", 3.0, 3.0, 0.0);

	pub_turtle1 = nh.advertise<geometry_msgs::Twist> ("turtle1/cmd_vel", 10); 
	pub_turtle2 = nh.advertise<geometry_msgs::Twist> ("turtle2/cmd_vel", 10); 

    std::string selected_turtle;
    float linear, angular;

    while(ros::ok()){
        std::cout << "Select turtle (turtle1/turtle2): ";
        std::cin >> selected_turtle;

        std::cout << "Enter linear velosity: ";
        std::cin >> linear;

        std::cout << "Enter angular velosity: ";
        std::cin >> angular;

        if (selected_turtle == "turtle1") {
            sendCommand(pub_turtle1, linear, angular);
        } else if (selected_turtle == "turtle2") {
            sendCommand(pub_turtle2, linear, angular);
        }
    }
    return 0;

}
