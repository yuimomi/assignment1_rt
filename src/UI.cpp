#include "ros/ros.h"
#include "turtlesim/Spawn.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <string>

ros::Publisher pub_turtle1, pub_turtle2;

void sendCommnad(const std::string &turtle_name, float linear, float angular){
    geometry_msgs::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    if(turtle_name == "turtle1"){
        pub_turtle1.publish(cmd);
    } else if(turtle_name == "turtle2"){
        pub_turtle2.publish(cmd);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ui_node");
    ros::NodeHandle nh;

    // Generate turtle2
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("/spawn");
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.x = 5.0;
	spawn_srv.request.y = 5.0;
 	spawn_srv.request.theta = 0.0;
    spawn_srv.request.name = "turtle2";
 	// spawn_client.waitForExistence();
	spawn_client.call(spawn_srv);

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

        // Send command for 1.0 sec
        sendCommnad(selected_turtle, linear, angular);
        ros::Duration(1.0).sleep();

        // Stop turtle
        sendCommnad(selected_turtle, 0, 0);
    }
    return 0;

}