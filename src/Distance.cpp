#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include <cmath>

ros::Publisher dist_pub;
ros::Publisher pub_turtle1, pub_turtle2;

double x_1,y_1,x_2,y_2;


void poseCallback1(const turtlesim::Pose::ConstPtr &msg){
    x_1 = msg ->x;
    y_1 = msg ->y;
}

void poseCallback2(const turtlesim::Pose::ConstPtr &msg){
    x_2 = msg ->x;
    y_2 = msg ->y;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    ros::Subscriber sub1 = nh.subscribe("turtle1/pose", 10, poseCallback1);
    ros::Subscriber sub2 = nh.subscribe("turtle2/pose", 10, poseCallback2);

    dist_pub = nh.advertise<std_msgs::Float32>("turtle_distance",10);
    pub_turtle1 = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",10);
    pub_turtle2 = nh.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",10);

    ros::Rate rate(10);
    while(ros::ok()){
        ros::spinOnce();
        double distance = std::sqrt(std::pow(x_2 - x_1, 2) + std::pow(y_2 - y_1, 2));

        // Publish distance
        std_msgs::Float32 dist_msg;
        dist_msg.data = distance;
        dist_pub.publish(dist_msg);

        // If the distance exceeds the threshold, stop the turtle
        if (distance < 1.0){
            geometry_msgs::Twist stop_cmd;
            stop_cmd.linear.x = 0;
            stop_cmd.angular.z = 0;
            pub_turtle1.publish(stop_cmd);
            pub_turtle2.publish(stop_cmd);
        }
        // If the turtle crosses the boundary, stop it
        if (x_1 < 1.0 || x_1 > 10.0 || y_1 < 1.0 || y_1 > 10.0 || 
            x_2 < 1.0 || x_2 > 10.0 || y_2 < 1.0 || y_2 > 10.0) {
            geometry_msgs::Twist stop_cmd;
            stop_cmd.linear.x = 0;
            stop_cmd.angular.z = 0;
            pub_turtle1.publish(stop_cmd);
            pub_turtle2.publish(stop_cmd);
        }
    rate.sleep();
    }
return 0;
}

