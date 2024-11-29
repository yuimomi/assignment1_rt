#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include <cmath>

ros::Publisher dist_pub;
ros::Publisher pub_turtle1, pub_turtle2;

// Global Variables
turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;

const float THRESHOLD = 2.0;  // Minimum Distance Threshold
const float BOUND_MIN = 1.0;        // Boundary minimum
const float BOUND_MAX = 10.0;       // Boundary maximum

void poseCallback1(const turtlesim::Pose::ConstPtr &msg){
     turtle1_pose = *msg;
}

void poseCallback2(const turtlesim::Pose::ConstPtr &msg){
    turtle2_pose = *msg;
}

// A function to calculate distance between turtle1 and turtle2
float calculateDistance(const turtlesim::Pose &pose1, const turtlesim::Pose &pose2) {
    float dx = pose1.x - pose2.x;
    float dy = pose1.y - pose2.y;
    return std::sqrt(dx * dx + dy * dy);
}

// A function to determine whether or not the object is within the boundary
bool isWithinBoundary(const turtlesim::Pose &pose) {
    return pose.x < BOUND_MIN || pose.x > BOUND_MAX || pose.y < BOUND_MIN || pose.y > BOUND_MAX;
}

void stopTurtle(ros::Publisher &pub) {
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    pub.publish(stop_cmd);
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

        float distance = calculateDistance(turtle1_pose, turtle2_pose);
        ROS_INFO("Distance between turtles: %.2f", distance);

        // Publish distance
        std_msgs::Float32 dist_msg;
        dist_msg.data = distance;
        dist_pub.publish(dist_msg);

        // If the distance exceeds the threshold, stop the turtle
        if (distance < THRESHOLD){
            stopTurtle(pub_turtle1);
            stopTurtle(pub_turtle2);
        }
        if (isWithinBoundary(turtle1_pose)) {
                ROS_WARN("Turtle1 is near the boundary! Stopping turtle1.");
                stopTurtle(pub_turtle1);
            }

        if (isWithinBoundary(turtle2_pose)) {
                ROS_WARN("Turtle2 is near the boundary! Stopping turtle2.");
                stopTurtle(pub_turtle2);
            }
        
    rate.sleep();
    }
return 0;
}

