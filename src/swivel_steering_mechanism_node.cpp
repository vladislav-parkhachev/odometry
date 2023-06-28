#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>

#define PI 3.14
#define QUANTITY_IMPULSES_PER_1_METER 33000
#define DISTANCE_BETWEEN_WHEEL 0.69

bool NewEncoderData = false;
double velocity_joint = 0;
double steering_joint = 0;

void message_callback_joint_states(const sensor_msgs::JointState &encoder_data)
{
    steering_joint = encoder_data.position[0];
    velocity_joint = encoder_data.velocity[1];
    NewEncoderData = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "swivel_steering_mechanism_node");
    ros::NodeHandle nh;
    ros::Subscriber subscriber_encoder_1 = nh.subscribe("/joint_states", 10, message_callback_joint_states);

    ros::Publisher publisher_odometry = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Publisher publisher_path_odometry = nh.advertise<nav_msgs::Path>("/path_odomerty", 50);

    geometry_msgs::PoseStamped pose_odomerty;
    nav_msgs::Path path_odomerty;
    path_odomerty.header.frame_id = "odom";

    double ICR_distance = 0;
    double steering_wheel_angle = 0;
    double distance_traveled = 0;

    double x = 0;
    double y = 0;
    double angle_theta = 0;

    ros::Rate r(10);

    while (nh.ok())
    {
        if (NewEncoderData)
        {
            distance_traveled = velocity_joint / QUANTITY_IMPULSES_PER_1_METER;
            steering_wheel_angle = steering_joint * 11 / 100 * PI / 180 ;
            ICR_distance = DISTANCE_BETWEEN_WHEEL / tan(steering_wheel_angle);

            angle_theta += distance_traveled / ICR_distance;
            x += cos(angle_theta) * distance_traveled;
            y += -sin(angle_theta) * distance_traveled;

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(-angle_theta);
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = ros::Time::now();
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";
            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            odom_broadcaster.sendTransform(odom_trans);

            nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time::now();
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";

            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            odom.twist.twist.linear.x = 0;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.angular.z = 0;

            publisher_odometry.publish(odom);

            pose_odomerty.pose = odom.pose.pose;
            path_odomerty.header.stamp = ros::Time::now();
            path_odomerty.poses.push_back(pose_odomerty);
            publisher_path_odometry.publish(path_odomerty);

            NewEncoderData = false;
        }
        ros::spinOnce();
    }
}
