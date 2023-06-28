#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Path.h>

#define PI 3.14
#define QUANTITY_IMPULSES 20
#define DISTANCE_BETWEEN_WHEELS 0.12
#define WHEEL_RADIUS 0.03

static int encoder_1_current = 0;
static int encoder_2_current = 0;
bool NewEncoder1Data= false;
bool NewEncoder2Data = false;

void callback_encoder_1(const std_msgs::Int32 &encoder_1)
{
    encoder_1_current = encoder_1.data;
    NewEncoder1Data = true;
}

void callback_encoder_2(const std_msgs::Int32 &encoder_2)
{
    encoder_2_current = encoder_2.data;
    NewEncoder2Data = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "differential_drive_kinematics_node");
    ros::NodeHandle nh;
    ros::Subscriber subscriber_encoder_1 = nh.subscribe("encoder_1", 50, callback_encoder_1);
    ros::Subscriber subscriber_encoder_2 = nh.subscribe("encoder_2", 50, callback_encoder_2);

    ros::Publisher publisher_odometry = nh.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Publisher publisher_path_odometry = nh.advertise<nav_msgs::Path>("/path_odomerty", 50);

    geometry_msgs::PoseStamped pose_odomerty;
    nav_msgs::Path path_odomerty;
    path_odomerty.header.frame_id = "odom";

    tf::TransformBroadcaster odom_broadcaster;

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.z = 0.0;

    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.z = 0.0;
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = 0;

    int encoder_1_old = 0;
    int encoder_2_old = 0;

    double wheel_1_distance = 0;
    double wheel_2_distance = 0;

    double midpoint_distance = 0;

    double x_coordinate = 0;
    double y_coordinate = 0;
    double angle_theta = 0;

    ros::Rate r(10);

    while (nh.ok())
    {
        if (NewEncoder1Data && NewEncoder2Data)
        {
            int encoder_1_difference = encoder_1_current - encoder_1_old;
            int encoder_2_difference = encoder_2_current - encoder_2_old;

            encoder_1_old = encoder_1_current;
            encoder_2_old = encoder_2_current;

            wheel_1_distance = 2 * PI * WHEEL_RADIUS * encoder_1_difference / QUANTITY_IMPULSES;
            wheel_2_distance = 2 * PI * WHEEL_RADIUS * encoder_2_difference / QUANTITY_IMPULSES;

            midpoint_distance = (wheel_1_distance + wheel_2_distance) / 2;

            angle_theta += (wheel_1_distance - wheel_2_distance) / DISTANCE_BETWEEN_WHEELS;
            x_coordinate += cos(angle_theta) * midpoint_distance;
            y_coordinate += -sin(angle_theta) * midpoint_distance;

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(angle_theta);
            odom_trans.header.stamp = ros::Time::now();
            odom_trans.transform.translation.x = x_coordinate;
            odom_trans.transform.translation.y = y_coordinate;
            
            odom_trans.transform.rotation = odom_quat;
            odom_broadcaster.sendTransform(odom_trans);

            odom.header.stamp = ros::Time::now();
            odom.pose.pose.position.x = x_coordinate;
            odom.pose.pose.position.y = y_coordinate;
            odom.pose.pose.orientation = odom_quat;

            pose_odomerty.pose = odom.pose.pose;
            path_odomerty.header.stamp = ros::Time::now();
            path_odomerty.poses.push_back(pose_odomerty);

            publisher_path_odometry.publish(path_odomerty);
            publisher_odometry.publish(odom);

            NewEncoder1Data = false;
            NewEncoder2Data = false;
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
