#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

nav_msgs::Path  path_gps;
nav_msgs::Path  path_carto;
ros::Publisher  pub_gps;
ros::Publisher  pub_carto;

void gpsCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = odom -> pose.pose.position.x;
    this_pose_stamped.pose.position.y = odom -> pose.pose.position.y;
    this_pose_stamped.pose.position.z = odom -> pose.pose.position.z;

    this_pose_stamped.pose.orientation = odom -> pose.pose.orientation;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "map";

    path_gps.poses.push_back(this_pose_stamped);

    path_gps.header.stamp = ros::Time::now();
    path_gps.header.frame_id = "map";
    pub_gps.publish(path_gps);
}

void fusedCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = odom -> pose.pose.position.x;
    this_pose_stamped.pose.position.y = odom -> pose.pose.position.y;
    this_pose_stamped.pose.position.z = odom -> pose.pose.position.z;

    this_pose_stamped.pose.orientation = odom -> pose.pose.orientation;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "map";

    path_carto.poses.push_back(this_pose_stamped);

    path_carto.header.stamp = ros::Time::now();
    path_carto.header.frame_id = "map";
    pub_carto.publish(path_carto);
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "odom2path");
    ros::NodeHandle ph;

    pub_gps = ph.advertise<nav_msgs::Path>("/synced_gnss_path", 10, true);
    ros::Subscriber sub_gps = ph.subscribe<nav_msgs::Odometry>("/synced_gnss", 10, gpsCallback);  
    pub_carto = ph.advertise<nav_msgs::Path>("/fused_localization_path", 10, true);
    ros::Subscriber sub_carto = ph.subscribe<nav_msgs::Odometry>("/fused_localization", 10, fusedCallback);  
    

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
