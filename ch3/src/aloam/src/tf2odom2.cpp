#include "ros/ros.h"
// #include "tf2_msgs/TFMessage.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>

using namespace std;
ros::Publisher pub;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf2odom2"); //初始化节点
    ros::NodeHandle n;
    pub = n.advertise<nav_msgs::Odometry>("gt_odom",1); //定义发送端
    ros::Rate loop_rate(10); //频率
    tf::TransformListener tfListener;
    tf::StampedTransform word2base_link;
    while (ros::ok()) {
        try {
            // 等待3s
            // tfListener.waitForTransform("world", "base_link", ros::Time(0), ros::Duration(0.01));
            // lidar系到baselink系的变换
            tfListener.lookupTransform("world", "base_link", ros::Time(0), word2base_link);
            nav_msgs::Odometry msg;
            msg.header.frame_id = "map";
            msg.child_frame_id = "base_link";
            msg.header.stamp = ros::Time::now();
            static int first = 1;
            if(first){
                msg.pose.pose.position.x = 0;
                msg.pose.pose.position.y = 0;
                msg.pose.pose.position.z = 0;
                msg.pose.pose.orientation.x = 0.0116170686661;
                msg.pose.pose.orientation.y = 0.00845793546045;
                msg.pose.pose.orientation.z = 0.481795918798;
                msg.pose.pose.orientation.w = 0.876165623425;
                first = 0;
            }
            else{
                msg.pose.pose.position.x = word2base_link.getOrigin().x();
                msg.pose.pose.position.y = word2base_link.getOrigin().y();
                msg.pose.pose.position.z = word2base_link.getOrigin().z();
                msg.pose.pose.orientation.x = word2base_link.getRotation().x();
                msg.pose.pose.orientation.y = word2base_link.getRotation().y();
                msg.pose.pose.orientation.z = word2base_link.getRotation().z();
                msg.pose.pose.orientation.w = word2base_link.getRotation().w();
            }
            pub.publish(msg);
        }
        catch (tf::TransformException ex) {
            // ROS_ERROR("%s",ex.what());
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}