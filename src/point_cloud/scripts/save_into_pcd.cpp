// saving pcd file
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
//Header file for writing PCD file
#include <pcl/io/pcd_io.h>

void cloudCB(const sensor_msgs::PointCloud2 &input)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(input, cloud);

//Save data as test.pcd file
    pcl::io::savePCDFileASCII ("human_replica.pcd", cloud);
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_save");

    ROS_INFO("Started PCL write node");

    ros::NodeHandle nh;
    ros::Subscriber bat_sub = nh.subscribe("/camera/depth/color/points", 10, cloudCB);

    ros::spin();
    ros::shutdown();
    return 0;
}