// simple Point cloud subscription 
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
ros::init(argc, argv, "pcl_filter");
ros::NodeHandle nh;
//ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth/color/points", 1, callback);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ> final_cloud;
pcl::io::loadPCDFile ("voxel_filtered.pcd", *cloud);
std::size_t k = 0;
 for(std::size_t i = 0; i < cloud->size (); i++){
    final_cloud[k].x = (*cloud)[i].x;
    final_cloud[k].y = (*cloud)[i].y;
    final_cloud[k].z = (*cloud)[i].z;
    printf ("\t(%f, %f, %f)\n",final_cloud[k].x, final_cloud[k].y, final_cloud[k].z);
    k++;
 }

}


// Point cloud filter and voxel downsampling
/*#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>

class cloudHandler
{
public:
    cloudHandler()
    {end program in ros
        
//Subscribing pcl_output topics from the publisher
//This topic can change according to the source of point cloud

    pcl_sub = nh.subscribe("/camera/depth/color/points", 10, &cloudHandler::cloudCB, this);
//Creating publisher for filtered cloud data
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1);
    }
//Creating cloud callback
    void cloudCB(const sensor_msgs::PointCloud2& input)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

     
       sensor_msgs::PointCloud2 output;
       pcl::fromROSMsg(input, cloud);

     //Creating VoxelGrid object
      pcl::VoxelGrid<pcl::PointXYZ> vox_obj;
     //Set input to voxel object
     vox_obj.setInputCloud (cloud.makeShared());
  
     //Setting parameters of filter such as leaf size
    vox_obj.setLeafSize (0.1f, 0.1f, 0.1f);
    
    //Performing filtering and copy to cloud_filtered variable
    vox_obj.filter(cloud_filtered);
      pcl::toROSMsg(cloud_filtered, output);
      output.header.frame_id = "point_cloud";
       pcl_pub.publish(output);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};
main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_filter");
    ROS_INFO("Started Filter Node");
    cloudHandler handler;
    ros::spin();
    return 0;
}
*/


/*#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <bits/stdc++.h>

class cloudHandler
{
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
    sensor_msgs::PointCloud2 data;
   // pcl_sub = nh.subscribe("/camera/depth/color/points", 10, cloudHandler::cloudCB);
//Creating publisher for filtered cloud data
    //pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1);

    public:
    void cloudCB(const sensor_msgs::PointCloud2 input)
    {
        data = input;
    }

    void cloudHandlr()
    {
        
//Subscribing pcl_output topics from the publisher
//This topic can change according to the source of point cloud

    pcl_sub = nh.subscribe("/camera/depth/color/points", 10, cloudHandler::cloudCB);
//Creating publisher for filtered cloud data
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1);

    pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

     
        sensor_msgs::PointCloud2 output;
        pcl::fromROSMsg(input, cloud);

    //Creating VoxelGrid object
        pcl::VoxelGrid<pcl::PointXYZ> vox_obj;
     //Set input to voxel object
        vox_obj.setInputCloud(cloud.makeShared());
  
     //Setting parameters of filter such as leaf size
        vox_obj.setLeafSize(0.1f, 0.1f, 0.1f);
    
    //Performing filtering and copy to cloud_filtered variable
        vox_obj.filter(cloud_filtered);
        pcl::toROSMsg(cloud_filtered, output);
        output.header.frame_id = "point_cloud";
        pcl_pub.publish(output);
    }
//Creating cloud callback


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_filter");
    ROS_INFO("Started Filter Node");
    cloudHandler handler;
    handler.cloudHandlr()
    ros::spin();
    return 0;
}*/












/*#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
  printf("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("points2", 1, callback);
  ros::spin();
}*/



















/*#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;

  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth/color/points", 10, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 10);

  // Spin
  ros::spin ();
}*/




/*#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <bits/stdc++.h>

class cloudHandler
{
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
    sensor_msgs::PointCloud2 data;
   // pcl_sub = nh.subscribe("/camera/depth/color/points", 10, cloudHandler::cloudCB);
//Creating publisher for filtered cloud data
    //pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1);

    public:
    void cloudCB(const sensor_msgs::PointCloud2 input)
    {
        data = input;
    }

    void cloudHandlr()
    {
        
//Subscribing pcl_output topics from the publisher
//This topic can change according to the source of point cloud

    pcl_sub = nh.subscribe("/camera/depth/color/points", 10, cloudHandler::cloudCB);
//Creating publisher for filtered cloud data
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1);

    pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

     
        sensor_msgs::PointCloud2 output;
        pcl::fromROSMsg(input, cloud);

    //Creating VoxelGrid object
        pcl::VoxelGrid<pcl::PointXYZ> vox_obj;
     //Set input to voxel object
        vox_obj.setInputCloud(cloud.makeShared());
  
     //Setting parameters of filter such as leaf size
        vox_obj.setLeafSize(0.1f, 0.1f, 0.1f);
    
    //Performing filtering and copy to cloud_filtered variable
        vox_obj.filter(cloud_filtered);
        pcl::toROSMsg(cloud_filtered, output);
        output.header.frame_id = "point_cloud";
        pcl_pub.publish(output);
    }
//Creating cloud callback


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_filter");
    ROS_INFO("Started Filter Node");
    cloudHandler handler;
    handler.cloudHandlr()
    ros::spin();
    return 0;
}*/