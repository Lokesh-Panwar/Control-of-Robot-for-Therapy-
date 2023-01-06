#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/point_cloud.h> // for PointCloud
#include <pcl/common/io.h> 
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/common/transforms.h>
#include <fstream>

using namespace std::chrono_literals;

// Reading a pcd file program
main(int argc, char **argv)
{
    ros::init (argc, argv, "pcl_processing");

    ROS_INFO("Started PCL read node");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vox_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Ransac_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    pcl::io::loadPCDFile ("human_replica.pcd", *cloud); // Loading a pcd file

    // Filtering the data
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.6, 2);
    //pass.setNegative (true);
    pass.filter(*cloud_filtered);

    pcl::PassThrough<pcl::PointXYZRGB> pasx;
    pasx.setInputCloud(cloud_filtered);
    pasx.setFilterFieldName ("x");
    pasx.setFilterLimits (-0.25, 0.2);
    //pasx.setNegative (true);
    pasx.filter(*cloud_filtered);

    pcl::PassThrough<pcl::PointXYZRGB> pasy;
    pasy.setInputCloud(cloud_filtered);
    pasy.setFilterFieldName ("y");
    pasy.setFilterLimits (0.0, 0.2);
    //pasx.setNegative (true);
    pasy.filter(*cloud_filtered);

    // Voxel Filter
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud (cloud_filtered);
    vox.setLeafSize (0.05f, 0.05f, 0.05f);
    vox.filter(*vox_filtered);
    // Statistical outlier removal 
    /*pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_filtered);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*SOR_filtered);*/

    //MLS surface reconstruction and normal 
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
    
    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud (vox_filtered);
    mls.setPolynomialOrder (2);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);

    // Reconstruct
    mls.process(*mls_points);


    // -----------------------------------------------
    // -----Create RangeImage from the PointCloud-----
    // -----------------------------------------------
    // --------------------
    // -----Parameters-----
    // --------------------
    float angular_resolution = 0.5f;
    float support_size = 0.2f;
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    bool setUnseenToMaxRange = true;
    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;

    scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f ((*vox_filtered).sensor_origin_[0],(*vox_filtered).sensor_origin_[1],(*vox_filtered).sensor_origin_[2])) * Eigen::Affine3f ((*vox_filtered).sensor_orientation_);

    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;   
    range_image.createFromPointCloud (*vox_filtered, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    range_image.integrateFarRanges (far_ranges);
    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange ();

    /*// --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
    viewer.setBackgroundColor (1, 1, 1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
    viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
    //viewer.addCoordinateSystem (1.0f, "global");
    //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
    //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
    viewer.initCameraParameters ();
    //setViewerPose (viewer, range_image.getTransformationToWorldSystem ());
 
    // --------------------------
    // -----Show range image-----
    // --------------------------
    pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    range_image_widget.showRangeImage (range_image);

    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage (&range_image);
    narf_keypoint_detector.getParameters ().support_size = support_size;
    //narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
    //narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;
    
    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute (keypoint_indices);
    std::cout << "Found "<<keypoint_indices.size ()<<" key points.\n";

    // -------------------------------------
    // -----Show keypoints in 3D viewer-----
    // -------------------------------------
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>& keypoints = *keypoints_ptr;
    keypoints.resize (keypoint_indices.size ());
    for (std::size_t i=0; i<keypoint_indices.size (); ++i)
       keypoints[i].getVector3fMap () = range_image[keypoint_indices[i]].getVector3fMap ();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> keypoints_color_handler (keypoints_ptr, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZRGB> (keyx1 = 0.33
    y1 = 0.11
    z1 = 0.564

    x2 = 0.33 + 0.1
    y2 = 0.11 - 0.1 
    z2 = 0.564 - 0.1points_ptr, keypoints_color_handler, "keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");
 
    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer.wasStopped ())
    {
        range_image_widget.spinOnce ();  // process GUI events
        viewer.spinOnce ();
        pcl_sleep(0.01);
    }*/


    //RANSAC 
    /*pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (vox_filtered));
    std::vector<int> inliers;
    pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
    ransac.setDistanceThreshold (0.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
    pcl::copyPointCloud (*vox_filtered, inliers, *Ransac_filtered);*/


    // Transformation Matrix

    // Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
 
    // //Define a rotation matrix 
    // //float theta = 20*(M_PI/180); // The angle of rotation in radians
    // transform_1 (0,0) = 1;
    // transform_1 (0,1) = 0;
    // transform_1 (0,2) = 0;
    // transform_1 (0,3) = -0.77;
    // transform_1 (1,0) = 0;
    // transform_1 (1,1) = 0;
    // transform_1 (1,2) = 1;
    // transform_1 (1,3) = -1.33;
    // transform_1 (2,0) = 0;
    // transform_1 (2,1) = -1;
    // transform_1 (2,2) = 0;
    // transform_1 (2,3) = 0;
    // transform_1 (3,0) = 0;
    // transform_1 (3,1) = 0;
    // transform_1 (3,2) = 0;
    // transform_1 (3,3) = 1;
    //    (row, column)

    // // Define a translation of 2.5 meters on the x axis.
    // transform_1 (0,3) = 0;

    // Print the transformation
    //printf ("Method #1: using a Matrix4f\n");
    //std::cout << transform_1 << std::endl;

     /*  METHOD #2: Using a Affine3f
       This method is easier and less error prone
     */
    // Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // // Define a translation of 2.5 meters on the x axis.
    // transform_2.translation() << 2, 0.0, 0.0;

    // The same rotation matrix as before; theta radians around Z axis
    //transform_2.rotate (Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitX()));
    //transform_2.rotate (Eigen::AngleAxisf (M_PI, Eigen::Vector3f::UnitZ()));

    // Print the transformation
    //printf ("\nMethod #2: using an Affine3f\n");
    //std::cout << transform_2.matrix() << std::endl;

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    // You can either apply transform_1 or transform_2; they are the same
    //pcl::transformPointCloud (*vox_filtered, *transformed_cloud, transform_1); 


    //pcl::io::savePCDFileASCII ("MLS_surface.pcd", *mls_points);
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Therapy activity Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(vox_filtered);
    viewer->addPointCloud<pcl::PointXYZRGB> (vox_filtered, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
    //viewer->addPointCloudNormals <pcl::PointXYZRGB, pcl::Normal> (vox_filtered, mls_points, 10, 0.05, "normals");
    // Visualise the transformed point cloud 
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> transformed_cloud_color_handler (transformed_cloud); // Red
    // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> transformed_cloud_color_handler (transformed_cloud);
    // viewer->addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while (!viewer->wasStopped()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
    pcl::toROSMsg(*transformed_cloud, output);
    output.header.frame_id = "point_cloud";
    
    // ofstream MyExcelFile;
    // MyExcelFile.open("human_replica_tr_data_4.csv");
    // MyExcelFile << "X, Y, Z" << endl;

    // Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
    // float theta = M_PI/2; // The angle of rotation in radians
    // transform_1 (0,0) = 1;
    // transform_1 (0,1) = 0;
    // transform_1 (0,2) = 0;
    // transform_1 (0,3) = 0.77;
    // transform_1 (1,0) = 0;
    // transform_1 (1,1) = cos(theta);
    // transform_1 (1,2) = -sin(theta);
    // transform_1 (1,3) = 0;
    // transform_1 (2,0) = 0;
    // transform_1 (2,1) = sin(theta);
    // transform_1 (2,2) = cos(theta);
    // transform_1 (2,3) = 1.33;
    // transform_1 (3,0) = 0;
    // transform_1 (3,1) = 0;
    // transform_1 (3,2) = 0;
    // transform_1 (3,3) = 1;

    // for(std::size_t i = 0; i < vox_filtered->size (); i++){


    //     MyExcelFile<<(-(*vox_filtered)[i].x + 0.77) << "," << (-(*vox_filtered)[i].z + 1.33) << "," << (-(*vox_filtered)[i].y) << endl;
        
    //     //MyExcelFile<<((*vox_filtered)[i].x - 0.77) << "," << ((*vox_filtered)[i].z - 1.33) << "," << (-(*vox_filtered)[i].y) << endl;
        
    //     printf ("\t(%f, %f, %f)\n",(*vox_filtered)[i].x, (*vox_filtered)[i].y, (*vox_filtered)[i].z);
    // }
    // MyExcelFile.close();
    ros::Rate loop_rate(1);
    /*while (ros::ok())
    {
//Publishing the cloud inside pcd file
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }*/

    return 0;
}


