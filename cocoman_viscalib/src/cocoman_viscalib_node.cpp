#include "ros/ros.h"
#include "ros/package.h"

// pcl
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

typedef pcl::PointXYZRGBNormal PointNT;
typedef pcl::PointCloud<PointNT> CloudN;
typedef CloudN::Ptr CloudNPtr;

#include <iostream>

using namespace std;

CloudPtr pc_in, pc_calibrated;
double lastT, nowT;

tf::TransformListener* tf_listener;
ros::Publisher pub_pc;
string baseFrame = "/base";
string camFrame = "/cam_calibrated";

void cb_rgbd(const pcl::PCLPointCloud2ConstPtr& input)
{
    static int cnt = 0;

    // input
    pc_in.reset(new Cloud);
    pc_calibrated.reset(new Cloud);
    pcl::fromPCLPointCloud2(*input, *pc_in);

    tf::StampedTransform transform;
    tf_listener->waitForTransform(baseFrame, camFrame, ros::Time(0), ros::Duration(3.0));
    tf_listener->lookupTransform(baseFrame, camFrame, ros::Time(0), transform);

    pcl::PCLPointCloud2 output_pc;
    pcl_ros::transformPointCloud(*pc_in, *pc_calibrated, transform);
    pcl::toPCLPointCloud2(*pc_calibrated, output_pc);
    output_pc.header.frame_id = baseFrame;
    pub_pc.publish (output_pc);
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init (argc, argv, "cocoman_viscalib");
    ros::NodeHandle n;
    tf_listener = new tf::TransformListener();
    ros::Subscriber sub_rgbd = n.subscribe("/xtion/depth_registered/points", 1, cb_rgbd);
    pub_pc = n.advertise<pcl::PCLPointCloud2>("cocoman/pc_calibrated", 1);
    ros::spin();
    return 0;
}
