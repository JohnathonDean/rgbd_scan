#ifndef RGBD_SCAN_H__
#define RGBD_SCAN_H__

/*********************
 ** ros
 **********************/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud_conversion.h>

/*********************
 ** pcl
 **********************/
#include "pcl_ros/point_cloud.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

/*********************
 ** class
 **********************/
class RgbdScan {
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pointcloud_publisher;
  ros::Publisher ground_publisher;
  ros::Publisher non_ground_publisher;
  float height, heightDev; //输入地面高度，以及其误差
  float maxDepth;          //输入使用摄像头视野最远距离
  float Distance1, Distance2; //1级安全距离，2级安全距离
  bool area1, area2, area3;   //分三个区域检测

public:
  RgbdScan() { ROS_INFO("START"); }
  ~RgbdScan() { ROS_INFO("END"); }
  void setGroundHeight(float h, float th); //输入地面高度，以及其误差-th/+th
  void setMaxDistance(float s); //输入使用摄像头视野最远距离
  void setDistanceArea(float s1, float s2);  //输入检测距离：1级安全距离和2级安全距离
  void PointCloudFilters(pcl::PCLPointCloud2::Ptr &cloud_in,
                         pcl::PCLPointCloud2::Ptr &cloud_out); //点云滤波预处理
  void
  GroundSegmentation(pcl::PCLPointCloud2::Ptr &cloud_in,
                     pcl::PCLPointCloud2::Ptr &cloud_out); //直通滤波，分离地面
  void NonGroundSegmentation(
      pcl::PCLPointCloud2::Ptr &cloud_in,
      pcl::PCLPointCloud2::Ptr &cloud_out); //直通滤波，分离非地面
  bool scanByDistance(pcl::PCLPointCloud2::Ptr &cloud_in, float minDistance,
                      float maxDistance); //扫描区间内的非地面点云是否为空
  void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr
                               &pointcloud_msg); //读取输入RGBD点云的回调函数
  void ScanerInit();                             //参数初始化函数
  void ScanerStart();                            //开始处理点云数据
};


#endif