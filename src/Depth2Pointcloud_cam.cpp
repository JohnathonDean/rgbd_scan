/*根据读取realsense D435发布的RGBD图像数据合成点云*/

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// PCL 库
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

// 定义点云类型
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 321.798;
const double camera_cy = 239.607;
const double camera_fx = 615.899;
const double camera_fy = 616.468;

// 全局变量：图像矩阵和点云
cv_bridge::CvImagePtr color_ptr, depth_ptr;
cv::Mat color_pic, depth_pic;
bool callbackflag1 = false;
bool callbackflag2 = false;
void color_Callback(const sensor_msgs::ImageConstPtr &color_msg) {
  try {
    color_ptr =
        cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
              color_msg->encoding.c_str());
  }
  color_pic = color_ptr->image;
  callbackflag1 = true;
  // output some info about the rgb image in cv format
  // cout << "output some info about the rgb image in cv format" << endl;
  // cout << "rows of the rgb image = " << color_pic.rows << endl;
  // cout << "cols of the rgb image = " << color_pic.cols << endl;
  // cout << "type of rgb_pic's element = " << color_pic.type() << endl;
}

void depth_Callback(const sensor_msgs::ImageConstPtr &depth_msg) {
  try {
    depth_ptr = cv_bridge::toCvCopy(depth_msg,
                                    sensor_msgs::image_encodings::TYPE_16UC1);

  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'TYPE_16UC1'.",
              depth_msg->encoding.c_str());
  }
  depth_pic = depth_ptr->image;
  callbackflag2 = true;
  // output some info about the depth image in cv format
  // cout << "output some info about the depth image in cv format" << endl;
  // cout << "rows of the depth image = " << depth_pic.rows << endl;
  // cout << "cols of the depth image = " << depth_pic.cols << endl;
  // cout << "type of depth_pic's element = " << depth_pic.type() << endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Depth2Pointcloud_cam1");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub =
      it.subscribe("/camera/color/image_raw", 1, color_Callback);
  image_transport::Subscriber sub1 =
      it.subscribe("/camera/depth/image_rect_raw", 1, depth_Callback);
  ros::Publisher pointcloud_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("generated_pointcloud", 1);
  std::string frame_id = "camera_depth_frame";
  bool transform = false;
  nh.getParam("frame_id", frame_id);
  nh.getParam("transform", transform);
  ROS_INFO("frame_id:%s\n", frame_id.c_str());
  ROS_INFO("transform:%i\n", transform);
  // 点云变量
  // 使用智能指针，创建一个空点云。这种指针用完会自动释放。
  PointCloudT::Ptr cloud(new PointCloudT);
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  sensor_msgs::PointCloud2 pub_pointcloud;

  double sample_rate = 30.0;      // HZ
  ros::Rate naptime(sample_rate); // use to regulate loop rate

  cout << "depth value of depth map : " << endl;

  while (ros::ok()) {
    if (callbackflag1 && callbackflag2) {
      // 遍历深度图
      for (int m = 0; m < depth_pic.rows; m++) {
        for (int n = 0; n < depth_pic.cols; n++) {
          // 获取深度图中(m,n)处的值
          ushort d = depth_pic.ptr<ushort>(m)[n];
          // d 可能没有值，若如此，跳过此点
          if (d == 0)
            continue;
          // d 存在值，则向点云增加一个点
          pcl::PointXYZRGB p;

          // 计算这个点的空间坐标
          p.z = double(d) / camera_factor;
          if (transform) {
            p.x = -(n - camera_cx) * p.z / camera_fx;
            p.y = -(m - camera_cy) * p.z / camera_fy;
          } else {
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;
          }

          // 从rgb图像中获取它的颜色
          // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
          p.b = color_pic.ptr<uchar>(m)[n * 3];
          p.g = color_pic.ptr<uchar>(m)[n * 3 + 1];
          p.r = color_pic.ptr<uchar>(m)[n * 3 + 2];

          // 把p加入到点云中
          cloud->points.push_back(p);
        }
      }

      // 设置并保存点云
      cloud->height = 1;
      cloud->width = cloud->points.size();
      // ROS_INFO("point cloud size = %d", cloud->width);
      cloud->is_dense = false;

      //直通滤波
      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.25, 2.5);
      pass.filter(*cloud_filtered);

      pcl::copyPointCloud(*cloud_filtered, *cloud); // PCL pointcloud Input
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("y");
      pass.setFilterLimitsNegative(true);
      pass.setFilterLimits(0.06, 0.5);
      pass.filter(*cloud_filtered);

      //体素滤波 VoxelGrid滤波器,对密集的点云进行稀疏
      pcl::copyPointCloud(*cloud_filtered, *cloud); // PCL pointcloud Input
      // Perform the actual filtering
      pcl::VoxelGrid<PointT> sor;
      // build the filter
      sor.setInputCloud(cloud);
      sor.setLeafSize(0.02, 0.02, 0.02);
      // apply filter
      sor.filter(*cloud_filtered);

      pcl::toROSMsg(*cloud_filtered, pub_pointcloud);
      for (sensor_msgs::PointCloud2Iterator<float> iter_z(pub_pointcloud, "y");
           iter_z != iter_z.end(); ++iter_z) {
        *iter_z = 0.0;
      }
      pub_pointcloud.header.frame_id =
          frame_id; //"camera1_color_optical_frame";
      pub_pointcloud.header.stamp = ros::Time::now();
      // cout << "publish point_cloud height = " << pub_pointcloud.height <<
      // endl;
      // cout << "publish point_cloud width = " << pub_pointcloud.width << endl;

      // 发布合成点云和原始点云
      pointcloud_publisher.publish(pub_pointcloud);

      // 清除数据并退出
      cloud->points.clear();
    }
    ros::spinOnce(); // allow data update from callback;
    naptime.sleep(); // wait for remainder of specified period;
  }
}