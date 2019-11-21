#include "rgbd_scan/rgbd_scan.h"

void RgbdScan::setGroundHeight(float h,
                               float th) //输入地面高度，以及其误差-th/+th
{
  height = h;
  heightDev = th;
}

void RgbdScan::setMaxDistance(float s) //输入使用摄像头视野最远距离
{
  maxDepth = s;
}

void RgbdScan::setDistanceArea(float s1, float s2)  //输入检测距离：1级安全距离和2级安全距离
{
  Distance1=s1;
  Distance2=s2;
}

void RgbdScan::PointCloudFilters(
    pcl::PCLPointCloud2::Ptr &cloud_in,
    pcl::PCLPointCloud2::Ptr &cloud_out) //点云滤波预处理
{
  pcl::PCLPointCloud2 cloud_temp;
  //直通滤波 去除远处的点
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  // build the filter
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, maxDepth); //滤除较远点
  // apply filter
  pass.filter(cloud_temp);

  //体素滤波 VoxelGrid滤波器,对密集的点云进行稀疏
  pcl::copyPointCloud(cloud_temp, *cloud_in); // PCL pointcloud Input
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // build the filter
  sor.setInputCloud(cloud_in);
  sor.setLeafSize(0.01, 0.01, 0.01);
  // apply filter
  sor.filter(cloud_temp);

  //去除离群点
  pcl::copyPointCloud(cloud_temp, *cloud_in); // PCL pointcloud Input
  // Perform the actual filtering
  pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sorem;
  sorem.setInputCloud(cloud_in);
  sorem.setMeanK(50);            //在进行统计时考虑查询点邻近点数
  sorem.setStddevMulThresh(1.5); //判断是否为离群点的阈值
  sorem.filter(cloud_temp);

  pcl::copyPointCloud(cloud_temp, *cloud_out); // PCL pointcloud Output
}

void RgbdScan::GroundSegmentation(
    pcl::PCLPointCloud2::Ptr &cloud_in,
    pcl::PCLPointCloud2::Ptr &cloud_out) //直通滤波，分离地面
{
  pcl::PCLPointCloud2 cloud_temp;
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  // build the filter
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(height - heightDev, height + heightDev);
  // apply filter
  pass.filter(cloud_temp);
  pcl::copyPointCloud(cloud_temp, *cloud_out); // PCL pointcloud Output
}

void RgbdScan::NonGroundSegmentation(
    pcl::PCLPointCloud2::Ptr &cloud_in,
    pcl::PCLPointCloud2::Ptr &cloud_out) //直通滤波，分离非地面
{
  pcl::PCLPointCloud2 cloud_temp;
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  // build the filter
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("y");
  pass.setFilterLimitsNegative(true);
  pass.setFilterLimits(height - heightDev, 5.0);
  // apply filter
  pass.filter(cloud_temp);
  
  pcl::copyPointCloud(cloud_temp, *cloud_out); // PCL pointcloud Output
}

bool RgbdScan::scanByDistance(pcl::PCLPointCloud2::Ptr &cloud_in, float minDistance,
                    float maxDistance) //扫描区间内的非地面点云是否为空
{
  float min = minDistance;
  float max = maxDistance;

  pcl::PCLPointCloud2 cloud_temp;

  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  // build the filter
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min, max);
  // apply filter
  pass.filter(cloud_temp);
  //ROS_INFO("%d %d", cloud_temp.width, cloud_temp.height);
  if (cloud_temp.width * cloud_temp.height == 0) {
    return (false);
  } else {
    return (true);
  }
}

void RgbdScan::ScanerInit() //参数初始化函数
{
  RgbdScan::setGroundHeight(0.15, 0.03);
  RgbdScan::setMaxDistance(2.5);
  RgbdScan::setDistanceArea(1.0, 1.8);
  //   ROS_INFO("height and heightDev", "%2f %2f %2f", height, heightDev);
  //   ROS_INFO("maxDepth", "%2f %2f %2f", maxDepth);
}

void RgbdScan::pointcloud_callback(
    const sensor_msgs::PointCloud2ConstPtr &pointcloud_msg) {

  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2::Ptr cloudPtr(cloud);
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr ground(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr non_ground(new pcl::PCLPointCloud2);

  pcl_conversions::toPCL(*pointcloud_msg, *cloud);
  RgbdScan::PointCloudFilters(cloudPtr, cloud_filtered);
  RgbdScan::GroundSegmentation(cloud_filtered, ground);
  RgbdScan::NonGroundSegmentation(cloud_filtered, non_ground);
  area1=RgbdScan::scanByDistance(non_ground, 0.0, Distance1);
  area2=RgbdScan::scanByDistance(non_ground, Distance1, Distance2);
  area3=RgbdScan::scanByDistance(non_ground, Distance2, maxDepth);
  
  pointcloud_publisher.publish(cloud_filtered);
  ground_publisher.publish(ground);

  // for(sensor_msgs::PointCloud2Iterator<float>
  //       iter_z(*non_ground,"z");
  //       iter_z!=iter_z.end();
  //       ++iter_z)
  //       {
  //           *iter_z = 0.0;
  //       }
  int z_idx = pcl::getFieldIndex(*non_ground,"z");
  int z_offset = non_ground->fields[z_idx].offset;
  uint8_t z_datatype = non_ground->fields[z_idx].datatype;
  size_t output_size = non_ground->width * non_ground->height;
  for(size_t cp =0;cp<output_size;++cp)
  {
    non_ground->data[cp*non_ground->point_step+z_offset] = 0.0;
  }

  non_ground_publisher.publish(non_ground);
}

void RgbdScan::ScanerStart() {
  sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "/camera/depth_registered/points", 1, &RgbdScan::pointcloud_callback,
      this);
  pointcloud_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("FilteredPointCloud", 1);
  ground_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("GroundSegmentationPointCloud", 1);
  non_ground_publisher =
      nh.advertise<sensor_msgs::PointCloud2>("NonGroundSegmentationCloud", 1);
}



