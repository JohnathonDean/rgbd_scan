/*启动两个相机的情况下，读取并显示深度图*/
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

cv_bridge::CvImagePtr image_ptr;
cv::Mat image_pic;

void image_Callback(const sensor_msgs::ImageConstPtr &image_msg) {
  // cv_bridge::CvImagePtr image_ptr;
  try {
    cv::imshow("image_view", cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8)->image);
    image_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

    cv::waitKey(1050); // 不断刷新图像，频率时间为int delay，单位为ms
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to '16UC1'.",
              image_msg->encoding.c_str());
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_input");
  ros::NodeHandle nh;
  cv::namedWindow("image_view");
  cv::startWindowThread();

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera1/color/image_raw", 1, image_Callback);

  double sample_rate = 30.0;       // HZ
  ros::Rate naptime(sample_rate); // use to regulate loop rate

  while (ros::ok()) {

    ros::spinOnce(); // allow data update from callback;
    naptime.sleep(); // wait for remainder of specified period;
  }

  cv::destroyWindow("image_view");
}
