#include "rgbd_scan/rgbd_scan.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "scanernode");

  RgbdScan scaner;
  scaner.ScanerInit();
  scaner.ScanerStart();

  double sample_rate = 30;        // 30HZ
  ros::Rate naptime(sample_rate); // use to regulate loop rate
  while (ros::ok()) {

    ros::spinOnce(); // allow data update from callback;
    naptime.sleep(); // wait for remainder of specified period;
  }
}

