#include "AdsLib.h"
#include "AdsNotification.h"
#include "AdsVariable.h"
#include "ros/ros.h"

int main(int argc, char *argv[]) {
  // ROS node init
  ros::init(argc, argv, "tctalker");
  ros::NodeHandle n;

  ROS_INFO_STREAM("running good");

  static const AmsNetId remoteNetId{172, 17, 64, 1, 1, 1};
  static const char remoteIpV4[] = "172.17.64.1";

  AdsDevice route{remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};

  AdsVariable<double> variable_to_write{route, "MAIN.myFloatVAR"};

  ros::Rate loop_rate(0.1);

  while (ros::ok()) {
    variable_to_write = 12.0;
    ROS_INFO_STREAM("changed variable on sps");
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
