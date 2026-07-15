#include "AdsLib.h"
#include "AdsVariable.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"

//通过ads读取twincat数据并发布为话题
// ─── ADS 连接参数 ───────────────────────────────────
static const AmsNetId remoteNetId { 127, 0, 0, 1, 2, 2   };
static const AmsNetId localNetId  { 192, 168, 10, 1, 1, 2 };
static const char     remoteIpV4[] = "169.254.207.231";


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "twincat_node");
  ros::NodeHandle n;

  // ── 初始化 ADS 路由 ──────────────────────────────
  AdsSetLocalAddress(localNetId);
  long ret = AdsAddRoute(remoteNetId, remoteIpV4);
  ROS_INFO_STREAM("AdsAddRoute_Read: " << (ret == 0 ? "OK" : "FAILED"));
  if (ret != 0) return 1;

  // ── 话题发布者 ──────────────────────────────────
  auto pub_joint  = n.advertise<sensor_msgs::JointState>("/twincat/joint_states", 1);

  try {
    AdsDevice route { remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3 };

    // ── 变量句柄（循环外创建一次）───────────────────
    AdsVariable<std::array<double, 16>> CurrentPositionReal     {route, "VariableMAIN.CurrentPositionReal"  };

    ROS_INFO("Variable Connected!");

    // ── 关节名称（示例，按实际轴数修改）─────────────
    std::vector<std::string> joint_names;
    for (int i = 14; i < 16; i++)
      joint_names.push_back("joint_" + std::to_string(i));

    ros::Rate loop_rate(200);  // 10Hz
    while (ros::ok()) {
      ros::Time now = ros::Time::now();  // 统一时间戳

      // 读取所有变量
      std::array<double, 16> axis = CurrentPositionReal;

      // ── 发布 JointState ──────────────────────────
      sensor_msgs::JointState joint_msg;
      joint_msg.header.frame_id = "base_link";
      joint_msg.header.stamp    = now;
      joint_msg.name            = joint_names;
      for (int i = 11; i < 13; i++) {
        joint_msg.position.push_back(axis[i]);
      }
      // ROS_INFO_STREAM(joint_msg.position[1]);
      pub_joint.publish(joint_msg);

      ros::spinOnce();
      loop_rate.sleep();
    }

  } catch (const AdsException& ex) {
    ROS_ERROR_STREAM("ADS Exception: " << ex.what());
    return 1;
  }
  return 0;
}
