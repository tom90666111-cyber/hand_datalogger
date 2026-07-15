#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

//负责从vrpn读取相机数据，然后发布为新的话题"/optitrack/pose*"

ros::Publisher pub_pose1;
ros::Publisher pub_pose2;

// ── 位姿回调 ─────────────────────────────────────
void poseCallback1(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  geometry_msgs::PoseStamped out = *msg;
  out.header.stamp    = ros::Time::now();  // 覆盖时间戳
  out.header.frame_id = "world";
  pub_pose1.publish(out);

  // ROS_INFO_STREAM("Position: "
  //   << out.pose.position.x << ", "
  //   << out.pose.position.y << ", "
  //   << out.pose.position.z);
}

void poseCallback2(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  geometry_msgs::PoseStamped out = *msg;
  out.header.stamp    = ros::Time::now();  // 覆盖时间戳
  out.header.frame_id = "world";
  pub_pose2.publish(out);

  // ROS_INFO_STREAM("Position: "
  //   << out.pose.position.x << ", "
  //   << out.pose.position.y << ", "
  //   << out.pose.position.z);
}

// ── 速度回调 ─────────────────────────────────────
// void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
//   geometry_msgs::TwistStamped out = *msg;
//   out.header.stamp    = ros::Time::now();
//   out.header.frame_id = "world";
//   pub_twist.publish(out);
// }

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "optitrack_node");
  ros::NodeHandle n;

  // 话题名中的 RigidBody1 改为 Motive 里定义的刚体名称
  pub_pose1 = n.advertise<geometry_msgs::PoseStamped> ("/optitrack/pose1", 1);
  pub_pose2 = n.advertise<geometry_msgs::PoseStamped> ("/optitrack/pose2", 1);

  // 队列长度为 1，保证永远处理最新帧
  auto sub_pose1 = n.subscribe("/vrpn_client/RigidBody1/pose", 1, poseCallback1);
  auto sub_pose2 = n.subscribe("/vrpn_client/RigidBody2/pose", 1, poseCallback2);

  ros::spin();
  return 0;
}
