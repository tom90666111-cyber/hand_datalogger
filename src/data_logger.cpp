#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <mutex>
#include <string>
#include <iomanip>

//从关节角和相机话题读取数据并构成结构体

struct JointAndPose {
    uint32_t time;
    double xyz[3];
    double RB1_quat[4];
    double MCP_PIP[2];
};

JointAndPose data;
std::mutex data_mutex;
bool data_ready = false;

void poseCallback1(const geometry_msgs::PoseStamped::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(data_mutex);
    data.time = msg->header.seq;

    data.xyz[0] = msg->pose.position.x;
    data.xyz[1] = msg->pose.position.y;
    data.xyz[2] = msg->pose.position.z;

    data.RB1_quat[0] = msg->pose.orientation.x;
    data.RB1_quat[1] = msg->pose.orientation.y;
    data.RB1_quat[2] = msg->pose.orientation.z;
    data.RB1_quat[3] = msg->pose.orientation.w;
    data_ready = true;
};

// void poseCallback2(const geometry_msgs::PoseStamped::ConstPtr& msg){
//     std::lock_guard<std::mutex> lock(data_mutex);
//     data.RB2_quat[0] = msg->pose.orientation.x;
//     data.RB2_quat[1] = msg->pose.orientation.y;
//     data.RB2_quat[2] = msg->pose.orientation.z;
//     data.RB2_quat[3] = msg->pose.orientation.w;
//     data_ready = true;
// };

void mdipCallback(const sensor_msgs::JointState::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(data_mutex);
    data.MCP_PIP[0] = msg->position[0];
    data.MCP_PIP[1] = msg->position[1];
    data_ready = true;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "data_logger");
    ros::NodeHandle nh;

    ros::Subscriber sub_pose1 = nh.subscribe("/vrpn_client/RigidBody1/pose", 1, poseCallback1);
    // ros::Subscriber sub_pose2 = nh.subscribe("/vrpn_client/RigidBody2/pose", 1, poseCallback2);
    ros::Subscriber sub_MDIP = nh.subscribe("/twincat/joint_states", 1, mdipCallback);

    //rosrun twincat_talker data_logger ori_data1.txt
    std::string filename = "JointAngleData/ori_data1.txt";
    if (argc > 1) {
        filename = argv[1];  // 从命令行读取
    }

    // 创建/打开文件（没有则自动创建，有则追加）
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        ROS_ERROR("create file fail!!!");
        return -1;
    }

    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();

        outfile << std::fixed << std::setprecision(6)
            << data.time << "," << data.xyz[0] << "," 
            << data.xyz[1] << "," << data.xyz[2] << ","
            << data.RB1_quat[0] << "," << data.RB1_quat[1] << "," 
            << data.RB1_quat[2] << "," << data.RB1_quat[3] << ","
            << data.MCP_PIP[0] << "," << data.MCP_PIP[1] 
            << std::endl;

        outfile.flush();  // 立即写入磁盘

        ROS_INFO("t-xyz: [%d, %.3f, %.3f, %.3f]", 
        data.time,data.xyz[0],data.xyz[1],data.xyz[2]);

        ROS_INFO("RB1_quat: [%.3f, %.3f, %.3f, %.3f]", 
        data.RB1_quat[0],data.RB1_quat[1],data.RB1_quat[2],data.RB1_quat[3]);

        ROS_INFO("MCP_PIP: [%.3f, %.3f]",data.MCP_PIP[0],data.MCP_PIP[1]);

        loop_rate.sleep();
    }
    return 0;
} 