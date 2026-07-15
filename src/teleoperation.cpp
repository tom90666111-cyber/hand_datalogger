#include "AdsLib.h"
#include "AdsNotification.h"
#include "AdsVariable.h"
// #include <std_msgs/Float64.h>
// #include <std_msgs/Float64MultiArray.h>
#include "ros/ros.h"
// #include <iostream>
// #include <string>
// #include <sstream>
// #include <iomanip>
// #include "sensor_msgs/JointState.h"
#include "../include/math.hpp"
# include <actionlib/server/simple_action_server.h>
# include <control_msgs/FollowJointTrajectoryAction.h>
# include "std_msgs/Float64MultiArray.h"
#include"std_msgs/Float32MultiArray.h"  

# include <moveit_msgs/RobotTrajectory.h>
#include "std_msgs/String.h"
// #include "Eigen/Dense"
// #include "Eigen/Core"
// #include "Eigen/Geometry"
// #include "Eigen/StdVector"
// #include "std_srvs/Empty.h"
// #include "std_srvs/SetBool.h"
#include "sensor_msgs/JointState.h"
// #include "geometry_msgs/Twist.h"
// #include "geometry_msgs/TwistStamped.h"
// #include <pthread.h>
// #include <unistd.h>
// #include <sstream>
#include <vector>
#include "Log.h"
#include <std_msgs/Bool.h>
#include <iostream>
#include <fstream>
#include <boost/thread/mutex.hpp>

 
// #include "libs/robot.h"
// #include "libs/conversion.h"
#include "time.h"
#include <map>
#include <string>

using namespace std;

#define DOF 7

bool start_control = false;
bool joint_state_received = false; // 用于确保至少收到一次数据
bool real_time_mode = false;
int NUMBER_PER_CIRCLE = 524288;
int joint_min = 0;
int joint_max = 360;
int joint_execute_index = 0;
double joint6_temp = 0;
double joint7_temp = 0;
// 定义一个全局变量来保存最新的 JointState 数据
sensor_msgs::JointState latest_joint_state;
std::array<double, DOF> joint_pos;
std::array<double, DOF> position_read;
std::array<double, DOF> send_pos;
std::array<double, DOF> last_pos = {0,0,0,0,0,0,0};
std::vector<std::vector<double>> joint_traj;

const AmsNetId remoteNetId{192, 168, 0, 2, 1, 1};
const char remoteIpV4[] = "192.168.0.2";
AdsDevice route{remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};
AdsVariable<std::array<double, DOF>> current_state{route, "VariableMAIN.Joint_Position"};
AdsVariable<std::array<double, DOF>> target_pos{route, "Maxon.AxisFeedPositionAbsolute"};
AdsVariable<std::array<double, DOF>> target_position{route, "Maxon.TargetPositionNext"};
AdsVariable<bool> ROSControl{route, "MAIN.ROSControl"};
AdsVariable<bool> executeDone{route, "MAIN.ROSExecuteDone"};
AdsVariable<bool> sendDone{route, "MAIN.ROSReceived"};
AdsVariable<uint16_t> state{route, "VariableMAIN.CurrentJob"};
AdsVariable<double> elbow_1{route, "MAIN.EncoderJoint1"};
AdsVariable<double> elbow_2{route, "MAIN.EncoderJoint2"};
AdsVariable<array<float, 16> > float_array{ route, "MAIN.ROS_A" };

uint16_t state_value=210;
array<float, 16>  readVAR_read;

// 重命名类型为 Server
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

string filename = "/home/raowentao/flexible_manipulator_ws/src/twincat_talker/files/motor3_data.txt";  // 输出文件名
// 初始化文件并写入标题（可选）
ofstream outFile(filename, ios::trunc);  // 清空文件并写入标题
// 创建互斥锁
boost::mutex callback_mutex;

ros::Publisher joint_states_pub;
void appendDataToFile(const string& filename,
    double motorAngle,
    double computedTheta,
    double measuredTheta);

// ANSI 清屏控制码
void clearScreen() {
    std::cout << "\033[2J\033[H"; // 清屏并将光标移动到左上角
}

string double2string(double number){
    ostringstream oss;
    oss << fixed << setprecision(7) << number;
    // string str = oss.str();
    return oss.str();
}

void printTable(const std::vector<std::vector<std::string>>& table) {
    clearScreen(); // 清除屏幕
    
    // 打印表头与内容
    for (const auto& row : table) {
        for (const auto& cell : row) {
            std::cout << std::setw(15) << cell << " | "; // 格式化输出
        }
        std::cout << "\n-------------------------------------------------\n";
    }
}

void execute_callback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goalPtr, Server* moveit_server)
{
    // boost::mutex::scoped_lock lock(callback_mutex);
    if(!start_control) start_control = true;
    joint_state_received = false;
    // 用于存储 moveit 发送出来的轨迹数据
    moveit_msgs::RobotTrajectory moveit_tra;
    // 1、解析提交的目标值
    int n_joints = goalPtr->trajectory.joint_names.size();
    int n_tra_Points = goalPtr->trajectory.points.size();
    // ROS_INFO_STREAM("data received!");
 
    moveit_tra.joint_trajectory.header.frame_id = goalPtr->trajectory.header.frame_id;
    moveit_tra.joint_trajectory.joint_names = goalPtr->trajectory.joint_names;
    moveit_tra.joint_trajectory.points.resize(n_tra_Points);
    // ROS_INFO_STREAM("start processing");
 
    for(int i=0; i<n_tra_Points; i++) // 遍历每组路点
    {
        // ROS_INFO_STREAM("i:" << i <<"n_tra_Points:" << n_tra_Points);
        moveit_tra.joint_trajectory.points[i].positions.resize(n_joints);
        moveit_tra.joint_trajectory.points[i].velocities.resize(n_joints);
        moveit_tra.joint_trajectory.points[i].accelerations.resize(n_joints);
 
        moveit_tra.joint_trajectory.points[i].time_from_start = goalPtr->trajectory.points[i].time_from_start;
        for(int j=0;j<n_joints; j++) // 遍历每组路点中的每个关节数据
        {
            moveit_tra.joint_trajectory.points[i].positions[j] = goalPtr->trajectory.points[i].positions[j];
            moveit_tra.joint_trajectory.points[i].velocities[j] = goalPtr->trajectory.points[i].velocities[j];
        };
    }
    joint_execute_index = 0;
    
    ROS_INFO("The number of joints is %d.",n_joints);
    ROS_INFO("The waypoints number of the trajectory is %d.",n_tra_Points);
    ROS_INFO("Receive trajectory successfully");

     // 转换轨迹
    convertTrajectoryToVector(moveit_tra, joint_traj);

    moveit_server->setSucceeded();
}

void send_position(const sensor_msgs::JointState::ConstPtr& msg) {
    if(!start_control) start_control = true;
    // 更新全局变量，保存最新的 JointState 数据
    latest_joint_state = *msg;
    joint_state_received = true; // 标记为已收到消息
}

void mode_transition(const std_msgs::Bool::ConstPtr &msg){
    real_time_mode = msg -> data;
    ROS_INFO_STREAM("real_time_mode changed to : " << real_time_mode);
}

void joint_states_callback(ros::Publisher joint_states_pub, std::array<double, DOF> position_read)
{
    //        clock_t t1 = clock();
    sensor_msgs::JointState joint_states;
    double elbow = (elbow_1 + elbow_2) / 2;
    double cal_theta = rad2real(motor2theta(real2rad(position_read[3])));
    double res = elbow - cal_theta;
    double res_cal_motor = position_read[3] - rad2real(theta2motor(real2rad(elbow)));
    string file_name = "/home/raowentao/flexible_manipulator_ws/src/twincat_talker/files/motor_theta_calib0430.txt";

    // ROS_INFO_STREAM("res_motor2theta: " << res << " ,res_theta2motor: " << res_cal_motor);
  
    //shoulder_1
    joint_states.position.push_back(real2rad(position_read[0])); // write data into standard ros msg
    joint_states.name.push_back("shoulder_1_joint");
    //shoulder_2
    joint_states.position.push_back(real2rad(position_read[1])); // write data into standard ros msg
    joint_states.name.push_back("shoulder_2_joint");
    //shoulder_3
    joint_states.position.push_back(real2rad(position_read[2])); // write data into standard ros msg
    joint_states.name.push_back("shoulder_3_joint");
    //elbow_1
    joint_states.position.push_back(motor2theta(real2rad(position_read[3]))); // write data into standard ros msg
    joint_states.name.push_back("elbow_1_joint");
    //elbow_2
    joint_states.position.push_back(motor2theta(real2rad(position_read[3]))); // write data into standard ros msg
    joint_states.name.push_back("elbow_2_joint");
    //wrist_1
    joint_states.position.push_back(real2rad(position_read[4])); // write data into standard ros msg
    joint_states.name.push_back("wrist_1_joint");
    //wrist_2
    joint_states.position.push_back(real2rad(position_read[5])); // write data into standard ros msg
    joint_states.name.push_back("wrist_2_joint");
    //wrist_3
    joint_states.position.push_back(real2rad(position_read[6])); // write data into standard ros msg
    joint_states.name.push_back("wrist_3_joint");
    joint_states.header.stamp = ros::Time::now(); //记录时间

    //记录数据时一定一定要把math.hpp里的补偿部分去掉！！！！！！
    // appendDataToFile(file_name, real2rad(position_read[3]), motor2theta(real2rad(position_read[3])), real2rad(elbow));

    joint_states_pub.publish(joint_states); // publish data
 
    //        cout << (clock() - t1) * 1.0 / CLOCKS_PER_SEC * 1000 << endl;
}

// 定时器回调函数：周期性保存 JointState 数据
void timerCallback(const ros::TimerEvent&) {
    // boost::mutex::scoped_lock lock(callback_mutex);
    position_read = current_state;
    // ROS_INFO_STREAM("theta: " << motor2theta(position_read[3]));
    // std::swap(position_read[5],position_read[6]);
    joint_states_callback(joint_states_pub, position_read);

    //一旦启用实时控制就无法执行下面程序，之后可以修改
    if(!joint_traj.empty() && !joint_state_received){
        std::copy(joint_traj[joint_execute_index].begin(), joint_traj[joint_execute_index].end(), joint_pos.begin());
        std::swap(joint_pos[0],joint_pos[1]);
        std::swap(joint_pos[1],joint_pos[2]);
        std::swap(joint_pos[2],joint_pos[3]);
        if(joint_execute_index < joint_traj.size() - 1){
            joint_execute_index += 1;
        }
    }

    if (joint_state_received) {
        // 保存最新数据（在此示例中，直接打印）
        // ROS_INFO("Saving JointState data:");
        for (size_t i = 0; i < latest_joint_state.name.size(); ++i) {
            joint_pos[i] = latest_joint_state.position[i];
        }
    }

    for(int i = 0;i < joint_pos.size(); i++){
        if(i == 3) joint_pos[i] = theta2motor(joint_pos[i]);
        joint_pos[i] = rad2real(joint_pos[i]);
    }
    if(start_control){
        send_pos = real2encoder(joint_pos);
        // double temp;
        // temp = send_pos[5];
        // send_pos[5] = send_pos[6];
        // send_pos[6] = temp;
        ROS_INFO_STREAM("joints: " << send_pos[0] << " , " << send_pos[1] << " , " << send_pos[2] << " , " << send_pos[3] << " , " << send_pos[4] << " , " << send_pos[5] << " , " << send_pos[6]);

    } 
    for(int i = 0;i < send_pos.size(); i++){
        last_pos[i] = send_pos[i];
    }

    // ROS_INFO_STREAM("sending start");
    target_position = send_pos;
    sendDone = true;
}

void appendDataToFile(const string& filename,
                       double motorAngle,
                       double computedTheta,
                       double measuredTheta) {
    // 以追加模式打开文件（ios::app）
    ofstream outFile(filename, ios::app);
    if (!outFile.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        return;
    }

    // 写入新的数据行
    outFile << std::fixed << std::setprecision(6)
            << motorAngle << " "
            << computedTheta << " "
            << measuredTheta << "\n";

    outFile.close();
}

// 定时器回调函数的线程执行
void timerThread(ros::NodeHandle& n) {
    ros::Timer timer = n.createTimer(ros::Duration(0.02), timerCallback);
    ros::spin();  // 启动事件循环，处理定时器回调
}
array<float , 16> joint_value_array = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
array<float , 20> joint_value_array_initial = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void Write_float_array_ros(const std_msgs::Float32MultiArray& msg)
{
    //通过msg获取并操作订阅到的数据
    std::move(std::begin(msg.data),std::end(msg.data),joint_value_array_initial.begin());

    for(int i=0;i<7;i++)
    {

        joint_value_array[i]=(48*sin((joint_value_array_initial[i]-17)* M_PI / 180.0)+14.0338)/3.5;
    }
    for(int i=7;i<10;i++)
    {
        joint_value_array[i]=(48*sin((joint_value_array_initial[i+1]-17)* M_PI / 180.0)+14.0338)/3.5;
    }
    for(int i=10;i<13;i++)
    {
        joint_value_array[i]=(48*sin((joint_value_array_initial[i+2]-17)* M_PI / 180.0)+14.0338)/3.5;
    }
    for(int i=13;i<16;i++)
    {
        joint_value_array[i]=(48*sin((joint_value_array_initial[i+3]-17)* M_PI / 180.0)+14.0338)/3.5;
    }
    for(int i=0;i<16;i++)
    {

        cout<<joint_value_array[i]<<" ";
    }    
    float_array=joint_value_array;
    state=state_value;
}
int main(int argc, char *argv[]) {
  
  // ROS node init
  ros::init(argc, argv, "read_information");
  ros::NodeHandle n;
  joint_states_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 10);
  ros::Subscriber sub = n.subscribe("/move_group/fake_controller_joint_states", 1000, send_position);
  ros::Subscriber sub_1 = n.subscribe("/move_group/real_time_mode", 1000, mode_transition);
  ros::Subscriber writter = n.subscribe("left_hand_data", 1000, &Write_float_array_ros);
  std_msgs::Float32MultiArray hand_msg;
  hand_msg.data.resize(16);
  ros::Publisher hand_reader = n.advertise<std_msgs::Float32MultiArray>("read", 1000);
//   ros::AsyncSpinner spinner(2);  // 使用2个线程
//   spinner.start();
  // 创建一个定时器，以 0.02 秒（20 毫秒）的周期调用定时器回调函数
//   ros::Timer timer = n.createTimer(ros::Duration(0.05), timerCallback);
  // 启动一个线程来运行定时器回调
  boost::thread timer_thread(timerThread, boost::ref(n));
  // 创建 action 对象(NodeHandle，话题名称，回调函数解析传入的目标值，服务器是否自启动)
  Server moveit_server(n,"gen3_rightarm_controller/follow_joint_trajectory", boost::bind(&execute_callback, _1, &moveit_server), false);

  // 手动启动服务器
  moveit_server.start();
  Logger::logLevel=4;
//   ROS_INFO_STREAM("333");
  
  std::array<double, DOF> position_write;
  bool bl;
  uint16_t ut;
  double db;

  ROS_INFO_STREAM("running good");

  ros::Rate loop_rate(50);

  std::vector<std::vector<std::string>> information_table = {
    {"Axis", "0", "1", "2", "3", "4", "5", "6"},
    {"Pos", "0", "0", "0", "0", "0", "0", "0"},
    {"Target_pos", "0", "0", "0", "0", "0", "0", "0"}
  };
//   ROS_INFO_STREAM("111");
  ROSControl = true;
  state = 200;
  send_pos = current_state;
  send_pos = real2encoder(send_pos);

//   readVAR_read=float_array;

//   for(int i=0;i<16;i++)
//   {
//       hand_msg.data[i] = readVAR_read[i];
//   }
//   hand_reader.publish(hand_msg);

  // 等待定时器线程结束（通常这一步是不会到达的，因为定时器会持续运行）
  timer_thread.join();
  return 0;
}