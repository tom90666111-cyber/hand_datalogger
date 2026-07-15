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

#define DOF 23
#define ARM_DOF 7
#define HAND_DOF 16

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
std::array<double, ARM_DOF> r_joint_pos;
std::array<double, ARM_DOF> l_joint_pos;
std::array<double, ARM_DOF> r_encoder_pos;
std::array<double, ARM_DOF> l_encoder_pos;
std::array<double, DOF> position_read;
std::array<double, HAND_DOF> hand_position_read;

std::array<double, DOF> send_pos;
std::vector<std::vector<double>> r_joint_traj;
std::vector<std::vector<double>> l_joint_traj;
double thre=1500;
double diff_elbow=561209;
double last_elbow=0;
bool elbow_flag=false;
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

ros::Time start_time;  
bool is_first_data = true;  // 是否是第一次接收手部遥操作数据
bool finish_interpolation=false;// 是否停止手部遥操作数据插值
float interpolation_time=5.0;

ros::Subscriber writter;
array<double , 16> joint_value_array = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
array<double , 20> joint_value_array_initial = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint32_t state_value=200;
array<double , 20> hand_joint_value_array_initial_read = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
std::vector<std::string> hand_joint_name = {"l_Jthumb1", "l_Jthumb2", "l_Jthumb3", "l_Jthumb4", "l_Jthumb5", "l_Jthumb6", "l_Jthumb7", "l_Jthumb8",
                                             "l_Jindex1", "l_Jindex2", "l_Jindex3", "l_Jindex4", "l_Jindex5", "l_Jindex6", "l_Jindex7", "l_Jindex8", 
                                             "l_Jmid1", "l_Jmid2", "l_Jmid3", "l_Jmid4", "l_Jmid5", "l_Jmid6", "l_Jmid7", "l_Jmid8", 
                                             "l_Jring1", "l_Jring2", "l_Jring3", "l_Jring4", "l_Jring5", "l_Jring6", "l_Jring7", "l_Jring8", 
                                            "l_Jlittle1", "l_Jlittle2", "l_Jlittle3", "l_Jlittle4", "l_Jlittle5", "l_Jlittle6", "l_Jlittle7", "l_Jlittle8"};

// 重命名类型为 Server
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

string filename = "/home/raowentao/flexible_manipulator_ws/src/twincat_talker/files/motor3_data.txt";  // 输出文件名
// 初始化文件并写入标题（可选）
ofstream outFile(filename, ios::trunc);  // 清空文件并写入标题
// 创建互斥锁
boost::mutex callback_mutex;

ros::Publisher joint_states_pub;
ros::Publisher all_joint_angle_pub;


class MovingAverageFilter {
private:
    static constexpr size_t WINDOW_SIZE = 9;  // 窗口大小
    using Window = std::deque<double>;
    std::array<Window, 7> windows;            // 每个维度一个窗口
    std::array<double, 7> sums;               // 每个窗口的当前和
    std::array<bool, 7> initialized;          // 各维度是否已初始化

public:
    MovingAverageFilter() {
        // 初始化标记为未初始化
        initialized.fill(false);
        // 初始时窗口为空
        for (auto& win : windows) {
            win.clear();
        }
        sums.fill(0.0);
    }

    std::array<double, 7> filter(const std::array<double, 7>& raw_pos) {
        std::array<double, 7> filtered_pos;

        for (size_t i = 0; i < 7; ++i) {
            if (!initialized[i]) {
                // 首次初始化：用原始值填充整个窗口
                windows[i].assign(WINDOW_SIZE, raw_pos[i]);
                sums[i] = raw_pos[i] * WINDOW_SIZE;  // 总和为初始值 * 窗口大小
                initialized[i] = true;
            } else {
                // 正常滑动窗口逻辑：移除最早值，添加新值，更新总和
                sums[i] -= windows[i].front();
                windows[i].pop_front();
                windows[i].push_back(raw_pos[i]);
                sums[i] += raw_pos[i];
            }

            // 计算平均值（始终使用固定窗口大小，因初始化后窗口大小固定为WINDOW_SIZE）
            filtered_pos[i] = sums[i] / WINDOW_SIZE;
        }

        return filtered_pos;
    }

    // 重置滤波器（可选）
    void reset() {
        initialized.fill(false);
        for (auto& win : windows) {
            win.clear();
        }
        sums.fill(0.0);
    }
};

MovingAverageFilter filter;



double compute_tip_angle(double dp_angle)
{
    return 2*asin(sin(real2rad(36))/sin(real2rad(45))*(sin(dp_angle/2)));
}

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

    bool use_right_arm = false;
    bool use_left_arm = true;
    
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
    
    // 定义本地关节名称列表和索引映射
    // std::vector<std::string> local_joint_names = {"r_shoulder_1_joint","r_shoulder_2_joint","r_shoulder_3_joint","r_elbow_1_joint","r_wrist_1_joint","r_wrist_2_joint","r_wrist_3_joint",
    // "l_shoulder_1_joint","l_shoulder_2_joint","l_shoulder_3_joint","l_elbow_1_joint","l_wrist_1_joint","l_wrist_2_joint","l_wrist_3_joint"}; // 根据实际情况修改
    // std::vector<int> joint_mapping(local_joint_names.size(), -1); // 存储目标关节在本地关节中的索引
    

    if(goalPtr->trajectory.joint_names[0] == "r_shoulder_1_joint"){
        use_right_arm = true;
                cout<<"use right"<<endl;

    }else if(goalPtr->trajectory.joint_names[0] == "l_shoulder_1_joint"){
        use_left_arm = true;
        cout<<"use left"<<endl;

    }
    
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
    if(use_right_arm) convertTrajectoryToVector(moveit_tra, r_joint_traj);
    else if(use_left_arm) convertTrajectoryToVector(moveit_tra, l_joint_traj);

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
    double cal_theta = rad2real(r_motor2theta(real2rad(position_read[3])));
    double res = elbow - cal_theta;
    double res_cal_motor = position_read[3] - rad2real(r_theta2motor(real2rad(elbow)));
    // string file_name = "/home/gst/gen3_robot/flexible_manipulator/src/twincat_talker/files/motor_theta_calib0515.txt";

    // ROS_INFO_STREAM("res_motor2theta: " << res << " ,res_theta2motor: " << res_cal_motor);
  
   //shoulder_1
    joint_states.position.push_back(real2rad(0)); // write data into standard ros msg
    joint_states.name.push_back("r_shoulder_1_joint");
    //shoulder_2
    joint_states.position.push_back(real2rad(0)); // write data into standard ros msg
    joint_states.name.push_back("r_shoulder_2_joint");
    //shoulder_3
    joint_states.position.push_back(real2rad(0)); // write data into standard ros msg
    joint_states.name.push_back("r_shoulder_3_joint");
    //elbow_1
    joint_states.position.push_back(real2rad(0)); // write data into standard ros msg
    joint_states.name.push_back("r_elbow_1_joint");
    //elbow_2
    joint_states.position.push_back(real2rad(0)); // write data into standard ros msg
    joint_states.name.push_back("r_elbow_2_joint");
    //wrist_1
    joint_states.position.push_back(real2rad(0)); // write data into standard ros msg
    joint_states.name.push_back("r_wrist_1_joint");
    //wrist_2
    joint_states.position.push_back(real2rad(0)); // write data into standard ros msg
    joint_states.name.push_back("r_wrist_2_joint");
    //wrist_3
    joint_states.position.push_back(real2rad(0)); // write data into standard ros msg
    joint_states.name.push_back("r_wrist_3_joint");


    //shoulder_1
    joint_states.position.push_back(real2rad(position_read[0])); // write data into standard ros msg
    joint_states.name.push_back("l_shoulder_1_joint");
    //shoulder_2
    joint_states.position.push_back(real2rad(position_read[1])); // write data into standard ros msg
    joint_states.name.push_back("l_shoulder_2_joint");
    //shoulder_3
    joint_states.position.push_back(real2rad(position_read[2])); // write data into standard ros msg
    joint_states.name.push_back("l_shoulder_3_joint");
    //elbow_1
    joint_states.position.push_back(l_motor2theta(real2rad(position_read[3]))); // write data into standard ros msg
    joint_states.name.push_back("l_elbow_1_joint");
    //elbow_2
    joint_states.position.push_back(l_motor2theta(real2rad(position_read[3]))); // write data into standard ros msg
    joint_states.name.push_back("l_elbow_2_joint");
    //wrist_1
    joint_states.position.push_back(real2rad(position_read[4])); // write data into standard ros msg
    joint_states.name.push_back("l_wrist_1_joint");
    //wrist_2
    joint_states.position.push_back(real2rad(position_read[5])); // write data into standard ros msg
    joint_states.name.push_back("l_wrist_2_joint");
    //wrist_3
    joint_states.position.push_back(real2rad(position_read[6])); // write data into standard ros msg
    joint_states.name.push_back("l_wrist_3_joint");
    joint_states.header.stamp = ros::Time::now(); //记录时间

    //记录数据时一定一定要把math.hpp里的补偿部分去掉！！！！！！
    // appendDataToFile(file_name, real2rad(position_read[3]), l_motor2theta(real2rad(position_read[3])), real2rad(elbow));

    joint_states_pub.publish(joint_states); // publish data
    int num=0;
    for(int i=0;i<HAND_DOF;i++)
    {
        if(i==0)
        {
            hand_joint_value_array_initial_read[i]=real2rad((asin((position_read[i+7] * M_PI/180.0 * 3.5 ) / 48) * 180.0/M_PI) * 2);
            joint_states.position.push_back(hand_joint_value_array_initial_read[i]/2.0); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;
            joint_states.position.push_back(hand_joint_value_array_initial_read[i]/2.0); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;
        }
        //拇指三个弯曲要变负，匹配urdfV3

        //也要处以缩放因子，但要减5度，所以在tactile data里面除了

        else if(i==1)
        {

            hand_joint_value_array_initial_read[i]=(real2rad((asin((position_read[i+7] * M_PI/180.0 * 3.5 - 14.0338) / 48) * 180.0/M_PI + 17.0) * 2)-10.0/180*M_PI)/1.08;
            joint_states.position.push_back(hand_joint_value_array_initial_read[i]/2.0); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;
            joint_states.position.push_back(hand_joint_value_array_initial_read[i]/2.0); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;

        }
        else if(i==2)
        {

            hand_joint_value_array_initial_read[i]=real2rad((asin((position_read[i+7] * M_PI/180.0 * 3.5 - 11.695) / 40.0) * 180.0/M_PI + 17.0) * 2)/1.36;
            joint_states.position.push_back(hand_joint_value_array_initial_read[i]/2.0); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;
            joint_states.position.push_back(hand_joint_value_array_initial_read[i]/2.0); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;

        }
        else if(i==3)
        {

            hand_joint_value_array_initial_read[i]=real2rad((asin((position_read[i+7] * M_PI/180.0 * 3.5 ) / 49.32) * 180.0/M_PI) * 2)/1.1889;
            joint_states.position.push_back(hand_joint_value_array_initial_read[i]/2); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;
            joint_states.position.push_back(hand_joint_value_array_initial_read[i]/2.0); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;

        }
        //手指侧展要变负，而且转换关系有变化，匹配urdfV3
        else if (i==4 || i==7 || i==10 || i==13)
        {
            hand_joint_value_array_initial_read[i]=real2rad((asin((position_read[i+7] * M_PI/180.0 * 3.5 ) / 48.0) * 180.0/M_PI) * 2);
            joint_states.position.push_back(hand_joint_value_array_initial_read[i]/2.0); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;
            joint_states.position.push_back(hand_joint_value_array_initial_read[i]/2.0); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;

        }
        else if(i==6||i==9||i==12||i==15)
        {
            hand_joint_value_array_initial_read[i]=real2rad((asin((position_read[i+7] * M_PI/180 * 3.5 - 11.695) / 40.0) * 180/M_PI + 17.0) * 2)/1.36;
            joint_states.position.push_back(hand_joint_value_array_initial_read[i]/2.0); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;
            joint_states.position.push_back(hand_joint_value_array_initial_read[i]/2.0); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;
        }
        

        else
        {
            hand_joint_value_array_initial_read[i]=real2rad((asin((position_read[i+7] * M_PI/180.0 * 3.5 - 14.0338) / 48.0) * 180/M_PI + 17.0) * 2)-10.0/180*M_PI;
            joint_states.position.push_back(hand_joint_value_array_initial_read[i]/2.0); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;
            joint_states.position.push_back(hand_joint_value_array_initial_read[i]/2.0); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;
        }
        //多一个指尖的处理，通过上一个关节的计算得到，给urdfV3
        if(i==6 || i==9 || i==12 || i==15)
        {
            joint_states.position.push_back(compute_tip_angle(hand_joint_value_array_initial_read[i])/2.0); // write data into standard ros msg
            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;

            joint_states.position.push_back(compute_tip_angle(hand_joint_value_array_initial_read[i])/2.0); // write data into standard ros msg

            joint_states.name.push_back(hand_joint_name[num]);
            joint_states.header.stamp = ros::Time::now(); //记录时间
            num++;


        }

    }
    all_joint_angle_pub.publish(joint_states); // publish data

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
    if(!r_joint_traj.empty() && !joint_state_received){
        std::copy(r_joint_traj[joint_execute_index].begin(), r_joint_traj[joint_execute_index].end(), r_joint_pos.begin());
        std::swap(r_joint_pos[0],r_joint_pos[1]);
        std::swap(r_joint_pos[1],r_joint_pos[2]);
        std::swap(r_joint_pos[2],r_joint_pos[3]);
        if(joint_execute_index < r_joint_traj.size() - 1){
            joint_execute_index += 1;
        }
    }

    if(!l_joint_traj.empty() && !joint_state_received){
        std::copy(l_joint_traj[joint_execute_index].begin(), l_joint_traj[joint_execute_index].end(), l_joint_pos.begin());
        std::swap(l_joint_pos[0],l_joint_pos[1]);
        std::swap(l_joint_pos[1],l_joint_pos[2]);
        std::swap(l_joint_pos[2],l_joint_pos[3]);
        if(joint_execute_index < l_joint_traj.size() - 1){
            joint_execute_index += 1;
        }
    }

    if (joint_state_received) {
        // 保存最新数据（在此示例中，直接打印）
        // ROS_INFO("Saving JointState data:");
        for (size_t i = 0; i < latest_joint_state.name.size(); ++i) {
            l_joint_pos[i] = latest_joint_state.position[i];

        }
    }

    for(int i = 0;i < r_joint_pos.size(); i++){
        if(i == 3){
            l_joint_pos[i] = l_theta2motor(l_joint_pos[i]);
        } 
        l_joint_pos[i] = rad2real(l_joint_pos[i]);
    }
    if(l_joint_pos[1]<-5)
    {
        l_joint_pos[1]=-5;
    }
    if(l_joint_pos[5]<0)
    {
        l_joint_pos[5]=0;
    }
    if(l_joint_pos[6]<-30)
    {
        l_joint_pos[6]=-30;
    }
        if(l_joint_pos[6]>30)
    {
        l_joint_pos[6]=30;
    }
    if(start_control){
        l_encoder_pos = l_real2encoder(l_joint_pos);

    } 
    auto filtered_pos = filter.filter(l_encoder_pos);
    for(int i = 0;i < ARM_DOF;i++){
        send_pos[i] = filtered_pos[i];
    }

    for(int j=0;j<HAND_DOF;j++)
    {
        send_pos[j+ARM_DOF] = joint_value_array[j];
    }
    cout<<endl;
    // if(!elbow_flag)
    // {
    //     elbow_flag=true;
    //     last_elbow=send_pos[3];
    // }
    // else
    // {
    //     diff_elbow=send_pos[3]-last_elbow;

    //     if(diff_elbow<thre &&diff_elbow>-thre)
    //     {
    //         send_pos[3]=last_elbow;
    //         last_elbow=send_pos[3];
    //     }
    // }

    for(int i=0;i<DOF;i++)
    {

        cout<<send_pos[i]<<" ";
    }    

        cout<<endl;

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


//手部的遥操作
void Write_float_array_ros(const std_msgs::Float32MultiArray& msg)
{
    
 
    // 如果是第一次接收数据，记录开始时间
    if (is_first_data) {
        start_time = ros::Time::now();
        is_first_data = false;
    }
    
    //通过msg获取并操作订阅到的数据
    std::move(std::begin(msg.data),std::end(msg.data),joint_value_array_initial.begin());
    cout<<"hand: ";
    for(int k=0;k<16;k++)
    {
        joint_value_array_initial[k]=joint_value_array_initial[k]*180.0/M_PI;
        cout<<joint_value_array_initial[k]<<" ";
    }
    cout<<endl;
    for (size_t i = 0; i < joint_value_array_initial.size(); ++i) {
        double value = joint_value_array_initial[i];
        
        if (i == 0) {
            joint_value_array[i] = (std::sin(value / 2.0 * M_PI / 180.0) * 48.0) / (3.5 * M_PI / 180.0);
        }
        else if (i == 1) {
            joint_value_array[i] = ((std::sin(((value * 1.08 + 10.0/180*M_PI) / 2.0 - 17.0) * M_PI / 180.0) * 48.0) + 14.0338) / (3.5 * M_PI / 180.0);
        }
        else if (i == 2) {
            joint_value_array[i] = ((std::sin((value * 1.36 / 2.0 - 17.0) * M_PI / 180.0) * 40.0) + 11.695) / (3.5 * M_PI / 180.0);
        }
        else if (i == 3) {
            joint_value_array[i] = (std::sin(value * 1.1889 / 2.0 * M_PI / 180.0) * 49.32) / (3.5 * M_PI / 180.0);
        }
        else if (i == 4 || i == 7 || i == 10 || i == 13) {
            joint_value_array[i] = (std::sin(value / 2.0 * M_PI / 180.0) * 48.0) / (3.5 * M_PI / 180.0);
        }
        else if (i == 6 || i == 9 || i == 12 || i == 15) {
            joint_value_array[i] = ((std::sin((value * 1.36 / 2.0 - 17.0) * M_PI / 180.0) * 40.0) + 11.695) / (3.5 * M_PI / 180.0);
        }
        else {
            joint_value_array[i] = ((std::sin(((value + 10.0/180*M_PI) / 2.0 - 17.0) * M_PI / 180.0) * 48.0) + 14.0338) / (3.5 * M_PI / 180.0);
        }
    }
   
        if(joint_value_array[0]<-300)
        {
            joint_value_array[0]=-300;
        }        
        if(joint_value_array[0]>300)
        {
            joint_value_array[0]=300;
        }
              if(joint_value_array[1]<5)
        {
            joint_value_array[1]=0;
        }
        if(joint_value_array[2]<5)
        {
            joint_value_array[2]=0;
        }
        if(joint_value_array[3]<5)
        {
            joint_value_array[3]=0;
        }
        if(joint_value_array[5]<0)
        {
            joint_value_array[5]=0;
        }
        if(joint_value_array[6]<5)
        {
            joint_value_array[6]=0;
        }
        //侧展1处理！！！

        if(joint_value_array[4]<-120)
        {
            joint_value_array[4]=-120;
        }
                if(joint_value_array[4]>10)
        {
            joint_value_array[4]=10;
        }

        //侧展2处理！！！

        if(joint_value_array[7]<-40)
        {
            joint_value_array[7]=-40;
        }
                if(joint_value_array[7]>30)
        {
            joint_value_array[7]=30;
        }


        if(joint_value_array[8]<0)
        {
            joint_value_array[8]=0;
        }
        if(joint_value_array[9]<5)
        {
            joint_value_array[9]=0;
        }

        //侧展3处理！！！
 
        if(joint_value_array[10]<-40)
        {
            joint_value_array[10]=-40;
        }
                if(joint_value_array[10]>40)
        {
            joint_value_array[10]=40;
        }



        if(joint_value_array[11]<0)
        {
            joint_value_array[11]=0;
        }
        if(joint_value_array[12]<5)
        {
            joint_value_array[12]=0;
        }

        //侧展4处理！！！

        if(joint_value_array[13]<-10)
        {
            joint_value_array[13]=-10;
        }
                if(joint_value_array[13]>120)
        {
            joint_value_array[13]=120;
        }



        
        if(joint_value_array[14]<0)
        {
            joint_value_array[14]=0;
        }
        if(joint_value_array[15]<5)
        {
            joint_value_array[15]=0;
        }



    for(int j=0;j<16;j++)
    {

        if(joint_value_array[j]>650)
        {
            joint_value_array[j]=650;
        }
    }
    if(joint_value_array[6]>580)
        {
            joint_value_array[6]=580;
        }
        // if(joint_value_array[9]>500)
        // {
        //     joint_value_array[9]=500;
        // }
    if(!finish_interpolation)
    {
        // 计算当前时间与开始时间的差值（秒）
        ros::Duration elapsed_time = ros::Time::now() - start_time;
        double elapsed_seconds = elapsed_time.toSec();
        // 如果时间小于3秒，进行插值
        if (elapsed_seconds < interpolation_time) {
            // 计算插值比例 (0到1之间)
            double ratio = elapsed_seconds / interpolation_time;
            
            // 对每个元素进行插值计算
            for (size_t i = 0; i < joint_value_array.size(); ++i) {
                joint_value_array[i] = 0.0 + ratio * joint_value_array[i];
            }
        }
        else{
            finish_interpolation=true;
        }
    }
    // float_array=joint_value_array;
    // ROSReceived=true;
    // int_single=state_value;
}

int main(int argc, char *argv[]) {
  
  // ROS node init
  ros::init(argc, argv, "read_information");
  ros::NodeHandle n;
    state = 200;

  joint_states_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 10);
  ros::Subscriber sub = n.subscribe("/move_group/fake_controller_joint_states", 1000, send_position);
  ros::Subscriber sub_1 = n.subscribe("/move_group/real_time_mode", 1000, mode_transition);
  writter = n.subscribe("/predicted_actions_hand", 10, &Write_float_array_ros);
  all_joint_angle_pub = n.advertise<sensor_msgs::JointState>("/all_joint_angle", 10);

//   ros::AsyncSpinner spinner(2);  // 使用2个线程
//   spinner.start();
  // 创建一个定时器，以 0.02 秒（20 毫秒）的周期调用定时器回调函数
//   ros::Timer timer = n.createTimer(ros::Duration(0.05), timerCallback);
  // 启动一个线程来运行定时器回调
  boost::thread timer_thread(timerThread, boost::ref(n));
  // 创建 action 对象(NodeHandle，话题名称，回调函数解析传入的目标值，服务器是否自启动)
  Server moveit_server(n,"gen3_leftarm_controller/follow_joint_trajectory", boost::bind(&execute_callback, _1, &moveit_server), false);

  // 手动启动服务器
  moveit_server.start();
  Logger::logLevel=4;
//   ROS_INFO_STREAM("333");
  
  std::array<double, DOF> position_write;
  bool bl;
  uint16_t ut;
  double db;

  ROS_INFO_STREAM("running good");

  ros::Rate loop_rate(300);

  std::vector<std::vector<std::string>> information_table = {
    {"Axis", "0", "1", "2", "3", "4", "5", "6"},
    {"Pos", "0", "0", "0", "0", "0", "0", "0"},
    {"Target_pos", "0", "0", "0", "0", "0", "0", "0"}
  };
//   ROS_INFO_STREAM("111");
  ROSControl = true;
  send_pos = current_state;
  for(int i = 0 ;i < ARM_DOF;i++){
    l_encoder_pos[i] = send_pos[i];
  }

  l_encoder_pos = l_real2encoder(l_encoder_pos);

  for(int i = 0 ;i < ARM_DOF;i++){
    send_pos[i] = l_encoder_pos[i];
  }
//     for(int j = 0 ;j< HAND_DOF;j++){
//     send_pos[j+HAND_DOF] = 0;
//   }
    // for(int i=0;i<HAND_DOF;i++)
    // {


    //     joint_value_array[i]=0;
    // }
  // 等待定时器线程结束（通常这一步是不会到达的，因为定时器会持续运行）
  timer_thread.join();
  return 0;
}