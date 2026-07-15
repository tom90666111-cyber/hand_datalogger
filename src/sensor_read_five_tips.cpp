#include <iostream>
#include <vector>
#include <serial/serial.h>
#include <chrono>
#include <thread>
#include <iomanip>
#include "ros/ros.h"
#include <signal.h>
#include <algorithm>
#include"std_msgs/Int32MultiArray.h"          
#include"std_msgs/String.h"
using namespace std;

uint8_t data_read_single;
int force_dp[219];
int force_ip[183];
serial::Serial ser;
int raw_data_dp[286]; 
int raw_data_ip[250]; 

// 读取数据
int* read_data_dp() 
{ 
    // 从串口读取数据 
    for(int i=0;i<286;i++)
    {
        ser.read(&data_read_single, 1);
        raw_data_dp[i]=data_read_single;
        if (raw_data_dp[i] & 0x80)
        {
            raw_data_dp[i]=raw_data_dp[i]-256;

        }
    }
    memcpy(force_dp,raw_data_dp+35,3*4);
    memcpy(force_dp+3,raw_data_dp+65,216*4);
        for(int i=0;i<73;i++)
    {
        if(force_dp[i*3+2]<0)
        {
            force_dp[i*3+2]=force_dp[i*3+2]+256;
        }
    }
    return force_dp; 
}

// 读取数据
int* read_data_ip() 
{   
    // 从串口读取数据 
    for(int i=0;i<250;i++)
    {
        
        ser.read(&data_read_single, 1);
        raw_data_ip[i]=data_read_single;
        if (raw_data_ip[i] & 0x80)
        {
            raw_data_ip[i]=raw_data_ip[i]-256;
        }
    }
    memcpy(force_ip,raw_data_ip+35,3*4);
    memcpy(force_ip+3,raw_data_ip+65,180*4);
    for(int i=0;i<61;i++)
    {
        if(force_ip[i*3+2]<0)
        {
            force_ip[i*3+2]=force_ip[i*3+2]+256;
        }
    }
    return force_ip; 
}

// 提取指腹数据
// pair<vector<int>, vector<int>> extract_data_ip(serial::Serial& ser) 
// {
//     vector<int32_t> data = read_data(ser,250); 
//     vector<int32_t> part1(data.begin() + 35, data.begin() + 38); 
//     vector<int32_t> part2(data.begin() + 65, data.begin() + 245); 
//     return {part1, part2}; 
// }

// // 提取指尖数据
// pair<vector<int>, vector<int>> extract_data_dp(serial::Serial& ser) 
// {
//     vector<int32_t> data = read_data(ser,286); 
//     vector<int32_t> part1(data.begin() + 35, data.begin() + 38); 
//     vector<int32_t> part2(data.begin() + 65, data.begin() + 281); 
//     return {part1, part2};
// }


// 检查 LRC 校验
bool lrc_check(const vector<uint8_t>& raw_data) 
{
    int checksum = 0;
    for (size_t i = 4; i < raw_data.size() - 1; ++i) {
        checksum += raw_data[i] & 0xFF;
    }
    uint8_t lrc = (~checksum + 1) & 0xFF;
    return lrc == raw_data[raw_data.size() - 1];
}
void MySigintHandler(int sig)
{
	//这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
	ROS_INFO("shutting down!");
	ros::shutdown();
}


int main(int argc, char* argv[]) 
{
    ros::init(argc, argv, "tactile_sensor_read_cpp");
    ros::NodeHandle n;
    ros::Rate rate(1000);
    ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("sensor_read_cpp", 1000); 
    std_msgs::Int32MultiArray msg;
    
    ROS_INFO_STREAM("running good");
    // 初始化串口通信
    signal(SIGINT, MySigintHandler);
    ser.setPort("/dev/ttyS3");
    ser.setBaudrate(460800);
    ser.setBytesize(serial::eightbits);
    ser.setParity(serial::parity_none);
    serial::Timeout to = serial::Timeout::simpleTimeout(10);
    ser.setTimeout(to);
    ser.open();
    if (ser.isOpen()) 
    {
        cout << "串口打开成功: " << "/dev/ttyS3" << endl;

    } 
    else{
        ser.setPort("/dev/ttyACM2");
        ser.setBaudrate(460800);
        ser.setBytesize(serial::eightbits);
        ser.setParity(serial::parity_none);
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        ser.setTimeout(to);
        ser.open();
        cout<<"启动第二个串口"<<endl;
    }

    // 校准和配置命令
    
    uint8_t calibration[18] = {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB0, 0x02, 0x02, 0x00, 0x03, 0x01,0xCA, 0x55, 0xAA, 0x7D, 0x7D};
    
    // for(int i=0;i<calibration.size();i++)
    // {
    //     cout<<dec<<setw(2)<<setfill('0')<<(int)calibration[i]<<endl;
    // }
    
    unsigned char initial_setting[17] = {0x55,0xAA,0x7B,0x7B,0x0E,0x00,0x70,0xC0,0x0C,0x01,0x00,0x02,0xB3,0x55,0xAA,0x7D,0x7D};
    ser.write(initial_setting,sizeof(initial_setting));
    
    // 配置选择命令
    
    unsigned char choose_con[20][17] = {
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x00, 0xC6, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x01, 0xC5, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x02, 0xC4, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x03, 0xC3, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x04, 0xC2, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x05, 0xC1, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x06, 0xC0, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x07, 0xBF, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x08, 0xBE, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x09, 0xBD, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x0A, 0xBC, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x0B, 0xBB, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x0C, 0xBA, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x0D, 0xB9, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x0E, 0xB8, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x0F, 0xB7, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x10, 0xB6, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x11, 0xB5, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x12, 0xB4, 0x55, 0xAA, 0x7D, 0x7D},
        {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xB1, 0x0A, 0x01, 0x00, 0x13, 0xB3, 0x55, 0xAA, 0x7D, 0x7D},
    };


    // 读数据命令
    
    unsigned char data_read_ip[21] = {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xC0, 0x06, 0x05, 0x00, 0x7B, 0xF0, 0x03, 0xD2, 0x00,0x77, 0x55, 0xAA, 0x7D, 0x7D};
    
    unsigned char data_read_dp[21] = {0x55, 0xAA, 0x7B, 0x7B, 0x0E, 0x00, 0x70, 0xC0, 0x06, 0x05, 0x00, 0x7B, 0xF0, 0x03, 0xF6, 0x00, 0x53,0x55, 0xAA, 0x7D, 0x7D};


    ser.write(choose_con[1],17);
    ser.write(calibration,18);
    ser.write(choose_con[3],17);
    ser.write(calibration,18);
    ser.write(choose_con[5],17);
    ser.write(calibration,18);
    ser.write(choose_con[7],17);
    ser.write(calibration,18);    
    ser.write(choose_con[9],17);
    ser.write(calibration,18);

    
    vector<int32_t> data; 
    uint8_t data2;
    for(int i=0;i<136;i++)
    {
        ser.read(&data2, 1);
        data.push_back(data2);
        // cout<<data[i]<<" ";
    }
    ser.flushOutput();
    sleep(1);

   int force[219*5];
   
   while(ros::ok())
{
    // 主循环
    // 发送选择配置命令并读取数据
    ser.write(choose_con[1],17);
    ser.write(data_read_dp,21);
    auto force0 =read_data_dp();
    cout<<"["<<force0[0]<<", "<<force0[1]<<", "<<force0[2]<<"]";
    copy(force0,force0+219,force);

    ser.write(choose_con[3],17);
    ser.write(data_read_dp,21);
    auto force1 =read_data_dp();
    cout<<"["<<force1[0]<<", "<<force1[1]<<", "<<force1[2]<<"]";
    copy(force1,force1+219,force+219);

    ser.write(choose_con[5],17);
    ser.write(data_read_dp,21);
    auto force2 =read_data_dp();
    cout<<"["<<force2[0]<<", "<<force2[1]<<", "<<force2[2]<<"]";
    copy(force2,force2+219,force+219*2);

    ser.write(choose_con[7],17);
    ser.write(data_read_dp,21);
    auto force3=read_data_dp();
    cout<<"["<<force3[0]<<", "<<force3[1]<<", "<<force3[2]<<"]";
    copy(force3,force3+219,force+219*3);

    ser.write(choose_con[9],17);
    ser.write(data_read_dp,21);
    auto force4 =read_data_dp();
    cout<<"["<<force4[0]<<", "<<force4[1]<<", "<<force4[2]<<"]"<<endl;
    copy(force4,force4+219,force+219*4);

    //将数组转为vector才能发布
    vector<int> force_vec(force,force+1095);
    cout<<endl;
    // cout<<"指腹con0数据：["<<force_vec[0]<<", "<<force_vec[1]<<", "<<force_vec[2]<<"]"<<endl;
    // cout<<"指腹con1数据：["<<force_vec[183]<<", "<<force_vec[183+1]<<", "<<force_vec[183+2]<<"]"<<endl;
    // cout<<"指腹con2数据：["<<force_vec[183+219]<<", "<<force_vec[183+219+1]<<", "<<force_vec[183+219+2]<<"]"<<endl;
    // cout<<"指腹con3数据：["<<force_vec[183+219*2]<<", "<<force_vec[183+219*2+1]<<", "<<force_vec[183+219*2+2]<<"]"<<endl;

    msg.data = force_vec;
    pub.publish(msg);
    rate.sleep();
}

    // auto [resultant_force0, force0] = extract_data_ip(ser);

        // write_data(ser, choose_con[3]); // choose_con3
        // write_data(ser, data_read_dp);  // Send data_read_dp
        // auto [resultant_force1, force1] = extract_data_dp(ser);

        // write_data(ser, choose_con[5]); // choose_con5
        // write_data(ser, data_read_dp);  // Send data_read_dp
        // auto [resultant_force2, force2] = extract_data_dp(ser);

        // 打印数据
        // cout << "Resultant Force (IP): " << resultant_force0[0] << ", " << resultant_force0[1] << ", " << resultant_force0[2] << endl;
        // cout << "Resultant Force (DP): " << resultant_force1[0] << ", " << resultant_force1[1] << ", " << resultant_force1[2] << endl;
    

    return 0;
}