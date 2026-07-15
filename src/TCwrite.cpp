#include "AdsLib.h"
#include "AdsVariable.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <iostream>
#include <limits>
#include <array>

//通过ads读取twincat数据并发布为话题
// ─── ADS 连接参数 ───────────────────────────────────
static const AmsNetId remoteNetId { 127, 0, 0, 1, 2, 2   };
static const AmsNetId localNetId  { 192, 168, 10, 1, 1, 3 };
static const char     remoteIpV4[] = "192.168.203.111";

int main(int argc, char *argv[]){
    ros::init(argc, argv, "TCwrite");
    ros::NodeHandle n;

    // ── 初始化 ADS 路由 ──────────────────────────────
    AdsSetLocalAddress(localNetId);
    long ret = AdsAddRoute(remoteNetId, remoteIpV4);
    ROS_INFO_STREAM("AdsAddRoute_Write: " << (ret == 0 ? "OK" : "FAILED"));
    if (ret != 0) return 1;

    try{
        AdsDevice route { remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3 };

        // ── 变量句柄（循环外创建一次）───────────────────
        AdsVariable<std::array<float, 16>>  RosTargetAngle          {route, "MAIN.ROS_A"};
        AdsVariable<int16_t>                CurrentJob              {route, "VariableMAIN.CurrentJob"};
        
        ROS_INFO("Variable Connected! start after enter");
        
        //写入目标关节角
        while(ros::ok()){
            std::array<float,16> AxisTarget = RosTargetAngle;
            
            //清空输入流
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            // 用户输入：关节索引和数值
            int idx; float val;
            std::cout << "Enter joint index (0-15) and value: ";
            std::cin >> idx >> val;

            ROS_INFO("input sucesss");

            if (idx >= 0 && idx < 16) {
                AxisTarget[idx] = val;          // 修改
                RosTargetAngle = AxisTarget;    // 写回 PLC
                std::cout << "Written " << val << " to joint " << idx << std::endl;

                // 询问是否触发运动
                char confirm;
                std::cout << "Trigger motion? (y/n): ";
                std::cin >> confirm;

                if (confirm == 'y')
                    CurrentJob = 210;       // 启动运动
                else if (confirm != 'n'){
                    ROS_INFO("Trigger motion Failed");
                    break;
                }
            }
        }   
        
    }catch (const AdsException& ex) {
    ROS_ERROR_STREAM("ADS Exception: " << ex.what());
    return 1;
    }
    return 0;
}   