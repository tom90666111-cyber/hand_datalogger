#include "AdsLib.h"
#include "AdsVariable.h"
#include "ros/ros.h"

// ─── ADS 连接参数 ───────────────────────────────────────────
static const AmsNetId remoteNetId { 127, 0, 0, 1, 2, 2   };  // TwinCAT NetId
static const AmsNetId localNetId  { 192, 168, 10, 1, 1, 2 };  // Linux ROS NetId
static const char     remoteIpV4[] = "127.0.0.1";        // Windows 真实IP

// ─── 初始化 ADS 路由 ─────────────────────────────────────────
bool initAdsRoute() {
  AdsSetLocalAddress(localNetId);

  long ret = AdsAddRoute(remoteNetId, remoteIpV4);
  ROS_INFO_STREAM("AdsAddRoute result: " << ret << (ret == 0 ? " (OK)" : " (FAILED)"));
  return ret == 0;
}

// ─── 打印本地 NetId（确认库实际使用的地址）────────────────────
void printLocalNetId() {
  long port = AdsPortOpenEx();
  AmsAddr addr;
  AdsGetLocalAddressEx(port, &addr);
  ROS_INFO_STREAM("Local NetId: "
    << (int)addr.netId.b[0] << "." << (int)addr.netId.b[1] << "."
    << (int)addr.netId.b[2] << "." << (int)addr.netId.b[3] << "."
    << (int)addr.netId.b[4] << "." << (int)addr.netId.b[5]);
  AdsPortCloseEx(port);
}

// ─── 连通性测试 ──────────────────────────────────────────────
bool testConnection(AdsDevice& route) {
  AdsDeviceState state = route.GetState();
  ROS_INFO_STREAM("ADS state: " << (int)state.ads
    << "  Device state: " << (int)state.device
    << (state.ads == 5 ? "  → PLC Running ✓" : "  → PLC NOT running ✗"));

  AdsVariable<int16_t> test_var { route, "MAIN.CurrentJobBefore" };
  int16_t val = test_var;
  ROS_INFO_STREAM("MAIN.CurrentJobBefore = " << val);

  return state.ads == 5;
}

// ─── 主循环 ──────────────────────────────────────────────────
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tc_reader");
  ros::NodeHandle n;

  if (!initAdsRoute()) {
    ROS_ERROR("Failed to add ADS route, exiting.");
    return 1;
  }
  printLocalNetId();

  try {
    AdsDevice route { remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3 };
    ROS_INFO("AdsDevice connected.");

    if (!testConnection(route)) {
      ROS_ERROR("PLC is not running, exiting.");
      return 1;
    }

    // 变量句柄在循环外创建一次
    AdsVariable<bool>     motionIsDoing   { route, "ADS.MotionIsDoing"   };
    // AdsVariable<bool>     motionIsDone    { route, "ADS.MotionIsDone"    };
    // AdsVariable<bool>     motionIsSuspend { route, "ADS.MotionIsSuspend" };
    // AdsVariable<bool>     motionIsAborted { route, "ADS.MotionIsAborted" };
    // AdsVariable<int16_t>  motionStatus    { route, "ADS.MotionStatus"    };
    // AdsVariable<std::array<char, 82>>    motionStateString { route, "ADS.MotionStateString" };
    // AdsVariable<std::array<double, 16>> speedBefore  { route, "ADS.SpeedBefore"  };
    // AdsVariable<std::array<double, 16>> radiusBefore { route, "ADS.RadiusBefore" };
    // AdsVariable<std::array<double, 16>> radiusAfter  { route, "ADS.RadiusAfter"  };
    AdsVariable<std::array<double, 16>> CurrentPositionReal  { route, "VariableMAIN.CurrentPositionReal"  };

    ros::Rate loop_rate(10);  // 10Hz
    while (ros::ok()) {
      // std::array<char, 82> strBuf = motionStateString;
      // std::array<double, 16> rb = radiusBefore;
      std::array<double, 16> axis = CurrentPositionReal;

      ROS_INFO_STREAM(
          // "Doing=" << motionIsDoing  
          // " Done="    << motionIsDone    
          // " Suspend=" << motionIsSuspend 
          // " Aborted=" << motionIsAborted 
          // " Status="  << (int)motionStatus 
          // " State="   << std::string(strBuf.data())
          // " axis16= " << rb[15]
          "axis16 = " << axis[15]
      );

      ros::spinOnce();
      loop_rate.sleep();
    }

  } catch (const AdsException& ex) {
    ROS_ERROR_STREAM("ADS Exception: " << ex.what());
    return 1;
  }

  return 0;
}