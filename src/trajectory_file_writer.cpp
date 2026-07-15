#include "AdsLib.h"
#include "AdsVariable.h"
#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <array>
#include <vector>
#include <cmath>

// ─── 与 PLC 中的常量完全一致，务必核对！───────────
const int MAX_CSV_ROWS    = 60000;   // 示例值，请根据实际 PLC 宏修改
const int MAX_CSV_COLUMNS = 15;      // 最后列索引，轴数 = MAX_CSV_COLUMNS + 1
const int AXIS_COUNT      = MAX_CSV_COLUMNS + 1;   // 16
const int MAX_DATA_SIZE   = (MAX_CSV_ROWS + 1) * AXIS_COUNT; // 数组总元素数

// ─── ADS 连接参数 ─────────────────────────────────
static const AmsNetId remoteNetId { 127, 0, 0, 1, 2, 2   };
static const AmsNetId localNetId  { 192, 168, 10, 1, 1, 3 };
static const char     remoteIpV4[] = "192.168.203.111";

// ─── 文件读取函数：支持空格/逗号分隔，第一行可选点数 ─────
int LoadTrajectoryFromFile(const std::string& filepath,
                           std::vector<double>& flat_data,
                           int src_cols)
{
    std::ifstream fin(filepath);
    if (!fin.is_open()) {
        ROS_ERROR("Cannot open: %s", filepath.c_str());
        return -1;
    }

    flat_data.clear();
    std::string first_line;
    if (!std::getline(fin, first_line)) {
        ROS_ERROR("Empty file");
        return -1;
    }

    first_line.erase(0, first_line.find_first_not_of(" \t"));
    first_line.erase(first_line.find_last_not_of(" \t") + 1);

    int num_points = 0;
    if (std::istringstream(first_line) >> num_points && num_points > 0) {
        // 格式1：第一行是点数
        auto replace_commas = [](std::string& s) {
            for (auto& c : s) if (c == ',') c = ' ';
        };
        for (int p = 0; p < num_points; ++p) {
            std::string line;
            if (!std::getline(fin, line)) {
                ROS_ERROR("Expected %d points, got %d", num_points, p);
                return -1;
            }
            replace_commas(line);
            std::istringstream iss(line);
            for (int a = 0; a < src_cols; ++a) {
                double val;
                if (!(iss >> val)) {
                    ROS_ERROR("Line %d: need %d values", p + 2, src_cols);
                    return -1;
                }
                flat_data.push_back(val);
            }
        }
    } else {
        // 格式2：纯数据流
        fin.clear();
        fin.seekg(0);
        std::stringstream buffer;
        buffer << fin.rdbuf();
        std::string all_content = buffer.str();
        for (auto& c : all_content) if (c == ',') c = ' ';
        std::istringstream data_stream(all_content);
        double val;
        while (data_stream >> val) {
            flat_data.push_back(val);
        }
        if (flat_data.empty()) {
            ROS_ERROR("No numbers found");
            return -1;
        }
        if (flat_data.size() % src_cols != 0) {
            ROS_ERROR("Data size %zu not a multiple of %d", flat_data.size(), src_cols);
            return -1;
        }
        num_points = flat_data.size() / src_cols;
    }
    return num_points;
}

// ─── 过滤函数：保留标志列从 start_val 到 end_val 之间的所有行 ─────
void FilterByColumn(const std::vector<double>& flat_data, int src_cols,
                    int filter_col, double start_val, double end_val,
                    std::vector<double>& filtered_data, int& points)
{
    filtered_data.clear();
    int total_points = flat_data.size() / src_cols;

    // 查找第一个 start_val
    int start_idx = -1;
    for (int p = 0; p < total_points; ++p) {
        double val = flat_data[p * src_cols + filter_col];
        if (std::fabs(val - start_val) < 1e-4) {
            start_idx = p;
            break;
        }
    }
    if (start_idx < 0) {
        ROS_ERROR("Filter start value %f not found in column %d", start_val, filter_col);
        points = 0;
        return;
    }

    // 向后查找最后一个 end_val
    int end_idx = -1;
    for (int p = start_idx + 1; p < total_points; ++p) {
        double val = flat_data[p * src_cols + filter_col];
        if (std::fabs(val - end_val) < 1e-9) {
            end_idx = p;
            break;
        }
    }
    if (end_idx < 0) {
        ROS_ERROR("Filter end value %f not found in column %d", end_val, filter_col);
        points = 0;
        return;
    }

    // 提取区间内所有行（包括首尾）
    for (int p = start_idx; p <= end_idx; ++p) {
        for (int c = 0; c < src_cols; ++c) {
            filtered_data.push_back(flat_data[p * src_cols + c]);
        }
    }
    points = end_idx - start_idx + 1;
    ROS_INFO("Filtered rows: %d to %d (%d points)", start_idx, end_idx, points);
}

// ─── 主函数 ────────────────────────────────────────
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "trajectory_file_writer");
    ros::NodeHandle nh("~");

    // ── 基本参数 ───────────────────────
    std::string filepath;
    nh.param<std::string>("file", filepath, "motor_trajectory_real.txt");

    int src_cols;
    nh.param<int>("src_cols", src_cols, 4);          // 文件每行列数

    int src_col;
    nh.param<int>("src_col", src_col, 1);            // 提取的源列索引（0‑based，默认 1 即第二列）

    int target_axis;
    nh.param<int>("target_axis", target_axis, 12);    // 写入 PLC 的轴索引（默认 8 即第 9 轴）

    // ── 过滤参数 ───────────────────────
    int filter_col;
    nh.param<int>("filter_col", filter_col, 3);      // 标志列索引（默认 3 即第四列）

    double filter_start;
    nh.param<double>("filter_start", filter_start, 0.000000);  // 标志起始值

    double filter_end;
    nh.param<double>("filter_end", filter_end, 0.000000);      // 标志结束值

    ROS_INFO("Reading: %s (src_cols=%d, src_col=%d, target_axis=%d, filter_col=%d, start=%f, end=%f)",
             filepath.c_str(), src_cols, src_col, target_axis, filter_col, filter_start, filter_end);

    // ── 初始化 ADS 路由 ────────────────
    AdsSetLocalAddress(localNetId);
    long ret = AdsAddRoute(remoteNetId, remoteIpV4);
    ROS_INFO_STREAM("AdsAddRoute: " << (ret == 0 ? "OK" : "FAILED"));
    if (ret != 0) return 1;

    try {
        AdsDevice route { remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3 };
        AdsVariable<uint32_t> NumTraj   { route, "MyTRAJ.SIZE_POS_TRAJ" };
        AdsVariable<std::array<double, MAX_DATA_SIZE>> TrajData { route, "MyTRAJ.database_read_LREAL2" };
        AdsVariable<int16_t>                CurrentJob              {route, "VariableMAIN.CurrentJob"};

        // 1. 读取原始文件
        std::vector<double> flat_data;
        int points = LoadTrajectoryFromFile(filepath, flat_data, src_cols);
        if (points < 0) return 1;

        // 2. 按标志列过滤
        if (filter_col >= 0 && filter_col < src_cols) {
            std::vector<double> filtered;
            int new_points = 0;
            FilterByColumn(flat_data, src_cols, filter_col,
                           filter_start, filter_end, filtered, new_points);
            if (new_points <= 0) {
                ROS_ERROR("Filtering returned no points, abort.");
                return 1;
            }
            flat_data = std::move(filtered);
            points = new_points;
        }

        if (points > (MAX_CSV_ROWS + 1)) {
            ROS_ERROR("Points %d exceed PLC capacity %d", points, MAX_CSV_ROWS + 1);
            return 1;
        }

        // 3. 构建目标缓冲区（全部填 0）
        static std::array<double, MAX_DATA_SIZE> buffer{};

        for (int p = 0; p < points; ++p) {
            int src_index = p * src_cols + src_col;          // 源列位置
            int dst_index = p * AXIS_COUNT + target_axis;    // 目标轴在 16 列中的位置
            buffer[dst_index] = flat_data[src_index];
        }

        // 4. 写入 PLC（先数据，后点数）
        TrajData = buffer;
        NumTraj  = static_cast<uint32_t>(points - 1);    // 注意符合轨迹逻辑：实际点数 - 1

        char confirm;
        std::cout << "Trigger motion? (y/n): ";
        std::cin >> confirm;
        if(confirm == 'y')
            CurrentJob = 114;

        ROS_INFO("Successfully wrote %d points (SIZE_POS_TRAJ=%d), col=%d -> axis=%d",
                 points, points - 1, src_col, target_axis);
    } catch (const AdsException& ex) {
        ROS_ERROR_STREAM("ADS Exception: " << ex.what());
        return 1;
    }

    ros::shutdown();
    return 0;
}