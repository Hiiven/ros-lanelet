#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <vector>

// 存储每辆车的订阅者和颜色信息
struct VehicleInfo {
    ros::Subscriber sub;
    std_msgs::ColorRGBA color;
    std::string vehicle_name;
    int marker_id_offset; // 用于避免不同车辆的Marker ID冲突
};

std::vector<VehicleInfo> vehicle_infos;
ros::Publisher global_pub;
int total_markers_processed = 0; // 用于计算ID偏移

// 回调函数：处理来自特定车辆的 MarkerArray
void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg, int vehicle_index) {
    visualization_msgs::MarkerArray modified_msg = *msg; // 复制消息

    // 修改颜色并调整ID (只修改特定类型的Marker)
    for (auto& marker : modified_msg.markers) {
        // --- 区分逻辑 ---
        // 检查 frame_id 和 marker type，假设 mission_planner 发布的路径Marker是TRIANGLE_LIST类型且在map坐标系下
        // 同时检查 namespace，确保只修改 "route_lanelets" 和 "current_lanelets"
        if (marker.header.frame_id == "map" && 
            (marker.type == visualization_msgs::Marker::TRIANGLE_LIST ) && // TRIANGLE_STRIP (type 11) 也常见
            (marker.ns == "route_lanelets" || marker.ns == "current_lanelets")) { // 添加 namespace 检查
            
            // 修改颜色 - 修改 Marker 顶层颜色（虽然可能不被直接使用）
            marker.color = vehicle_infos[vehicle_index].color;

            // 修改颜色 - 修改 Marker 的 colors 数组（这是关键！）
            // 将数组中的每一个颜色都修改为对应车辆的颜色
            for (auto& color : marker.colors) {
                color = vehicle_infos[vehicle_index].color;
            }

            // 修改ID以避免冲突
            marker.id += vehicle_infos[vehicle_index].marker_id_offset;

            //修改命名空间 (namespace) 以区分来源
            marker.ns = vehicle_infos[vehicle_index].vehicle_name + "_" + marker.ns;
            // 如果不想修改ns，保持原样即可
        }
        // --- 区分逻辑结束 ---
    }

    // 发布修改后的 MarkerArray
    global_pub.publish(modified_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_vehicle_marker_relay");
    ros::NodeHandle nh;

    // 从参数服务器获取车辆数量 (例如，在launch文件中设置)
    int num_vehicles = 3; // 默认值
    nh.param<int>("num_vehicles", num_vehicles, num_vehicles);

    // 定义颜色 (可以根据需要调整)
    std::vector<std_msgs::ColorRGBA> colors;
    // 为每辆车定义一种颜色
    for (int i = 0; i < num_vehicles; ++i) {
        std_msgs::ColorRGBA color;
        // 使用简单的颜色区分，例如：红、绿、蓝、黄、紫、青...
        if (i % 6 == 0) { color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 0.8; } // Red
        else if (i % 6 == 1) { color.r = 0.0; color.g = 1.0; color.b = 0.0; color.a = 0.8; } // Green
        else if (i % 6 == 2) { color.r = 0.0; color.g = 0.0; color.b = 1.0; color.a = 0.8; } // Blue
        else if (i % 6 == 3) { color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = 0.8; } // Yellow
        else if (i % 6 == 4) { color.r = 1.0; color.g = 0.0; color.b = 1.0; color.a = 0.8; } // Magenta
        else if (i % 6 == 5) { color.r = 0.0; color.g = 1.0; color.b = 1.0; color.a = 0.8; } // Cyan
        colors.push_back(color);
    }

    // 计算每辆车的Marker ID偏移量 (假设每辆车的最大Marker ID不会超过10000)
    const int MARKER_ID_OFFSET_PER_VEHICLE = 10000;

    // 创建订阅者和存储信息
    for (int i = 1; i <= num_vehicles; ++i) { // 假设车辆编号从1开始
        std::string vehicle_name = "vehicle_" + std::to_string(i);
        std::string topic_name = "/" + vehicle_name + "/planning/mission_planning/route_marker";

        VehicleInfo info;
        info.sub = nh.subscribe<visualization_msgs::MarkerArray>(
            topic_name, 10,
            boost::bind(&markerCallback, _1, i - 1) // 绑定回调函数和车辆索引 (0-based)
        );
        info.color = colors[i - 1];
        info.vehicle_name = vehicle_name;
        info.marker_id_offset = (i - 1) * MARKER_ID_OFFSET_PER_VEHICLE; // 计算偏移量
        vehicle_infos.push_back(info);

        ROS_INFO("Subscribing to %s with color R:%f, G:%f, B:%f, A:%f", topic_name.c_str(),
                info.color.r, info.color.g, info.color.b, info.color.a);
    }

    // 创建发布者，发布到全局话题
    global_pub = nh.advertise<visualization_msgs::MarkerArray>("/planning/mission_planning/route_marker", 10);

    ROS_INFO("Multi-vehicle marker relay node started. Publishing to /planning/mission_planning/route_marker");

    ros::spin();

    return 0;
}