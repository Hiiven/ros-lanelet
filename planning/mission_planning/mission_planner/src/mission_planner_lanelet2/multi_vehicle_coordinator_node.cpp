#include <ros/ros.h>
#include <autoware_planning_msgs/Route.h>
#include <lanelet2_routing/Types.h> // For lanelet::Id
#include <unordered_set>
#include <mutex>
#include <string>
#include <functional> // For std::function and std::placeholders

// 全局共享的已覆盖Lanelet集合
std::unordered_set<lanelet::Id> g_globally_covered_lanelets;
std::mutex g_global_mutex; // 保护全局集合

// 回调函数：接收任意车辆的路线
void routeCallback(const autoware_planning_msgs::Route::ConstPtr& msg, const std::string& vehicle_name) {
    std::lock_guard<std::mutex> lock(g_global_mutex);

    for (const auto& route_section : msg->route_sections) {
        for (const auto& lane_id : route_section.lane_ids) {
            g_globally_covered_lanelets.insert(lane_id);
        }
    }
    ROS_INFO("Coordinator: Received route from %s, updated global coverage set. Size: %ld", vehicle_name.c_str(), g_globally_covered_lanelets.size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_vehicle_coordinator");
    ros::NodeHandle nh;

    // 订阅所有车辆的路线 (例如 /vehicle_1/planning/mission_planning/route, /vehicle_2/...)
    int num_vehicles = 3; // 从参数服务器获取
    nh.param<int>("num_vehicles", num_vehicles, num_vehicles);

    std::vector<ros::Subscriber> route_subscribers;
    for (int i = 1; i <= num_vehicles; ++i) {
        std::string topic_name = "/vehicle_" + std::to_string(i) + "/planning/mission_planning/route";
        std::string vehicle_name = "vehicle_" + std::to_string(i);

        // 使用 Lambda 将车辆名称绑定到回调
        route_subscribers.push_back(nh.subscribe<autoware_planning_msgs::Route>(topic_name, 10,
            [vehicle_name](const autoware_planning_msgs::Route::ConstPtr& msg) {
                routeCallback(msg, vehicle_name);
            }));
        ROS_INFO("Coordinator: Subscribing to %s", topic_name.c_str());
    }

    ROS_INFO("Multi-vehicle coordinator node (in mission_planner pkg) started. Listening for routes.");
    ros::spin();

    return 0;
}