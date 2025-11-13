#ifndef TRAFFIC_LIGHT_SIMULATOR_NODE_H
#define TRAFFIC_LIGHT_SIMULATOR_NODE_H

#include <ros/ros.h>
#include <autoware_perception_msgs/traffic_light_recognition/TrafficLightStateArray.h> // 或其他合适的消息类型
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_extension/utility/query.h>

#include <vector>

// 前向声明 (Forward Declaration) 如果不需要完整定义类型
// namespace lanelet { class TrafficLightConstPtr; } // 如果直接使用指针

class TrafficLightSimulator {
public:
    explicit TrafficLightSimulator(ros::NodeHandle& nh); // 构造函数声明

    // 如果有需要暴露给外部的公有方法，可以在这里声明
    // void somePublicMethod();

private:
    void timerCallback(const ros::TimerEvent& event); // 私有方法声明

    ros::NodeHandle nh_; // 私有成员变量
    ros::Publisher light_state_pub_;
    ros::Timer timer_;
    std::vector<lanelet::TrafficLightConstPtr> traffic_lights_;
    // 可能还需要其他成员变量，例如当前状态、计时器等
};

#endif // TRAFFIC_LIGHT_SIMULATOR_NODE_H