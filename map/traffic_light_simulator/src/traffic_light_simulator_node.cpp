// traffic_light_simulator_node.cpp (示例框架)
#include <traffic_light_simulator/traffic_light_simulator_node.h> // 包含头文件

#include <iostream> // 或其他需要的头文件

class TrafficLightSimulator {
public:
    TrafficLightSimulator(ros::NodeHandle& nh) : nh_(nh) {
        // 1. 加载地图 (可能通过服务或参数服务器获取)
        std::string map_path = nh_.param<std::string>("map_path", "");
        lanelet::LaneletMapPtr lanelet_map = lanelet::load(map_path, lanelet::Origin());

        // 2. 查询地图上的所有交通信号灯
        traffic_lights_ = lanelet::utils::query::trafficLights(lanelet_map->laneletLayer);
        // 或者 autowareTrafficLights

        // 3. 设置发布器
        light_state_pub_ = nh_.advertise<autoware_lanelet2_msgs::TrafficLightStateArray>("/light_states", 1);

        // 4. 设置定时器，定期发布状态
        timer_ = nh_.createTimer(ros::Duration(1.0), &TrafficLightSimulator::timerCallback, this);
    }

private:
    void timerCallback(const ros::TimerEvent& event) {
        autoware_lanelet2_msgs::TrafficLightStateArray state_array_msg;
        state_array_msg.header.stamp = ros::Time::now();
        state_array_msg.header.frame_id = "map";

        // 5. 根据预设时序更新每个信号灯的状态
        for (const auto& tl : traffic_lights_) {
            autoware_lanelet2_msgs::TrafficLightState state_msg;
            state_msg.id = tl->id(); // 或其他唯一标识符
            // 这里需要一个简单的时序逻辑来决定是红灯、黄灯还是绿灯
            // 例如，使用系统时间或内部计时器
            auto now = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
            int cycle_time = duration.count() % 60; // 60秒为一个周期

            if (cycle_time < 30) {
                state_msg.state = autoware_lanelet2_msgs::TrafficLightState::RED;
            } else if (cycle_time < 55) {
                state_msg.state = autoware_lanelet2_msgs::TrafficLightState::GREEN;
            } else {
                state_msg.state = autoware_lanelet2_msgs::TrafficLightState::YELLOW;
            }

            state_array_msg.traffic_light_states.push_back(state_msg);
        }

        light_state_pub_.publish(state_array_msg);
    }

    ros::NodeHandle nh_;
    ros::Publisher light_state_pub_;
    ros::Timer timer_;
    std::vector<lanelet::TrafficLightConstPtr> traffic_lights_; // 或其他类型
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "traffic_light_simulator");
    ros::NodeHandle nh;
    TrafficLightSimulator sim(nh);
    ros::spin();
    return 0;
}