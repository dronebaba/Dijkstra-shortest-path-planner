#ifndef GRITT_ROBOTICS__DIJKSTRA_PLANNER_HPP_
#define GRITT_ROBOTICS__DIJKSTRA_PLANNER_HPP_

#include <vector>
#include <queue>
#include <utility>
#include <limits>
#include <algorithm>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DijkstraPlanner : public rclcpp::Node {
public:
    DijkstraPlanner();

private:
    void stateCallback(const std_msgs::msg::String::SharedPtr msg);
    void planPath();
    void initializeGrid();

    int width_;
    int height_;
    int robot_x_;
    int robot_y_;
    int goal_x_;
    int goal_y_;
    double obstacle_density_;
    bool goal_reached_; 
    bool path_actions_cal;

    std::vector<std::pair<int, std::string>> path_actions_cal_;
    std::vector<std::vector<int>> grid_;
    std::vector<std::pair<int, int>> path_coordinates_;

    std::vector<std::pair<int, std::string>> path_;
    std::string actions_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // GRITT_ROBOTICS__DIJKSTRA_PLANNER_HPP_
