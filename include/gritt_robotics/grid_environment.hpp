#ifndef GRITT_ROBOTICS__GRID_ENVIRONMENT_HPP_
#define GRITT_ROBOTICS__GRID_ENVIRONMENT_HPP_

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class GridEnvironment : public rclcpp::Node
{
    public:
        GridEnvironment();
    
    private:
        void initializeGrid();
        void updateState();
        void moveRobot(const std_msgs::msg::String::SharedPtr msg);

        int width_;
        int height_;
        int robot_x_;
        int robot_y_;
        int goal_x_;
        int goal_y_;
        double obstacle_density_;
        
        std::vector<std::vector<int>> grid_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
};

#endif //GRITT_ROBOTICS__GRID_ENVIRONMENT_HPP_


