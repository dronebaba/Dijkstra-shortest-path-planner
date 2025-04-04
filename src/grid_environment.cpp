#include "gritt_robotics/grid_environment.hpp"
#include <ctime>
#include <cstdlib>
using namespace std;

GridEnvironment::GridEnvironment() : Node("grid_environment")
{
    width_ = this->declare_parameter<int>("width",10);
    height_ = this->declare_parameter<int>("height",10);
    obstacle_density_ = this->declare_parameter<double>("obstacle_density",0.1);
    robot_x_ = this->declare_parameter<int>("start_x",0);
    robot_y_ = this->declare_parameter<int>("start_y",0);
    goal_x_ = this->declare_parameter<int>("goal_x",width_-1);
    goal_y_ = this->declare_parameter<int>("goal_y",height_-1);

    initializeGrid();

    // State publisher
    state_publisher_ = this->create_publisher<std_msgs::msg::String>("robot_state",10);
    
    // subscriber for move commands
    command_subscriber_ = this-> create_subscription<std_msgs::msg::String>("move_command",10,
        bind(&GridEnvironment::moveRobot, this, placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "GridEnvironment node has been started.");
}

void GridEnvironment::initializeGrid(){

    // resize the grid to the specified width and height
    grid_.resize(height_,std::vector<int>(width_,0));

    // seed the random number generation
    srand(time(0));

    // Fill the grid with random obstacles based on the density
    for ( int i = 0; i< height_ ; ++i)
    {
        for( int j = 0; j<width_ ; ++j)
        {
            if((double)rand()/RAND_MAX < obstacle_density_)
            {
                grid_[i][j] = 1; // obstacle
            }else{
                grid_[i][j] = 0; // free space
            }
        }
    }

    // Ensure the starting position and goal position are free space
    grid_[robot_y_][robot_x_] = 0;
    grid_[goal_y_][goal_x_] = 0;

}

void GridEnvironment::updateState(){
    // Publish the current state of the robot
    auto message = std_msgs::msg::String();
    message.data = "Robot at (" + std::to_string(robot_x_) + "," + std::to_string(robot_y_)+")";
    state_publisher_->publish(message);
}

void GridEnvironment::moveRobot(const std_msgs::msg::String::SharedPtr msg)
{
    std::string command = msg->data;
    int new_x = robot_x_;
    int new_y = robot_y_;

    // Move in the commanded direction with probability 0.75

    if((double)rand()/RAND_MAX < 0.75)
    {
        if(command == "up")
        {
            new_y -= 1;
        }else if (command == "down")
        {
            new_y += 1;
        }else if(command == "right")
        {
            new_x += 1;
        }else if (command == "left")
        {
            new_x -= 1;
        }
        
    }else{
        // Move in a random direction with probability 0.25
        int random_direction = rand() % 4;
        if(random_direction == 0){
            new_y -= 1; // up
        }else if (random_direction == 1)
        {
            new_y += 1; // down
        }else if (random_direction == 2)
        {
            new_x -= 1; // left
        }else if (random_direction == 3)
        {
            new_x += 1; // right
        }  
    }

    // check for boundary and obstacles
    if (new_x >= 0 && new_x < width_ && new_y >= 0 && new_y < height_ && grid_[new_y][new_x] == 0) {
        robot_x_ = new_x;
        robot_y_ = new_y;
        RCLCPP_INFO(this->get_logger(), "Moved robot to (%d, %d)", robot_x_, robot_y_);

    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid move to (%d, %d)", new_x, new_y);
    }

    // Publish updated robot state
    updateState();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // Create and run the GridEnvironment node
    rclcpp::spin(std::make_shared<GridEnvironment>());
    rclcpp::shutdown();
   
    return 0;
}