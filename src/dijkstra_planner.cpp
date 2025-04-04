#include "gritt_robotics/dijkstra_planner.hpp"
using namespace std;
using namespace std::chrono_literals;

DijkstraPlanner::DijkstraPlanner() : Node("dijkstra_planner") {
    
    // Declare and initialize parameters
    width_ = this->declare_parameter<int>("width", 10);
    height_ = this->declare_parameter<int>("height", 10);
    goal_x_ = this->declare_parameter<int>("goal_x", width_ - 1);
    goal_y_ = this->declare_parameter<int>("goal_y", height_ - 1);
    obstacle_density_ = this->declare_parameter<double>("obstacle_density", 0.1);
    path_actions_cal = false;
    goal_reached_ = false;

    // Initialize the grid
    initializeGrid();

    // Print the initial grid
    RCLCPP_INFO(this->get_logger(), "Initial Grid: Width = %d, Height = %d", width_, height_);
    
    for (int i = 0; i < height_; ++i) {
        std::stringstream ss;
        for (int j = 0; j < width_; ++j) {
            if (i == robot_y_ && j == robot_x_) {
                ss << "S "; // Start
            } else if (i == goal_y_ && j == goal_x_) {
                ss << "G "; // Goal
            } else {
                ss << grid_[i][j] << " ";
            }
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    // Create a subscriber for robot state
    state_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "robot_state", 10, bind(&DijkstraPlanner::stateCallback, this, placeholders::_1));
    
    // Create a publisher for move commands
    command_publisher_ = this->create_publisher<std_msgs::msg::String>("move_command", 10);

    // Create a timer to periodically plan the path
    timer_ = this->create_wall_timer(
        500ms, bind(&DijkstraPlanner::planPath, this));

    RCLCPP_INFO(this->get_logger(), "DijkstraPlanner node has been started.");
}

// Function to initialize the grid with random obstacles
void DijkstraPlanner::initializeGrid() {
    
    grid_.resize(height_, vector<int>(width_, 0));
    
    // Seed the random number generator
    srand(time(0));

    // Fill the grid with random obstacles based on the density
    for (int i = 0; i < height_; ++i) {
        for (int j = 0; j < width_; ++j) {
            if ((double)rand() / RAND_MAX < obstacle_density_) {
                grid_[i][j] = 1; // Obstacle
            } else {
                grid_[i][j] = 0; // Free space
            }
        }
    }

    // Ensure the starting position and goal position are free
    grid_[robot_y_][robot_x_] = 0;
    grid_[goal_y_][goal_x_] = 0;
}

// Callback function for state updates
void DijkstraPlanner::stateCallback(const std_msgs::msg::String::SharedPtr msg) {
    
    // Extract and process the robot state message
    sscanf(msg->data.c_str(), "Robot at (%d, %d)", &robot_x_, &robot_y_);
    RCLCPP_INFO(this->get_logger(), "Robot state received: (%d, %d)", robot_x_, robot_y_);

}

// Function to plan the path using Dijkstra's Algorithm
void DijkstraPlanner::planPath() {

    if (goal_reached_) {
        // If the goal has been reached, do not plan further
        return;
    }

    // Dijkstra's Algorithm initialization
    vector<vector<int>> dist(height_, vector<int>(width_, numeric_limits<int>::max()));
    vector<vector<pair<int, int>>> prev(height_, vector<pair<int, int>>(width_, make_pair(-1, -1)));
    dist[robot_y_][robot_x_] = 0;

    // Priority queue for path
    typedef pair<int, pair<int, int>> Node; // (distance, (x, y))
    priority_queue<Node, vector<Node>, greater<Node>> pq;
    pq.push(make_pair(0, make_pair(robot_x_, robot_y_)));
    
    // Path Planning
    while (!pq.empty()) {
        Node node = pq.top();
        pq.pop();
        int d = node.first;
        int x = node.second.first;
        int y = node.second.second;

        // This can happen if a shorter path to this node was found and processed after it was added to the priority queue
        if (d > dist[y][x]) continue; 

        // Check all direction movements
        vector<pair<int, int>> neighbours = {make_pair(x - 1, y), make_pair(x + 1, y), make_pair(x, y - 1), make_pair(x, y + 1)};

        // Iterate over all possible neighboring cells (up, down, left, right)
        for (const auto& neighbour : neighbours) {
            int nx = neighbour.first;
            int ny = neighbour.second;
            if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_ && grid_[ny][nx] == 0) {
                // Calculate the new distance to this neighbor
                int new_dist = dist[y][x] + 1;
                if (new_dist < dist[ny][nx]) {
                    dist[ny][nx] = new_dist;
                    prev[ny][nx] = make_pair(x, y);
                    // Push the neighbor with the new distance onto the priority queue
                    pq.push(make_pair(new_dist, make_pair(nx, ny)));
                }
            }
        }
    }

    // Reconstruct the path from target to start
    path_.clear(); // Clear the previous path and actions
    actions_.clear();

    // Vector to hold the coordinates of the reconstructed path
    vector<pair<int, int>> coordinates;

    for (int x = goal_x_, y = goal_y_; x != -1 && y != -1;) {
        coordinates.push_back(make_pair(x, y));
        tie(x, y) = prev[y][x];
    }
    // Reverse the coordinates vector to get the path from start to goal
    reverse(coordinates.begin(), coordinates.end());

    // Calculate actions and print path
    int move_x, move_y;
    for (size_t i = 1; i < coordinates.size(); ++i) {
        move_x = coordinates[i].first;
        move_y = coordinates[i].second;
        string move;

        if (move_x == coordinates[i-1].first && move_y == coordinates[i-1].second - 1) move = "up";
        else if (move_x == coordinates[i-1].first && move_y == coordinates[i-1].second + 1) move = "down";
        else if (move_x == coordinates[i-1].first - 1 && move_y == coordinates[i-1].second) move = "left";
        else if (move_x == coordinates[i-1].first + 1 && move_y == coordinates[i-1].second) move = "right";

        path_.push_back(make_pair(i, move));
        actions_ += "(" + to_string(coordinates[i-1].first) + ", " + to_string(coordinates[i-1].second) + ", " + move + ")\n";
    }

    // Store the robot path and movement
    if (!path_actions_cal) {
        path_actions_cal = true;
        path_actions_cal_ = path_; 
        path_coordinates_ = coordinates;
    }

    // Check if the robot has reached the goal
    if (robot_x_ == goal_x_ && robot_y_ == goal_y_) {
        
        RCLCPP_INFO(this->get_logger(), "!!!! Robot has successfully reached the target at (%d, %d) !!!!!", goal_x_, goal_y_);
        RCLCPP_INFO(this->get_logger(), " Number of actions: %zu", path_actions_cal_.size());

        for (size_t i = 0; i < path_actions_cal_.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "((%d, %d), %s)", path_coordinates_[i].first, path_coordinates_[i].second, path_actions_cal_[i].second.c_str());
            cout << "((" << path_coordinates_[i].first << ", " << path_coordinates_[i].second << "), " << path_actions_cal_[i].second << ")\n";
        }

        goal_reached_ = true;

        return;
    }

    // Publish the next move command if a valid path exists
    if (!path_.empty()) {
        // Create a new String message to publish the next move
        auto message = std_msgs::msg::String();
        message.data = path_.front().second;
        command_publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Publishing move command: %s", path_.front().second.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "No valid path to the goal found!");
    }
}

int main(int argc, char **argv) {
    
    rclcpp::init(argc, argv);

    // Create and run the DijkstraPlanner node
    rclcpp::spin(make_shared<DijkstraPlanner>());
    rclcpp::shutdown();

    return 0;
}
