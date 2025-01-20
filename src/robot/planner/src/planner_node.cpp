#include "planner_node.hpp"
#include <queue>
#include <tuple>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <algorithm>

    PlannerNode::PlannerNode() : Node("planner_node"), planner_(robot::PlannerCore(this->get_logger())), state_(State::WAITING_FOR_GOAL) {
        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
 
        // Publisher
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
 
        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
        goal_received_ = false;

    }
 
 
 
 
    void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        PlannerNode::current_map_ = *msg;
        if (state_ == PlannerNode::State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
            planPath();
        }
    }
 
    void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        goal_ = *msg;
        goal_received_ = true;
        state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
        planPath();
    }
 
    void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_pose_ = msg->pose.pose;
    }
 
    void PlannerNode::timerCallback() {
        if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
            if (goalReached()) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                state_ = State::WAITING_FOR_GOAL;
            } else {
                RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
                planPath();
            }
        }
    }
 
    bool PlannerNode::goalReached() {
        double dx = goal_.point.x - robot_pose_.position.x;
        double dy = goal_.point.y - robot_pose_.position.y;
        return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
    }
 
    void PlannerNode::planPath() {
        if (!goal_received_ || current_map_.data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
            return;
        }
 
        // A* Implementation (pseudo-code)
        std::priority_queue<std::tuple<int, int, int, int, int>, std::vector<std::tuple<int, int, int, int, int>>, std::greater<std::tuple<int, int, int, int, int>>
    > min_heap;
        std::map<std::tuple<int, int>, double> seen_map; // (x, y) -> distance
        std::map<std::tuple<int, int>, std::tuple<int, int>> pred_map; // (x, y) -> (pred_x, pred_y)

        // double inf = std::numeric_limits<double>::infinity();

         auto calculateDistance = [](int x1, int y1, int x2, int y2) -> double {
          return static_cast<double>(std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2)));
        };

        int start_x = static_cast<int>(round(this->robot_pose_.position.x*10));
        int start_y = static_cast<int>(round(this->robot_pose_.position.y*10));

        int goal_x = static_cast<int>(round(this->goal_.point.x*10));
        int goal_y = static_cast<int>(round(this->goal_.point.y*10));
        
        seen_map[std::make_tuple(start_x, start_y)] = 0;
        pred_map[std::make_tuple(start_x, start_y)] = std::make_tuple(start_x, start_y);

        double h_cost = calculateDistance(start_x, start_y, goal_.point.x, goal_.point.y);
        min_heap.push(std::make_tuple(0, h_cost, 0, start_x, start_y));

        std::vector<std::pair<int, int>> directions = {
            {0, 1}, {1, 0}, {0, -1}, {-1, 0}, 
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
        };

        while (!min_heap.empty()) {
            auto [f_cost, h_cost, g_cost, x, y] = min_heap.top();
            min_heap.pop();

            if (x == goal_x && y == goal_y) {
                break;
            }
            
            for (const auto& dir : directions) {
                int nx = x + dir.first;
                int ny = y + dir.second;

                if (nx < 0 || ny < 0 || nx >= current_map_.info.width || ny >= current_map_.info.height || seen_map[std::make_tuple(x, y)] < g_cost
                || current_map_.data[ny * current_map_.info.width + nx] != 0) {
                    continue;
                }

                double new_g_cost = g_cost + calculateDistance(x, y, nx, ny);
                double new_h_cost = calculateDistance(nx, ny, goal_.point.x, goal_.point.y);
                double new_f_cost = new_g_cost + new_h_cost;

                min_heap.push(std::make_tuple(new_f_cost, new_h_cost, new_g_cost, nx, ny));
                seen_map[std::make_tuple(nx, ny)] = new_g_cost;
                pred_map[std::make_tuple(nx, ny)] = std::make_tuple(x, y);
            }

            
        }

        // Compute path using A* on current_map_

        std::vector<std::tuple<int, int>> path_vector;
        std::tuple<int, int> current = std::make_tuple(goal_.point.x, goal_.point.y);
        while (current != std::make_tuple(start_x, start_y)) {
            path_vector.push_back(current);
            current = pred_map[current];
        }
        std::reverse(path_vector.begin(), path_vector.end());


        // Fill path.poses with the resulting waypoints.
        nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "map";

        for (const auto& point : path_vector) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = std::get<0>(point);
            pose.pose.position.y = std::get<1>(point);
            path.poses.push_back(pose);
        }

        path_pub_->publish(path);

 
        
 
    }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}