#pragma once 
#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/int32.hpp>
#include<sensor_msgs/msg/laser_scan.hpp>
#include<geometry_msgs/msg/twist.hpp>
#include<memory>

class WalkerContext;
//State Interface
class WalkerState{
    public:
        virtual ~WalkerState() = default;
        float min_distance;
         // obstacle detected
        bool is_obstacle_detected_=false;
        virtual void execute(WalkerContext &context, float) = 0;
};

//Concrete States
class MoveForward : public WalkerState {
    public:
    void execute(WalkerContext &context, float) override;
};

class TurnLeft : public WalkerState {
    public:
    void execute(WalkerContext &context, float) override;
};

class TurnRight : public WalkerState {
    public:
    void execute(WalkerContext &context, float) override;
};

class WalkerContext: public rclcpp::Node{
    public:
    WalkerContext(std::string node_name) : Node(node_name){
        // Create a publisher to publish Twist messages to cmd_vel topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        // Create a subscription to subscribe to LaserScan messages from scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10,
            std::bind(&WalkerContext::laser_callback, this, std::placeholders::_1));
        // Initialize the state of the robot to MovingForward
        state_ = std::make_shared<TurnLeft>();
        // Initialize the current and previous state of the robot
        current_state_ = "MoveForward";
        previous_state_ = "MoveForward";
        // Initialize the rotating direction to false
        turn_right_ = false;
        RCLCPP_INFO(this->get_logger(), "Walker node initialized.");
        RCLCPP_INFO(this->get_logger(), "Current state: %s", current_state_.c_str());

    }
        // Callback function for the laser scan data
        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        // Function to change the state of the robot
        void change_state(std::shared_ptr<WalkerState> new_state);
        // create a publisher to publish the velocity commands
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        // create a subscription to subscribe to the laser scan data
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
        // string to store the current state of the robot
        std::string current_state_;
        // string to store the previous state of the robot
        std::string previous_state_;
        // state of the robot
        std::shared_ptr<WalkerState> state_;
        // variable to store the previous rotation direction
        bool turn_right_=false;
        bool is_obstacle_detected_=false;

       


};