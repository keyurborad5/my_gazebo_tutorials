#include "walker/walker_node.hpp"

#define MAX_FORWARD_SPEED 0.05
#define MAX_ROTATION_SPEED 0.1
#define DISTANCE_THRESHOLD 0.8

void WalkerContext::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    float min_distance = std::numeric_limits<float>::infinity();

    // Define the forward angular range
    double forward_angle_min = -M_PI / 3;  // -45 degrees
    double forward_angle_max = M_PI / 3;   // +45 degrees

    // Iterate through the ranges within the forward angular range
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double angle = msg->angle_min + i * msg->angle_increment;

        // Check if the angle is within the forward range
        if (angle >= forward_angle_min && angle <= forward_angle_max) {
            float range = msg->ranges[i];
            if (std::isfinite(range) && range < min_distance) {
                min_distance = range;
            }
        }
    }



     // Log the minimum distance within the forward range
    RCLCPP_INFO(this->get_logger(),
                "Minimum distance from LIDAR (forward): %.2f meters",
                min_distance);

    // Update obstacle detection
    is_obstacle_detected_ = (min_distance < 0.9);

    // Check if an obstacle is detected and log the status
    if (is_obstacle_detected_) {
        RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2f meters",
                    min_distance);
    } else {
        RCLCPP_INFO(this->get_logger(), "No obstacle detected in forward range.");
    }
    state_->execute(*this, min_distance);
}

void WalkerContext::change_state(std::shared_ptr<WalkerState> new_state){
    previous_state_ = current_state_;
    current_state_ = typeid(new_state).name();
    state_ = new_state;
    RCLCPP_INFO(this->get_logger(), "State changed from %s to %s",
                previous_state_.c_str(), current_state_.c_str());
}
    
    

void MoveForward::execute(WalkerContext &context, float min_distance){
    // Check if the minimum distance is less than the threshold
  if (min_distance < DISTANCE_THRESHOLD) {
    // check if the robot was rotating clockwise
    if (context.turn_right_) {
        // change the state to rotating counter clockwise
      context.change_state(std::make_shared<TurnLeft>());
      context.turn_right_ = false;
      
    } else {
    // change the state to rotating clockwise
      context.change_state(std::make_shared<TurnRight>());
      context.turn_right_ = true;
      
      
    }
  } else {
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    // Set the linear velocity to the maximum forward speed
    msg->linear.x = MAX_FORWARD_SPEED;
    msg->angular.z = 0.0;
    // std::cout<<"MOVING forward ."<<std::endl;
    RCLCPP_INFO(context.get_logger(), "MOVING forward .");


    context.publisher_->publish(*msg);
  }
}




void TurnLeft::execute(WalkerContext &context,   float min_distance){
    // Check if the minimum distance is greater than the threshold
    if (min_distance > DISTANCE_THRESHOLD) {
        // change the state to moving forward
        context.change_state(std::make_shared<MoveForward>());
    } else {
        // Publish the angular velocity to rotate clockwise
        auto msg = std::make_shared<geometry_msgs::msg::Twist>();
        msg->linear.x = 0.0;
        msg->angular.z = MAX_ROTATION_SPEED;
        RCLCPP_INFO(context.get_logger(), "Turning LEFT .");
        // std::cout<<"MOVING fLEFT ."<<std::endl;


        context.publisher_->publish(*msg);
    }
}
    
    
  
void TurnRight::execute(WalkerContext &context,  float min_distance){
    // Check if the minimum distance is greater than the threshold
    if (min_distance > DISTANCE_THRESHOLD) {
        // change the state to moving forward
        context.change_state(std::make_shared<MoveForward>());
    } else {
        // Publish the angular velocity to rotate clockwise
        auto msg = std::make_shared<geometry_msgs::msg::Twist>();
        msg->linear.x = 0.0;
        msg->angular.z = -MAX_ROTATION_SPEED;
        RCLCPP_INFO(context.get_logger(), "Turning Right .");
        // std::cout<<"MOVING RIGHT ."<<std::endl;
        context.publisher_->publish(*msg);
    }
}
    
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WalkerContext>("walker_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}