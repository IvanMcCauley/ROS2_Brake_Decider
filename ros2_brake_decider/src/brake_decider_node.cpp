#include <rclcpp/rclcpp.hpp> // gives us ROS 2 C++ API 
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>


class BrakeDeciderNode : public rclcpp::Node {
public:                                                     // public: can be used outside class
  BrakeDeciderNode() : rclcpp::Node("brake_decider"){        // Constructor, and node named set to brake_decider

    // 1) DECLARE PARAMETERS AND DEFAULTS
      // syntax is declare_parameter<T>("name", value)
    this->declare_parameter<double>("reaction_time", 1.0);
    this->declare_parameter<double>("decel", 9.0);
    this->declare_parameter<double>("safety_margin", 0.10);
    
    // 2) INITIALIZE VARIABLES, THEN READ PARAMETER VALUES FROM NODE INTO THE VARIABLES AT STARTUPP
    double reaction_time{};
    double decel{};
    double safety_margin{};
    this->get_parameter("reaction_time", reaction_time);
    this->get_parameter("decel", decel);
    this->get_parameter("safety_margin", safety_margin);
    

    // 3) REACT TO LIVE PARAMETER CHANGES
    param_cb_handle_ = this->add_on_set_parameters_callback(  // registers a func that ROS callls when a someone tries to set a parameter 
      [this](const std::vector<rclcpp::Parameter> & params) {
        // Loop over each changed paramEter
        for (const auto & p : params) {
    //----IF IT'S REACTION_TIME ------------
          if (p.get_name() == "reaction_time") {
            if (p.as_double() <= 0.0){
                RCLCPP_WARN(this->get_logger(), "Rejecting reaction_time %.3f (must be > 0)", p.as_double()); // warning message
                return rcl_interfaces::msg::SetParametersResult().set__successful(false); //reject change
            }
            RCLCPP_INFO(this->get_logger(), "reaction_time updated to %.3f", p.as_double()); // p.as_double turns value into a double for printing 

    //----IF IT'S DECEL ------------
          } else if (p.get_name() == "decel") {
            if (p.as_double() <= 0.0) {
              RCLCPP_WARN(this->get_logger(), "Rejecting decel %.3f (must be > 0)", p.as_double());
              return rcl_interfaces::msg::SetParametersResult().set__successful(false); // reject change
            }
            RCLCPP_INFO(this->get_logger(), "decel updated to %.3f", p.as_double());
            
    //----IF IT'S SAFETY_MARGIN --------
          } else if (p.get_name() == "safety_margin") {
            if (p.as_double() < 0.0 || p.as_double() > 1.0){
              RCLCPP_WARN(this->get_logger(), "Rejecting safety_margin %.3f (must be between 0 and 1)", p.as_double());
              return rcl_interfaces::msg::SetParametersResult().set__successful(false);// reject change
            }
            RCLCPP_INFO(this->get_logger(), "safety_margin updated to %.3f", p.as_double());
          }
        }
        return rcl_interfaces::msg::SetParametersResult().set__successful(true); // accept changes
      }
    );
    
    // 4) Create subscriptions (/ego_speed and /obstacle_distance)
    ego_speed_sub_= this->create_subscription<std_msgs::msg::Float64>(
      "/ego_speed", // topic name
      rclcpp::QoS(10), // QoS: small buffer of 10
      [this](const std_msgs::msg::Float64::SharedPtr msg){  // callback (similar to params but runs as new message arrives)
        last_speed_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Recieved ego_speed: %.2f", last_speed_); // logs the value
      }
    );
   
    obstacle_dist_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/obstacle_distance", // topic name
      rclcpp::QoS(10), // QoS: small buffer of 10
      [this](const std_msgs::msg::Float64::SharedPtr msg){
        last_distance_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Recieved obstacle_distance: %.2f", last_distance_);
      }
    );
    

    // 5) 20Hz timer: compute real brake decision and publish
    // Publisher for /brake_cmd
    brake_pub_ = this->create_publisher<std_msgs::msg::Bool>("/brake_cmd", rclcpp::QoS(10));

    tick_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      [this](){
        // 5.1) Read current params (So live updates apply)
        double reaction_time{}, decel{}, safety_margin{};
        this->get_parameter("reaction_time", reaction_time);
        this->get_parameter("decel", decel);
        this->get_parameter("safety_margin", safety_margin);
        
        // 5.2) Grab latese sensor values
        const double v = last_speed_;  // m/s
        const double d = last_distance_; // m

        // 5.3) Compute stoppping distance: react + brake (a = decel magnitude)
        // d_stop = v * t_react +v^2/(2*a)
        double d_react = v * reaction_time;
        double d_brake = (decel > 0.0) ? (v * v) / (2.0 * decel) : std::numeric_limits<double>::infinity();
        double d_stop = d_react + d_brake;

        // 5.4) Apply safety margin to available distance
        double d_available = d * (1.0 - safety_margin);

        // 5.5) Decide: brake if stopping distance >= available
        std_msgs::msg::Bool out;
        out.data = (d_stop >= d_available);

        brake_pub_->publish(out);

        // Uncomment to see decisions streaming (noisy!)
         // RCLCPP_INFO(this->get_logger(), "v=%.2f m/s, d=%.2f m, d_stop=%.2f, d_avail=%.2f -> brake=%s",
         //             v, d, d_stop, d_available, out.data ? "TRUE" : "false");
    
      }
    );

    // 6) LOG WHAT WE HAVE AT STARTUP
    RCLCPP_INFO(this->get_logger(),
      "brake_decider node started (reaction_time=%.3f, decel=%.3f, safety_margin=%.3f)",
      reaction_time, decel, safety_margin);
  }


private: // only the class itself can see this. 
// Keeps the param callback alive for the whole node
rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

// Subscription to /ego_speed (Float64 messages)
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ego_speed_sub_;

// Subscription to /obstacle_distance (Float 64 messages)
rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr obstacle_dist_sub_;

// Publisher for /brake_cmd and timer to publish at 20 Hz
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brake_pub_;
rclcpp::TimerBase::SharedPtr tick_timer_;

// Latest sensor values (default 0 until first message arrives)
double last_speed_= 0.0;
double last_distance_ = 0.0;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);  // turns ROS on
  rclcpp::spin(std::make_shared<BrakeDeciderNode>());  //keeps program alive so callbacks can run
  rclcpp::shutdown(); // cleans up when Ctrl+C pressed
  return 0;
}
