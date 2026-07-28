#include <algorithm>
#include <atomic>
#include <memory>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

using unitree::robot::go2::SportClient;

class SafeCmdVelBridge {
 public:
  explicit SafeCmdVelBridge(const std::string& interface)
      : last_command_(ros::Time(0)), stopped_(true) {
    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    // Acquire the Sport service lease after a robot reboot so this bridge is
    // the active high-level motion controller.
    client_ = std::make_shared<SportClient>(true);
    client_->SetTimeout(1.0f);
    client_->Init();
    client_->WaitLeaseApplied();
    state_subscriber_ = std::make_shared<
        unitree::robot::ChannelSubscriber<
            unitree_go::msg::dds_::SportModeState_>>("rt/sportmodestate");
    state_subscriber_->InitChannel(
        std::bind(&SafeCmdVelBridge::stateCallback, this,
                  std::placeholders::_1),
        1);
    subscriber_ =
        nh_.subscribe("/cmd_vel", 1, &SafeCmdVelBridge::commandCallback, this);
    watchdog_ =
        nh_.createTimer(ros::Duration(0.05), &SafeCmdVelBridge::watchdog, this);
  }

  ~SafeCmdVelBridge() { stop(); }

 private:
  void stateCallback(const void* raw) {
    const auto& state =
        *static_cast<const unitree_go::msg::dds_::SportModeState_*>(raw);
    ROS_INFO_THROTTLE(
        1.0,
        "Sport state: mode=%u gait=%u vx=%.3f vy=%.3f yaw=%.3f",
        static_cast<unsigned>(state.mode()),
        static_cast<unsigned>(state.gait_type()), state.velocity()[0],
        state.velocity()[1], state.yaw_speed());
  }

  void commandCallback(const geometry_msgs::Twist::ConstPtr& message) {
    const float vx =
        std::max(-0.15f, std::min(0.50f, static_cast<float>(message->linear.x)));
    const float vy =
        std::max(-0.12f, std::min(0.12f, static_cast<float>(message->linear.y)));
    const float wz = std::max(
        -0.30f, std::min(0.30f, static_cast<float>(message->angular.z)));
    last_command_ = ros::Time::now();
    const int32_t result = client_->Move(vx, vy, wz);
    if (result != 0) {
      ROS_ERROR_THROTTLE(1.0, "Unitree Move command failed: %d", result);
      stop();
      return;
    }
    ROS_INFO_THROTTLE(1.0, "Unitree Move accepted: vx=%.2f vy=%.2f wz=%.2f", vx, vy, wz);
    stopped_ = (vx == 0.0f && vy == 0.0f && wz == 0.0f);
  }

  void watchdog(const ros::TimerEvent&) {
    if (!stopped_ && (ros::Time::now() - last_command_).toSec() > 0.25) {
      ROS_ERROR("cmd_vel timeout; forcing StopMove");
      stop();
    }
  }

  void stop() {
    if (client_) client_->StopMove();
    stopped_ = true;
  }

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  ros::Timer watchdog_;
  std::shared_ptr<SportClient> client_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<
      unitree_go::msg::dds_::SportModeState_>>
      state_subscriber_;
  ros::Time last_command_;
  std::atomic<bool> stopped_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "unitree_safe_cmd_vel_bridge");
  if (argc != 2) {
    ROS_FATAL("Usage: go2_move_sub NETWORK_INTERFACE");
    return 2;
  }
  SafeCmdVelBridge bridge(argv[1]);
  ros::spin();
  return 0;
}
