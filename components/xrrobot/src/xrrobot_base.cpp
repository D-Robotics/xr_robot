#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "XRMiddleware.cpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "xrrobot_msgs/msg/battery.hpp"
#include "xrrobot_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class XRMiddleWare : public rclcpp::Node {
 public:
  XRMiddleWare() : Node("xr_middle_ware"), count_(0) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    battery_pub_ =
        this->create_publisher<xrrobot_msgs::msg::Battery>("voltage", qos);
    imu_pub_ = this->create_publisher<xrrobot_msgs::msg::Imu>("raw_imu", qos);
    odom_pub_ =
        this->create_publisher<nav_msgs::msg::Odometry>("raw_odom", qos);

    update_timer_ = this->create_wall_timer(
        25ms, std::bind(&XRMiddleWare::timer_callback, this));

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        qos,
        std::bind(&XRMiddleWare::command_velocity_callback,
                  this,
                  std::placeholders::_1));

    xr = std::make_shared<XRMiddleware>("/dev/xrbase", 115200);
    // xr->Deinit();
    // usleep(20000);
    // xr->SetPID(20, 0, 0);
    // usleep(20000);
    // xr->SetParams(3150, 0.022, 0.0545, 0.051);
    usleep(20000);
    xr->Init();
    usleep(20000);
  }
  void InitTf() {
    tf_broadcaster_ =
        std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
  }

 private:
  void handle_imu() {
    float imu_list[9] = {0};
    xr->GetIMU(imu_list);

    xrrobot_msgs::msg::Imu imu_data;
    imu_data.angular_velocity.x = imu_list[0];
    imu_data.angular_velocity.y = imu_list[1];
    imu_data.angular_velocity.z = imu_list[2];
    //  printf("imu z: %f\n", imu_list[2]);
    imu_data.linear_acceleration.x = imu_list[3];
    imu_data.linear_acceleration.y = imu_list[4];
    imu_data.linear_acceleration.z = imu_list[5];
    imu_data.magnetic_field.x = imu_list[6];
    imu_data.magnetic_field.y = imu_list[7];
    imu_data.magnetic_field.z = imu_list[8];
    imu_pub_->publish(imu_data);
  }

  void handle_odom() {
    float factor = 1.0f;
    float odom_list[6] = {0};
    xr->GetOdom(odom_list);

    geometry_msgs::msg::TransformStamped tf_transfer;
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = odom_list[0] / factor;
    odom.pose.pose.position.y = odom_list[1] / factor;
    //  printf("odom p: [x, y]: [%f, %f]\n", odom_list[0], odom_list[1]);
    odom.pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odom_list[2] / factor);
    //  printf("odom yaw: %f\n", odom_list[2]);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // printf("odom orientation: [x, y, z, w]: [%f, %f, %f, %f]\n",
    // 		    q.x(), q.y(), q.z(), q.w());

    odom.twist.twist.linear.x = odom_list[3] / factor;
    odom.twist.twist.linear.y = odom_list[4] / factor;
    odom.twist.twist.angular.z = odom_list[5] / factor;
    // printf("odom linear: [x, y, angularz]: [%f, %f, %f]\n",
    // 		    odom_list[3], odom_list[4], odom_list[5]);
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;

    odom_pub_->publish(odom);

    tf_transfer.header.stamp = odom.header.stamp;
    tf_transfer.header.frame_id = odom.header.frame_id;
    tf_transfer.child_frame_id = odom.child_frame_id;
    tf_transfer.transform.translation.x = odom.pose.pose.position.x;
    tf_transfer.transform.translation.y = odom.pose.pose.position.y;
    tf_transfer.transform.translation.z = odom.pose.pose.position.z;
    tf_transfer.transform.rotation = odom.pose.pose.orientation;

    //  tf_broadcaster_->sendTransform(tf_transfer);
  }

  void handle_bat() {
    auto battery_data = xr->GetBattery();
    xrrobot_msgs::msg::Battery bat;
    bat.battery = battery_data;
    battery_pub_->publish(bat);
  }

  void timer_callback() {
    count_++;
    handle_imu();
    handle_odom();
    if (count_ > 60) {
      count_ = 0;
      handle_bat();
    }
  }

  void command_velocity_callback(
      const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) {
    //	  printf("set z: %f\n", cmd_vel_msg->angular.z);
    xr->SetVelocity(cmd_vel_msg->linear.x, 0, cmd_vel_msg->angular.z);
  }

  rclcpp::TimerBase::SharedPtr update_timer_;

  rclcpp::Publisher<xrrobot_msgs::msg::Battery>::SharedPtr battery_pub_;
  rclcpp::Publisher<xrrobot_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  size_t count_;

  std::shared_ptr<XRMiddleware> xr;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<XRMiddleWare>();
  node->InitTf();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
