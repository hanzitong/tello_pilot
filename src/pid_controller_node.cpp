
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "pid_controller.hpp"


using namespace std::chrono_literals;

// マーカーロスト判定: TF がこの秒数より古ければ制御を停止する（Bug F 対策）
static constexpr double kMaxStaleSec = 0.5;

class PidVelPublisher : public rclcpp::Node
{
  public:
    PidVelPublisher()
    : Node("pid_controller"),
    pid_x_{1.0, 0., 0.},
    pid_y_{1.0, 0., 0.},
    last_time_(this->now())
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("pid_vel", rclcpp::QoS(1));
      timer_ = this->create_wall_timer(
        100ms, std::bind(&PidVelPublisher::timer_callback, this)
      );
      tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

  private:
    void timer_callback()
    {
        const rclcpp::Time now = this->now();
        const double dt = (now - last_time_).seconds();
        last_time_ = now;

        geometry_msgs::msg::Twist pid_vel_msg;
        pid_vel_msg.linear.x  = 0.;
        pid_vel_msg.linear.y  = 0.;
        pid_vel_msg.linear.z  = 0.;
        pid_vel_msg.angular.x = 0.;
        pid_vel_msg.angular.y = 0.;
        pid_vel_msg.angular.z = 0.;

        try {
            // camera_frame から見た drone_frame の位置と姿勢を取得する
            // = drone の3D位置 [m] in camera_frame
            const auto t = tf_buffer_->lookupTransform(
                "camera_frame", "drone_frame", tf2::TimePointZero);

            // --- Bug F: staleness チェック ---
            // tf2::TimePointZero はキャッシュ内の最新値を返すが、
            // マーカーロスト後も最大 10 秒キャッシュに残り続ける。
            // TF のタイムスタンプを直接確認して古い値を排除する。
            const rclcpp::Time tf_stamp(
                t.header.stamp.sec, t.header.stamp.nanosec, RCL_ROS_TIME);
            if ((now - tf_stamp).seconds() > kMaxStaleSec) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Stale TF (%.2fs): marker lost. Stopping PID.",
                    (now - tf_stamp).seconds());
                publisher_->publish(pid_vel_msg);  // ゼロ出力
                return;
            }

            // --- PID を camera_frame（固定フレーム）で計算し、出力を drone_frame に変換する ---
            //
            // 設計根拠:
            //   誤差の積分・微分は固定フレーム (camera_frame) で行う必要がある。
            //   drone_frame は yaw とともに回転するため、drone_frame で積分すると
            //   yaw 変化のたびに積分軸が変わり、I・D 項が誤った方向に働く。
            //   正しい手順:
            //     1. camera_frame での位置誤差 (tx, ty) を PID に入力する
            //     2. PID 出力（camera_frame 表現の速度）を drone_frame に回転してから publish する
            //
            // t.translation = camera_frame でのドローン位置 [m]（目標は原点）
            // t.rotation    = drone_frame → camera_frame への回転 (q_drone_to_cam)

            const double tx = t.transform.translation.x;
            const double ty = t.transform.translation.y;

            // Step 1: camera_frame で PID 計算（単位: [m]、出力: 無次元速度指令）
            // NOTE: ゲイン (kp=1.0) は実機テストで調整が必要
            const double vel_x_cam = pid_x_.compute(tx, 0., dt);
            const double vel_y_cam = pid_y_.compute(ty, 0., dt);

            // Step 2: 速度指令を camera_frame から drone_frame に回転変換する
            // q_drone_to_cam.inverse(): camera_frame → drone_frame への回転
            const tf2::Quaternion q_drone_to_cam(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w
            );
            const tf2::Vector3 vel_drone = tf2::quatRotate(
                q_drone_to_cam.inverse(),
                tf2::Vector3(vel_x_cam, vel_y_cam, 0.0)
            );

            pid_vel_msg.linear.x = vel_drone.x();
            pid_vel_msg.linear.y = vel_drone.y();

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "lookupTransform failed: %s", ex.what());
        }

        // 出力クランプ: 過大な速度指令を制限する
        auto clamp = [](double v) -> double {
            if (v >  0.8) return  0.5;
            if (v < -0.8) return -0.5;
            return v;
        };
        pid_vel_msg.linear.x = clamp(pid_vel_msg.linear.x);
        pid_vel_msg.linear.y = clamp(pid_vel_msg.linear.y);

        publisher_->publish(pid_vel_msg);
    }


    rclcpp::TimerBase::SharedPtr                                  timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       publisher_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener>                   tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer>                              tf_buffer_;

    PIDController pid_x_;
    PIDController pid_y_;
    rclcpp::Time  last_time_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PidVelPublisher>());
  rclcpp::shutdown();
  return 0;
}
