// tello_pilot cmd_multiplexer_node.cpp

#include <memory>
#include <functional>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tello_msgs/srv/tello_action.hpp"

using namespace std::chrono_literals;


class CmdMultiplexer : public rclcpp::Node
{
public:
    CmdMultiplexer() : Node("cmd_multiplexer")
    {
        // KeepLast(N): 履歴ポリシー。キューに最新 N 件のみ保持し、超過分は破棄する。
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", rclcpp::SensorDataQoS(rclcpp::KeepLast(1)),
            std::bind(&CmdMultiplexer::joy_callback, this, std::placeholders::_1)
        );

        pid_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "pid_vel", rclcpp::QoS(1),
            std::bind(&CmdMultiplexer::pid_callback, this, std::placeholders::_1)
        );

        selected_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::QoS(1)
        );

        action_client_ = this->create_client<tello_msgs::srv::TelloAction>("tello_action");

        timer_ = this->create_wall_timer(
            30ms, std::bind(&CmdMultiplexer::timer_callback, this)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr     joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pid_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    selected_twist_pub_;
    rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr    action_client_;
    rclcpp::TimerBase::SharedPtr                               timer_;

    sensor_msgs::msg::Joy     last_joy_;  // buttons/axes は空ベクトルで構築される。
                                          // resize しないと timer_callback での直接アクセスが UB になるため、
                                          // コンストラクタで resize するか got_joy_ で未受信をガードする必要がある。
    geometry_msgs::msg::Twist last_pid_;
    bool                      got_joy_{false};

private:
    // テイクオフ/着陸はボタン変化時に1回だけ呼ばれる joy_callback で処理する。
    // joy_node は autorepeat_rate=0（デフォルト）のとき状態変化時のみメッセージを送るため、
    // ボタン押下で1回だけ send_action が呼ばれる。
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->buttons[7] == 1) send_action("takeoff");  // Start
        if (msg->buttons[6] == 1) send_action("land");     // Back
        last_joy_ = *msg;
        got_joy_  = true;
    }

    void pid_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_pid_ = *msg;
    }

    void timer_callback()
    {
        if (!got_joy_) return;

        geometry_msgs::msg::Twist twist;

        if (last_joy_.buttons[5] == 1) {         // RB: 自動モード
            twist.linear.x  = last_pid_.linear.x;
            twist.linear.y  = last_pid_.linear.y;
            twist.angular.z = last_pid_.angular.z;
        } else if (last_joy_.buttons[4] == 1) {  // LB: テスト前進
            twist.linear.x = 0.5;
        } else {                                   // 手動モード
            twist.linear.x  = last_joy_.axes[4];
            twist.linear.y  = last_joy_.axes[3];
            twist.angular.z = last_joy_.axes[0];
        }
        twist.linear.z = last_joy_.axes[1];

        selected_twist_pub_->publish(twist);
    }

    void send_action(const std::string & cmd)
    {
        auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
        request->cmd = cmd;
        action_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Sent tello_action: %s", cmd.c_str());
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdMultiplexer>());
    rclcpp::shutdown();
    return 0;
}
