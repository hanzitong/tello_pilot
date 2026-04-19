// auto_lander.cpp
//
// 収束検出 → 自動着陸ノード
// - 自動モード(RBボタン)のとき、マーカーが画像中心付近に一定時間とどまったら "land" を送信する
// - 着陸後は再トリガーしない

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tello_msgs/srv/tello_action.hpp>

using namespace std::chrono_literals;

// 収束とみなす誤差 [ピクセル/100]（= 100ピクセル以内）
static constexpr double kConvergeThreshold = 1.0;
// 着陸トリガーに必要な連続収束フレーム数（10フレーム × 100ms = 1秒）
static constexpr int kConvergeFrames = 10;


class AutoLander : public rclcpp::Node
{
public:
    AutoLander() : Node("auto_lander")
    {
        tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&AutoLander::joy_callback, this, std::placeholders::_1)
        );

        land_client_ = this->create_client<tello_msgs::srv::TelloAction>("tello_action");

        timer_ = this->create_wall_timer(
            100ms, std::bind(&AutoLander::timer_callback, this)
        );
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        last_joy_ = *msg;
        got_joy_  = true;
    }

    void timer_callback()
    {
        if (landed_) return;

        // RBボタン(buttons[5])が押されていないときはカウンタリセット
        if (!got_joy_ || last_joy_.buttons[5] != 1) {
            converge_count_ = 0;
            return;
        }

        // マーカーのカメラ中心からの誤差を取得
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(
                "camera_center_frame", "marker_23_frame", tf2::TimePointZero);
        } catch (const tf2::TransformException &) {
            // マーカー未検出 → カウンタリセット
            converge_count_ = 0;
            return;
        }

        double x = t.transform.translation.x;
        double y = t.transform.translation.y;

        if (std::abs(x) < kConvergeThreshold && std::abs(y) < kConvergeThreshold) {
            converge_count_++;
        } else {
            converge_count_ = 0;
        }

        RCLCPP_DEBUG(this->get_logger(),
            "converge_count=%d  x=%.2f y=%.2f", converge_count_, x, y);

        if (converge_count_ >= kConvergeFrames) {
            RCLCPP_INFO(this->get_logger(),
                "Converged for %d frames. Sending land command.", kConvergeFrames);
            landed_ = true;

            auto request  = std::make_shared<tello_msgs::srv::TelloAction::Request>();
            request->cmd  = "land";
            land_client_->async_send_request(request);
        }
    }

    rclcpp::TimerBase::SharedPtr                              timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr    joy_sub_;
    rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr   land_client_;
    std::shared_ptr<tf2_ros::TransformListener>               tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer>                          tf_buffer_;

    sensor_msgs::msg::Joy last_joy_;
    bool got_joy_{false};
    int  converge_count_{0};
    bool landed_{false};
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoLander>());
    rclcpp::shutdown();
    return 0;
}
