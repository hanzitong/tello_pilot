// auto_lander_node.cpp
//
// 収束検出 → 自動着陸ノード
// - 自動モード(RBボタン)のとき、ドローンが camera_frame 上の目標位置（XY 原点）
//   付近に一定時間とどまったら "land" を送信する
// - 着陸後は再トリガーしない

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tello_msgs/srv/tello_action.hpp>

using namespace std::chrono_literals;

// 収束とみなす XY 誤差 [m]
static constexpr double kConvergeThreshold = 0.10;
// 着陸トリガーに必要な連続収束フレーム数（10 フレーム × 100ms = 1 秒）
static constexpr int    kConvergeFrames    = 10;
// マーカーロスト判定: TF がこの秒数より古ければロスト扱い（Bug F 対策）
static constexpr double kMaxStaleSec       = 0.5;


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
    rclcpp::TimerBase::SharedPtr                              timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr    joy_sub_;
    rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr   land_client_;
    std::shared_ptr<tf2_ros::TransformListener>               tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer>                          tf_buffer_;

    sensor_msgs::msg::Joy last_joy_;
    bool got_joy_{false};
    int  converge_count_{0};
    bool landed_{false};

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        last_joy_ = *msg;
        got_joy_  = true;
    }

    void timer_callback()
    {
        if (landed_) return;

        // RB ボタン (buttons[5]) が押されていないときはカウンタリセット
        const bool rb_pressed =
            got_joy_ &&
            static_cast<int>(last_joy_.buttons.size()) > 5 &&
            last_joy_.buttons[5] == 1;

        if (!rb_pressed) {
            converge_count_ = 0;
            return;
        }

        // drone_frame の camera_frame に対する位置を取得する
        // = camera_frame でのドローン XY 変位 [m]
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(
                "camera_frame", "drone_frame", tf2::TimePointZero);
        } catch (const tf2::TransformException &) {
            converge_count_ = 0;
            return;
        }

        // --- Bug F: staleness チェック ---
        const rclcpp::Time tf_stamp(
            t.header.stamp.sec, t.header.stamp.nanosec, RCL_ROS_TIME);
        if ((this->now() - tf_stamp).seconds() > kMaxStaleSec) {
            converge_count_ = 0;
            return;
        }

        // XY 誤差のみで収束判定（Z = 高度は現時点では使わない）
        const double x = t.transform.translation.x;
        const double y = t.transform.translation.y;

        if (std::abs(x) < kConvergeThreshold && std::abs(y) < kConvergeThreshold) {
            converge_count_++;
        } else {
            converge_count_ = 0;
        }

        RCLCPP_DEBUG(this->get_logger(),
            "converge_count=%d  x=%.3f y=%.3f [m]", converge_count_, x, y);

        if (converge_count_ >= kConvergeFrames) {
            RCLCPP_INFO(this->get_logger(),
                "Converged for %d frames (x=%.3f y=%.3f [m]). Sending land command.",
                kConvergeFrames, x, y);
            landed_ = true;

            auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
            request->cmd = "land";
            land_client_->async_send_request(request);
        }
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoLander>());
    rclcpp::shutdown();
    return 0;
}
