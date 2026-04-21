
#pragma once

class PIDController
{
public:
    PIDController(
        double kp, double ki, double kd, 
        // double i_max = 1e6, double i_min = -1e6
        double i_max = 0.2, double i_min = -0.2
    )
    : kp_(kp), ki_(ki), kd_(kd), 
    integral_(0.0), prev_error_(0.0), i_max_(i_max), i_min_(i_min)
    {}

    double compute(double current_pos, double goal_pos, double dt);
    void reset();

private:
    double kp_, ki_, kd_;
    double i_max_, i_min_;
    double integral_, prev_error_;
};

/// @param current_pos   current position
/// @param goal_pos      goal position
/// @param dt            time between the last time this program called
/// @return              control input
double PIDController::compute(double current_pos, double goal_pos, double dt)
{
    if (dt <= 0.0) return 0.0;

    double error = goal_pos - current_pos;

    integral_ += error * dt;
    if (integral_ > i_max_) integral_ = i_max_;
    if (integral_ < i_min_) integral_ = i_min_;

    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    return kp_ * error + ki_ * integral_ + kd_ * derivative;




}


void PIDController::reset()
{
    integral_   = 0.0;
    prev_error_ = 0.0;
}



