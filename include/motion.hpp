#pragma once
#include "util.hpp"
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/config.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <tuple>
namespace ob = ompl::base;

class Motion {
public:
    Motion();
    
    // Public methods (referenced from other files)
    bool computeCommandForReedsShepp(size_t &target_id, double position_threshold, double angle_threshold,
                                    const std::vector<Pose2D> &waypoints, const std::vector<Pose2D> &path,
                                    Pose2D &robot, double dt, int path_tracking_mode, Velocity &cmd);
    
    bool computeCommandForRTR(int &rtr_state, size_t &target_id, double position_threshold, double angle_threshold,
                              const std::vector<Pose2D> &waypoints, const std::vector<Pose2D> &path,
                              Pose2D &robot, double dt, int path_tracking_mode, Velocity &cmd, bool allow_backward = false);
    
    bool computeCommandForGuidelessAGV_pprtr(int &rtr_state, size_t &target_id, double position_threshold, double angle_threshold,
                                           const std::vector<Pose2D> &waypoints, const std::vector<Pose2D> &path,
                                           Pose2D &robot, double dt, int path_tracking_mode, Velocity &cmd);
    
    bool computeCommandForGuidelessAGV(int &rtr_state, size_t &target_id, double position_threshold, double angle_threshold,
                                      const std::vector<Pose2D> &waypoints, const std::vector<Pose2D> &path,
                                      Pose2D &robot, double dt, int path_tracking_mode, Velocity &cmd);
    
    std::vector<Pose2D> generateReedsSheppPathFromWaypoints(const std::vector<Pose2D> &waypoints);

private:
    // Member variables (converted from global variables)
    bool rtr_reverse_ = false;
    double rtr_moved_distance_ = 0.0;
    Pose2D prev_robot_pose_ = {0.0, 0.0, 0.0};
    double i_lat_err = 0.0;
    double i_yaw_err = 0.0;
    int prev_target_id = -1;
    bool is_forward = true;
    bool is_recovery_backward = false;
    
    // Private helper methods
    Velocity computeVelocityProportionalControl(const Pose2D& current, const Pose2D& target);
    Velocity computeVelocityPurePursuit(const Pose2D& current, const std::vector<Pose2D>& path, size_t target_id, bool allow_backward = false);
    Velocity computeVelocityLinetrace(const Pose2D& robot, const std::vector<Pose2D>& goal, size_t target_id, bool allow_backward = false);
    std::vector<Pose2D> generateReedsSheppPath(Pose2D start, Pose2D goal, double step_size = 0.1);
    double calc_distance(const Pose2D& a, const Pose2D& b);
    std::tuple<double, double> calc_error(Pose2D next, Pose2D previous, Pose2D robot);
};

// Constructor implementation
Motion::Motion() {
    // Initialize member variables with default values
    rtr_reverse_ = false;
    rtr_moved_distance_ = 0.0;
    prev_robot_pose_ = {0.0, 0.0, 0.0};
    i_lat_err = 0.0;
    i_yaw_err = 0.0;
    prev_target_id = -1;
    is_forward = true;
    is_recovery_backward = false;
}

// Public methods implementation
bool Motion::computeCommandForReedsShepp(size_t &target_id, double position_threshold, double angle_threshold,
                                        const std::vector<Pose2D> &waypoints, const std::vector<Pose2D> &path,
                                        Pose2D &robot, double dt, int path_tracking_mode, Velocity &cmd)
{
    bool is_finished = false;
    if (target_id >= path.size())
    {
        double angle_error = normalizeAngle(waypoints.back().theta - robot.theta);
        if (std::abs(angle_error) < angle_threshold) {
            is_finished = true;
            return is_finished;
        }

        cmd.linear = 0.0;
        cmd.angular = std::min(std::max(2.0 * angle_error, -1.0), 1.0);
    }
    else
    {
        Pose2D target = path[target_id];
        double dx = target.x - robot.x;
        double dy = target.y - robot.y;
        double dist = std::hypot(dx, dy);

        if (dist < position_threshold)
        {
            target_id++;
            cmd = {0.0, 0.0};
        }
        else
        {
            cmd =   path_tracking_mode == 0 ? computeVelocityProportionalControl(robot, target)
                  : path_tracking_mode == 1 ? computeVelocityPurePursuit(robot, path, target_id)
                                            : Velocity{0.0, 0.0};
        }
    }
    return is_finished;
}

bool Motion::computeCommandForGuidelessAGV_pprtr(int &rtr_state, size_t &target_id, double position_threshold, double angle_threshold,
                                   const std::vector<Pose2D> &waypoints, const std::vector<Pose2D> &path,
                                   Pose2D &robot, double dt, int path_tracking_mode, Velocity &cmd)
{
    bool is_finished = false;

    Pose2D target = path[target_id];
    double dx = target.x - robot.x;
    double dy = target.y - robot.y;
    double dist = std::hypot(dx, dy);
    double final_adjustment_distance = 0.3;
    if (target_id == path.size() - 1 && dist < final_adjustment_distance)
    {
        return computeCommandForRTR(rtr_state, target_id, position_threshold, angle_threshold,
                                    waypoints, path, robot, dt, path_tracking_mode, cmd, true);
    }
    else
    {
        rtr_state = 0;
        if (dist < position_threshold)
        {
            target_id++;
            cmd = {0.0, 0.0};
        }
        else
        {
            cmd = computeVelocityPurePursuit(robot, path, target_id, true);
        }
    }
    return is_finished;
}

bool Motion::computeCommandForGuidelessAGV(int &rtr_state, size_t &target_id, double position_threshold, double angle_threshold,
                                   const std::vector<Pose2D> &waypoints, const std::vector<Pose2D> &path,
                                   Pose2D &robot, double dt, int path_tracking_mode, Velocity &cmd)
{
    if (target_id == path.size()) {
        cmd = {0.0, 0.0};
        Pose2D goal = path.back();
        Pose2D prev_goal = path[path.size() - 2];
        const double goal_angle = std::atan2(goal.y - prev_goal.y, goal.x - prev_goal.x);
        std::cout << "goal pose: " << goal.x << ", " << goal.y << ", " << goal_angle << std::endl;
        std::cout << "robot pose: " << robot.x << ", " << robot.y << ", " << robot.theta << std::endl;
        std::cout << "deviation x: " << goal.x - robot.x << ", y: " << goal.y - robot.y << ", "
                  << "theta: " << normalizeAngle(std::atan2(goal.y - prev_goal.y, goal.x - prev_goal.x) - robot.theta) << std::endl;
        return true;
    }

    Pose2D target = path[target_id];
    double dx = target.x - robot.x;
    double dy = target.y - robot.y;
    double dist = std::hypot(dx, dy);

    if (target_id != prev_target_id)
    {
        i_lat_err = 0.0;
        i_yaw_err = 0.0;
        prev_target_id = target_id;
    }

    if (dist < position_threshold)
    {
        target_id++;
    }
    else if (target_id >= 1)
    {
        // from previous goal to current goal
        Pose2D dpc = {path[target_id].x - path[target_id - 1].x,
                      path[target_id].y - path[target_id - 1].y,
                      path[target_id].theta - path[target_id - 1].theta};

        // from previous goal to robot
        Pose2D dpr = {robot.x - path[target_id - 1].x,
                      robot.y - path[target_id - 1].y,
                      robot.theta - path[target_id - 1].theta};
        // from current goal to robot
        Pose2D dcr = {robot.x - path[target_id].x,
                      robot.y - path[target_id].y,
                      robot.theta - path[target_id].theta};

        // check if both inner products are positive
        const double inner_product1 = dpc.x * dpr.x + dpc.y * dpr.y;
        const double inner_product2 = dpc.x * dcr.x + dpc.y * dcr.y;
        constexpr double recovery_threshold = 0.1;
        if (inner_product1 > 0.0 && inner_product2 > 0.0)
        {
            if (target_id == path.size() - 1)
            {
                if (dist > recovery_threshold) {
                    is_recovery_backward = true;
                    i_lat_err = 0.0;
                    i_yaw_err = 0.0;
                }
            }
            else
            {
                target_id++;
            }
        }
        if (is_recovery_backward && inner_product2 < 0.0)
        {
            if (target_id == path.size() - 1)
            {
                if (dist > recovery_threshold) {
                    is_recovery_backward = false;
                    i_lat_err = 0.0;
                    i_yaw_err = 0.0;
                }
            }
        }
    }
    cmd = computeVelocityLinetrace(robot, path, target_id, false);
    return false;
}

bool Motion::computeCommandForRTR(int &rtr_state, size_t &target_id, double position_threshold, double angle_threshold,
                          const std::vector<Pose2D> &waypoints, const std::vector<Pose2D> &path,
                          Pose2D &robot, double dt, int path_tracking_mode, Velocity &cmd, bool allow_backward)
{
    bool is_finished = false;
    double vx = 0.2;
    double wz = 0.2;

    position_threshold = 0.02;
    angle_threshold = 0.05;
    Pose2D target = path[target_id];
    if (rtr_state == 0)
    {
        // Step 1: rotate to face goal
        double dx = target.x - robot.x;
        double dy = target.y - robot.y;
        double target_theta = std::atan2(dy, dx);
        double angle_error = normalizeAngle(target_theta - robot.theta);
        if (std::cos(angle_error) < 0.0)
        {
            angle_error = normalizeAngle(angle_error + M_PI);
            rtr_reverse_ = true;
        } else {
            rtr_reverse_ = false;
        }

        if (std::abs(angle_error) < angle_threshold)
        {
            rtr_state = 1;
        }
        else
        {
            cmd.linear = 0.0;
            cmd.angular = std::clamp(2.0 * angle_error, -wz, wz);
        }
    }
    else if (rtr_state == 1)
    {
        // Step 2: translate toward goal
        double dx = target.x - robot.x;
        double dy = target.y - robot.y;
        double dist = std::hypot(dx, dy);
        double mdx = robot.x - prev_robot_pose_.x;
        double mdy = robot.y - prev_robot_pose_.y;
        rtr_moved_distance_ += std::sqrt(mdx * mdx + mdy * mdy);

        if (dist < position_threshold)
        {
            rtr_state = 2;
        }
        else if (rtr_moved_distance_ > 0.3) {
            rtr_state = 0;
            rtr_moved_distance_ = 0.0;
        }
        else
        {
            if (rtr_reverse_)
            {
                vx = -vx;
            }
            cmd.linear = vx;
            cmd.angular = 0.0;
        }
    }
    else if (rtr_state == 2)
    {
        // Step 3: rotate to match goal orientation
        double angle_error = normalizeAngle(target.theta - robot.theta);
        if (std::abs(angle_error) < angle_threshold) is_finished = true;

        cmd.linear = 0.0;
        cmd.angular = std::clamp(2.0 * angle_error, -wz, wz);
    }
    prev_robot_pose_ = robot;
    return is_finished;
}

std::vector<Pose2D> Motion::generateReedsSheppPathFromWaypoints(const std::vector<Pose2D> &waypoints) {
    std::vector<Pose2D> path;
    for (size_t i = 0; i < waypoints.size() - 1; ++i)
    {
        auto segment = generateReedsSheppPath(waypoints[i], waypoints[i + 1], 0.3);
        // Remove the last point of each segment except the final one to avoid duplicates
        if (!path.empty())
            segment.erase(segment.begin());
        path.insert(path.end(), segment.begin(), segment.end());
    }

    // output path info to console
    std::cout << "Generated path:" << std::endl;
    for (const auto &p : path)
    {
        std::cout << "x: " << p.x << ", y: " << p.y << ", theta: " << p.theta << std::endl;
    }
    return path;
}

std::vector<Pose2D> Motion::generateReedsSheppPath(Pose2D start, Pose2D goal, double step_size) {
    auto space = std::make_shared<ob::ReedsSheppStateSpace>(0.5);

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);

    ob::ScopedState<> s(space), g(space);
    s[0] = start.x; s[1] = start.y; s[2] = start.theta;
    g[0] = goal.x;  g[1] = goal.y;  g[2] = goal.theta;

    ob::ReedsSheppStateSpace::ReedsSheppPath rs_path = space->reedsShepp(s(), g());
    double length = rs_path.length();

    std::vector<Pose2D> result;
    for (double t = 0; t <= length; t += step_size) {
        ob::State *interpolated = space->allocState();
        space->interpolate(s(), g(), t / length, interpolated);

        const auto *se2 = interpolated->as<ob::SE2StateSpace::StateType>();
        Pose2D pose;
        pose.x = se2->getX();
        pose.y = se2->getY();
        pose.theta = se2->getYaw();
        result.push_back(pose);

        space->freeState(interpolated);
    }

    return result;
}

Velocity Motion::computeVelocityProportionalControl(const Pose2D& current, const Pose2D& target) {
    const double K_linear = 2.0;
    const double K_angular = 2.0;

    double dx = target.x - current.x;
    double dy = target.y - current.y;
    double target_angle = std::atan2(dy, dx);
    double angle_diff = normalizeAngle(target_angle - current.theta);
    double distance = std::hypot(dx, dy);

    Velocity cmd;
    cmd.linear = K_linear * distance;
    cmd.angular = K_angular * angle_diff;

    cmd.linear = std::min(cmd.linear, 1.0);
    cmd.angular = std::min(std::max(cmd.angular, -M_PI/2.0), M_PI/2.0);

    return cmd;
}

Velocity Motion::computeVelocityPurePursuit(const Pose2D& current, const std::vector<Pose2D>& path, size_t target_id, bool allow_backward) {
    Velocity cmd = {0.0, 0.0};
    const double lookahead_distance = 0.5;
    const double linear_velocity = 1.0;
    const double max_linear = 1.0;
    const double max_angular = M_PI / 2.0;

    // Select a target point that is at least lookahead_distance away
    Pose2D target = path[target_id];
    std::cout << target_id << std::endl;
    /*
    Pose2D target = path.back();
    for (size_t i = target_id; i < path.size(); ++i) {
        double dx = path[i].x - current.x;
        double dy = path[i].y - current.y;
        if (std::hypot(dx, dy) >= lookahead_distance) {
            target = path[i];
            target_id = i;
            break;
        }
    }
    */

    double dx = target.x - current.x;
    double dy = target.y - current.y;
    double angle_to_target = std::atan2(dy, dx);
    double angle_diff = std::atan2(std::sin(angle_to_target - current.theta), std::cos(angle_to_target - current.theta));

    // Determine whether to reverse if the angular difference exceeds 90 degrees
    bool reverse = std::abs(angle_diff) > M_PI / 2.0;

    // Convert target point to robot-local coordinates (reverse if necessary)
    double x_r = std::cos(-current.theta) * dx - std::sin(-current.theta) * dy;
    double y_r = std::sin(-current.theta) * dx + std::cos(-current.theta) * dy;
    if (reverse && allow_backward) {
        x_r = -x_r;
        y_r = -y_r;
    }

    double alpha = std::atan2(y_r, x_r);
    double kappa = 2.0 * std::sin(alpha) / lookahead_distance;

    double dist_to_target = std::hypot(dx, dy);
    double linear = linear_velocity; // * (dist_to_target / lookahead_distance);
    linear = std::clamp(linear, 0.0, max_linear);

    cmd.linear = linear;
    cmd.angular = std::clamp(linear * kappa, -max_angular, max_angular);
    if (reverse && allow_backward) cmd.linear = -cmd.linear;

    // If heading deviation is large, rotate in place instead of moving forward/backward
    if (std::cos(alpha) < std::cos(M_PI / 10.0)) {
        cmd.linear = 0.0;
        cmd.angular = std::clamp(2.0 * alpha, -max_angular, max_angular);
    }

    return cmd;
}

double Motion::calc_distance(const Pose2D& a, const Pose2D& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

std::tuple<double, double> Motion::calc_error(Pose2D next, Pose2D previous, Pose2D robot) {
    double dpn_x = next.x - previous.x;
    double dpn_y = next.y - previous.y;
    double dpr_x = robot.x - previous.x;
    double dpr_y = robot.y - previous.y;

    double numerator = std::abs(dpn_x * dpr_y - dpn_y * dpr_x);
    double denominator = std::sqrt(dpn_x * dpn_x + dpn_y * dpn_y);
    double cross_product = dpn_x * dpr_y - dpn_y * dpr_x;

    double lat = numerator / denominator * sign(cross_product);
    double yaw = normalizeAngle(robot.theta - std::atan2(dpn_y, dpn_x));

    return {lat, yaw};
}

Velocity Motion::computeVelocityLinetrace(const Pose2D& robot, const std::vector<Pose2D>& goal, size_t target_id, bool allow_backward) {
    double sign_vel = is_forward ? 1.0 : -1.0;
    Velocity cmd_vel = {0.0, 0.0};

    // Parameters
    double max_vx = 1.0;
    double min_vx = 0.02;
    double max_wz = M_PI / 4.0;
    double i_lat_max_err = 1.0;
    double i_yaw_max_err = M_PI;
    double lookahead_distance = is_forward ? 0.15 : -0.15;
    double P_lat = 10.0;
    double I_lat = 1.0;
    double P_yaw = 2.0;
    double I_yaw = 1.0;

    // Accurate tracking in the vicinity of the goal
    if (target_id == goal.size() - 1) {
        max_vx = calc_distance(robot, goal[target_id]);
        min_vx = 0.0;
        lookahead_distance = 0.15;
        P_lat = 50.0;
        I_lat = 5.0;
        if (is_recovery_backward) {
            P_lat = 10.0;
            I_lat = 1.0;
        }
    }

    // Calculate the lookahead point
    Pose2D robot_ahead = robot;
    robot_ahead.x += lookahead_distance * std::cos(robot.theta);
    robot_ahead.y += lookahead_distance * std::sin(robot.theta);

    // Check if target_id is within bounds
    std::cout << target_id << std::endl;
    if (target_id < 1 || target_id >= goal.size()) {
        return cmd_vel;
    }

    // Calculate lat and yaw error
    double lat_err = std::get<0>(calc_error(goal[target_id], goal[target_id - 1], robot_ahead));
    double yaw_err = std::get<1>(calc_error(goal[target_id], goal[target_id - 1], robot_ahead));
    if (!is_forward) {
        lat_err = -lat_err;
        if (yaw_err > M_PI / 2.0)  yaw_err = M_PI - yaw_err;
        if (yaw_err < -M_PI / 2.0) yaw_err = -M_PI - yaw_err;
    }
    i_lat_err += lat_err;
    i_lat_err = std::clamp(i_lat_err, -i_lat_max_err, i_lat_max_err);
    i_yaw_err += yaw_err;
    i_yaw_err = std::clamp(i_yaw_err, -i_yaw_max_err, i_yaw_max_err);

    // Determine whether to reverse if the angular difference exceeds 90 degrees
    bool reverse = std::abs(yaw_err) > M_PI / 2.0;
    if (reverse && allow_backward) {
    }

    // Calculate linear and angular velocities
    constexpr double vx_lat_err_gain = 7.0;
    constexpr double vx_yaw_err_gain = 1.0;
    double vx = max_vx * std::max(0.0, 1.0 - vx_lat_err_gain * std::abs(lat_err)
                                           - vx_yaw_err_gain * std::abs(yaw_err));
    double wz = -(P_lat * lat_err + I_lat * i_lat_err) - (P_yaw * yaw_err + I_yaw * i_yaw_err);
    if (is_recovery_backward) { wz = (P_lat * lat_err + I_lat * i_lat_err) - (P_yaw * yaw_err + I_yaw * i_yaw_err); }
    cmd_vel.linear = sign_vel * std::clamp(vx, min_vx, max_vx); // cmd_vel.linear = max_vx;  // simple command
    cmd_vel.angular = sign_vel * std::clamp(wz, -max_wz, max_wz);
    if (is_recovery_backward) { cmd_vel.linear = -cmd_vel.linear; }

    // If heading error is large or heading for reverse direction,
    // rotate in place instead of moving forward/backward
    /*
    if (std::abs(std::cos(yaw_err)) < std::cos(M_PI / 4.0)) {
        std::cout << "rotate in place" << std::endl;
        const double wz = sign_vel * -(P_yaw * yaw_err);
        cmd_vel.linear = 0.0;
        cmd_vel.angular = std::clamp(wz, -max_wz, max_wz);
    }
    */

    return cmd_vel;
}
