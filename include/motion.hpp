#pragma once
#include "util.hpp"
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/config.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
namespace ob = ompl::base;

Velocity computeVelocityProportionalControl(const Pose2D& current, const Pose2D& target);
Velocity computeVelocityPurePursuit(const Pose2D& current, const std::vector<Pose2D>& path, size_t target_index);
std::vector<Pose2D> generateReedsSheppPath(Pose2D start, Pose2D goal, double step_size);
bool computeCommandForRTR(int &rtr_state, size_t &target_index, double position_threshold, double angle_threshold,
                          const std::vector<Pose2D> &waypoints, const std::vector<Pose2D> &path,
                          Pose2D &robot, double dt, int path_tracking_mode, Velocity &cmd);

bool computeCommandForReedsShepp(size_t &target_index, double position_threshold, double angle_threshold,
                                 const std::vector<Pose2D> &waypoints, const std::vector<Pose2D> &path,
                                 Pose2D &robot, double dt, int path_tracking_mode, Velocity &cmd)
{
    bool is_finished = false;
    if (target_index >= path.size())
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
        Pose2D target = path[target_index];
        double dx = target.x - robot.x;
        double dy = target.y - robot.y;
        double dist = std::hypot(dx, dy);

        if (dist < position_threshold)
        {
            target_index++;
            cmd = {0.0, 0.0};
        }
        else
        {
            cmd =   path_tracking_mode == 0 ? computeVelocityProportionalControl(robot, target)
                  : path_tracking_mode == 1 ? computeVelocityPurePursuit(robot, path, target_index)
                                            : Velocity{0.0, 0.0};
        }
    }
    return is_finished;
}

bool computeCommandForGuidelessAGV(int &rtr_state, size_t &target_index, double position_threshold, double angle_threshold,
                                   const std::vector<Pose2D> &waypoints, const std::vector<Pose2D> &path,
                                   Pose2D &robot, double dt, int path_tracking_mode, Velocity &cmd)
{
    bool is_finished = false;

    Pose2D target = path[target_index];
    double dx = target.x - robot.x;
    double dy = target.y - robot.y;
    double dist = std::hypot(dx, dy);
    double final_adjustment_distance = 0.3;
    if (target_index == path.size() - 1 && dist < final_adjustment_distance)
    {
        return computeCommandForRTR(rtr_state, target_index, position_threshold, angle_threshold,
                                    waypoints, path, robot, dt, path_tracking_mode, cmd);
    }
    else
    {
        rtr_state = 0;
        if (dist < position_threshold)
        {
            target_index++;
            cmd = {0.0, 0.0};
        }
        else
        {
            cmd = computeVelocityPurePursuit(robot, path, target_index);
        }
    }
    return is_finished;
}

bool computeCommandForRTR(int &rtr_state, size_t &target_index, double position_threshold, double angle_threshold,
                         const std::vector<Pose2D> &waypoints, const std::vector<Pose2D> &path,
                         Pose2D &robot, double dt, int path_tracking_mode, Velocity &cmd)
{
    bool is_finished = false;
    double vx = 0.2;
    double wz = 0.2;

    position_threshold = 0.02;
    angle_threshold = 1e-5;
    Pose2D target = path[target_index];
    if (rtr_state == 0)
    {
        // Step 1: rotate to face goal
        double dx = target.x - robot.x;
        double dy = target.y - robot.y;
        double target_theta = std::atan2(dy, dx);
        double angle_error = normalizeAngle(target_theta - robot.theta);

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

        if (dist < position_threshold)
        {
            rtr_state = 2;
        }
        else
        {
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
    return is_finished;
}

std::vector<Pose2D> generateReedsSheppPathFromWaypoints(const std::vector<Pose2D> &waypoints) {
    std::vector<Pose2D> path;
    for (size_t i = 0; i < waypoints.size() - 1; ++i)
    {
        auto segment = generateReedsSheppPath(waypoints[i], waypoints[i + 1], 0.3);
        // Remove the last point of each segment except the final one to avoid duplicates
        if (!path.empty())
            segment.erase(segment.begin());
        path.insert(path.end(), segment.begin(), segment.end());
    }
    // path = waypoints; // use waypoints directly

    // output path info to console
    /*
    std::cout << "Generated path:" << std::endl;
    for (const auto &p : path)
    {
        std::cout << "x: " << p.x << ", y: " << p.y << ", theta: " << p.theta << std::endl;
    }
    */
    return path;
}

std::vector<Pose2D> generateReedsSheppPath(Pose2D start, Pose2D goal, double step_size = 0.1) {
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

Velocity computeVelocityProportionalControl(const Pose2D& current, const Pose2D& target) {
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

Velocity computeVelocityPurePursuit(const Pose2D& current, const std::vector<Pose2D>& path, size_t target_index) {
    Velocity cmd = {0.0, 0.0};
    const double lookahead_distance = 0.01;
    const double linear_velocity = 0.3;
    const double max_linear = 1.0;
    const double max_angular = M_PI / 2.0;

    // Select a target point that is at least lookahead_distance away
    Pose2D target = path.back();
    for (size_t i = target_index; i < path.size(); ++i) {
        double dx = path[i].x - current.x;
        double dy = path[i].y - current.y;
        if (std::hypot(dx, dy) >= lookahead_distance) {
            target = path[i];
            break;
        }
    }

    double dx = target.x - current.x;
    double dy = target.y - current.y;
    double angle_to_target = std::atan2(dy, dx);
    double angle_diff = std::atan2(std::sin(angle_to_target - current.theta), std::cos(angle_to_target - current.theta));

    // Determine whether to reverse if the angular difference exceeds 90 degrees
    bool reverse = std::abs(angle_diff) > M_PI / 2.0;

    // Convert target point to robot-local coordinates (reverse if necessary)
    double x_r = std::cos(-current.theta) * dx - std::sin(-current.theta) * dy;
    double y_r = std::sin(-current.theta) * dx + std::cos(-current.theta) * dy;
    if (reverse) {
        x_r = -x_r;
        y_r = -y_r;
    }

    double alpha = std::atan2(y_r, x_r);
    double kappa = 2.0 * std::sin(alpha) / lookahead_distance;

    double dist_to_target = std::hypot(dx, dy);
    double linear = linear_velocity * (dist_to_target / lookahead_distance);
    linear = std::clamp(linear, 0.0, max_linear);
    if (reverse) linear = -linear;

    cmd.linear = linear;
    cmd.angular = std::clamp(linear * kappa, -max_angular, max_angular);

    // If heading deviation is large, rotate in place instead of moving forward/backward
    if (std::abs(std::cos(alpha)) < std::cos(M_PI / 10.0)) {
        cmd.linear = 0.0;
        cmd.angular = std::clamp(2.0 * alpha, -max_angular, max_angular);
    }

    return cmd;
}