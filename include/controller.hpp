#include "util.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

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
    const double lookahead_distance = 0.2;
    const double linear_velocity = 0.3;
    const double max_linear = 1.0;
    const double max_angular = M_PI / 2.0;

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
    double x_r = std::cos(-current.theta) * dx - std::sin(-current.theta) * dy;
    double y_r = std::sin(-current.theta) * dx + std::cos(-current.theta) * dy;
    double alpha = std::atan2(y_r, x_r);
    double kappa = 2.0 * std::sin(alpha) / lookahead_distance;

    double dist_to_target = std::hypot(target.x - current.x, target.y - current.y);
    double linear = linear_velocity * (dist_to_target / lookahead_distance);
    linear = std::clamp(linear, 0.0, max_linear);

    // Reverse motion handling
    /*
    if (x_r < 0) {
        linear *= -1.0;  // Reverse direction if target is behind
    }
    */

    Velocity cmd;
    cmd.linear = linear;
    cmd.angular = std::clamp(linear * kappa, -max_angular, max_angular);

    // Rotate in place when angular deviation is large
    /*
    if (std::abs(std::cos(alpha)) < std::cos(M_PI / 10.0)) {
        cmd.linear = 0.0;
        cmd.angular = std::clamp(2.0 * alpha, -max_angular, max_angular);
    }
    */
    return cmd;
}