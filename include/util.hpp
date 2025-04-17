#pragma once
#include <iostream>
#include <cmath>
#include <vector>

struct Pose2D {
    double x;
    double y;
    double theta;
};

struct Velocity {
    double linear;
    double angular;
};

double normalizeAngle(double angle) {
    while (angle > M_PI)  angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void updatePose(Pose2D& pose, const Velocity& cmd, double dt) {
    pose.x += cmd.linear * std::cos(pose.theta) * dt;
    pose.y += cmd.linear * std::sin(pose.theta) * dt;
    pose.theta += cmd.angular * dt;
    pose.theta = normalizeAngle(pose.theta);
}