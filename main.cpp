#include <opencv2/opencv.hpp>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/config.h>
#include <iostream>
#include <vector>
#include <cmath>

namespace ob = ompl::base;

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

Velocity computeVelocity(const Pose2D& current, const Pose2D& target) {
    const double K_linear = 0.8;
    const double K_angular = 2.0;

    double dx = target.x - current.x;
    double dy = target.y - current.y;
    double target_angle = std::atan2(dy, dx);
    double angle_diff = normalizeAngle(target_angle - current.theta);
    double distance = std::hypot(dx, dy);

    Velocity cmd;
    cmd.linear = K_linear * distance;
    cmd.angular = K_angular * angle_diff;

    cmd.linear = std::min(cmd.linear, 0.5);
    cmd.angular = std::min(std::max(cmd.angular, -1.0), 1.0);

    return cmd;
}

void updatePose(Pose2D& pose, const Velocity& cmd, double dt) {
    pose.x += cmd.linear * std::cos(pose.theta) * dt;
    pose.y += cmd.linear * std::sin(pose.theta) * dt;
    pose.theta += cmd.angular * dt;
    pose.theta = normalizeAngle(pose.theta);
}

cv::Point2i toPixel(const Pose2D& pose, int scale, int offsetX, int offsetY) {
    return cv::Point2i(
        static_cast<int>(pose.x * scale) + offsetX,
        static_cast<int>(pose.y * scale) + offsetY
    );
}

void drawRobotPolygon(cv::Mat& canvas, const Pose2D& pose, int scale, int offsetX, int offsetY) {
    std::vector<cv::Point> shape_local = {
        {  20,   0  },  // Front
        {  12,  12  },
        { -12,  12  },
        { -12, -12  },
        {  12, -12  },
    };

    std::vector<cv::Point> shape_world;

    double cos_theta = std::cos(pose.theta);
    double sin_theta = std::sin(pose.theta);

    for (const auto& p : shape_local) {
        double x_rot = p.x * cos_theta - p.y * sin_theta;
        double y_rot = p.x * sin_theta + p.y * cos_theta;

        int x = static_cast<int>(pose.x * scale + x_rot) + offsetX;
        int y = static_cast<int>(pose.y * scale + y_rot) + offsetY;

        shape_world.emplace_back(x, y);
    }

    const cv::Point* pts = shape_world.data();
    int npts = static_cast<int>(shape_world.size());
    cv::polylines(canvas, &pts, &npts, 1, true, cv::Scalar(0, 0, 0), 2);
    cv::fillPoly(canvas, &pts, &npts, 1, cv::Scalar(0, 0, 0));
}

std::vector<Pose2D> generateReedsSheppPath(Pose2D start, Pose2D goal, double step_size = 0.1) {
    auto space = std::make_shared<ob::ReedsSheppStateSpace>(1.0);

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

int main() {
    Pose2D start = {-1.0, -1.0, M_PI / 2.0};
    Pose2D goal  = {1.0, 1.0, 0.0};

    std::vector<Pose2D> path = generateReedsSheppPath(start, goal);

    Pose2D robot = start;
    double dt = 0.1;
    const double position_threshold = 0.05;
    const double angle_threshold = 0.05;

    const int scale = 200;
    const int width = 600, height = 600;
    const int offsetX = width / 2, offsetY = height / 2;
    cv::Mat canvas(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    std::vector<cv::Point> trajectory;
    size_t target_index = 0;

    while (true) {
        if (target_index >= path.size()) {
            double angle_error = normalizeAngle(goal.theta - robot.theta);
            if (std::abs(angle_error) < angle_threshold) break;

            Velocity cmd;
            cmd.linear = 0.0;
            cmd.angular = std::min(std::max(2.0 * angle_error, -1.0), 1.0);
            updatePose(robot, cmd, dt);
        } else {
            Pose2D target = path[target_index];
            double dx = target.x - robot.x;
            double dy = target.y - robot.y;
            double dist = std::hypot(dx, dy);

            if (dist < position_threshold) {
                target_index++;
            } else {
                Velocity cmd = computeVelocity(robot, target);
                updatePose(robot, cmd, dt);
            }
        }

        trajectory.push_back(toPixel(robot, scale, offsetX, offsetY));
        canvas = cv::Scalar(255, 255, 255);

        for (size_t i = 1; i < trajectory.size(); ++i) {
            cv::line(canvas, trajectory[i - 1], trajectory[i], cv::Scalar(0, 0, 255), 2);
        }

        drawRobotPolygon(canvas, robot, scale, offsetX, offsetY);
        cv::imshow("Reeds-Shepp Path Tracking", canvas);
        if (cv::waitKey(30) == 27) break;
    }

    std::cout << "Reached goal and aligned!" << std::endl;
    cv::waitKey(0);
    return 0;
}
