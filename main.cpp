#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

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
    const double K_linear = 0.5;
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

int main() {
    std::vector<Pose2D> path = {
        {0, 0, 0},
        {1.0, 1.0, 0}
    };

    Pose2D robot = {-1.0, -1.0, 0};
    double dt = 0.1;
    const double threshold = 0.05;
    size_t target_index = 0;

    const int scale = 200;
    const int width = 600, height = 600;
    const int offsetX = width / 2, offsetY = height / 2;
    cv::Mat canvas(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    std::vector<cv::Point> trajectory;

    while (target_index < path.size()) {
        Pose2D target = path[target_index];
        double dx = target.x - robot.x;
        double dy = target.y - robot.y;
        double dist = std::hypot(dx, dy);

        if (dist < threshold) {
            std::cout << "Reached waypoint " << target_index << std::endl;
            target_index++;
            continue;
        }

        Velocity cmd = computeVelocity(robot, target);
        updatePose(robot, cmd, dt);

        trajectory.push_back(toPixel(robot, scale, offsetX, offsetY));
        canvas = cv::Scalar(255, 255, 255);

        for (size_t i = 1; i < trajectory.size(); ++i) {
            cv::line(canvas, trajectory[i - 1], trajectory[i], cv::Scalar(0, 0, 255), 2);
        }

        drawRobotPolygon(canvas, robot, scale, offsetX, offsetY);
        cv::circle(canvas, toPixel(target, scale, offsetX, offsetY), 5, cv::Scalar(0, 255, 0), -1);

        cv::imshow("Robot Path", canvas);
        int key = cv::waitKey(50);
        if (key == 27) break;
    }

    std::cout << "Reached goal!" << std::endl;
    cv::waitKey(0);
    return 0;
}