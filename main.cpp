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

    Velocity cmd;
    cmd.linear = linear;
    cmd.angular = std::clamp(linear * kappa, -max_angular, max_angular);
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

int main() {
    Pose2D start = {-1.0, -1.0, 0.0};
    Pose2D goal  = {1.0, 1.0, M_PI / 2.0};

    int mode = 0; // 0: Reeds-Shepp tracking, 1: rotate-translate-rotate
    int path_tracking_mode = 1; // 0: P control, 1: Pure pursuit

    std::vector<Pose2D> path;
    if (mode == 0) {
        path = generateReedsSheppPath(start, goal, 0.3);
    }

    Pose2D robot = start;
    double dt = 0.1;
    double position_threshold = 0.05;
    double angle_threshold = 0.05;

    const int scale = 200;
    const int width = 600, height = 600;
    const int offsetX = width / 2, offsetY = height / 2;
    cv::Mat canvas(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

    // --- MP4 video output setup ---
    std::string filename = "output.mp4";
    int codec = cv::VideoWriter::fourcc('a', 'v', 'c', '1');  // MP4 (H.264) codec, requires ffmpeg support
    double fps = 30.0;
    cv::VideoWriter video(filename, codec, fps, cv::Size(width, height));
    if (!video.isOpened()) {
        std::cerr << "Failed to open video file for writing." << std::endl;
        return -1;
    }

    std::vector<cv::Point> trajectory;
    size_t target_index = 0;
    int rtr_state = 0;  // 0: rotate to face goal, 1: move forward, 2: rotate to goal orientation
    int frame_count = 0;

    // Draw planned path once (for mode 0)
    if (mode == 0) {
        for (size_t i = 1; i < path.size(); ++i) {
            cv::line(canvas,
                     toPixel(path[i - 1], scale, offsetX, offsetY),
                     toPixel(path[i], scale, offsetX, offsetY),
                     cv::Scalar(255, 0, 0), 2);  // Blue
        }
    }

    while (true) {
        // Clear canvas
        canvas = cv::Scalar(255, 255, 255);

        // Redraw planned path (for mode 0)
        if (mode == 0) {
            for (size_t i = 1; i < path.size(); ++i) {
                cv::line(canvas,
                         toPixel(path[i - 1], scale, offsetX, offsetY),
                         toPixel(path[i], scale, offsetX, offsetY),
                         cv::Scalar(255, 0, 0), 2);  // Blue
            }
        }

        // --- Robot motion logic ---
        if (mode == 0) {
            // Reeds-Shepp tracking
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
                    Velocity cmd = path_tracking_mode == 0 ? computeVelocityProportionalControl(robot, target)
                                 : path_tracking_mode == 1 ? computeVelocityPurePursuit(robot, path, target_index)
                                 : Velocity{0.0, 0.0};
                    updatePose(robot, cmd, dt);
                }
            }

        } else if (mode == 1) {
            // Rotate-Translate-Rotate motion
            double vx = 0.3;
            double wz = 0.25;

            if (rtr_state == 0) {
                // Step 1: rotate to face goal
                double dx = goal.x - robot.x;
                double dy = goal.y - robot.y;
                double target_theta = std::atan2(dy, dx);
                double angle_error = normalizeAngle(target_theta - robot.theta);

                if (std::abs(angle_error) < angle_threshold) {
                    rtr_state = 1;
                } else {
                    Velocity cmd;
                    cmd.linear = 0.0;
                    cmd.angular = std::min(std::max(2.0 * angle_error, -wz), wz);
                    updatePose(robot, cmd, dt);
                }

            } else if (rtr_state == 1) {
                // Step 2: translate toward goal
                double dx = goal.x - robot.x;
                double dy = goal.y - robot.y;
                double dist = std::hypot(dx, dy);
                position_threshold = 0.2;

                if (dist < position_threshold) {
                    rtr_state = 2;
                } else {
                    Velocity cmd;
                    cmd.linear = vx;
                    cmd.angular = 0.0;
                    updatePose(robot, cmd, dt);
                }

            } else if (rtr_state == 2) {
                // Step 3: rotate to match goal orientation
                double angle_error = normalizeAngle(goal.theta - robot.theta);
                if (std::abs(angle_error) < angle_threshold) break;

                Velocity cmd;
                cmd.linear = 0.0;
                cmd.angular = std::min(std::max(2.0 * angle_error, -wz), wz);
                updatePose(robot, cmd, dt);
            }
        }

        // --- Drawing ---
        trajectory.push_back(toPixel(robot, scale, offsetX, offsetY));
        for (size_t i = 1; i < trajectory.size(); ++i) {
            cv::line(canvas, trajectory[i - 1], trajectory[i], cv::Scalar(0, 0, 255), 2);  // Red
        }

        drawRobotPolygon(canvas, robot, scale, offsetX, offsetY);

        // Show window and write to video
        cv::imshow("Reeds-Shepp Path Tracking", canvas);
        video.write(canvas);  // Save current frame to video

        if (cv::waitKey(30) == 27) break;

        frame_count++;
    }

    // --- Finalize ---
    video.release();
    std::cout << "Reached goal and aligned!" << std::endl;
    std::cout << "Simulated motion time: " << frame_count * dt << " seconds" << std::endl;
    cv::waitKey(0);
    return 0;
}