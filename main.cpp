#include "include/util.hpp"
#include "include/controller.hpp"
#include "include/draw.hpp"
#include "include/path_planner.hpp"

int main() {
    // start and goal information
    std::vector<Pose2D> waypoints = {
        {-1.0, -1.0, M_PI},
        {-0.5, 0.5, M_PI / 4},
        {1.0, 1.0, M_PI / 2}};

    // parameters
    int mode = 0; // 0: Reeds-Shepp tracking, 1: rotate-translate-rotate
    int path_tracking_mode = 1; // 0: P control, 1: Pure pursuit
    double dt = 0.1;
    double position_threshold = 0.05;
    double angle_threshold = 0.05;

    std::vector<Pose2D> path;
    if (mode == 0) {
        for (size_t i = 0; i < waypoints.size() - 1; ++i)
        {
            auto segment = generateReedsSheppPath(waypoints[i], waypoints[i + 1], 0.3);
            // Remove the last point of each segment except the final one to avoid duplicates
            if (!path.empty())
                segment.erase(segment.begin());
            path.insert(path.end(), segment.begin(), segment.end());
        }
        // path = waypoints; // For testing, use waypoints directly

        // output path info to console
        /*
        std::cout << "Generated path:" << std::endl;
        for (const auto &p : path)
        {
            std::cout << "x: " << p.x << ", y: " << p.y << ", theta: " << p.theta << std::endl;
        }
        */
    }

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

    Pose2D robot = waypoints[0];
    std::vector<cv::Point> trajectory;
    size_t target_index = 0;
    int rtr_state = 0;  // 0: rotate to face goal, 1: move forward, 2: rotate to goal orientation
    int frame_count = 0;

    while (true) {
        // Clear canvas
        canvas = cv::Scalar(255, 255, 255);

        // Draw planned path (for mode 0)
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
                double angle_error = normalizeAngle(waypoints.back().theta - robot.theta);
                if (std::abs(angle_error) < angle_threshold) break;

                Velocity cmd;
                cmd.linear = 0.0;
                cmd.angular = std::min(std::max(2.0 * angle_error, -1.0), 1.0);
                updatePose(robot, cmd, dt);
                mode = 1;
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
                double dx = waypoints.back().x - robot.x;
                double dy = waypoints.back().y - robot.y;
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
                double dx = waypoints.back().x - robot.x;
                double dy = waypoints.back().y - robot.y;
                double dist = std::hypot(dx, dy);
                position_threshold = 0.02;

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
                double angle_error = normalizeAngle(waypoints.back().theta - robot.theta);
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