#include "include/util.hpp"
#include "include/draw.hpp"
#include "include/motion.hpp"

int main() {
    // start and goal information
    std::vector<Pose2D> waypoints = {
        {-1.0, -1.0, M_PI},
        {-0.5, 0.5, M_PI / 4},
        {1.0, 1.0, M_PI / 2}};

    // parameters
    int mode = 1; // 0: Reeds-Shepp tracking, 1: rotate-translate-rotate
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
        // path = waypoints; // use waypoints directly

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
        Velocity cmd;
        bool is_finished = false;
        if (mode == 0) {
            is_finished = computeCommandForReedsShepp(target_index, position_threshold, angle_threshold,
                                                      waypoints, path, robot, dt, path_tracking_mode, cmd);
        } else if (mode == 1) {
            is_finished = computeCommandForRTR(rtr_state, target_index, position_threshold, angle_threshold,
                                               waypoints, path, robot, dt, path_tracking_mode, cmd);
        }
        updatePose(robot, cmd, dt);

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