#include "include/util.hpp"
#include "include/draw.hpp"
#include "include/motion.hpp"

int main() {
    // start and goal information
    std::vector<Pose2D> waypoints = {
        {-1.0, -1.0, 0.0},
        {-0.5, 0.5, M_PI / 4},
        {1.0, 1.0, M_PI / 2}};

    // Draw setup
    const int scale = 200;
    const int width = 600, height = 600;
    Draw draw(width, height, scale);
    const std::string video_filename = "output.mp4";
    draw.setupVideoWriter(video_filename, 30);

    // parameters
    int mode = 0; // 0: Reeds-Shepp tracking, 1: rotate-translate-rotate
    int path_tracking_mode = 1; // 0: P control, 1: Pure pursuit
    double dt = 0.1;
    double position_threshold = 0.05;
    double angle_threshold = 0.05;

    // state
    Pose2D robot = waypoints[0];
    size_t target_index = 0;
    int rtr_state = 0;  // 0: rotate to face goal, 1: move forward, 2: rotate to goal orientation
    int frame_count = 0;

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

    while (true) {
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
        
        draw.draw(mode, robot, path);

        if (cv::waitKey(30) == 27 || is_finished) break;

        frame_count++;
    }

    // --- Finalize ---
    draw.releaseVideo();
    std::cout << "Reached goal and aligned!" << std::endl;
    std::cout << "Simulated motion time: " << frame_count * dt << " seconds" << std::endl;
    cv::waitKey(0);
    return 0;
}