#include "include/util.hpp"
#include "include/draw.hpp"
#include "include/motion.hpp"

int main() {
    // Draw setup
    const int scale = 200;
    const int width = 600, height = 600;
    Draw draw(width, height, scale);
    const std::string video_filename = "output.mp4";
    draw.setupVideoWriter(video_filename, 30);

    // start and goal information
    std::vector<Pose2D> waypoints = {
        {1.0, -1.0, 0.0},
        {-1.0, -1.0, 0.0},
        {-0.75, 0.25, M_PI / 4},
        {-0.5, 0.5, M_PI / 4},
        {0.25, 0.25, M_PI / 4},
        {1.0, 1.0, M_PI / 2}};

    // parameters
    int mode = 2; // 0: Reeds-Shepp tracking, 1: rotate-translate-rotate, 2: guideless AGV
    int path_tracking_mode = 1; // 0: P control, 1: Pure pursuit
    double dt = 0.1;
    double position_threshold = 0.05;
    double angle_threshold = 0.05;

    // state
    Pose2D robot = waypoints[0];
    size_t target_index = 0;
    int rtr_state = 0;  // 0: rotate to face goal, 1: move forward, 2: rotate to goal orientation
    int frame_count = 0;

    // waypoints creation
    std::vector<Pose2D> path;
    if (mode == 0) {
        path = generateReedsSheppPathFromWaypoints(waypoints);
    }
    path = waypoints;

    while (true) {
        Velocity cmd;
        bool is_finished = false;
        if (mode == 0) {
            is_finished = computeCommandForReedsShepp(target_index, position_threshold, angle_threshold,
                                                      waypoints, path, robot, dt, path_tracking_mode, cmd);
        } else if (mode == 1) {
            is_finished = computeCommandForRTR(rtr_state, target_index, position_threshold, angle_threshold,
                                               waypoints, path, robot, dt, path_tracking_mode, cmd);
        } else if (mode == 2) {
            is_finished = computeCommandForGuidelessAGV(rtr_state, target_index, position_threshold, angle_threshold,
                                                        waypoints, path, robot, dt, path_tracking_mode, cmd);
        }
        updatePose(robot, cmd, dt);
        
        draw.draw(mode, robot, path);
        frame_count++;

        if (cv::waitKey(30) == 27 || is_finished) break;
    }

    // --- Finalize ---
    draw.releaseVideo();
    std::cout << "Reached goal and aligned!" << std::endl;
    std::cout << "Simulated motion time: " << frame_count * dt << " seconds" << std::endl;
    cv::waitKey(0);
    return 0;
}