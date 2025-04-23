#include "include/util.hpp"
#include "include/draw.hpp"
#include "include/motion.hpp"
#include <csignal> // For signal handling

Draw* global_draw_ptr = nullptr; // Pointer for signal handler

// Signal handler to release video on Ctrl+C
void handle_sigint(int) {
    if (global_draw_ptr) {
        global_draw_ptr->releaseVideo();
        std::cout << "\nVideo saved on SIGINT (Ctrl+C)." << std::endl;
    }
    std::_Exit(0); // Immediate exit
}

int main() {
    // Draw setup
    const int scale = 200;
    const int width = 600, height = 600;
    Draw draw(width, height, scale);
    global_draw_ptr = &draw; // Register global pointer for signal handler

    const std::string video_filename = "output.mp4";
    draw.setupVideoWriter(video_filename, 30);

    // Register SIGINT handler
    std::signal(SIGINT, handle_sigint);

    // Start and goal information
    std::vector<Pose2D> waypoints = {
        /*
        {1.05, -0.94, 0.0},
        {-1.035, -0.96, 0.0},
        {-1.05, -0.405, 0.0},
        {-0.545, 0.15, 0.0},
        {0.28, 0.22, 0.0},
        {1.05, 0.625, 0.0},
        {1.125, 1.225, 0.0},
        */

        // S-curve
        {1.0, -1.0, 0.0},
        {-1.0, -1.0, 0.0},
        {-0.9, -0.5, M_PI / 4},
        {-0.5, -0.1, M_PI / 4},
        {0.0, 0.0, M_PI / 4},
        {0.5, 0.1, M_PI / 4},
        {0.9, 0.5, M_PI / 4},
        {1.0, 1.0, M_PI / 2},

        /*
        // N-curve
        {-1.0, -1.0, 0.0},
        {-1.0, 1.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, -1.0, 0.0},
        {1.0, 1.0, M_PI / 2},
        */
    };

    // Parameters
    int mode = 2; // 0: Reeds-Shepp tracking, 1: rotate-translate-rotate, 2: guideless AGV
    int path_tracking_mode = 1; // 0: P control, 1: Pure pursuit
    double dt = 0.1;
    double position_threshold = 0.05;
    double angle_threshold = 0.05;

    // State
    Pose2D robot = waypoints[0];
    size_t target_index = 0;
    int rtr_state = 0;
    int frame_count = 0;

    // Waypoints creation
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
                                               waypoints, path, robot, dt, path_tracking_mode, cmd, false);
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