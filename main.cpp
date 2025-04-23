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
        // zig-zag
        /*
        {0.82, -0.525, 0.0},
        {0.215, -0.96, 0.0},
        {-0.405, -0.63, 0.0},
        {0.085, -0.265, 0.0},
        {-0.68, -0.205, 0.0},
        {-1.03, -0.56, 0.0},
        {-1.1, 0.05, 0.0},
        {-0.64, 0.285, 0.0},
        {-0.745, 0.935, 0.0},
        {-0.155, 0.775, 0.0},
        {0.155, 0.35, 0.0},
        {0.56, -0.12, 0.0},
        {1.055, 0.2, 0.0},
        {1.145, 0.76, 0.0},
        {0.665, 1.105, 0.0},
        {1.095, -0.58, 0.0},
        {0.855, -0.835, 0.0},
        */

        // spiral
        {0.295, -1.08, 0.0},
        {-0.225, -1.015, 0.0},
        {-0.59, -0.805, 0.0},
        {-0.895, -0.47, 0.0},
        {-1.07, -0.05, 0.0},
        {-1.085, 0.435, 0.0},
        {-0.87, 0.82, 0.0},
        {-0.5, 1.125, 0.0},
        {0.17, 1.205, 0.0},
        {0.685, 1.045, 0.0},
        {1.065, 0.8, 0.0},
        {1.28, 0.435, 0.0},
        {1.315, 0.02, 0.0},
        {1.095, -0.33, 0.0},
        {0.91, -0.47, 0.0},
        {0.375, -0.66, 0.0},
        {0.0, -0.425, 0.0},
        {-0.135, -0.035, 0.0},
        {-0.03, 0.36, 0.0},
        {0.325, 0.515, 0.0},
        {0.945, 0.265, 0.0},
        {1.05, -0.01, 0.0},
        {0.945, -0.285, 0.0},
        {0.615, -0.415, 0.0},
        {0.495, -0.225, 0.0},
        {0.59, 0.005, 0.0},

        /*
        // S-curve
        {1.0, -1.0, 0.0},
        {-1.0, -1.0, 0.0},
        {-0.9, -0.5, M_PI / 4},
        {-0.5, -0.1, M_PI / 4},
        {0.0, 0.0, M_PI / 4},
        {0.5, 0.1, M_PI / 4},
        {0.9, 0.5, M_PI / 4},
        {1.0, 1.0, M_PI / 2},

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