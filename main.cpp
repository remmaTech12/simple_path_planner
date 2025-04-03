#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

struct Pose2D {
    double x;
    double y;
    double theta; // heading in radians
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

// 描画用の座標変換（メートル→ピクセル）
cv::Point2i toPixel(const Pose2D& pose, int scale, int offsetX, int offsetY) {
    return cv::Point2i(
        static_cast<int>(pose.x * scale) + offsetX,
        static_cast<int>(pose.y * scale) + offsetY
    );
}

int main() {
    std::vector<Pose2D> path = {
        {0, 0, 0},       // start
        {-1.0, 0.5, 0},   // intermediate
        {1.0, 1.0, 0}    // goal
    };

    Pose2D robot = {0, 0, 0};
    double dt = 0.1;
    const double threshold = 0.05;
    size_t target_index = 1;

    // OpenCV ウィンドウ準備
    const int scale = 200; // 1m = 200px
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

        // 軌跡を保存
        trajectory.push_back(toPixel(robot, scale, offsetX, offsetY));

        // 画面描画
        canvas = cv::Scalar(255, 255, 255); // 画面クリア

        // 軌跡描画
        for (size_t i = 1; i < trajectory.size(); ++i) {
            cv::line(canvas, trajectory[i - 1], trajectory[i], cv::Scalar(0, 0, 255), 2);
        }

        // ロボット描画
        cv::circle(canvas, toPixel(robot, scale, offsetX, offsetY), 5, cv::Scalar(0, 0, 0), -1);

        // ターゲット描画
        cv::circle(canvas, toPixel(target, scale, offsetX, offsetY), 5, cv::Scalar(0, 255, 0), -1);

        // 表示
        cv::imshow("Robot Path", canvas);
        int key = cv::waitKey(50);
        if (key == 27) break; // ESCで終了
    }

    std::cout << "Reached goal!" << std::endl;
    cv::waitKey(0);
    return 0;
}

