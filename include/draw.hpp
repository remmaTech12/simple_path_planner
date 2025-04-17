#include "util.hpp"
#include <opencv2/opencv.hpp>

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