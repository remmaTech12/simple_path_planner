#pragma once
#include "util.hpp"
#include <opencv2/opencv.hpp>

class Draw {
public:
    Draw(int width, int height, int scale)
       : width_(width), height_(height), scale_(scale),
         canvas_(height, width, CV_8UC3, cv::Scalar(255, 255, 255)) {

        offsetX_ = width_ / 2;
        offsetY_ = height_ / 2;
    }

    cv::Point2i toPixel(const Pose2D& pose) const {
        return cv::Point2i(
            static_cast<int>(pose.x * scale_) + offsetX_,
            static_cast<int>(pose.y * scale_) + offsetY_
        );
    }

    void drawRobotPolygon(const Pose2D& pose) const {
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

            int x = static_cast<int>(pose.x * scale_ + x_rot) + offsetX_;
            int y = static_cast<int>(pose.y * scale_ + y_rot) + offsetY_;

            shape_world.emplace_back(x, y);
        }

        const cv::Point* pts = shape_world.data();
        int npts = static_cast<int>(shape_world.size());
        cv::polylines(canvas_, &pts, &npts, 1, true, cv::Scalar(0, 0, 0), 2);
        cv::fillPoly(canvas_, &pts, &npts, 1, cv::Scalar(0, 0, 0));
    }

    void drawTrajectory() const {
        for (size_t i = 1; i < trajectory_.size(); ++i) {
            cv::line(canvas_, trajectory_[i - 1], trajectory_[i], cv::Scalar(0, 0, 255), 2);  // Red
        }
    }

    void drawPlannedPath(const std::vector<Pose2D>& path) const {
        for (size_t i = 1; i < path.size(); ++i) {
            cv::line(canvas_, toPixel(path[i - 1]), toPixel(path[i]), cv::Scalar(0, 255, 0), 2);  // Green
        }
    }
    
    void clearCanvas() {
        canvas_ = cv::Scalar(255, 255, 255);
    }

    void imshow() const {
        cv::Mat flipped;
        cv::flip(canvas_, flipped, 0);
        cv::imshow("Robot Simulation", flipped);
    }

    void setupVideoWriter(const std::string& filename, int fps) {
        video_.open(filename, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps, cv::Size(width_, height_));
        if (!video_.isOpened()) {
            throw std::runtime_error("Could not open the output video file.");
        }
    }

    void writeVideoFrame() {
        video_.write(canvas_);
    }

    void releaseVideo() {
        video_.release();
    }

    void draw(int mode, const Pose2D& robot, const std::vector<Pose2D>& path) {
        clearCanvas();

        // Draw planned path (for mode 0)
        if (mode == 0) {
            drawPlannedPath(path);
        }
        trajectory_.push_back(toPixel(robot));
        drawTrajectory();
        drawRobotPolygon(robot);

        // Show window and write to video
        imshow();
        writeVideoFrame();
    }


private:
    int scale_;
    int width_;
    int height_;
    int offsetX_;
    int offsetY_;

    std::vector<cv::Point> trajectory_;

    cv::Mat canvas_;
    cv::VideoWriter video_;
};