#pragma once
#include "util.hpp"
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/config.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
namespace ob = ompl::base;

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