//
// Created by nimpng on 7/25/21.
//

#ifndef POPLARDIGIT_FOOTPLANNER_H
#define POPLARDIGIT_FOOTPLANNER_H

#include "Planner.h"
#include "Trajectory/FootSwingTrajectory.h"

class FootPlanner : public Planner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FootPlanner();

    virtual void
    plan(size_t iter, const RobotState &state, RobotWrapper &robot, const GaitData &gaitData, Tasks &tasks);

    enum class CoordinateAxis {
        X, Y, Z
    };

private:
    Mat3 coordinateRotation(CoordinateAxis axis, double theta);

    FootSwingTrajectory lfTraj, rfTraj;
    string base, lf, rf;
    Scalar swingHeight;
    bool firstSwing[2];
    Vec3 pHipBody[2];
    Vec3 lf_shift, rf_shift;
};


#endif //POPLARDIGIT_FOOTPLANNER_H
