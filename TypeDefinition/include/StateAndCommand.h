//
// Created by nimpng on 6/15/21.
//

#ifndef POPLARDIGIT_STATEANDCOMMAND_H
#define POPLARDIGIT_STATEANDCOMMAND_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "Configuration.h"
#include "EigenTypes.h"

using namespace Eigen;

struct JointsState {
    Vector<RealNum, ROBOT_NU> qpos;
    Vector<RealNum, ROBOT_NU> qvel;
};

struct FloatingBaseState {
    Poplar::Vec3 pos, vel, omega; // vel in world frame, omega in local frame;
    Quaternion<RealNum> quat;
};

struct RobotState {
    FloatingBaseState floatingBaseState;
    JointsState jointsState;
};

struct Reference {
    // TODO:
};

struct JointsCmd {
    // TODO:
};

struct UserCmd {
    Vector<RealNum, ROBOT_NU> tau;
};

#endif //POPLARDIGIT_STATEANDCOMMAND_H
