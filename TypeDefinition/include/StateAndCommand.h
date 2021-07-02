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
    Matrix<RealNum, ROBOT_NU, 1> qpos;
    Matrix<RealNum, ROBOT_NU, 1> qvel;
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
    Matrix<RealNum, ROBOT_NU, 1> tau_ff;
};

struct UserCmd {
    Matrix<RealNum, ROBOT_NU, 1> tau;
};

#endif //POPLARDIGIT_STATEANDCOMMAND_H
