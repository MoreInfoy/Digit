//
// Created by nimpng on 6/15/21.
//

#ifndef POPLARDIGIT_STATEANDCOMMAND_H
#define POPLARDIGIT_STATEANDCOMMAND_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "PoplarConfig.h"
#include "Configuration.h"

using namespace Poplar;

struct JointsState {
    Matrix<Scalar, ROBOT_NJ, 1> qpos;
    Matrix<Scalar, ROBOT_NJ, 1> qvel;
};

struct FloatingBaseState {
    Poplar::Vec3 pos, vel, omega, acc; // vel in world frame, omega in local frame;
    Quaternion<Scalar> quat;
};

struct RobotState {
    FloatingBaseState floatingBaseState;
    JointsState jointsState;
};

struct GaitData {
    void zero() {
        swingTime.setZero();
        stanceTime.setZero();
        swingTimeRemain.setZero();
        stanceTimeRemain.setZero();
    }

    Vec2 swingTime;
    Vec2 stanceTime;
    Vec2 swingTimeRemain;
    Vec2 stanceTimeRemain;
    MatInt contactTable;
};

struct LinkTask {
    string link_name;
    Vec3 pos = Vec3::Zero();
    Vec3 vel = Vec3::Zero();
    Vec3 acc = Vec3::Zero();
    Mat3 R_wb = Mat3::Identity();
    Vec3 omega = Vec3::Zero();
    Vec3 omega_dot = Vec3::Zero();
};

struct Tasks {
    // pos in world frame, vel,acc in local frame
    LinkTask floatingBaseTask;

    LinkTask leftFootTask;

    LinkTask rightFootTask;

    LinkTask leftFootContact;

    LinkTask rightFootContact;

    Vec forceTask;

    Vec3 desired_vel = Vec3::Zero();
};

struct JointsCmd {
    // TODO:
    Matrix<Scalar, ROBOT_NU, 1> tau_ff;
};

struct UserCmd {
    Matrix<Scalar, ROBOT_NU, 1> tau;
};

#endif //POPLARDIGIT_STATEANDCOMMAND_H
