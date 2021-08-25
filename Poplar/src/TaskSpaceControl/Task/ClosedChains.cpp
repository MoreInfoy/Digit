//
// Created by nimapng on 8/23/21.
//

#include "TaskSpaceControl/Task/ClosedChains.h"

using namespace TSC;

ClosedChains::ClosedChains(RobotWrapper &robot, string name) : Task(robot, name) {
}

void ClosedChains::update() {
    int input_dims = robot().nv() + 3 * robot().nc() + robot().ncf();
    if (robot().ncf() > 0) {
        int input_dims = robot().nv() + 3 * robot().nc() + robot().ncf();
        Mat S, A;
        S.resize(robot().nv(), input_dims);
        S.setZero();
        S.leftCols(robot().nv()).setIdentity();

        _acc_fb = -robot().connectPointBiasAcc() - 0.1 * robot().constraintForceJacobia() * robot().qvel();

        Mat _Q = 1e4 * Mat::Identity(robot().ncf(), robot().ncf());
        A.noalias() = robot().constraintForceJacobia() * S;
        _H.noalias() = A.transpose() * _Q * A;
        _g.noalias() = -A.transpose() * _Q * _acc_fb;
        //     std::cout << "acc_fb:\n" << acc_fb.transpose() << std::endl;
    } else {
        _H = Mat::Zero(input_dims, input_dims);
        _g = Vec(input_dims);
    }
}

ConstMatRef ClosedChains::H() {
    return ConstMatRef(_H);
}

ConstVecRef ClosedChains::g() {
    return ConstVecRef(_g);
}