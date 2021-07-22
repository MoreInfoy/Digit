//
// Created by nimapng on 7/1/21.
//

#include "TaskSpaceControl/Task/AngularMomentumTask.h"

using namespace TSC;

TSC::AngularMomentumTask::AngularMomentumTask(RobotWrapper &robot, string name) : Task(robot, name) {
    _Q.setZero();
    _Kp.setZero();
    _ref.setZero();
    _ref_dot.setZero();
}

void AngularMomentumTask::update() {
    int u_dims = robot().nv() + 3 * robot().nc() + robot().ncf();
    Mat S(robot().nv(), u_dims);
    S.setZero();
    S.leftCols(robot().nv()).setIdentity();
    ConstMatRef J_am = _robot.momentumJacobia().bottomRows(3) * S;
    Vec3 mdot_des = _Kp * (_ref - _robot.momentumJacobia().bottomRows(3) * robot().qvel()) + _ref_dot -
                    robot().momentumTimeVariation().tail(3);
    _H = J_am.transpose() * _Q * J_am;
    _g = -J_am.transpose() * _Q * mdot_des;
}


ConstMatRef AngularMomentumTask::H() {
    return ConstMatRef(_H);
}

ConstVecRef AngularMomentumTask::g() {
    return ConstVecRef(_g);
}

Mat3Ref AngularMomentumTask::Kp() {
    return Mat3Ref(_Kp);
}

Vec3Ref AngularMomentumTask::ref() {
    return Vec3Ref(_ref);
}

Vec3Ref AngularMomentumTask::ref_dot() {
    return Vec3Ref(_ref_dot);
}

Mat3Ref AngularMomentumTask::weightMatrix() {
    return Mat3Ref(_Q);
}