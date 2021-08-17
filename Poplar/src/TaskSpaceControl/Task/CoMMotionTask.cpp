//
// Created by nimapng on 6/11/21.
//

#include "TaskSpaceControl/Task/CoMMotionTask.h"

using namespace TSC;

CoMMotionTask::CoMMotionTask(RobotWrapper &robot, string name) : Task(robot, name) {
    _posRef.setZero();
    _velRef.setZero();
    _accRef.setZero();
    _Kp.setZero();
    _Kd.setZero();
    _Q.setIdentity();
}

void CoMMotionTask::update() {
    int input_dims = robot().nv() + 3 * robot().nc()+ robot().ncf();
    Mat S, A;
    S.resize(robot().nv(), input_dims);
    S.setZero();
    S.leftCols(robot().nv()).setIdentity();

    acc_fb = _Kp * (_posRef - robot().CoM_pos()) +
             _Kd * (_velRef - robot().CoM_vel()) + _accRef;

    A.noalias() = robot().Jacobia_CoM() * S;
    Vec a = acc_fb - robot().CoM_acc();
    _H.noalias() = A.transpose() * _Q * A;
    _g.noalias() = -A.transpose() * _Q * a;
//     std::cout << "acc_fb:\n" << acc_fb.transpose() << std::endl;
}

ConstMatRef CoMMotionTask::H() {
    return ConstMatRef(_H);
}

ConstVecRef CoMMotionTask::g() {
    return ConstVecRef(_g);
}

Mat3Ref CoMMotionTask::Kp() {
    return Mat3Ref(_Kp);
}

Mat3Ref CoMMotionTask::Kd() {
    return Mat3Ref(_Kd);
}

Vec3Ref CoMMotionTask::posRef() {
    return Vec3Ref(_posRef);
}

Vec3Ref CoMMotionTask::velRef() {
    return Vec3Ref(_velRef);
}

Vec3Ref CoMMotionTask::accRef() {
    return Vec3Ref(_accRef);
}

Mat3Ref CoMMotionTask::weightMatrix() {
    return Mat3Ref(_Q);
}