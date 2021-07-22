//
// Created by nimapng on 6/11/21.
//

#include "TaskSpaceControl/Task/CoMMotionTask.h"

using namespace TSC;

TSC::CoMMotionTask::CoMMotionTask(RobotWrapper &robot, string name) : Task(robot, name) {
    _posRef.setZero();
    _velRef.setZero();
    _accRef.setZero();
    _Kp.setZero();
    _Kd.setZero();
    _Q.setIdentity();
}

void TSC::CoMMotionTask::update() {
    int input_dims = robot().nv() + 3 * robot().nc()+ robot().ncf();
    Mat S;
    S.resize(robot().nv(), input_dims);
    S.setZero();
    S.leftCols(robot().nv()).setIdentity();

    acc_fb = _Kp * (_posRef - robot().CoM_pos()) +
             _Kd * (_velRef - robot().CoM_vel()) + _accRef;

    Mat A = robot().Jacobia_CoM() * S;
    Vec a = acc_fb - robot().CoM_acc();
    _H = A.transpose() * _Q * A;
    _g = -A.transpose() * _Q * a;
//     std::cout << "acc_fb:\n" << acc_fb.transpose() << std::endl;
}

TSC::ConstMatRef TSC::CoMMotionTask::H() {
    return TSC::ConstMatRef(_H);
}

TSC::ConstVecRef TSC::CoMMotionTask::g() {
    return TSC::ConstMatRef(_g);
}

TSC::Mat3Ref TSC::CoMMotionTask::Kp() {
    return TSC::Mat3Ref(_Kp);
}

TSC::Mat3Ref TSC::CoMMotionTask::Kd() {
    return TSC::Mat3Ref(_Kd);
}

TSC::Vec3Ref TSC::CoMMotionTask::posRef() {
    return TSC::Vec3Ref(_posRef);
}

TSC::Vec3Ref TSC::CoMMotionTask::velRef() {
    return TSC::Vec3Ref(_velRef);
}

TSC::Vec3Ref TSC::CoMMotionTask::accRef() {
    return TSC::Vec3Ref(_accRef);
}

TSC::Mat3Ref TSC::CoMMotionTask::weightMatrix() {
    return TSC::Mat3Ref(_Q);
}