//
// Created by nimapng on 7/1/21.
//

#include "TaskSpaceControl/Task/JointsNominalTask.h"

using namespace TSC;

JointsNominalTask::JointsNominalTask(RobotWrapper &robot, string name) : Task(robot, name) {
    _q_n.resize(robot.na());
    _q_n.setZero();
    _Kp.resize(robot.na(), robot.na());
    _Kp.setZero();
    _Kd.resize(robot.na(), robot.na());
    _Kd.setZero();
    _Q.resize(robot.na(), robot.na());
    _Q.setZero();
}

void JointsNominalTask::update() {
    int u_dims = robot().nv() + 3 * robot().nc() + robot().ncf();
    Mat S(robot().na(), u_dims);
    S.setZero();
    S.middleCols(robot().nv() - robot().na(), robot().na()).setIdentity();
    Vec qa_acc_des = _Kp * (_q_n - robot().qpos().tail(robot().na())) - _Kd * robot().qvel().tail(robot().na());
    _H.noalias() = S.transpose() * _Q * S;
    _g.noalias() = -S.transpose() * _Q * qa_acc_des;
    /* cout << "------------JointsNominalTask: H ----------------\n"
         << _H << endl;
    cout << "------------JointsNominalTask: g ----------------\n"
         << _g.transpose() << endl; */
}

ConstMatRef JointsNominalTask::H() {
    return ConstMatRef(_H);
}

ConstVecRef JointsNominalTask::g() {
    return ConstVecRef(_g);
}

MatRef JointsNominalTask::Kp() {
    return MatRef(_Kp);
}

MatRef JointsNominalTask::Kd() {
    return MatRef(_Kd);
}

MatRef JointsNominalTask::weightMatrix() {
    return MatRef(_Q);
}

VecRef JointsNominalTask::norminalPosition() {
    return VecRef(_q_n);
}
