//
// Created by nimpng on 8/7/21.
//

#include "TaskSpaceControl/Task/ForceTask.h"

using namespace TSC;

ForceTask::ForceTask(RobotWrapper &robot, string name) : Task(robot, name) {
    _Qf.setZero();
}

void ForceTask::update() {
    int u_dims = robot().nv() + 3 * robot().nc() + robot().ncf();
    assert(_forceRef.size() == 3 * robot().nc());

    if (robot().nc() > 0) {
        Mat S(3 * robot().nc(), u_dims);
        S.setZero();
        S.middleCols(robot().nv(), 3 * robot().nc()).setIdentity();
        _Q.resize(3 * robot().nc(), 3 * robot().nc());
        _Q.setZero();
        for (int i = 0; i < robot().nc(); i++) {
            _Q.block<3, 3>(3 * i, 3 * i) = _Qf;
        }
        _H = S.transpose() * _Q * S;
        _g = -S.transpose() * _Q * _forceRef;
    } else {
        _H = Mat::Zero(u_dims, u_dims);
        _g = Vec::Zero(u_dims);
    }
}

ConstMatRef ForceTask::H() {
    return ConstMatRef(_H);
}

ConstVecRef ForceTask::g() {
    return ConstVecRef(_g);
}

Mat3Ref ForceTask::weightMatrix() {
    return Mat3Ref(_Qf);
}

void ForceTask::setForceRef(ConstVecRef forceRef) {
    _forceRef = forceRef;
}