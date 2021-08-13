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
    assert(_forceRef.size() >= robot().nc() * 3);
    if (_forceRef.size() != 3 * robot().nc()) {
        Vec forceRef = _forceRef;
        _forceRef.resize(robot().nc() * 3);
        int index = 0;
        for (int i = 0; i < robot().contactMask().size(); i++) {
            if (robot().contactMask()(i) != 0) {
                _forceRef.segment(index * 3, 3) = forceRef.segment(i * 3, 3);
                index++;
            }
        }
    }

    if (robot().nc() > 0) {
        Mat S(3 * robot().nc(), u_dims);
        S.setZero();
        S.middleCols(robot().nv(), 3 * robot().nc()).setIdentity();
        _Q.resize(3 * robot().nc(), 3 * robot().nc());
        _Q.setZero();
        for (int i = 0; i < robot().nc(); i++) {
            _Q.block<3, 3>(3 * i, 3 * i) = _Qf;
        }
        _H.noalias() = S.transpose() * _Q * S;
        _g.noalias() = -S.transpose() * _Q * _forceRef;
    } else {
        _H.noalias() = Mat::Zero(u_dims, u_dims);
        _g.noalias() = Vec::Zero(u_dims);
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