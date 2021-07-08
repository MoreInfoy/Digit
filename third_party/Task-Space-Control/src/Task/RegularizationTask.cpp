//
// Created by nimpng on 6/10/21.
//

#include "Task/RegularizationTask.h"

//#define REGULARIZE_TORQUE

using namespace TSC;

TSC::RegularizationTask::RegularizationTask(RobotWrapper &robot, string name) : Task(robot, name),
                                                                                _Q_isUpdated(false) {
    _Q_qacc.setZero();
    _Q_f.setZero();
}

void RegularizationTask::update() {
    int input_dims = robot().nv() + 3 * robot().nc();
    if (_Q_qacc.rows() != robot().nv() || _Q_qacc.cols() != robot().nv()) {
        throw runtime_error(name() + " task  weight matrix _Q_qacc dimension is wrong");
    }
    if (!_Q_isUpdated || _Q.rows() != input_dims || _Q.cols() != input_dims) {
        _Q.resize(input_dims, input_dims);
        _Q.setZero();
        _Q.topLeftCorner(robot().nv(), robot().nv()) = _Q_qacc;
        for (int i = 0; i < robot().nc(); i++) {
            _Q.block<3, 3>(robot().nv() + 3 * i, robot().nv() + 3 * i) =_Q_f;
        }
        _Q_isUpdated = true;
        _g = Vec::Zero(input_dims);
    }

#ifdef REGULARIZE_TORQUE
    Mat A;
    A.resize(_robot.na(), _robot.nv() + 3 * _robot.nc());
    ConstMatRef Ma = _robot.M().bottomRows(_robot.na());
    ConstVecRef ba = _robot.nonLinearEffects().tail(_robot.na());

    ConstMatRef Jca = robot().contactJacobia().rightCols(_robot.na());
    A << Ma, -Jca.transpose();
    _H = _Q + 1e-12 * A.transpose() * A;
    _g = 1e-12 * A.transpose() * ba;
#else
    _H = _Q;
    _g.setZero();
#endif
}

ConstMatRef RegularizationTask::H() {
    return TSC::ConstMatRef(_H);
}

ConstVecRef RegularizationTask::g() {
    return TSC::ConstVecRef(_g);
}

Mat3 &RegularizationTask::forceWeight() {
    _Q_isUpdated = false;
    _Q_f.setZero();
    return _Q_f;
}

MatRef RegularizationTask::qaccWeight() {
    if (_Q_qacc.rows() != _robot.nv() || _Q_qacc.cols() != _robot.nv()) {
        _Q_qacc.resize(_robot.nv(), _robot.nv());
    }
    _Q_qacc.setZero();
    _Q_isUpdated = false;
    return TSC::MatRef(_Q_qacc);
}
