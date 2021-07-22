//
// Created by nimapng on 6/29/21.
//

#include "TaskSpaceControl/Constraints/ActuatorLimit.h"

using namespace TSC;

ActuatorLimit::ActuatorLimit(RobotWrapper &robot, string name) : LinearConstraints(robot, name, false) {
    _lb = -robot.actuatorsEffortLimit();
    _ub = robot.actuatorsEffortLimit();
}

void ActuatorLimit::update() {
    if (_C.rows() != _robot.na() || _C.cols() != (_robot.nv() + 3 * _robot.nc() + _robot.ncf())) {
        _C.resize(_robot.na(), _robot.nv() + 3 * _robot.nc() + _robot.ncf());
    }
    ConstMatRef Ma = _robot.M().bottomRows(_robot.na());
    ConstVecRef ba = _robot.nonLinearEffects().tail(_robot.na());
    ConstMatRef Jca = _robot.contactJacobia().rightCols(_robot.na());
    if (_robot.ncf() > 0) {
        ConstMatRef Ka = _robot.constraintForceJacobia().rightCols(_robot.na());
        _C << Ma, -Jca.transpose(), -Ka.transpose();
    } else {
        _C << Ma, -Jca.transpose();
    }
    /*cout << "tau lb: " << _lb.transpose() << endl;
    cout << "tau ub: " << _ub.transpose() << endl;*/
#ifdef DAMPING_TERM
    _c_lb = _lb - ba + _robot.jointsSpringForce() + _robot.actuatorsDampingForce();
    _c_ub = _ub - ba + _robot.jointsSpringForce() + _robot.actuatorsDampingForce();
#else
    _c_lb = _lb - ba + _robot.jointsSpringForce();
    _c_ub = _ub - ba + _robot.jointsSpringForce();
#endif
}

ConstMatRef ActuatorLimit::C() {
    return TSC::ConstMatRef(_C);
}

ConstVecRef ActuatorLimit::c_lb() {
    return TSC::ConstVecRef(_c_lb);
}

ConstVecRef ActuatorLimit::c_ub() {
    return TSC::ConstVecRef(_c_ub);
}

VecRef ActuatorLimit::lb() {
    if (_lb.size() != robot().na()) {
        _lb.resize(robot().na());
    }
    return VecRef(_lb);
}

VecRef ActuatorLimit::ub() {
    if (_ub.size() != robot().na()) {
        _ub.resize(robot().na());
    }
    return VecRef(_ub);
}
