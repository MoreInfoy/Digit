//
// Created by nimapng on 6/29/21.
//

#include "TaskSpaceControl/Constraints/QaccBound.h"

using namespace TSC;

TSC::QaccBound::QaccBound(RobotWrapper &robot, string name) : LinearConstraints(robot, name) {
    _lb.resize(robot.nv());
    _lb.fill(-1e3);
    _ub.resize(robot.nv());
    _ub.fill(1e3);
}

void QaccBound::update() {
    _C.resize(robot().nv(), robot().nv() + 3 * robot().nc() + robot().ncf());
    _C.leftCols(robot().nv()).setIdentity();
    assert((_ub - _lb).all() > 0);
    _c_lb = _lb;
    _c_ub = _ub;
}

ConstMatRef QaccBound::C() {
    return TSC::ConstMatRef(_C);
}

ConstVecRef QaccBound::c_lb() {
    return TSC::ConstVecRef(_c_lb);
}

ConstVecRef QaccBound::c_ub() {
    return TSC::ConstVecRef(_c_ub);
}

VecRef QaccBound::lb() {
    return TSC::VecRef(_lb);
}

VecRef QaccBound::ub() {
    return TSC::VecRef(_ub);
}

