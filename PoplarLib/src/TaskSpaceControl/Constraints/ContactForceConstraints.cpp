//
// Created by nimapng on 6/11/21.
//

#include "TaskSpaceControl/Constraints/ContactForceConstraints.h"

using namespace TSC;

ContactForceConstraints::ContactForceConstraints(RobotWrapper &robot, string name) : LinearConstraints(robot, name, false) {
    _n = Vec3::UnitZ();
    _mu = 0.5;
    _min = 0;
    _max = 500;
}

void ContactForceConstraints::update() {
    int input_dims = robot().nv() + 3 * robot().nc() + robot().ncf();
    _C.resize(robot().nc() * 5, input_dims);
    _c_ub.resize(robot().nc() * 5);
    _c_lb.resize(robot().nc() * 5);

    Vec3 t1, t2;
    const int n_in = 4 * 1 + 1;
    const int n_var = 3 * 1;
    Mat B = Mat::Zero(n_in, n_var);
    Vec lb = -1e10 * Vec::Ones(n_in);
    Vec ub = Vec::Zero(n_in);
    t1 = _n.cross(Vec3::UnitX());
    if (t1.norm() < 1e-5)
        t1 = _n.cross(Vec3::UnitY());
    t2 = _n.cross(t1);
    t1.normalize();
    t2.normalize();

    B.block<1, 3>(0, 0) = (-t1 - _mu * _n).transpose();
    B.block<1, 3>(1, 0) = (t1 - _mu * _n).transpose();
    B.block<1, 3>(2, 0) = (-t2 - _mu * _n).transpose();
    B.block<1, 3>(3, 0) = (t2 - _mu * _n).transpose();

    B.block<1, 3>(n_in - 1, 0) = _n.transpose();
    ub(n_in - 1) = _max;
    lb(n_in - 1) = _min;
    auto &mask = robot().contactMask();
    Mat S;
    S.resize(3, input_dims);
    for (int index = 0; index < robot().nc(); index++) {
        S.setZero();
        S.middleCols(index * 3 + robot().nv(), 3).setIdentity();
        _C.middleRows(index * 5, 5).noalias() = B * S;
        _c_ub.segment(index * 5, 5) = ub;
        _c_lb.segment(index * 5, 5) = lb;
    }
}

ConstMatRef ContactForceConstraints::C() {
    return TSC::ConstMatRef(_C);
}

ConstVecRef ContactForceConstraints::c_lb() {
    return TSC::ConstVecRef(_c_lb);
}

ConstVecRef ContactForceConstraints::c_ub() {
    return TSC::ConstVecRef(_c_ub);
}

Vec3Ref ContactForceConstraints::normal() {
    return TSC::Vec3Ref(_n);
}

RealNum &ContactForceConstraints::mu() {
    return ref(_mu);
}

RealNum &ContactForceConstraints::min() {
    return ref(_min);
}

RealNum &ContactForceConstraints::max() {
    return ref(_max);
}