//
// Created by nimpng on 7/10/21.
//

#include "TaskSpaceControl/Constraints/ClosedChainsConstraints.h"

using namespace TSC;

TSC::ClosedChainsConstraints::ClosedChainsConstraints(RobotWrapper &robot, string name) :
        LinearConstraints(robot, name, true) {

}

void ClosedChainsConstraints::update() {
    if (robot().ncf() > 0) {
        int input_dims = robot().nv() + 3 * robot().nc() + robot().ncf();
        Mat S;
        S.resize(robot().nv(), input_dims);
        S.setZero();
        S.leftCols(robot().nv()).setIdentity();

        Mat Par(robot().constraintForceJacobia().cols() + 1, robot().constraintForceJacobia().rows());
        Par << robot().constraintForceJacobia().transpose(), -robot().connectPointBiasAcc().transpose();
        FullPivLU<Mat> rank_check(Par);
        Mat ParN = rank_check.image(Par).transpose();
        Mat Jcf = ParN.leftCols(robot().constraintForceJacobia().cols());

        /*cout << "constraintForceJacobia: \n" << robot().constraintForceJacobia() << endl;
        cout << "constraintForceJacobia new: \n" << Jcf << endl;*/

        _C.noalias() = Jcf * S;
        _c_ub = ParN.rightCols(1);
        _c_lb = _c_ub;
    } else {
        _C = Mat::Zero(0, robot().nv() + 3 * robot().nc() + robot().ncf());
        _c_ub = Vec(0);
        _c_lb = Vec(0);
    }
}

ConstMatRef ClosedChainsConstraints::C() {
    return TSC::ConstMatRef(_C);
}

ConstVecRef ClosedChainsConstraints::c_lb() {
    return TSC::ConstVecRef(_c_lb);
}

ConstVecRef ClosedChainsConstraints::c_ub() {
    return TSC::ConstVecRef(_c_ub);
}
