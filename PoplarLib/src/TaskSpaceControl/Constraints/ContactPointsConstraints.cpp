//
// Created by nimapng on 6/11/21.
//

#include "TaskSpaceControl/Constraints/ContactPointsConstraints.h"

using namespace TSC;

TSC::ContactPointsConstraints::ContactPointsConstraints(RobotWrapper &robot, string name) : LinearConstraints(
        robot, name, true) {

}

void ContactPointsConstraints::update() {
    int input_dims = robot().nv() + 3 * robot().nc() + robot().ncf();
    Mat S;
    S.resize(robot().nv(), input_dims);
    S.setZero();
    S.leftCols(robot().nv()).setIdentity();

    Mat Par(robot().contactJacobia().cols() + 1, robot().contactJacobia().rows());
    Par << robot().contactJacobia().transpose(), -robot().activeContactPointBiasAcc().transpose();
    FullPivLU<Mat> rank_check(Par);
    Mat ParN = rank_check.image(Par).transpose();
    Mat Jc = ParN.leftCols(robot().contactJacobia().cols());

    /*cout << "Jc: "<< robot().contactJacobia() << endl;
    cout << "Jc_new: "<< Jc << endl;*/

    _C = Jc * S;
    _c_ub = ParN.rightCols(1);
    _c_lb = _c_ub;
}

ConstMatRef ContactPointsConstraints::C() {
    return TSC::ConstMatRef(_C);
}

ConstVecRef ContactPointsConstraints::c_lb() {
    return TSC::ConstVecRef(_c_lb);
}

ConstVecRef ContactPointsConstraints::c_ub() {
    return TSC::ConstVecRef(_c_ub);
}
