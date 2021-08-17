//
// Created by nimapng on 6/11/21.
//

#include "TaskSpaceControl/Constraints/ContactPointsConstraints.h"

using namespace TSC;

ContactPointsConstraints::ContactPointsConstraints(RobotWrapper &robot, string name) : LinearConstraints(
        robot, name, true) {

}

void ContactPointsConstraints::update() {
    int input_dims = robot().nv() + 3 * robot().nc() + robot().ncf();
    if (robot().nc() > 0) {
        Mat S;
        S.resize(robot().nv(), input_dims);
        S.setZero();
        S.leftCols(robot().nv()).setIdentity();

        Mat Par(robot().contactJacobia().cols() + 1, robot().contactJacobia().rows());
        Par << robot().contactJacobia().transpose(), -robot().activeContactPointBiasAcc().transpose();
        FullPivLU<Mat> rank_check(Par);
        Mat ParN, Jc;
        ParN.noalias() = rank_check.image(Par).transpose();
        Jc.noalias() = ParN.leftCols(robot().contactJacobia().cols());

        /*cout << "Jc: "<< robot().contactJacobia() << endl;
        cout << "Jc_new: "<< Jc << endl;*/

        _C.noalias() = Jc * S;
        _c_ub = ParN.rightCols(1);
        _c_lb = _c_ub;
    } else {
        _C.resize(0, input_dims);
        _c_ub.resize(0);
        _c_lb.resize(0);
    }
}

ConstMatRef ContactPointsConstraints::C() {
    return ConstMatRef(_C);
}

ConstVecRef ContactPointsConstraints::c_lb() {
    return ConstVecRef(_c_lb);
}

ConstVecRef ContactPointsConstraints::c_ub() {
    return ConstVecRef(_c_ub);
}
