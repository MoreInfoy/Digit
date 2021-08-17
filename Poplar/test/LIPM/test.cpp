//
// Created by nimpng on 8/14/21.
//

#include "LIPM/LIPM_MPC.h"

#include <iostream>

using namespace std;

int main() {
    LIPM_Parameters param;
    param.Qx << 1, 10, 1, 10;
    param.Qu << 1e-5, 1e-5;
    param.mpc_dt = 0.05;
    param.mpc_horizons = 100;
    param.height = 0.892442;
    LIPM_MPC lipmMpc;
    lipmMpc.setParamters(param);

    Mat C = Mat::Zero(param.mpc_horizons * 2, 2 * param.mpc_horizons);
    Vec c_lb = Vec::Zero(param.mpc_horizons * 2);
    Vec c_ub = Vec::Zero(param.mpc_horizons * 2);
    for (int i = 0; i < param.mpc_horizons; i++) {
        C.block<2, 2>(i * 2, i * 2).setIdentity();
        c_lb.segment(i * 2, 2) << -0.5, -0.5;
        c_ub.segment(i * 2, 2) << 0.5, 0.5;
    }
    lipmMpc.updateZMP_constraints(C, c_lb, c_ub);

    lipmMpc.setDesiredVel(Vec2(0, 0));

    Vec4 x0;
    x0 << 0.0, 0.5, 0, 0;
    lipmMpc.x0() = x0;
    lipmMpc.run();
    cout << "xopt: " << lipmMpc.optimalTraj().transpose() << endl;
    return 0;
}