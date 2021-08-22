//
// Created by nimpng on 8/14/21.
//

#ifndef POPLARDIGIT_LIPM_MPC_H
#define POPLARDIGIT_LIPM_MPC_H

#include "PoplarConfig.h"
#include "qpOASES.hpp"
#include "eiquadprog/eiquadprog-fast.hpp"

//#define USE_QPOASES

using namespace Poplar;

struct LIPM_Parameters {
    Scalar mpc_dt = 0.05;
    Poplar::Index mpc_horizons = 100;
    Scalar height = 0.8;
    Scalar gravity = 9.8;
    Scalar mu = 0.5;
    Vec3 normal_dir = Vec3::UnitZ();
    Scalar max_force = 500;
    Scalar mass = 0.0;
    Mat3 inertia = Mat3::Identity();
    Scalar px = 0.10;
    Scalar nx = 0.12;
    Scalar py = 0.04;
    Scalar ny = 0.04;
    Vec2 Qx = Vec2::Ones();
    Vec2 Qu = 1e-3 * Vec2::Ones();
};

class LIPM_MPC {
public:
    LIPM_MPC();

    VecRef x0();

    void setZMPRef(ConstVecRef zmpRef);

    void updateTerminalZMPConstraints(ConstMatRef C, ConstVecRef c_lb, ConstVecRef c_ub);

    void setContactPoints(const vector<Vec3> &points);

    void run();

    Vec forceDistribute(Poplar::Index ith_horizon, Vec3 pos, Vec3 linear_vel, Vec3 angular_momentum, ConstVecXiRef mask);

    ConstVecRef optimalTraj();

    LIPM_Parameters &parameters();

private:

    void setup();

    Mat At, Bt, Ct;
    Poplar::Index ar, bc, cr;
    vector<Vec3> _contactPoints;

    LIPM_Parameters _param;
    Vec _x0, _zmpRef, _xOptimal, _uOptimal;
    bool _updatedTerminalZMPConstraints, _updatedZMPRef;

    Mat _C, _Cz, Sx, Su, Rx, Par;
    Vec _c_lb, _c_ub, _cz_lb, _cz_ub;

    Mat_R _Q, _R;
    Mat_R _H, _g;

#ifdef USE_QPOASES
    qpOASES::QProblem solver;
#else
    eiquadprog::solvers::EiquadprogFast eiquadprog_solver;
    eiquadprog::solvers::EiquadprogFast_status solver_state;
#endif
};


#endif //POPLARDIGIT_LIPM_H
