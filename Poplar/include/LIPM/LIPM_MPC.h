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
    Scalar mass = 0.0;
    Mat3 inertia = Mat3::Identity();
    /*Scalar _px = 0.10;
    Scalar _nx = -0.06;
    Scalar _py = 0.04;
    Scalar _ny = -0.04;*/
    Vec4 Qx = Vec4::Ones();
    Vec2 Qu = 1e-3 * Vec2::Ones();
};

class LIPM_MPC {
public:
    LIPM_MPC();

    VecRef x0();

    void setDesiredVel(Vec2 vel);

    void updateZMP_constraints(ConstMatRef C, ConstVecRef c_lb, ConstVecRef c_ub);

    void updateContactPoints(const vector<Vec3>& points);

    void run();

    void forceDistribute();

    ConstVecRef optimalTraj();

    ConstVecRef optimalTrajDot();

    void setParameters(const LIPM_Parameters &param);

    const LIPM_Parameters &parameters();

private:

    void setup();

    Mat At, Bt;
    vector<Vec3> _contactPoints;

    LIPM_Parameters _param;
    Vec _x0, _xRef, _xOptimal, _uOptimal, _xdotOptimal;
    Vec2 _v_des;
    bool _updatedZMPConstraints;

    Mat _C, _C1, Sx, Su, Par;
    Vec _c_lb, _c_ub, _c_lb1, _c_ub1;

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
