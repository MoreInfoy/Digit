//
// Created by nimpng on 8/14/21.
//

#include "LIPM/LIPM_MPC.h"

#include <eigen3/unsupported/Eigen/MatrixFunctions>

LIPM_MPC::LIPM_MPC() {
    _x0.noalias() = Vec4::Zero();
    _updatedTerminalZMPConstraints = false;
    _updatedZMPRef = false;
    At = Mat::Zero(4, 4);
    Bt = Mat::Zero(4, 2);
    Ct = Mat::Zero(2, 4);
    Dt = Mat::Zero(2, 2);

    At << 0, 1, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 1,
            0, 0, 0, 0;
    Bt << 0, 0,
            1, 0,
            0, 0,
            0, 1;
    Ct << 1, 0, 0, 0,
            0, 0, 1, 0;
    setup();
}

void LIPM_MPC::setParameters(const LIPM_Parameters &param) {
    _param = param;
    setup();
}

const LIPM_Parameters &LIPM_MPC::parameters() {
    return ref(_param);
}

void LIPM_MPC::setup() {

    Scalar w = _param.gravity / _param.height;
    Dt << -1.0 / w, 0,
            0, -1.0 / w;

    Mat e;
    e.noalias() = Mat::Zero(6, 6);
    e.topRows(4) << At, Bt;
    e.noalias() = _param.mpc_dt * e;
    Mat E(6, 6);
    E.noalias() = e.exp();
    Mat Ak(4, 4);
    Mat Bk(4, 2);
    Ak.noalias() = E.topLeftCorner<4, 4>();
    Bk.noalias() = E.topRightCorner<4, 2>();
    /*cout << "Ak: " << endl << Ak << endl << "Bk: " << endl << Bk << endl;*/

    _xOptimal = Vec::Zero(4 * _param.mpc_horizons);

    Sx.noalias() = Mat::Zero(4 * _param.mpc_horizons, 4);
    Su.noalias() = Mat::Zero(4 * _param.mpc_horizons, 2 * _param.mpc_horizons);
    Rx.noalias() = Mat::Zero(2 * _param.mpc_horizons, 4 * _param.mpc_horizons);
    Ru.noalias() = Mat::Zero(2 * _param.mpc_horizons, 2 * _param.mpc_horizons);
    Mat P0_exp;
    for (size_t k = 0; k < _param.mpc_horizons; k++) {
        if (k == 0) {
            Sx.middleRows(k * 4, 4).noalias() = Ak;
            Su.topLeftCorner(4, 2).noalias() = Bk;
        } else {
            Sx.middleRows(k * 4, 4).noalias() = Ak * Sx.middleRows((k - 1) * 4, 4);
            Su.block(k * 4, 0, 4, 2 * k).noalias() =
                    Ak * Su.block((k - 1) * 4, 0, 4, 2 * k);
            Su.block(k * 4, 2 * k, 4, 2).noalias() = Bk;
        }
        Rx.block<2, 4>(2 * k, 4 * k) = Ct;
        Ru.block<2, 2>(2 * k, 2 * k) = Dt;
    }

    /* for capture point */
    Par = Mat::Zero(2, 4);
    Par << 1, 1 / sqrt(w), 0, 0,
            0, 0, 1, 1 / sqrt(w);

}

VecRef LIPM_MPC::x0() {
    return Poplar::VecRef(_x0);
}

void LIPM_MPC::setZMPRef(ConstVecRef zmpRef) {
    assert(zmpRef.size() == _param.mpc_horizons * 2);
    _updatedZMPRef = true;
    _zmpRef = zmpRef;
}

void LIPM_MPC::run() {
    /* check whether zmp constraints has been updated */
    assert(_updatedTerminalZMPConstraints);
    assert(_updatedZMPRef);

    /* capture point constraints */
    _C.noalias() = _Cz * Par * Su.bottomRows(4);
    _c_lb.noalias() = _cz_lb - _Cz * Par * Sx.bottomRows(4) * _x0;
    _c_ub.noalias() = _cz_ub - _Cz * Par * Sx.bottomRows(4) * _x0;

    /* QP problem setup */
    _Q.resize(_param.Qx.size() * _param.mpc_horizons, _param.Qx.size() * _param.mpc_horizons);
    _Q.setZero();
    _Q.diagonal() = _param.Qx.replicate(_param.mpc_horizons, 1);
    _R.resize(_param.Qu.size() * _param.mpc_horizons, _param.Qu.size() * _param.mpc_horizons);
    _R.setZero();
    _R.diagonal() = _param.Qu.replicate(_param.mpc_horizons, 1);
    /*cout << "_Q: " << endl << _Q << endl << "_R: " << endl << _R << endl;*/

    // formulate QP
    Mat Mx, Mu;
    Mx.noalias() = Rx * Sx;
    Mu.noalias() = Rx * Su + Ru;
    _H.noalias() = _R + Mu.transpose() * _Q * Mu;
    _g.noalias() = Mu.transpose() * _Q * Mx * _x0 - Mu.transpose() * _Q * _zmpRef;

#ifdef USE_QPOASES
    solver = qpOASES::QProblem(_C.cols(), _C.rows());
    qpOASES::int_t nWSR = 1000;
    qpOASES::Options opt;
    opt.setToMPC();
    opt.enableEqualities = qpOASES::BT_TRUE;
    opt.printLevel = qpOASES::PL_NONE;
    solver.setOptions(opt);
    solver.init(_H.data(), _g.data(), _C.data(), nullptr, nullptr, _c_lb.data(), _c_ub.data(), nWSR);
    _uOptimal.resize(_C.cols());
    if (solver.isSolved()) {
        solver.getPrimalSolution(_uOptimal.data());
    } else {
        throw std::runtime_error("SRGB_MPC::solve() qp solver failed");
    }
#else
    eiquadprog_solver.reset(_C.cols(), 0, _C.rows() * 2);
    Mat Cin(_C.rows() * 2, 2 * _param.mpc_horizons);
    Cin << _C, -_C;
    Vec cin(_C.rows() * 2);
    cin << -_c_lb, _c_ub;
    solver_state = eiquadprog_solver.solve_quadprog(_H, _g, Mat(0, _C.cols()), Vec(0), Cin, cin,
                                                    _uOptimal);
    printf("solver state: %d\n", solver_state);
    if (solver_state != eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL) {
        //      throw runtime_error("TaskSpaceControl::solve() qp failed, related data has been saved in qp_failed.txt");
        std::cerr << "LIPM_MPC::solve() qp failed" << endl;
    }
#endif

    _xOptimal.noalias() = Sx * _x0 + Su * _uOptimal;

    _xdotOptimal.resize(4 * _param.mpc_horizons);
    _xdotOptimal.head(4) = At * _x0 + Bt * _uOptimal.head(2);
    for (int i = 1; i < _param.mpc_horizons; i++) {
        _xdotOptimal.segment(4 * i, 4) = At * _xOptimal.segment(i * 4 - 4, 4) + Bt * _uOptimal.segment(2 * i, 2);
    }

    _updatedTerminalZMPConstraints = false;
    _updatedZMPRef = false;
}

ConstVecRef LIPM_MPC::optimalTraj() {
    return ConstVecRef(_xOptimal);
}

void LIPM_MPC::updateTerminalZMPConstraints(ConstMatRef C, ConstVecRef c_lb, ConstVecRef c_ub) {
    assert(C.cols() == 2 && C.rows() == c_lb.size() && c_lb.size() == c_ub.size());

    /* input constraints */
    _Cz.noalias() = C;
    _cz_lb = c_lb;
    _cz_ub = c_ub;
    _updatedTerminalZMPConstraints = true;
}

ConstVecRef LIPM_MPC::optimalTrajDot() {
    return ConstVecRef(_xdotOptimal);
}

void LIPM_MPC::updateContactPoints(const vector<Vec3> &points) {
    _contactPoints = points;
}

