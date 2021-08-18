//
// Created by nimpng on 8/14/21.
//

#include "LIPM/LIPM_MPC.h"

#include <eigen3/unsupported/Eigen/MatrixFunctions>

LIPM_MPC::LIPM_MPC() {
    _x0.noalias() = Vec4::Zero();
    _v_des.setZero();
    _updatedZMPConstraints = false;
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

    At = Mat::Zero(4, 4);
    Bt = Mat::Zero(4, 2);

    At << 0, 1, 0, 0,
            w, 0, 0, 0,
            0, 0, 0, 1,
            0, 0, w, 0;
    Bt << 0, 0,
            -w, 0,
            0, 0,
            0, -w;

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
    }

    /* for capture point */
    Par = Mat::Zero(2, 4);
    Par << 1, 1 / sqrt(w), 0, 0,
            0, 0, 1, 1 / sqrt(w);

}

VecRef LIPM_MPC::x0() {
    return Poplar::VecRef(_x0);
}

void LIPM_MPC::setDesiredVel(Vec2 vel) {
    _v_des = vel;
}

void LIPM_MPC::run() {
    /* check whether zmp constraints has been updated */
    assert(_updatedZMPConstraints);

    /* capture point constraints */
    _C = Mat::Zero(_C1.rows() + 2, _C1.cols());
    _c_lb = Vec::Zero(_c_lb1.size() + 2);
    _c_ub = Vec::Zero(_c_ub1.size() + 2);
    _C << _C1, _C1.bottomRightCorner<2, 2>() * Par * Su.bottomRows(4);
    _c_lb << _c_lb1, _c_lb1.tail(2) - _C1.bottomRightCorner<2, 2>() * Par * Sx.bottomRows(4) * _x0;
    _c_ub << _c_ub1, _c_ub1.tail(2) - _C1.bottomRightCorner<2, 2>() * Par * Sx.bottomRows(4) * _x0;
    /*_C = _C1;
    _c_lb = _c_lb1;
    _c_ub = _c_ub1;*/

    /* generate desired com trajectory */
    _xRef.resize(4 * _param.mpc_horizons);
    for (int i = 0; i < _param.mpc_horizons; i++) {
        _xRef.segment(4 * i, 4) << _x0(0) + Scalar(i) * _param.mpc_dt * _v_des(0), _v_des(0),
                _x0(1) + Scalar(i) * _param.mpc_dt * _v_des(1), _v_des(1);
    }

    /* QP problem setup */
    _Q.resize(4 * _param.mpc_horizons, 4 * _param.mpc_horizons);
    _Q.setZero();
    _Q.diagonal() = _param.Qx.replicate(_param.mpc_horizons, 1);
    _R.resize(2 * _param.mpc_horizons, 2 * _param.mpc_horizons);
    _R.setZero();
    _R.diagonal() = _param.Qu.replicate(_param.mpc_horizons, 1);
    /*cout << "_Q: " << endl << _Q << endl << "_R: " << endl << _R << endl;*/

    // formulate QP
    _H.noalias() = _R + Su.transpose() * _Q * Su;
    _g.noalias() = Su.transpose() * _Q * Sx * _x0 - Su.transpose() * _Q * _xRef;

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
    _updatedZMPConstraints = false;
}

ConstVecRef LIPM_MPC::optimalTraj() {
    return ConstVecRef(_xOptimal);
}

void LIPM_MPC::updateZMP_constraints(ConstMatRef C, ConstVecRef c_lb, ConstVecRef c_ub) {
    assert(C.cols() == 2 * _param.mpc_horizons && C.rows() == c_lb.size() && c_lb.size() == c_ub.size());

    /* input constraints */
    _C1.noalias() = C;
    _c_lb1 = c_lb;
    _c_ub1 = c_ub;
    _updatedZMPConstraints = true;
}

ConstVecRef LIPM_MPC::optimalTrajDot() {
    return ConstVecRef(_xdotOptimal);
}

void LIPM_MPC::updateContactPoints(const vector<Vec3> &points) {
    _contactPoints = points;
}

