//
// Created by nimpng on 8/14/21.
//

#include "LIPM/LIPM_MPC.h"

#include <eigen3/unsupported/Eigen/MatrixFunctions>

LIPM_MPC::LIPM_MPC() {
    _updatedTerminalZMPConstraints = false;
    _updatedZMPRef = false;
    ar = 6;
    bc = 2;
    cr = 2;
    At = Mat::Zero(ar, ar);
    Bt = Mat::Zero(ar, bc);
    Ct = Mat::Zero(cr, ar);

    _x0.noalias() = Vec::Zero(ar);

    At << 0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1,
            0, 0, 0, 0, 0, 0;
    Bt << 0, 0,
            0, 0,
            1, 0,
            0, 0,
            0, 0,
            0, 1;

    setup();
}

LIPM_Parameters &LIPM_MPC::parameters() {
    return ref(_param);
}

void LIPM_MPC::setup() {

    Scalar w = _param.gravity / _param.height;
    Ct << 1, 0, -1.0 / w, 0, 0, 0,
            0, 0, 0, 1, 0, -1.0 / w;

    Mat e;
    e.noalias() = Mat::Zero(ar + bc, ar + bc);
    e.topRows(ar) << At, Bt;
    e.noalias() = _param.mpc_dt * e;
    Mat E(ar + bc, ar + bc);
    E.noalias() = e.exp();
    Mat Ak(ar, ar);
    Mat Bk(ar, bc);
    Ak.noalias() = E.topLeftCorner(ar, ar);
    Bk.noalias() = E.topRightCorner(ar, bc);
//    cout << "Ak: " << endl << Ak << endl << "Bk: " << endl << Bk << endl;

    _xOptimal = Vec::Zero(ar * _param.mpc_horizons);

    Sx.noalias() = Mat::Zero(ar * _param.mpc_horizons, ar);
    Su.noalias() = Mat::Zero(ar * _param.mpc_horizons, bc * _param.mpc_horizons);
    Rx.noalias() = Mat::Zero(cr * _param.mpc_horizons, ar * _param.mpc_horizons);
    Mat P0_exp;
    for (size_t k = 0; k < _param.mpc_horizons; k++) {
        if (k == 0) {
            Sx.middleRows(k * ar, ar).noalias() = Ak;
            Su.topLeftCorner(ar, bc).noalias() = Bk;
        } else {
            Sx.middleRows(k * ar, ar).noalias() = Ak * Sx.middleRows((k - 1) * ar, ar);
            Su.block(k * ar, 0, ar, bc * k).noalias() =
                    Ak * Su.block((k - 1) * ar, 0, ar, bc * k);
            Su.block(k * ar, bc * k, ar, bc).noalias() = Bk;
        }
        Rx.block(cr * k, ar * k, cr, ar) = Ct;
    }

    /* for capture point */
    Par = Mat::Zero(cr, ar);
    Par << 1, 1 / sqrt(w), 0, 0, 0, 0,
            0, 0, 0, 1, 1 / sqrt(w), 0;
}

VecRef LIPM_MPC::x0() {
    return Poplar::VecRef(_x0);
}

void LIPM_MPC::setZMPRef(ConstVecRef zmpRef) {
    assert(zmpRef.size() == _param.mpc_horizons * cr);
    _updatedZMPRef = true;
    _zmpRef = zmpRef;
}

void LIPM_MPC::run() {
    /* check whether zmp constraints has been updated */
    assert(_updatedTerminalZMPConstraints);
    assert(_updatedZMPRef);
    setup();

    /* capture point constraints */
    _C.noalias() = _Cz * Par * Su.bottomRows(ar);
    _c_lb.noalias() = _cz_lb - _Cz * Par * Sx.bottomRows(ar) * _x0;
    _c_ub.noalias() = _cz_ub - _Cz * Par * Sx.bottomRows(ar) * _x0;

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
    Mu.noalias() = Rx * Su;
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
    // printf("solver state: %d\n", solver_state);
    if (solver_state != eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL) {
        //      throw runtime_error("TaskSpaceControl::solve() qp failed, related data has been saved in qp_failed.txt");
        std::cerr << "LIPM_MPC::solve() qp failed" << endl;
    }
#endif

    _xOptimal.noalias() = Sx * _x0 + Su * _uOptimal;

    _updatedTerminalZMPConstraints = false;
    _updatedZMPRef = false;
}

ConstVecRef LIPM_MPC::optimalTraj() {
    return ConstVecRef(_xOptimal);
}

void LIPM_MPC::updateTerminalZMPConstraints(ConstMatRef C, ConstVecRef c_lb, ConstVecRef c_ub) {
    assert(C.cols() == cr && C.rows() == c_lb.size() && c_lb.size() == c_ub.size());

    /* input constraints */
    _Cz.noalias() = C;
    _cz_lb = c_lb;
    _cz_ub = c_ub;
    _updatedTerminalZMPConstraints = true;
}

void LIPM_MPC::setContactPoints(const vector<Vec3> &points) {
    _contactPoints = points;
}

Vec LIPM_MPC::forceDistribute(Poplar::Index ith_horizon, Vec3 pos, Vec3 linear_vel, Vec3 angular_momentum,
                              ConstVecXiRef mask) {
    assert(mask.size() == _contactPoints.size());
    Poplar::Index n_dims = mask.cwiseEqual(1).cast<Poplar::Index>().sum();
    if (n_dims == 0) {
        return Vec::Zero(_contactPoints.size() * 3);
    }
    Poplar::Index nc = _contactPoints.size();

    /* feedback */
    Mat3 K = 16 * Mat3::Identity();
    Mat3 D = 4 * Mat3::Identity();
    Vec3 pos_des, vel_des, acc_des, AgDot_des, r;
    pos_des << _xOptimal(ith_horizon * 6),
            _xOptimal(3 + ith_horizon * 6),
            _param.height;
    vel_des << _xOptimal(1 + ith_horizon * 6), _xOptimal(4 + ith_horizon * 6), 0;
    acc_des << _xOptimal(2 + ith_horizon * 6), _xOptimal(5 + ith_horizon * 6), 0;
    acc_des += K * (pos_des - pos) + D * (vel_des - linear_vel);
    AgDot_des = -10.0 * angular_momentum;

    /* QP */
    Mat_R Ce(3, 3 * n_dims);
    Mat_R J_A(3, 3 * n_dims);

    Poplar::Index k = 0;
    for (int i = 0; i < nc; i++) {
        if (mask(i) == 1) {
            Ce.middleCols(3 * k, 3).setIdentity();
            r = _contactPoints[i] - pos;
            J_A.middleCols(3 * k, 3) << 0., -r(2), r(1),
                    r(2), 0., -r(0),
                    -r(1), r(0), 0.;
            k++;
        }
    }
    Vec3 ce = _param.mass * (acc_des + Vec3(0, 0, _param.gravity));

    Mat_R Cin = Mat::Zero(n_dims * 5, n_dims * 3);
    Vec cin_ub = Vec::Zero(n_dims * 5);
    Vec cin_lb = Vec::Zero(n_dims * 5);

    Vec3 t1, t2;
    const int n_in = 4 * 1 + 1;
    const int n_var = 3 * 1;
    Mat B = Mat::Zero(n_in, n_var);
    Vec lb = -1e10 * Vec::Ones(n_in);
    Vec ub = Vec::Zero(n_in);
    t1 = _param.normal_dir.cross(Vec3::UnitX());
    if (t1.norm() < 1e-5)
        t1 = _param.normal_dir.cross(Vec3::UnitY());
    t2 = _param.normal_dir.cross(t1);
    t1.normalize();
    t2.normalize();

    B.block<1, 3>(0, 0) = (-t1 - _param.mu * _param.normal_dir).transpose();
    B.block<1, 3>(1, 0) = (t1 - _param.mu * _param.normal_dir).transpose();
    B.block<1, 3>(2, 0) = (-t2 - _param.mu * _param.normal_dir).transpose();
    B.block<1, 3>(3, 0) = (t2 - _param.mu * _param.normal_dir).transpose();

    B.block<1, 3>(n_in - 1, 0) = _param.normal_dir.transpose();
    ub(n_in - 1) = _param.max_force;
    lb(n_in - 1) = 0;
    Mat S = Mat::Zero(3, 3 * n_dims);
    for (int index = 0; index < n_dims; index++) {
        S.setZero();
        S.middleCols(index * 3, 3).setIdentity();
        Cin.middleRows(index * 5, 5).noalias() = B * S;
        cin_ub.segment(index * 5, 5) = ub;
        cin_lb.segment(index * 5, 5) = lb;
    }

    // angular momentum weight matrix
    Mat3 Q = Mat3::Zero();
    Mat R = 1e-3 * Mat::Identity(3 * n_dims, 3 * n_dims);
    Q.diagonal() << 20, 20, 10;
    Mat3 W = 1e2 * Q;
    Mat_R H;
    Vec g;
    H.noalias() = J_A.transpose() * Q * J_A + R + Ce.transpose() * W * Ce;
    g.noalias() = -J_A.transpose() * Q * AgDot_des - Ce.transpose() * W * ce;
//    H.noalias() = J_A.transpose() * Q * J_A + R;
//    g.noalias() = -J_A.transpose() * Q * AgDot_des;

#ifdef USE_QPOASES
    Mat_R Cin_R(3 + 5 * n_dims, 3 * n_dims);
    Vec clb(3 + 5 * n_dims);
    Vec cub(3 + 5 * n_dims);
    Cin_R << Ce, Cin;
    clb << ce, cin_lb;
    cub << ce, cin_ub;

    /*    Mat Par(Cin.cols() + 2, Cin.rows());
    Par << Cin.transpose(), clb.transpose(), cub.transpose();
    FullPivLU<Mat> rank_check(Par);
    Mat ParN = rank_check.image(Par).transpose();
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> _Cin = ParN.leftCols(Cin.cols());
    Vec _clb = -ParN.col(ParN.cols() - 2);
    Vec _cub = ParN.col(ParN.cols() - 1);*/
    qpOASES::QProblem solver_fd = qpOASES::QProblem(3 * n_dims, 5 * n_dims);
    qpOASES::int_t nWSR = 1000;
    qpOASES::Options opt;
    opt.setToMPC();
    opt.enableEqualities = qpOASES::BT_TRUE;
    opt.printLevel = qpOASES::PL_DEBUG_ITER;
    solver_fd.setOptions(opt);
    solver_fd.init(H.data(), g.data(), Cin.data(), nullptr, nullptr, cin_lb.data(), cin_ub.data(), nWSR);
    Vec force_optimal(n_dims * 3);
    if (solver_fd.isSolved()) {
        solver_fd.getPrimalSolution(force_optimal.data());
    } else {
        throw std::runtime_error("LIPM_MPC::forceDistribute() qp solver failed");
    }
#else
    eiquadprog::solvers::EiquadprogFast eiquadprog_solver_fd;
//    eiquadprog_solver_fd.reset(n_dims * 3, 0, 10 * n_dims);
    Mat CI(10 * n_dims, 3 * n_dims);
    CI << Cin, -Cin;
    Vec cI(10 * n_dims);
    cI << -cin_lb, cin_ub;
    Vec force_optimal(n_dims * 3);
//    solver_state = eiquadprog_solver_fd.solve_quadprog(H, g, Ce, -ce, CI, cI, force_optimal);
    solver_state = eiquadprog_solver_fd.solve_quadprog(H, g, Mat::Zero(0, 3 * n_dims), Vec::Zero(0), CI, cI,
                                                       force_optimal);
    // printf("solver state: %d\n", solver_state);
    if (solver_state != eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL) {
        throw runtime_error("LIPM_MPC::forceDistribute() qp failed, related data has been saved in qp_failed.txt");
        std::cerr << "LIPM_MPC::forceDistribute() qp failed" << endl;
    }
#endif

    Vec force_all = Vec::Zero(_contactPoints.size() * 3);
    k = 0;
    for (int i = 0; i < nc; i++) {
        if (mask(i) == 1) {
            force_all.segment(3 * i, 3) = force_optimal.segment(3 * k, 3);
            k++;
        }
    }

    /*cout << "------------------J_A------------------" << endl
         << J_A << endl;
    cout << "------------------angular_momentum------------------" << endl
         << angular_momentum.transpose() << endl;

    cout << "------------------force_optimal------------------" << endl
         << force_optimal.transpose() << endl;

    cout << "------------------Ce------------------" << endl
         << Ce << endl;
    cout << "------------------ce------------------" << endl
         << ce.transpose() << endl;

    cout << "------------------Cin------------------" << endl
         << Cin << endl;
    cout << "------------------c_lb------------------" << endl
         << cin_lb.transpose() << endl;
    cout << "------------------c_ub------------------" << endl
         << cin_ub.transpose() << endl;
    getchar();*/

    return force_all;
}
