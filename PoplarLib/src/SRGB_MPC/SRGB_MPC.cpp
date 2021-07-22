//
// Created by nimpng on 7/4/21.
//

#include "SRGB_MPC/SRGB_MPC.h"
#include <eigen3/unsupported/Eigen/MatrixFunctions>

using namespace SRGB_MPC;

#define BIG_NUMBER 1e10

SRGB_MPC_IMPL::SRGB_MPC_IMPL(size_t horizon, RealNum dt) : _horizon(horizon), _dt(dt), _gravity(-9.81), _mu(0.4),
                                                           _fmax(500), _setDesiredTraj(false),
                                                           solver(12 * horizon, 20 * horizon) {

    _mass = 0;
    _inertia.setZero();
    _Qx.setIdentity();
    _Qf.setZero();
    _Qf.diagonal().fill(1e-3);
    _Q.resize(13 * _horizon, 13 * _horizon);
    _R.resize(12 * _horizon, 12 * _horizon);
    _contactTable = Mat::Ones(4, horizon);
    _desiredDiscreteTraj = Vec::Zero(13 * horizon);
    _vel_des.setZero();

    _At.resize(13, 13);
    _At.setZero();
    _Bt.resize(13, 12);
    _Bt.setZero();

    _C.resize(20 * _horizon, 12 * _horizon);
    _C.setZero();
    _ub.resize(20 * _horizon);
    _ub.setZero();
    _lb.resize(20 * _horizon);
    _lb.setZero();

    Sx.resize(13 * _horizon, 13);
    Sx.setZero();
    Su.resize(13 * _horizon, 12 * _horizon);
    Su.setZero();

    _optimalContactForce.resize(12 * _horizon);
    _optimalContactForce.setZero();
    _x0.setZero();
    _x0.tail(1) << _gravity;
}

void SRGB_MPC_IMPL::setCurrentState(ConstVecRef x0) {
    assert(x0.size() == 12);
    _x0 << x0, _gravity;
}

void SRGB_MPC_IMPL::setVelocityCmd(Vec6 vel_des) {
    _vel_des = vel_des;
}

void SRGB_MPC_IMPL::setContactTable(ConstMatRef &contactTable) {
    if (contactTable.rows() != 4 || contactTable.cols() != _horizon) {
        throw std::runtime_error("[SRGB_MPC_IMPL::setContactTable] contactTable size is wrong");
    }
    _contactTable = contactTable;
}

void SRGB_MPC_IMPL::setWeight(ConstVecRef Qx, ConstVecRef Qf) {
    assert(Qx.size() == 12 && Qf.size() == 12);
    _Qx.diagonal() << Qx, 0.0;
    _Qf.diagonal() << Qf;
}

void SRGB_MPC_IMPL::setDesiredTrajectory(const SixDimsPose_Trajectory &traj) {
    _desiredTraj = traj;
    _setDesiredTraj = true;
}

const SixDimsPose_Trajectory &SRGB_MPC_IMPL::getContinuousOptimizedTrajectory() {
    return _continuousOptimizedTraj;
}

ConstVecRef SRGB_MPC_IMPL::getDiscreteOptimizedTrajectory() {
    _discreteOptimizedTraj = Sx * _x0 + Su * _optimalContactForce;
    return SRGB_MPC::ConstVecRef(_discreteOptimizedTraj);
}

ConstVecRef SRGB_MPC_IMPL::getOptimalContactForce() {
    return SRGB_MPC::ConstVecRef(_optimalContactForce);
}

void SRGB_MPC_IMPL::setMassAndInertia(RealNum mass, Mat3Ref inertia) {
    _mass = mass;
    _inertia = inertia;
}

void SRGB_MPC_IMPL::solve(RealNum t_now) {
    if (_setDesiredTraj) {
        _setDesiredTraj = false;
        for (int i = 0; i < _horizon; i++) {
            _desiredDiscreteTraj.segment(i * 13, 13) << _desiredTraj.roll(t_now + RealNum(i) * _dt).data(),
                    _desiredTraj.pitch(t_now + RealNum(i) * _dt).data(),
                    _desiredTraj.yaw(t_now + RealNum(i) * _dt).data(),
                    _desiredTraj.x(t_now + RealNum(i) * _dt).data(),
                    _desiredTraj.y(t_now + RealNum(i) * _dt).data(),
                    _desiredTraj.z(t_now + RealNum(i) * _dt).data(),
                    _desiredTraj.roll(t_now + RealNum(i) * _dt).derivative(),
                    _desiredTraj.pitch(t_now + RealNum(i) * _dt).derivative(),
                    _desiredTraj.yaw(t_now + RealNum(i) * _dt).derivative(),
                    _desiredTraj.x(t_now + RealNum(i) * _dt).derivative(),
                    _desiredTraj.y(t_now + RealNum(i) * _dt).derivative(),
                    _desiredTraj.z(t_now + RealNum(i) * _dt).derivative(),
                    _gravity;
        }
    } else {
        for (int i = 0; i < _horizon; i++) {
            _desiredDiscreteTraj.segment(i * 13, 13) << _x0.head(6) + RealNum(i) * _vel_des * _dt, _vel_des, _gravity;
        }
    }

    computeSxSu();

    // set input constraints
    double mu = 1 / _mu;
    Mat Ci(5, 3);
    Ci << mu, 0, 1.,
            -mu, 0, 1.,
            0, mu, 1.,
            0, -mu, 1.,
            0, 0, 1.;

    for (int i = 0; i < _horizon * 4; i++) {
        _C.block(i * 5, i * 3, 5, 3) = Ci;
    }
    int k = 0;
    for (int i = 0; i < _horizon; i++) {
        for (int j = 0; j < 4; j++) {
            _ub(5 * k + 0) = BIG_NUMBER;
            _ub(5 * k + 1) = BIG_NUMBER;
            _ub(5 * k + 2) = BIG_NUMBER;
            _ub(5 * k + 3) = BIG_NUMBER;
            _ub(5 * k + 4) = _contactTable.col(i)(j) * _fmax;
            k++;
        }
    }
    // set weight matrix
    _Q.diagonal() = _Qx.diagonal().replicate(_horizon, 1);
    _R.diagonal() = _Qf.diagonal().replicate(_horizon, 1);

    // formulate QP
    _H.noalias() = _R + Su.transpose() * _Q * Su;
    _g.noalias() = Su.transpose() * _Q * Sx * _x0 - Su.transpose() * _Q * _desiredDiscreteTraj;

    int_t nWSR = 1000;
    solver.reset();
    Options opt;
    opt.setToMPC();
    opt.enableEqualities = BT_TRUE;
    opt.printLevel = PL_NONE;
    solver.setOptions(opt);
    solver.init(_H.data(), _g.data(), _C.data(), nullptr, nullptr, _lb.data(), _ub.data(), nWSR, nullptr,
                _optimalContactForce.data());
    if (solver.isSolved()) {
        solver.getPrimalSolution(_optimalContactForce.data());
    } else {
        throw std::runtime_error("qp solver failed");
    }
}

void SRGB_MPC_IMPL::computeSxSu() {

    for (size_t k = 0; k < _horizon; k++) {
        computeAtBt(k);
        Mat P0(_At.rows() + _Bt.cols(), _At.cols() + _Bt.cols());
        P0.setZero();
        P0.topRows(_At.rows()) << _At, _Bt;
        P0 *= _dt;
        Mat P0_exp = P0.exp();
        _Ak = P0_exp.topLeftCorner(_At.rows(), _At.cols());
        _Bk = P0_exp.topRightCorner(_Bt.rows(), _Bt.cols());

        if (k == 0) {
            Sx.middleRows(k * 13, 13).noalias() = _Ak;
            Su.topLeftCorner<13, 12>().noalias() = _Bk;
        } else {
            Sx.middleRows(k * 13, 13).noalias() = _Ak * Sx.middleRows((k - 1) * 13, 13);
            Su.block(k * 13, 0, 13, k * 12).noalias()
                    = _Ak * (Su.block((k - 1) * 13, 0, 13, k * 12));
            Su.block<13, 12>(k * 13, k * 12).noalias() = _Bk;
        }
    }

}

void SRGB_MPC_IMPL::setFrictionCoefficient(double mu) {
    _mu = mu;
}

void SRGB_MPC_IMPL::setContactPointPos(vector<Vec3> contactPointPos) {
    assert(contactPointPos.size() == 4);
    _contactPointPos = contactPointPos;
}

void SRGB_MPC_IMPL::setMaxForce(double fmax) {
    _fmax = fmax;
}

void SRGB_MPC_IMPL::computeAtBt(size_t) {
    // TODO: compute At, Bt

    double pc = cos(_x0(1));
    double yc = cos(_x0(2));
    double ys = sin(_x0(2));
    double pt = tan(_x0(1));

    Mat3 T_inv, R_wb;
    T_inv << yc / pc, ys / pc, 0,
            -ys, yc, 0,
            yc * pt, ys * pt, 1;

    R_wb = rpyToRotMat(_x0.head(3));

    _At.setZero();
    _At.block(0, 6, 3, 3) = T_inv;
    _At.block(3, 9, 3, 3).setIdentity();
    _At(11, 12) = 1.;

    Mat3 Iworld;
    Iworld = R_wb * _inertia * R_wb.transpose();
    auto Iw_inv = Iworld.inverse();

    _Bt.setZero();
    Vec3 r;
    for (int leg(0); leg < 4; leg++) {
        r = _contactPointPos[leg] - _x0.segment(3, 3);
        // std::cout << "r " << leg << ": " << r.transpose() << "\n";
        Mat3 r_skew;
        r_skew << 0., -r(2), r(1),
                r(2), 0., -r(0),
                -r(1), r(0), 0.;
        _Bt.block(6, leg * 3, 3, 3) = Iw_inv * r_skew;
        _Bt.block(9, leg * 3, 3, 3) = 1 / _mass * Mat3::Identity();
    }
}

Mat3 SRGB_MPC_IMPL::coordinateRotation(CoordinateAxis axis, double theta) {
    RealNum s = std::sin(theta);
    RealNum c = std::cos(theta);

    Mat3 R;

    if (axis == CoordinateAxis::X) {
        R << 1, 0, 0, 0, c, s, 0, -s, c;
    } else if (axis == CoordinateAxis::Y) {
        R << c, 0, -s, 0, 1, 0, s, 0, c;
    } else if (axis == CoordinateAxis::Z) {
        R << c, s, 0, -s, c, 0, 0, 0, 1;
    }

    return R;
}

Mat3 SRGB_MPC_IMPL::rpyToRotMat(Vec3Ref v) {
    static_assert(v.ColsAtCompileTime == 1 && v.RowsAtCompileTime == 3,
                  "must have 3x1 vector");
    Mat3 m = coordinateRotation(CoordinateAxis::X, v[0]) *
             coordinateRotation(CoordinateAxis::Y, v[1]) *
             coordinateRotation(CoordinateAxis::Z, v[2]);
    return m;
}
