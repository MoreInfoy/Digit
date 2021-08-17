//
// Created by nimpng on 7/4/21.
//

#ifndef SRGB_MPC_SRGB_MPC_H
#define SRGB_MPC_SRGB_MPC_H

#include "Trajectory/TrajectoryInterpolation.h"
#include "qpOASES.hpp"
#include "eiquadprog/eiquadprog-fast.hpp"
#include "PoplarConfig.h"

//#define USE_QPOASES

using namespace Poplar;


namespace SRGB_MPC {
    struct SixDimsPose_Trajectory {
        TrajectoryInterpolation x;
        TrajectoryInterpolation y;
        TrajectoryInterpolation z;
        TrajectoryInterpolation roll;
        TrajectoryInterpolation pitch;
        TrajectoryInterpolation yaw;
    };

    enum class CoordinateAxis {
        X, Y, Z
    };

    class SRGB_MPC_IMPL {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit SRGB_MPC_IMPL(size_t horizons, Scalar dt, size_t ns_contact);

        void setMassAndInertia(Scalar mass, Mat3Ref inertia);

        void setCurrentState(ConstVecRef x0);

        void setExternalWrench(ConstVec6Ref ext_wrench);

        void setExternalWrench(ConstVec6Ref ext_wrench, size_t k);

        void setVelocityCmd(Vec6 vel_des);

        void setContactTable(ConstMatIntRef &contactTable);

        void setContactPointPos(vector<Vec3> contactPointPos);

        void setFrictionCoefficient(Scalar mu);

        void setMaxForce(Scalar fmax);

        void setWeight(ConstVecRef Qx, ConstVecRef Qu);

        void setDesiredTrajectory(const SixDimsPose_Trajectory &traj);

        void setDesiredDiscreteTrajectory(ConstVecRef traj);

        void solve(Scalar t_now);

        const SixDimsPose_Trajectory &getContinuousOptimizedTrajectory();

        ConstVecRef getDiscreteOptimizedTrajectory();

        ConstVecRef getOptimalContactForce();

        ConstVecRef getCurrentDesiredContactForce();

        ConstVecRef getCurrentDesiredActiveContactForce();

        ConstVecRef getXDot();

        size_t horizons();

        Scalar dt();

    private:
        void computeSxSu();

        void computeAtBt(size_t i_horizons);

        void computeAtBtAndBiasTraj(size_t i_horizons);

        Mat3 coordinateRotation(CoordinateAxis axis, Scalar theta);

        Mat3 rpyToRotMat(Vec3Ref v);

        size_t _horizons, _ns_contact;
        Scalar _dt, _gravity, _mu, _fmax;
        Scalar _mass;
        Mat3 _inertia;
        SixDimsPose_Trajectory _continuousOptimizedTraj, _desiredTraj;
        bool _setDesiredTraj, _setDesiredDiscreteTraj;
        Vec _desiredDiscreteTraj, _desiredDiscreteTraj_bias;
        Vec _discreteOptimizedTraj, _optimalContactForce;
        Vec _force_des;
        MatInt _contactTable;
        vector<Vec3> _contactPointPos;
        Vec6 _vel_des;
        Vec13 _x0;
        vector<Vec6> _ext_wrench;
        Mat3 T_inv, R_wb, Iworld, Iw_inv;
        Mat_R _At, _Bt, _Ak, _Bk, Sx, Su;
        Mat_R _C;
        Vec _ub, _lb;
        Vec _xDot;

        Vec13 _Qx;
        Vec3 _Qf;
        Mat_R _Q, _R;
        Mat_R _H, _g;
        size_t _n_contact;

#ifdef USE_QPOASES
        qpOASES::QProblem solver;
#else
        eiquadprog::solvers::EiquadprogFast eiquadprog_solver;
        eiquadprog::solvers::EiquadprogFast_status solver_state;
#endif
    };

}


#endif //SRGB_MPC_SRGB_MPC_H