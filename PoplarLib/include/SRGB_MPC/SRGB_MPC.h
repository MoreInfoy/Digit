//
// Created by nimpng on 7/4/21.
//

#ifndef SRGB_MPC_SRGB_MPC_H
#define SRGB_MPC_SRGB_MPC_H

#include "Trajectory/TrajectoryInterpolation.h"
#include "qpOASES.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace qpOASES;

namespace SRGB_MPC {

    using Vec = typename Eigen::Matrix<RealNum, Eigen::Dynamic, 1>;

    using VecInt = typename Eigen::Matrix<int, Eigen::Dynamic, 1>;

    using Vec3 = typename Eigen::Matrix<RealNum, 3, 1>;

    using Vec6 = typename Eigen::Matrix<RealNum, 6, 1>;

    using Vec12 = typename Eigen::Matrix<RealNum, 12, 1>;

    using Vec13 = typename Eigen::Matrix<RealNum, 13, 1>;

    using Mat = typename Eigen::Matrix<RealNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    using Mat6 = typename Eigen::Matrix<RealNum, 6, 6, Eigen::RowMajor>;

    using Mat12 = typename Eigen::Matrix<RealNum, 12, 12, Eigen::RowMajor>;

    using Mat13 = typename Eigen::Matrix<RealNum, 13, 13, Eigen::RowMajor>;

    using Mat3 = typename Eigen::Matrix<RealNum, 3, 3, Eigen::RowMajor>;

    typedef const Eigen::Ref<const Vec> ConstVecRef;

    typedef const Eigen::Ref<const VecInt> ConstVecIntRef;

    typedef const Eigen::Ref<const Mat> ConstMatRef;

    typedef Eigen::Ref<Vec> VecRef;

    typedef Eigen::Ref<VecInt> VecIntRef;

    typedef Eigen::Ref<Vec6> Vec6Ref;

    typedef Eigen::Ref<Vec3> Vec3Ref;

    typedef Eigen::Ref<Mat6> Mat6Ref;

    typedef Eigen::Ref<Mat3> Mat3Ref;

    typedef Eigen::Ref<Mat> MatRef;

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

        explicit SRGB_MPC_IMPL(size_t horizon, RealNum dt);

        void setMassAndInertia(RealNum mass, Mat3Ref inertia);

        void setCurrentState(ConstVecRef x0);

        void setVelocityCmd(Vec6 vel_des);

        void setContactTable(ConstMatRef &contactTable);

        void setContactPointPos(vector<Vec3> contactPointPos);

        void setFrictionCoefficient(RealNum mu);

        void setMaxForce(RealNum fmax);

        void setWeight(ConstVecRef Qx, ConstVecRef Qu);

        void setDesiredTrajectory(const SixDimsPose_Trajectory &traj);

        void solve(RealNum t_now);

        const SixDimsPose_Trajectory &getContinuousOptimizedTrajectory();

        ConstVecRef getDiscreteOptimizedTrajectory();

        ConstVecRef getOptimalContactForce();

    private:
        void computeSxSu();

        void computeAtBt(size_t i_horizon);

        Mat3 coordinateRotation(CoordinateAxis axis, RealNum theta);

        Mat3 rpyToRotMat(Vec3Ref v);

        size_t _horizon;
        RealNum _dt, _gravity, _mu, _fmax;
        RealNum _mass;
        Mat3 _inertia;
        SixDimsPose_Trajectory _continuousOptimizedTraj, _desiredTraj;
        bool _setDesiredTraj;
        Vec _desiredDiscreteTraj;
        Vec _discreteOptimizedTraj, _optimalContactForce;
        Mat _contactTable;
        vector<Vec3> _contactPointPos;
        Vec6 _vel_des;
        Vec13 _x0;
        Mat _At, _Bt, _Ak, _Bk, Sx, Su;
        Mat _C;
        Vec _ub, _lb;

        Mat13 _Qx;
        Mat12 _Qf;
        Mat _Q, _R;
        Mat _H, _g;

        qpOASES::QProblem solver;

    };

}


#endif //SRGB_MPC_SRGB_MPC_H
