//
// Created by nimpng on 7/25/21.
//

#ifndef POPLARDIGIT_FLOATINGBASEPLANNER_H
#define POPLARDIGIT_FLOATINGBASEPLANNER_H


#include "Planner.h"
#include "SRGB_MPC/SRGB_MPC.h"
#include "LIPM/LIPM_MPC.h"
#include "Timer.h"

class FloatingBasePlanner : public Planner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FloatingBasePlanner(Poplar::Index horizons,
                        Scalar mpc_dt, Scalar dt);

    virtual void plan(size_t iter,
                      const RobotState &state,
                      RobotWrapper &robot,
                      const GaitData &gaitData,
                      Tasks &tasks);

    ConstVecRef getOptimalTraj();

    ConstVecRef getDesiredTraj();

    ConstVecRef getZMPRef();

private:
    void generateRefTraj(const RobotState &state,
                         RobotWrapper &robot);

    void generateContactTable(const GaitData &gaitData);

    void srgb_mpc(size_t iter,
                  const RobotState &state,
                  RobotWrapper &robot,
                  const GaitData &gaitData,
                  Tasks &tasks);

    void lipm_mpc(size_t iter,
                  const RobotState &state,
                  RobotWrapper &robot,
                  const GaitData &gaitData,
                  Tasks &tasks);

    string base;
    Scalar mass;
    Scalar fmax;
    Scalar _dt, _mpc_dt;
    Poplar::Index _appliedIndex;
    Mat3 Ibody;

    Scalar xDes;
    Scalar yDes;
    Scalar maxPosError;
    Scalar bodyHeight; // TODO
    Scalar yawDes;
    Scalar rollDes;
    Scalar pitchDes;
    Vec2 rpInt;
    Vec2 rpCom;
    Scalar yawdDes;
    Vec12 trajInit;

    Matrix<Scalar, 13, 1> x0;
    Matrix<Scalar, Dynamic, 1> X_d;
    Matrix<Poplar::Index, 8, Dynamic> contactTable;

    Matrix<qpOASES::real_t, Dynamic, 1> q_soln;
    SRGB_MPC::SRGB_MPC_IMPL srgbMpc;
    LIPM_MPC lipmMpc;
    Vec zmpRef;
};


#endif //POPLARDIGIT_FLOATINGBASEPLANNER_H
