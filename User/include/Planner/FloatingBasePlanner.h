//
// Created by nimpng on 7/25/21.
//

#ifndef POPLARDIGIT_FLOATINGBASEPLANNER_H
#define POPLARDIGIT_FLOATINGBASEPLANNER_H


#include "Planner.h"
#include "SRGB_MPC/SRGB_MPC.h"
#include "Timer.h"

class FloatingBasePlanner : public Planner {
public:
    FloatingBasePlanner(Poplar::Index horizon,
                        Scalar mpc_dt, Scalar dt);

    virtual void plan(size_t iter,
                      const RobotState &state,
                      RobotWrapper &robot,
                      const GaitData &gaitData,
                      Tasks &tasks);

    ConstVecRef getOptimalTraj();

    ConstVecRef getDesiredTraj();

private:
    void generateRefTraj(const RobotState &state,
                         RobotWrapper &robot);

    void generateContactTable(const GaitData &gaitData);

    string base;
    Scalar mass;
    Scalar fmax;
    Scalar _dt, _mpc_dt;
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
};


#endif //POPLARDIGIT_FLOATINGBASEPLANNER_H
