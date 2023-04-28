//
// Created by nimpng on 7/25/21.
//

#include "Planner/FloatingBasePlanner.h"

FloatingBasePlanner::FloatingBasePlanner(Poplar::Index horizons, Scalar mpc_dt, Scalar dt) :
        base("torso"), srgbMpc(horizons, mpc_dt, 8) {
    _dt = dt;
    _mpc_dt = mpc_dt;
    _appliedIndex = 0;
    contactTable = MatInt(8, horizons);

    xDes = 0;
    yDes = 0;
    maxPosError = 0.1;
    bodyHeight = 0.892442; // TODO
    yawDes = 0.;
    rollDes = 0.;
    pitchDes = 0.;
    yawdDes = 0;

    rpInt.setZero();
    rpCom.setZero();
    trajInit.setZero();

    x0.setZero();

    X_d.resize(13 * horizons, Eigen::NoChange);
    X_d.setZero();
    contactTable.setZero();

    q_soln.resize(12 * horizons);
    q_soln.setZero();

    Vec Qx(12);
    Qx.segment(0, 3) << 100.5, 100.5, 10.0;
    Qx.segment(3, 3) << 10, 10, 50;
    Qx.segment(6, 3) << 0.1, 0.1, 0.3;
    Qx.segment(9, 3) << 10.1, 10.1, 0.3;
    Vec Qf(3);
    Qf.fill(4e-5);
    srgbMpc.setWeight(Qx, Qf);
    srgbMpc.setMassAndInertia(mass, Ibody);
    srgbMpc.setMaxForce(500);

    LIPM_Parameters &param = lipmMpc.parameters();
    param.Qx << 10, 10;
    param.Qu << 1e-5, 1e-5;
    param.mpc_dt = mpc_dt;
    param.mpc_horizons = horizons;
    param.height = bodyHeight;
    param.px = 0.10;
    param.nx = 0.12;
    param.py = 0.04;
    param.ny = 0.04;
    param.mu = 0.7;
    param.max_force = 300;
}

void FloatingBasePlanner::plan(size_t iter, const RobotState &state, RobotWrapper &robot, const GaitData &gaitData,
                               Tasks &tasks) {
    lipm_mpc(iter, state, robot, gaitData, tasks);
//    srgb_mpc(iter, state, robot, gaitData, tasks);
}

void FloatingBasePlanner::lipm_mpc(size_t iter, const RobotState &state,
                                   RobotWrapper &robot, const GaitData &gaitData,
                                   Tasks &tasks) {
    auto &param = lipmMpc.parameters();
    param.mass = robot.totalMass();
    param.inertia = robot.Ig();

    Vec3 r;
    vector<Vec3> cp;
    auto cl = robot.contactVirtualLinks();
    for (int i(0); i < cl.size(); i++) {
        r = robot.frame_pose(cl[i]).translation();
        cp.push_back(r);
    }
    lipmMpc.setContactPoints(cp);

    /* Vec3 c = robot.CoM_pos();
     Vec3 cdot = robot.CoM_vel();
     Vec3 cddot = robot.CoM_acc();*/

    Vec3 c = state.floatingBaseState.pos;
    Vec3 cdot = state.floatingBaseState.vel;
    Vec3 cddot = state.floatingBaseState.acc;

    if (iter % 30 == 0) {
        lipmMpc.x0() << c(0), cdot(0), cddot(0), c(1), cdot(1), cddot(1);
        /* zmp reference */
        Vec3 lpos, rpos;
        zmpRef = Vec::Zero(2 * param.mpc_horizons);
        Scalar ts_l, ts_r;
        if (gaitData.stanceTimeRemain[0] > 0) {
            lpos = tasks.leftFootTask.pos;
            ts_l = gaitData.stanceTimeRemain[0];
        } else {
            lpos = tasks.leftFootTask.pos + gaitData.swingTimeRemain[0] * tasks.desired_vel;
//            lpos = tasks.leftFootContact.pos;
            ts_l = gaitData.swingTimeRemain[0] + gaitData.stanceTime[0];
        }
        if (gaitData.stanceTimeRemain[1] > 0) {
            rpos = tasks.rightFootTask.pos;
            ts_r = gaitData.stanceTimeRemain[1];
        } else {
            rpos = tasks.rightFootTask.pos + gaitData.swingTimeRemain[1] * tasks.desired_vel;
//            rpos = tasks.rightFootContact.pos;
            ts_r = gaitData.swingTimeRemain[1] + gaitData.stanceTime[1];
        }
        for (int i = 0; i < param.mpc_horizons; i++) {
            if (_mpc_dt * i > ts_l && gaitData.contactTable(0, i) > 0) {
                lpos += gaitData.swingTime[0] * tasks.desired_vel;
                ts_l += gaitData.swingTime[0] + gaitData.stanceTime[0];
            }
            if (_mpc_dt * i > ts_r && gaitData.contactTable(1, i) > 0) {
                rpos += gaitData.swingTime[1] * tasks.desired_vel;
                ts_r += gaitData.swingTime[1] + gaitData.stanceTime[1];
            }
            if (gaitData.contactTable(0, i) > 0 && gaitData.contactTable(1, i) > 0) {
                zmpRef.segment(i * 2, 2) << 0.5 * (lpos.x() + rpos.x()),
                        0.5 * (lpos.y() + rpos.y());
            } else if (gaitData.contactTable(0, i) > 0) {
                zmpRef.segment(i * 2, 2) << lpos.x(), lpos.y();
            } else if (gaitData.contactTable(1, i) > 0) {
                zmpRef.segment(i * 2, 2) << rpos.x(), rpos.y();
            } else {
                zmpRef.segment(i * 2, 2) << 0.5 * (lpos.x() + rpos.x()),
                        0.5 * (lpos.y() + rpos.y());
            }
        }
        lipmMpc.setZMPRef(zmpRef);

        /* terminal zmp constraints */
        Mat C = Mat::Zero(2, 2);
        Vec c_lb = Vec::Zero(2);
        Vec c_ub = Vec::Zero(2);
        VecXi ctr = gaitData.contactTable.rightCols(1);

        if (ctr(0) > 0 && ctr(1) > 0) {
            C.setIdentity();
            c_lb(1) = rpos.y() - param.ny;
            c_ub(1) = lpos.y() + param.py;
            Scalar k = (lpos.x() - rpos.x()) /
                       (lpos.y() - rpos.y());
            C.row(0) << -1, k;
            c_ub(0) = k * lpos.y() - lpos.x() + param.nx;
            c_lb(0) = k * lpos.y() - lpos.x() - param.px;
        } else if (ctr(0) > 0) {
            C.setIdentity();
            c_lb << lpos.x() - param.nx, lpos.y() - param.ny;
            c_ub << lpos.x() + param.px, lpos.y() + param.py;
        } else if (ctr(1) > 0) {
            C.setIdentity();
            c_lb << rpos.x() - param.nx, rpos.y() - param.ny;
            c_ub << rpos.x() + param.px, rpos.y() + param.py;
        }
        lipmMpc.updateTerminalZMPConstraints(C, c_lb, c_ub);

        /*cout << "------------------C------------------" << endl
             << C << endl;
        cout << "------------------c_lb------------------" << endl
             << c_lb.transpose() << endl;
        cout << "------------------c_ub------------------" << endl
             << c_ub.transpose() << endl;
        getchar();*/

        /*cout << "------------------lf------------------" << endl
             << robot.frame_pose("left_toe_roll").translation().transpose() << endl;
        cout << "------------------c1------------------" << endl
             << robot.frame_pose("contact1").translation().transpose() << endl;
        cout << "------------------c3------------------" << endl
             << robot.frame_pose("contact3").translation().transpose() << endl;
        getchar();*/

        Timer timer;
        lipmMpc.run();
        // printf("LIPM MPC solve time: %f\n", timer.getMs());

        _appliedIndex = 0;
    }

    auto xopt = lipmMpc.optimalTraj();
    /*tasks.floatingBaseTask.pos << xopt(_appliedIndex * 4),
            xopt(2 + _appliedIndex * 4),
            (lipmMpc.parameters().height - c(2)) / param.mpc_horizons + c(2);
    tasks.floatingBaseTask.vel << xopt(1 + _appliedIndex * 4), xopt(3 + _appliedIndex * 4),
            (lipmMpc.parameters().height - c(2)) /
            (param.mpc_dt * param.mpc_horizons);*/
    tasks.floatingBaseTask.pos << xopt(_appliedIndex * 6),
            xopt(3 + _appliedIndex * 6),
            lipmMpc.parameters().height;
    tasks.floatingBaseTask.vel << xopt(1 + _appliedIndex * 6), xopt(4 + _appliedIndex * 6), 0;
    tasks.floatingBaseTask.acc << xopt(2 + _appliedIndex * 6), xopt(5 + _appliedIndex * 6), 0;
    tasks.floatingBaseTask.omega_dot.setZero();
    tasks.floatingBaseTask.R_wb.setIdentity();
    tasks.floatingBaseTask.omega.setZero();

    // cout << "acc_des: " << tasks.floatingBaseTask.acc.transpose() << endl;


    VecXi mask = VecXi::Zero(8);
    if (gaitData.stanceTimeRemain(0) > 0) {
        mask.head(4).setOnes();
    }
    if (gaitData.stanceTimeRemain(1) > 0) {
        mask.tail(4).setOnes();
    }
    Vec force = lipmMpc.forceDistribute(_appliedIndex, c, cdot, robot.angularMomentum(), mask);
    tasks.forceTask = force;

    // cout << "distributed force: " << force.transpose() << endl;

    if (iter % Poplar::Index(param.mpc_dt / _dt) == 0) {
        _appliedIndex++;
    }
}

void FloatingBasePlanner::srgb_mpc(size_t iter, const RobotState &state, RobotWrapper &robot,
                                   const GaitData &gaitData, Tasks &tasks) {
    mass = robot.totalMass();
    Ibody = robot.Ig();
    srgbMpc.setMassAndInertia(mass, Ibody);

    auto base_pose = robot.frame_pose(base);
    auto base_vel = robot.frame_6dVel_localWorldAligned(base);
    Vec3 rpy = pin::rpy::matrixToRpy(base_pose.rotation());
    Vec3 c = robot.CoM_pos();
    Vec3 cdot = robot.CoM_vel();
    /*Vec3 c = base_pose.translation();
    Vec3 cdot = robot.frame_6dVel_localWorldAligned(base).linear();*/

    // TODO:
    Vec3 vBodyDes(0, 0, 0);
    Scalar yawd_des = 0;
    Vec3 vWorldDes = robot.frame_pose(base).rotation() * vBodyDes;
    if (iter == 0) {
        xDes = c[0];
        yDes = c[1];
        yawDes = rpy[2];
    } else {
        xDes += vWorldDes[0] * _dt;
        yDes += vWorldDes[1] * _dt;
        yawDes = rpy[2] + yawd_des * _dt;
    }
    rpInt[0] = _dt * (rollDes - rpy[0]);
    rpInt[1] = _dt * (rollDes - rpy[1]);
    rpCom[0] = rollDes + rpInt[0];
    rpCom[1] = pitchDes + 0.5 * (rollDes - rpy[0]) + rpInt[1];

    if (iter % 15 == 0) {
        x0 << rpy, c, base_vel.angular(), cdot, -9.81;
        srgbMpc.setCurrentState(x0);
        generateRefTraj(state, robot);
        srgbMpc.setDesiredDiscreteTrajectory(X_d);
        generateContactTable(gaitData);
        srgbMpc.setContactTable(contactTable);

        Vec3 r;
        vector<Vec3> cp;
        auto cl = robot.contactVirtualLinks();
        for (int i(0); i < cl.size(); i++) {
            r = robot.frame_pose(cl[i]).translation();
            /*std::cout << "r " << i << ": " << r.transpose() << "\n";*/
            cp.push_back(r);
        }
        srgbMpc.setContactPointPos(cp);
        Timer timer;
        srgbMpc.solve(0.0);
        // printf("mpc solve time: %f\n", timer.getMs());
        /*std::cout << "contact force: " << srgbMpc.getCurrentDesiredContactForce().transpose() << std::endl;
        std::cout << "optimal traj: " << srgbMpc.getDiscreteOptimizedTrajectory().transpose() << std::endl;*/
    }
    auto xopt = srgbMpc.getDiscreteOptimizedTrajectory();
    auto xdd = srgbMpc.getXDot();
    tasks.floatingBaseTask.pos = xopt.segment(3, 3);
    tasks.floatingBaseTask.vel = xopt.segment(9, 3);
    tasks.floatingBaseTask.acc = xdd.segment(9, 3);
    tasks.floatingBaseTask.omega_dot = xdd.segment(6, 3);
    tasks.floatingBaseTask.R_wb = pin::rpy::rpyToMatrix(xopt.segment(0, 3));
    tasks.floatingBaseTask.omega = xopt.segment(6, 3);
    tasks.forceTask = srgbMpc.getCurrentDesiredContactForce();
}

void FloatingBasePlanner::generateRefTraj(const RobotState &state, RobotWrapper &robot) {
    Vec3 vBodyDes(0, 0, 0); // TODO:
    Vec3 vWorldDes = robot.frame_pose(base).rotation() * vBodyDes;
    Vec3 pos = robot.frame_pose(base).translation();

    if (xDes - pos[0] > maxPosError)
        xDes = pos[0] + maxPosError;
    if (pos[0] - xDes > maxPosError)
        xDes = pos[0] - maxPosError;

    if (yDes - pos[1] > maxPosError)
        yDes = pos[1] + maxPosError;
    if (pos[1] - yDes > maxPosError)
        yDes = pos[1] - maxPosError;

    trajInit << rpCom[0], rpCom[1], yawDes,
            xDes, yDes, bodyHeight,
            0.0, 0.0, yawdDes,
            vWorldDes;
    X_d.head(12) = trajInit;
    for (int i = 1; i < srgbMpc.horizons(); i++) {
        for (int j = 0; j < 12; j++)
            X_d[13 * i + j] = trajInit[j];

        X_d[13 * i + 2] = trajInit[2] + i * yawdDes * _mpc_dt;
        X_d.segment(13 * i + 3, 3) = trajInit.segment(3, 3) +
                                     i * vWorldDes * _mpc_dt;
        //        std::cout << "x_d " << i << ": " << X_d.segment(13 * i, 13).transpose() << std::endl;
    }
}

void FloatingBasePlanner::generateContactTable(const GaitData &gaitData) {
    contactTable.setZero();
    for (int i = 0; i < srgbMpc.horizons(); i++) {
        if (gaitData.contactTable(0, i) == 1) {
            contactTable.col(i).head(4).fill(1);
        }
        if (gaitData.contactTable(1, i) == 1) {
            contactTable.col(i).tail(4).fill(1);
        }
    }
}

ConstVecRef FloatingBasePlanner::getOptimalTraj() {
//    return srgbMpc.getDiscreteOptimizedTrajectory();
    return lipmMpc.optimalTraj();
}

ConstVecRef FloatingBasePlanner::getDesiredTraj() {
    return ConstVecRef(X_d);
}

ConstVecRef FloatingBasePlanner::getZMPRef() {
    return Poplar::ConstVecRef(zmpRef);
}
