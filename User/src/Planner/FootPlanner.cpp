//
// Created by nimpng on 7/25/21.
//

#include "Planner/FootPlanner.h"

FootPlanner::FootPlanner() : base("torso"), lf("left_toe_roll"), rf("right_toe_roll"), swingHeight(0.08) {
    for (int i(0); i < 2; i++) {
        firstSwing[i] = true;
    }
    lf_shift.setZero();
    rf_shift.setZero();
    pHipBody[0] << 0.000549614, 0.137757, -0.829919;
    pHipBody[1] << -0.000742945, -0.137698, 0.829925;
    vBodyDes.setZero();
}

void
FootPlanner::plan(size_t iter, const RobotState &state, RobotWrapper &robot, const GaitData &gaitData, Tasks &tasks) {
    auto lf_pose = robot.frame_pose(lf);
    auto rf_pose = robot.frame_pose(rf);
    //    auto com_pos = robot.frame_pose(base).translation();
    auto com_pos = robot.CoM_pos();
    auto R_wb = robot.frame_pose(base).rotation();
    //    auto com_vel = robot.frame_6dVel_localWorldAligned(base).linear();
    auto com_vel = robot.CoM_vel();

    if (iter == 0) {
        lfTraj.setHeight(swingHeight);
        lfTraj.setInitialPosition(lf_pose.translation());
        lfTraj.setFinalPosition(lf_pose.translation());
        //        pHipBody[0] = lf_pose.translation() - com_pos;

        rfTraj.setHeight(swingHeight);
        rfTraj.setInitialPosition(rf_pose.translation());
        rfTraj.setFinalPosition(rf_pose.translation());
        //        pHipBody[1] = rf_pose.translation() - com_pos;

        tasks.leftFootTask.R_wb = lf_pose.rotation();
        tasks.rightFootTask.R_wb = rf_pose.rotation();

        tasks.leftFootContact.R_wb = lf_pose.rotation();
        tasks.rightFootContact.R_wb = rf_pose.rotation();

        // TODO: check for every robot
        lf_shift.z() = lf_pose.translation().z();
        rf_shift.z() = rf_pose.translation().z();
    }


    Scalar ratio = 1 / exp(1e-4 * iter);

    vBodyDes.x() = 0.6 * (1.0 - ratio); // TODO: get desired velocity
    Vec3 vWorldDes = R_wb * vBodyDes;
    Scalar yawd_des = 0; // TODO:

    if (iter != 0) {
        /*// compute foot placement]
        Vec3 pYawCorrected, Pf;
        for (int i(0); i < 2; i++) {
            if (gaitData.swingTimeRemain(i) > 0) {
                pYawCorrected =
                        coordinateRotation(CoordinateAxis::Z, -yawd_des * gaitData.stanceTime[i] / 2) * pHipBody[i];
                Pf.x() = com_pos.x() + 0.5 * com_vel.x() / sqrt(9.81 / com_pos.z());
                Pf.y() = com_pos.y() + 0.5 * com_vel.y() / sqrt(9.81 / com_pos.z());
                Pf += R_wb * (pHipBody[i] + vBodyDes * gaitData.swingTimeRemain[i]);
                Pf.z() = 0.003;
            } else {
                // double p_rel_max = 0.3f;
                double py_rel_max = 0.5f;
                double px_rel_max = 0.5f;

                pYawCorrected =
                        coordinateRotation(CoordinateAxis::Z, -yawd_des * gaitData.stanceTime[i] / 2) * pHipBody[i];

                Pf = com_pos + R_wb * (pYawCorrected + vBodyDes * gaitData.swingTime[i]);

                // Using the estimated velocity is correct
                double pfx_rel = .28f * (com_vel[0] - vWorldDes[0]) +
                                 (0.5f * com_pos[2] / 9.81f) *
                                 (com_vel[1] * yawd_des);

                double pfy_rel = .28f * (com_vel[1] - vWorldDes[1]) +
                                 (0.5f * com_pos[2] / 9.81f) *
                                 (com_vel[0] * yawd_des);

                pfx_rel = fminf(fmaxf(pfx_rel, -px_rel_max), px_rel_max);
                pfy_rel = fminf(fmaxf(pfy_rel, -py_rel_max), py_rel_max);
                Pf[0] += pfx_rel;
                //            Pf[0] = 0.06;
                Pf[1] += pfy_rel;
                Pf[2] = 0.003;
            }

            if (i == 0) {
                lfTraj.setHeight(swingHeight);
                Pf += lf_shift;
                lfTraj.setFinalPosition(Pf);
                tasks.leftFootContact.pos = Pf;
            } else {
                rfTraj.setHeight(swingHeight);
                Pf += rf_shift;
                rfTraj.setFinalPosition(Pf);
                tasks.rightFootContact.pos = Pf;
            }

        }
        cout << "capture point: ["
             << robot.CoM_pos().x() + robot.CoM_vel().x() / sqrt(9.8 / robot.CoM_pos().z())
             << ", "
             << robot.CoM_pos().y() + robot.CoM_vel().y() / sqrt(9.8 / robot.CoM_pos().z())
             << "]" << endl;
        cout << "lpf: " << (robot.CoM_pos() - lf_pose.translation()).transpose() << endl;
        cout << "rpf: " << (robot.CoM_pos() - rf_pose.translation()).transpose() << endl;
        cout << "CoM pos: " << robot.CoM_pos().transpose() << endl;*/


        // double p_rel_max = 0.3f;
        double py_rel_max = 0.3;
        double px_rel_max = 0.3;

        // compute foot placement
        for (int i(0); i < 2; i++) {
            double stanceTime = gaitData.stanceTime[i];
            Vec3 pYawCorrected =
                    coordinateRotation(CoordinateAxis::Z, -yawd_des * stanceTime / 2) * pHipBody[i];

            Vec3 Pf = robot.frame_pose(base).translation() +
                      R_wb * (pYawCorrected + vBodyDes * gaitData.swingTimeRemain[i]);


            // Using the estimated velocity is correct
            double pfx_rel = com_vel[0] * 0.5 * gaitData.stanceTime[i] +
                             .15f * (com_vel[0] - vWorldDes[0]) +
                             (0.5f * com_pos[2] / 9.81f) *
                             (com_vel[1] * yawd_des);

            double pfy_rel = com_vel[1] * 0.5 * gaitData.stanceTime[i] +
                             .25f * (com_vel[1] - vWorldDes[1]) +
                             (0.5f * com_pos[2] / 9.81f) *
                             (com_vel[0] * yawd_des);

            pfx_rel = fminf(fmaxf(pfx_rel, -px_rel_max), px_rel_max);
            pfy_rel = fminf(fmaxf(pfy_rel, -py_rel_max), py_rel_max);
            Pf[0] += pfx_rel;
//            Pf[0] = 0.06;
            Pf[1] += pfy_rel;
            Pf[2] = 0.003;

            if (i == 0) {
                lfTraj.setHeight(swingHeight);
                Pf += lf_shift;
                lfTraj.setFinalPosition(Pf);
                tasks.leftFootContact.pos = Pf;
            } else {
                rfTraj.setHeight(swingHeight);
                Pf += rf_shift;
                rfTraj.setFinalPosition(Pf);
                tasks.rightFootContact.pos = Pf;
            }

            cout << "capture point: ["
                 << robot.CoM_pos().x() + robot.CoM_vel().x() / sqrt(9.8 / robot.CoM_pos().z())
                 << ", "
                 << robot.CoM_pos().y() + robot.CoM_vel().y() / sqrt(9.8 / robot.CoM_pos().z())
                 << "]" << endl;
            cout << "lpf: " << lf_pose.translation().transpose() << endl;
            cout << "rpf: " << rf_pose.translation().transpose() << endl;
            cout << "v_des: " << vBodyDes.transpose() << endl;
        }
    }

    // generating foot trajectory
    FootSwingTrajectory *footSwingTrajectory = &lfTraj;
    pinocchio::SE3 *foot_pose = &lf_pose;
    LinkTask *footTaskData = &(tasks.leftFootTask);
    for (int i = 0; i < 2; i++) {

        if (i == 1) {
            footSwingTrajectory = &rfTraj;
            foot_pose = &rf_pose;
            footTaskData = &(tasks.rightFootTask);
        }

        if (gaitData.swingTimeRemain[i] > 0.) // swing
        {
            if (firstSwing[i]) {
                firstSwing[i] = false;
                footSwingTrajectory->setInitialPosition(foot_pose->translation());
            }

            double swingPhase = 1 - gaitData.swingTimeRemain[i] / gaitData.swingTime[i];
            footSwingTrajectory->computeSwingTrajectoryBezier(swingPhase, gaitData.swingTime[i]);

            footTaskData->pos = footSwingTrajectory->getPosition();
            footTaskData->vel = foot_pose->rotation().transpose() * footSwingTrajectory->getVelocity();
            footTaskData->acc = foot_pose->rotation().transpose() * footSwingTrajectory->getAcceleration();
        } else // stance
        {
            firstSwing[i] = true;
            footTaskData->pos = foot_pose->translation();
            footTaskData->vel.setZero();
            footTaskData->acc.setZero();
        }
    }
    cout << "task: rf pos: " << tasks.rightFootTask.pos.transpose() << endl;
    cout << "task: rf vel: " << tasks.rightFootTask.vel.transpose() << endl;
    cout << "task: rf acc: " << tasks.rightFootTask.acc.transpose() << endl;
    cout << "task: rf R: " << tasks.rightFootTask.R_wb << endl;
    cout << "task: rf omega: " << tasks.rightFootTask.omega.transpose() << endl;
    cout << "task: rf omega_dot: " << tasks.rightFootTask.omega_dot.transpose() << endl;

    cout << "rf pos: " << rf_pose.translation().transpose() << endl;
    cout << "rf R: " << rf_pose.rotation() << endl;


}


Mat3 FootPlanner::coordinateRotation(CoordinateAxis axis, double theta) {
    Scalar s = std::sin(theta);
    Scalar c = std::cos(theta);

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

