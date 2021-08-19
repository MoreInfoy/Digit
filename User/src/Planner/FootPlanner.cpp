//
// Created by nimpng on 7/25/21.
//

#include "Planner/FootPlanner.h"

FootPlanner::FootPlanner() : base("torso"), lf("left_toe_roll"), rf("right_toe_roll"), swingHeight(0.10) {
    for (int i(0); i < 2; i++) {
        firstSwing[i] = true;
    }
    lf_shift.setZero();
    rf_shift.setZero();
}

void
FootPlanner::plan(size_t iter, const RobotState &state, RobotWrapper &robot, const GaitData &gaitData, Tasks &tasks) {
    auto lf_pose = robot.frame_pose(lf);
    auto rf_pose = robot.frame_pose(rf);
    auto base_pose = robot.frame_pose(base);
    auto base_vel = robot.frame_6dVel_localWorldAligned(base).linear();

    if (iter == 0) {
        lfTraj.setHeight(swingHeight);
        lfTraj.setInitialPosition(lf_pose.translation());
        lfTraj.setFinalPosition(lf_pose.translation());
        pHipBody[0] = lf_pose.translation() - base_pose.translation();

        rfTraj.setHeight(swingHeight);
        rfTraj.setInitialPosition(rf_pose.translation());
        rfTraj.setFinalPosition(rf_pose.translation());
        pHipBody[1] = rf_pose.translation() - base_pose.translation();

        tasks.leftFootTask.R_wb = lf_pose.rotation();
        tasks.rightFootTask.R_wb = rf_pose.rotation();

        // TODO: check for every robot
        lf_shift.z() = lf_pose.translation().z();
        rf_shift.z() = rf_pose.translation().z();
    }


    Vec3 vBodyDes(0, 0, 0.); // TODO: get desired velocity
    Vec3 vWorldDes = base_pose.rotation() * vBodyDes;
    Scalar yawd_des = 0; // TODO:
    if (iter != 0) {
        // compute foot placement
        for (int i(0); i < 2; i++) {


            double stanceTime = gaitData.stanceTime[i];
            Vec3 pYawCorrected =
                    coordinateRotation(CoordinateAxis::Z, -yawd_des * stanceTime / 2) * pHipBody[i];

            Vec3 Pf = base_pose.translation() + base_pose.rotation() * (pYawCorrected
                                                                        + vBodyDes *
                                                                          gaitData.swingTimeRemain[0]);
            // double p_rel_max = 0.3f;
            double p_rel_max = 0.15f;

            // Using the estimated velocity is correct
            double pfx_rel = base_vel[0] * 0.5 * gaitData.stanceTime[i] +
                             .03f * (base_vel[0] - vWorldDes[0]) +
                             (0.5f * base_pose.translation()[2] / 9.81f) *
                             (base_vel[1] * yawd_des);

            double pfy_rel = base_vel[1] * .5 * gaitData.stanceTime[i] +
                             .03f * (base_vel[1] - vWorldDes[1]) +
                             (0.5f * base_pose.translation()[2] / 9.81f) *
                             (base_vel[0] * yawd_des);

            pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
            pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
            // Pf[0] += pfx_rel;
            Pf[1] += pfy_rel;
            Pf[2] = 0.003;

            if (i == 0) {
                lfTraj.setHeight(swingHeight);
                Pf += lf_shift;
                // lfTraj.setFinalPosition(Pf);
            } else {
                rfTraj.setHeight(swingHeight);
                Pf += rf_shift;
                // rfTraj.setFinalPosition(Pf);
            }
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

