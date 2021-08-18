//
// Created by nimpng on 6/16/21.
//


#include "Manager.h"

#ifdef FIXED_BASE
bool fixedBase = true;
#else
bool fixedBase = false;
#endif

Manager::Manager(const RobotState &state) : _state(state), mpc_horizons(20), mpc_dt(0.05), dt(0.001),
                                            robot(URDF, SRDF, fixedBase),
                                            gaitScheduler(dt),
                                            footPlanner(),
                                            tsc(robot),
                                            floatingBasePlanner(mpc_horizons, mpc_dt, dt),
                                            _iter(0) {
    tasks.floatingBaseTask.link_name = "torso";
    tasks.leftFootTask.link_name = "left_toe_roll";
    tasks.rightFootTask.link_name = "right_toe_roll";
    qpos.resize(_state.jointsState.qpos.size() + 7);
    qdot.resize(_state.jointsState.qvel.size() + 6);
}

Manager::~Manager() {
}

void Manager::init() {

}

void Manager::update() {
    if (robot.isFixedBase()) {
        qpos = _state.jointsState.qpos;
        qdot = _state.jointsState.qvel;
    } else {
        qpos.resize(_state.jointsState.qpos.size() + 7);
        Vec quat(4);
        quat << _state.floatingBaseState.quat.x(),
                _state.floatingBaseState.quat.y(),
                _state.floatingBaseState.quat.z(),
                _state.floatingBaseState.quat.w();
        qpos << _state.floatingBaseState.pos,
                quat,
                _state.jointsState.qpos;
        qdot.resize(_state.jointsState.qvel.size() + 6);

        qdot << _state.floatingBaseState.quat.toRotationMatrix().transpose() * _state.floatingBaseState.vel,
                _state.floatingBaseState.quat.toRotationMatrix().transpose() *
                _state.floatingBaseState.omega,
                _state.jointsState.qvel;
    }
    robot.update(ConstVecRef(qpos), ConstVecRef(qdot));

}

void Manager::run() {
    update();
    gaitScheduler.run(_iter, _state, robot);
    gaitScheduler.updateContactTable(mpc_horizons, mpc_dt / dt);
    footPlanner.plan(_iter, _state, robot, gaitScheduler.data(), tasks);
    floatingBasePlanner.plan(_iter, _state, robot, gaitScheduler.data(), tasks);
    tsc.run(_iter, _state, gaitScheduler.data(), tasks);
    runLCM();
    _iter++;
}

Poplar::ConstVecRef Manager::output() {
    return tsc.jointsCmd().tau_ff;
}

void Manager::runLCM() {
    RobotMessage robotMsg;
    Trajectory_LCM trajectoryLcm;

    robotMsg.timeStamp = 0.001 * _iter;

    /* swing trajectory */
    /*robotMsg.data_size = 6;
    robotMsg.data.resize(6);
    robotMsg.data[0] = robot.frame_pose(tasks.leftFootTask.link_name).translation().x();
    robotMsg.data[1] = robot.frame_pose(tasks.leftFootTask.link_name).translation().y();
    robotMsg.data[2] = robot.frame_pose(tasks.leftFootTask.link_name).translation().z();
    robotMsg.data[3] = tasks.leftFootTask.pos.x();
    robotMsg.data[4] = tasks.leftFootTask.pos.y();
    robotMsg.data[5] = tasks.leftFootTask.pos.z();*/

    /* gait trajectory */
    /*robotMsg.data_size = 3 * mpc_horizons;
    printf("size: %d\n", robotMsg.data_size);
    robotMsg.data.resize(robotMsg.data_size);
    *//*robotMsg.data[0] = gaitScheduler->data().swingTimeRemain(0) > 0 ? 0 : 1;
    robotMsg.data[1] = gaitScheduler->data().swingTimeRemain(1) > 0 ? 0 : 1;*//*
    robotMsg.data[0] = tasks.floatingBaseTask.pos.x();
    robotMsg.data[1] = tasks.floatingBaseTask.pos.y();
    robotMsg.data[2] = tasks.floatingBaseTask.pos.z();
    robotMsg.data[3] = robot.CoM_pos().x();
    robotMsg.data[4] = robot.CoM_pos().y();
    robotMsg.data[5] = robot.CoM_pos().z();*/

    /* com trajectory */
    /*trajectoryLcm.n_point = mpc_horizons;
    trajectoryLcm.data.resize(6 * mpc_horizons);
    auto x_opt = floatingBasePlanner.getOptimalTraj();
    auto x_des = floatingBasePlanner.getDesiredTraj();

    for (int i = 0; i < mpc_horizons; i++) {
        trajectoryLcm.data[i * 3 + 0] = x_des(13 * i + 3);
        trajectoryLcm.data[i * 3 + 1] = x_des(13 * i + 4);
        trajectoryLcm.data[i * 3 + 2] = x_des(13 * i + 5);
        robotMsg.data[i * 6 + 0] = x_des(13 * i + 3);
        robotMsg.data[i * 6 + 1] = x_des(13 * i + 4);
        robotMsg.data[i * 6 + 2] = x_des(13 * i + 5);
        robotMsg.data[i * 6 + 3] = x_opt(13 * i + 3);
        robotMsg.data[i * 6 + 4] = x_opt(13 * i + 4);
        robotMsg.data[i * 6 + 5] = x_opt(13 * i + 5);
    }

    if (lcm1.good()) {
        lcm1.publish("ROBOT_MESSAGE_TOPIC", &robotMsg);
    }
    if (lcm2.good()) {
        lcm2.publish("TRAJECTORY_LCM", &trajectoryLcm);
    }*/

    /* com trajectory */
    auto x_opt = floatingBasePlanner.getOptimalTraj();
    robotMsg.data_size = x_opt.size();
    robotMsg.data.resize(robotMsg.data_size);
    for (int i = 0; i < x_opt.size() / 4; i++) {
        robotMsg.data[i * 4 + 0] = x_opt(4 * i + 0);
        robotMsg.data[i * 4 + 1] = x_opt(4 * i + 1);
        robotMsg.data[i * 4 + 2] = x_opt(4 * i + 2);
        robotMsg.data[i * 4 + 3] = x_opt(4 * i + 3);
    }

    if (lcm1.good()) {
        lcm1.publish("ROBOT_MESSAGE_TOPIC", &robotMsg);
    }
    /*if (lcm2.good()) {
        lcm2.publish("TRAJECTORY_LCM", &trajectoryLcm);
    }*/
}

