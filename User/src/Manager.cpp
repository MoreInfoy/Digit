//
// Created by nimpng on 6/16/21.
//


#include "Manager.h"

#ifdef FIXED_BASE
bool fixedBase = true;
#else
bool fixedBase = false;
#endif

Manager::Manager(const RobotState &state) : _state(state), robot(URDF, SRDF, fixedBase), _iter(0) {
    mpc_horizon = 10;
    mpc_dt = 0.1;
    dt = 0.001;
    gaitScheduler = new GaitScheduler(dt);
    floatingBasePlanner = new FloatingBasePlanner(mpc_horizon, mpc_dt, dt);
    footPlanner = new FootPlanner();
    tsc = new TSC_IMPL(robot);
    tasks.floatingBaseTask.link_name = "torso";
    tasks.leftFootTask.link_name = "left_toe_roll";
    tasks.rightFootTask.link_name = "right_toe_roll";
}

Manager::~Manager() {
    delete gaitScheduler;
    delete floatingBasePlanner;
    delete footPlanner;
    delete tsc;
}

void Manager::init() {

}

void Manager::update() {
    Vec qpos, qdot;
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

    VecXi mask(8);
    if (robot.isFixedBase()) {
        mask.setZero();
        robot.setContactMask(mask);
    } else {
        mask.setOnes();
        if (gaitScheduler->data().swingTimeRemain(0) > 0) {
            mask.head(4).setZero();
        }

        if (gaitScheduler->data().swingTimeRemain(1) > 0) {
            mask.tail(4).setZero();
        }
        mask.setOnes();
        robot.computeAllData(qpos, qdot, mask);
    }
}

void Manager::run() {
    gaitScheduler->run(_iter, _state, robot);
    gaitScheduler->updateContactTable(mpc_horizon, mpc_dt / dt);
    update();
    footPlanner->plan(_iter, _state, robot, gaitScheduler->data(), tasks);
    floatingBasePlanner->plan(_iter, _state, robot, gaitScheduler->data(), tasks);
    tsc->run(_iter, _state, gaitScheduler->data(), tasks);

    runLCM();
    _iter++;
}

Poplar::Vec Manager::output() {
    return tsc->jointsCmd().tau_ff;
}

void Manager::runLCM() {
    robotMsg.timeStamp = 0.001 * _iter;
    robotMsg.data_size = 3 * mpc_horizon;
    robotMsg.data.resize(robotMsg.data_size);
    /*robotMsg.data[0] = gaitScheduler->data().swingTimeRemain(0) > 0 ? 0 : 1;
    robotMsg.data[1] = gaitScheduler->data().swingTimeRemain(1) > 0 ? 0 : 1;*/
    robotMsg.data[0] = tasks.floatingBaseTask.pos.x();
    robotMsg.data[1] = tasks.floatingBaseTask.pos.y();
    robotMsg.data[2] = tasks.floatingBaseTask.pos.z();
    robotMsg.data[3] = robot.CoM_pos().x();
    robotMsg.data[4] = robot.CoM_pos().y();
    robotMsg.data[5] = robot.CoM_pos().z();

    trajectoryLcm.n_point = mpc_horizon;
    trajectoryLcm.data.resize(6 * mpc_horizon);
    auto x_opt = floatingBasePlanner->getOptimalTraj();
    auto x_des = floatingBasePlanner->getDesiredTraj();

    for (int i = 0; i < mpc_horizon; i++) {
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

    if (lcm.good()) {
        lcm.publish("ROBOT_MESSAGE_TOPIC", &robotMsg);
        lcm.publish("TRAJECTORY_LCM", &trajectoryLcm);
    }
}

