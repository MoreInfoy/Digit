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

    robotMsg.timeStamp = 0.001 * _iter;
    robotMsg.data_size = 6;
    robotMsg.data.resize(robotMsg.data_size);
    /*robotMsg.data[0] = gaitScheduler->data().swingTimeRemain(0) > 0 ? 0 : 1;
    robotMsg.data[1] = gaitScheduler->data().swingTimeRemain(1) > 0 ? 0 : 1;*/
    robotMsg.data[0] = tasks.floatingBaseTask.pos.x();
    robotMsg.data[1] = tasks.floatingBaseTask.pos.y();
    robotMsg.data[2] = tasks.floatingBaseTask.pos.z();
    robotMsg.data[3] = robot.CoM_pos().x();
    robotMsg.data[4] = robot.CoM_pos().y();
    robotMsg.data[5] = robot.CoM_pos().z();

    if (lcm.good()) {
        lcm.publish("ROBOT_MESSAGE_TOPIC", &robotMsg);
    }
    _iter++;
}

Poplar::Vec Manager::output() {
    return tsc->jointsCmd().tau_ff;
}