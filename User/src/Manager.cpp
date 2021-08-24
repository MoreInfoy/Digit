//
// Created by nimpng on 6/16/21.
//


#include "Manager.h"


#ifdef FIXED_BASE
bool fixedBase = true;
#else
bool fixedBase = false;
#endif

Manager::Manager(const RobotState &state) : _state(state), mpc_horizons(100), mpc_dt(0.03), dt(0.001),
                                            out_state("datasets_state.txt", std::ios::ate | std::ios::out),
                                            out_gait("datasets_gait.txt", std::ios::ate | std::ios::out),
                                            out_planning("datasets_planning.txt", std::ios::ate | std::ios::out),
                                            out_tsc("datasets_tsc.txt", std::ios::ate | std::ios::out),
                                            robot(URDF, SRDF, fixedBase),
                                            gaitScheduler(dt),
                                            footPlanner(),
                                            tsc(robot),
                                            floatingBasePlanner(mpc_horizons, mpc_dt, dt),
                                            _iter(0) {
    tasks.floatingBaseTask.link_name = "torso";
    tasks.leftFootTask.link_name = "left_toe_roll";
    tasks.rightFootTask.link_name = "right_toe_roll";
    tasks.desired_vel.setZero();
    qpos.resize(_state.jointsState.qpos.size() + 7);
    qdot.resize(_state.jointsState.qvel.size() + 6);
    com_pos_des << 0, 0, 0.892442;
    com_vel_des.setZero();

    if (!out_state.is_open()) {
        throw runtime_error("state datasets file open failed");
    }
    if (!out_gait.is_open()) {
        throw runtime_error("gait datasets file open failed");
    }
    if (!out_planning.is_open()) {
        throw runtime_error("planning datasets file open failed");
    }
    if (!out_tsc.is_open()) {
        throw runtime_error("tsc datasets file open failed");
    }
}

Manager::~Manager() {
    cout << "datasets file closed" << endl;
    out_state.close();
    out_gait.close();
    out_planning.close();
    out_tsc.close();
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

    if (gaitScheduler.gait_type() == GAIT_TYPE::WALK) {
        com_vel_des << 0.6, 0, 0;
        com_pos_des += dt * com_vel_des;
    }
    tasks.desired_vel = 0.1 * (com_pos_des - robot.CoM_pos()) + 0.1 * (com_vel_des - robot.CoM_vel()) + com_vel_des;
}

void Manager::run() {
    update();
    gaitScheduler.run(_iter, _state, robot);
    gaitScheduler.updateContactTable(mpc_horizons, mpc_dt / dt);
    footPlanner.plan(_iter, _state, robot, gaitScheduler.data(), tasks);
    floatingBasePlanner.plan(_iter, _state, robot, gaitScheduler.data(), tasks);
    tsc.run(_iter, _state, gaitScheduler.data(), tasks);
    runLCM();
//    saveAllData();
    _iter++;
}

Poplar::ConstVecRef Manager::output() {
    return tsc.jointsCmd().tau_ff;
}

void Manager::runLCM() {
    RobotMessage robotMsg;
    Trajectory_LCM trajectoryLcm;

    robotMsg.timeStamp = 0.001 * _iter;
    /* joint torque */
    robotMsg.data_size = 6;
    robotMsg.data.resize(6);
    /*robotMsg.data[0] = robot.frame_pose(tasks.rightFootTask.link_name).translation().x();
    robotMsg.data[1] = robot.frame_pose(tasks.rightFootTask.link_name).translation().y();
    robotMsg.data[2] = robot.frame_pose(tasks.rightFootTask.link_name).translation().z();
    robotMsg.data[3] = tasks.rightFootTask.pos.x();
    robotMsg.data[4] = tasks.rightFootTask.pos.y();
    robotMsg.data[5] = tasks.rightFootTask.pos.z();*/

    /*robotMsg.data[0] = tsc.jointsCmd().tau_ff(18);
    robotMsg.data[1] = tsc.jointsCmd().tau_ff(19);*/

    /* swing trajectory */
    /*robotMsg.data_size = 6;
    robotMsg.data.resize(6);*/
    /*robotMsg.data[0] = robot.frame_pose(tasks.rightFootTask.link_name).translation().x();
    robotMsg.data[1] = robot.frame_pose(tasks.rightFootTask.link_name).translation().y();
    robotMsg.data[2] = robot.frame_pose(tasks.rightFootTask.link_name).translation().z();
    robotMsg.data[3] = tasks.rightFootTask.pos.x();
    robotMsg.data[4] = tasks.rightFootTask.pos.y();
    robotMsg.data[5] = tasks.rightFootTask.pos.z();*/

    robotMsg.data[0] = robot.CoM_pos().x();
    robotMsg.data[1] = robot.CoM_pos().y();
    robotMsg.data[2] = robot.CoM_pos().z();
    robotMsg.data[3] = tasks.floatingBaseTask.pos.x();
    robotMsg.data[4] = tasks.floatingBaseTask.pos.y();
    robotMsg.data[5] = tasks.floatingBaseTask.pos.z();

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
    /*robotMsg.data_size = 6 * mpc_horizons;
    robotMsg.data.resize(6 * mpc_horizons);
    trajectoryLcm.n_point = mpc_horizons;
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
    auto zmpRef = floatingBasePlanner.getZMPRef();
    robotMsg.data_size = x_opt.size() + zmpRef.size();
    robotMsg.data.resize(robotMsg.data_size);
    for (int i = 0; i < x_opt.size() / 6; i++) {
        robotMsg.data[i * 8 + 0] = x_opt(6 * i + 0);
        robotMsg.data[i * 8 + 1] = x_opt(6 * i + 1);
        robotMsg.data[i * 8 + 2] = x_opt(6 * i + 2);
        robotMsg.data[i * 8 + 3] = x_opt(6 * i + 3);
        robotMsg.data[i * 8 + 4] = x_opt(6 * i + 4);
        robotMsg.data[i * 8 + 5] = x_opt(6 * i + 5);
        robotMsg.data[i * 8 + 6] = zmpRef(2 * i);
        robotMsg.data[i * 8 + 7] = zmpRef(2 * i + 1);
    }

    if (lcm1.good()) {
        lcm1.publish("ROBOT_MESSAGE_TOPIC", &robotMsg);
    }
    /*if (lcm2.good()) {
        lcm2.publish("TRAJECTORY_LCM", &trajectoryLcm);
    }*/
}

void Manager::saveAllData() {
    out_state << dt * _iter << ", ";
    out_gait << dt * _iter << ", ";
    out_planning << dt * _iter << ", ";
    out_tsc << dt * _iter << ", ";

    /* state */
    for (int i = 0; i < 3; i++) {
        out_state << _state.floatingBaseState.pos(i) << ", ";
    }
    for (int i = 0; i < 3; i++) {
        out_state << _state.floatingBaseState.vel(i) << ", ";
    }
    out_state << _state.floatingBaseState.quat.x() << ", "
              << _state.floatingBaseState.quat.y() << ", "
              << _state.floatingBaseState.quat.z() << ", "
              << _state.floatingBaseState.quat.w() << ", ";

    for (int i = 0; i < _state.jointsState.qvel.size(); i++) {
        out_state << _state.jointsState.qpos(i) << ", ";
    }
    for (int i = 0; i < _state.jointsState.qvel.size(); i++) {
        out_state << _state.jointsState.qvel(i) << ", ";
    }

    /* gait */
    for (int i = 0; i < 2; i++) {
        out_gait << gaitScheduler.data().stanceTimeRemain(i) << ", ";
        out_gait << gaitScheduler.data().swingTimeRemain(i) << ", ";
    }

    /* planning */
    for (int i = 0; i < 3; i++) {
        out_planning << tasks.floatingBaseTask.pos(i) << ", ";
    }
    for (int i = 0; i < 3; i++) {
        out_planning << tasks.floatingBaseTask.vel(i) << ", ";
    }
    for (int i = 0; i < 3; i++) {
        out_planning << tasks.floatingBaseTask.acc(i) << ", ";
    }
    for (int i = 0; i < 3; i++) {
        out_planning << tasks.floatingBaseTask.omega(i) << ", ";
    }
    for (int i = 0; i < 3; i++) {
        out_planning << tasks.floatingBaseTask.omega_dot(i) << ", ";
    }
    for (int i = 0; i < 3; i++) {
        out_planning << tasks.leftFootTask.pos(i) << ", ";
    }
    for (int i = 0; i < 3; i++) {
        out_planning << tasks.leftFootTask.vel(i) << ", ";
    }
    for (int i = 0; i < 3; i++) {
        out_planning << tasks.leftFootTask.acc(i) << ", ";
    }
    for (int i = 0; i < 3; i++) {
        out_planning << tasks.rightFootTask.pos(i) << ", ";
    }
    for (int i = 0; i < 3; i++) {
        out_planning << tasks.rightFootTask.vel(i) << ", ";
    }
    for (int i = 0; i < 3; i++) {
        out_planning << tasks.rightFootTask.acc(i) << ", ";
    }
    for (int i = 0; i < 24; i++) {
        out_planning << tasks.forceTask(i) << ", ";
    }

    /* tsc */
    for (int i = 0; i < tsc.jointsCmd().tau_ff.size(); i++) {
        out_tsc << tsc.jointsCmd().tau_ff(i) << ", ";
    }
    Vec contact_force(24);
    if (gaitScheduler.data().stanceTimeRemain(0) > 0 && gaitScheduler.data().stanceTimeRemain(1) > 0) {
        contact_force = tsc.getOptimalContactForce();
    } else if (gaitScheduler.data().stanceTimeRemain(0) > 0) {
        contact_force.head(12) = tsc.getOptimalContactForce();
    } else if (gaitScheduler.data().stanceTimeRemain(1) > 0) {
        contact_force.tail(12) = tsc.getOptimalContactForce();
    } else {
        contact_force.setZero();
    }
    for (int i = 0; i < contact_force.size(); i++) {
        out_tsc << contact_force(i) << ", ";
    }
    for (int i = 0; i < tsc.getOptimalQacc().size(); i++) {
        out_tsc << tsc.getOptimalQacc()(i) << ", ";
    }

    out_state << endl;
    out_gait << endl;
    out_planning << endl;
    out_tsc << endl;
}

