//
// Created by nimapng on 6/25/21.
//

#include "RobotWrapper/RobotWrapper.h"
#include "path_PoplarLib.h"

int main() {
    RobotWrapper robot(string(PoplarLib_PATH) + "/test/RobotWrapper/model/digit_ysp.urdf", true);
    vector<pair<string, string>> link_pairs;
    link_pairs.push_back(pair<string, string>("cp_left_achillies_rod", "cp_left_heel_spring"));
    Mat T(6, 3);
    T.setZero();
    T.topRows(3).setIdentity();
    Mat T_dot = Mat::Zero(6, 3);
    robot.setConstraintForceSubspace(T, T_dot);
    robot.setConnectVirtualLink(link_pairs);

    RobotWrapperMath::Vec qpos(robot.nq()), qvel(robot.nv());
    qpos.setZero();
    qvel.fill(0.2);
    robot.computeAllData(qpos, qvel);
    cout << robot.M() << endl;
    return 0;
}