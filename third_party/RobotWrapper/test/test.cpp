//
// Created by nimapng on 6/25/21.
//

#include "RobotWrapper/RobotWrapper.h"
#include "RW_path.h"

int main() {
    RobotWrapper robot(string(ROBOT_WRAPPER_PATH) + "/test/model/ur5_joint_limited_robot.urdf", true);
    RobotWrapperMath::Vec qpos(6), qvel(6);
    qpos << -3.48886e-05, -1.57029, 0.000309709, 0.00020329, -7.50051e-07, 5.90164;
    qvel << 6.01388e-05, -0.000496574, -0.000202142, -4.02428e-05, -3.05742e-07, -1.18901;
    robot.computeAllData(qpos, qvel);
    RobotWrapperMath::Mat6x J;
    robot.Jacobia_local("wrist_3_link", J);
    cout << robot.M() << endl;
    return 0;
}