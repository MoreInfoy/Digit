//
// Created by nimapng on 6/25/21.
//

#ifndef TASKSPACECONTROL_ROBOTWRAPPER_H
#define TASKSPACECONTROL_ROBOTWRAPPER_H

#include "RobotWrapper/pinocchio.h"

using namespace RobotWrapperMath;

class RobotWrapper {
public:
    explicit RobotWrapper(string urdf_file, bool isFixedBase = false);

    void setContactMask(const VecXi &mask);

    ConstRefVec actuatorsEffortLimit();

    void setContactVirtualLink(const vector<string> &names);

    void setConnectVirtualLink(const pair<string, string> &link_pairs);

    void computeAllData(ConstRefVec qpos, ConstRefVec qvel, const VecXi &mask = VecXi::Zero(0));

    Vec3 CoM_pos();

    Vec3 CoM_vel();

    Vec3 CoM_acc();

    const Mat3x &Jacobia_CoM();

    void Jacobia_local(string frame_name, Mat6x &J);

    void Jacobia_world(string frame_name, Mat6x &J);

    void analyticalJacobia(string frame_name, Mat6x &J);

    const Mat6x &momentumJacobia();

    ConstRefMat M();

    ConstRefMat Minv();

    ConstRefVec nonLinearEffects();

    const pin::SE3 &frame_pose(string frame_name);

    pin::Motion frame_6dVel_local(string frame_name);

    pin::Motion frame_6dAcc_local(string frame_name);

    pin::Motion frame_6dClassicalAcc_local(string frame_name);

    pin::Motion frame_6dClassicalAcc_world(string frame_name);

    Vec6 momentumTimeVariation();

    int nq();

    int nv();

    int na();

    int nc();

    int ncf(); // n-DoFs constraints force

    const VecXi &contactMask();

    ConstRefMat contactJacobia();

    ConstRefVec activeContactPointBiasAcc();

    ConstRefVec qpos();

    ConstRefVec qvel();

    bool isFixedBase();

private:
    void computeContactJacobia();

    void computeActiveContactPointBiasAcc();

    pin::FrameIndex getFrameID(string frame_name);

    pin::Model _model;
    pin::Data _data;
    string _urdf_file;
    Vec _qpos;
    Vec _qvel;
    bool _isFixedBase;
    vector<string> _contactPoint_virtual_link;
    pair<string, string> _connect_point_pairs;
    VecXi _mask;
    Mat _Jc;
    Vec _contactPointsBiasAcc;
};


#endif //TASKSPACECONTROL_ROBOTWRAPPER_H
