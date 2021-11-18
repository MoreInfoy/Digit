//
// Created by nimapng on 6/25/21.
//

#ifndef TASKSPACECONTROL_ROBOTWRAPPER_H
#define TASKSPACECONTROL_ROBOTWRAPPER_H

#include "PoplarConfig.h"

#define CONSIDER_ROTOR_INERTIA

using namespace Poplar;

class RobotWrapper {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RobotWrapper(string urdf_file, bool isFixedBase = false);

    RobotWrapper(string urdf_file, string srdf_file, bool isFixedBase = false);

    void setContactMask(const VecXi &mask);

    ConstVecRef homeConfigurations();

    ConstVecRef gear_ratio();

    ConstVecRef rotorInertia();

    ConstVecRef actuatorsEffortLimit();

    ConstVecRef actuatorsDamping();

    ConstVecRef actuatorsDampingForce();

    ConstVecRef actuatorsFriction();

    void setSpringJoints(const vector<pair<string, Scalar>> spring_joints);

    ConstVecRef jointsSpringForce();

    void setContactVirtualLink(const vector<string> &names);

    const vector<string> &contactVirtualLinks();

    void setConnectVirtualLink(const vector<pair<string, string>> &link_pairs);

    void setConstraintForceSubspace(ConstMatRef T, ConstMatRef T_dot); // in local frame

    void update(Vec qpos, Vec qvel);

    void compute(const VecXi &mask = VecXi::Zero(0));

    void computeClosedChainTerm(); // call this function after call computeAllData

    Vec3 CoM_pos();

    Vec3 CoM_vel();

    Vec3 CoM_acc();

    const Mat3x &Jacobia_CoM();

    void Jacobia_local(string frame_name, Mat6x &J);

    void Jacobia_world(string frame_name, Mat6x &J);

    void analyticalJacobia(string frame_name, Mat6x &J);

    const Mat6x &momentumJacobia();

    ConstMatRef M();

    ConstMatRef Minv();

    ConstVecRef nonLinearEffects();

    const pin::SE3 &frame_pose(string frame_name);

    pin::Motion frame_6dVel_local(string frame_name);

    pin::Motion frame_6dVel_localWorldAligned(string frame_name);

    pin::Motion frame_6dAcc_local(string frame_name);

    pin::Motion frame_6dAcc_localWorldAligned(string frame_name);

    pin::Motion frame_6dClassicalAcc_local(string frame_name);

    pin::Motion frame_6dClassicalAcc_world(string frame_name);

    Vec6 momentumTimeVariation();

    Vec3 angularMomentum();

    Scalar totalMass();

    Mat3 Ig();

    int nq();

    int nv();

    int na();

    int nc();

    int ncf(); // ncf = Cols(T)

    const VecXi &contactMask();

    ConstMatRef contactJacobia();

    ConstVecRef activeContactPointBiasAcc();

    ConstMatRef constraintForceJacobia();

    ConstVecRef connectPointBiasAcc();

    ConstMatRef connectPointRelativeJacobia();

    ConstVecRef qpos();

    ConstVecRef qvel();

    bool isFixedBase();

private:
    void computeContactJacobia();

    void computeConstraintForceJacobia();

    void computeActiveContactPointBiasAcc();

    void
    computeConnectPointBiasAcc(); // call this function after computeConstraintForceJacobia() because _T is updated in lateral one

    pin::FrameIndex getFrameID(string frame_name);

    pin::Model _model;
    pin::Data _data;
    string _urdf_file;
    bool has_srdf;
    string _srdf_file;
    Vec _qpos;
    Vec _qvel;
    bool _isFixedBase;
    vector<string> _contactPoint_virtual_link;
    vector<pair<string, string>> _connect_point_pairs;
    vector<pair<string, Scalar>> _spring_joints;
    Mat _T, _T_dot, _Jps;
    VecXi _mask;
    Mat _Jc, _K;
    Vec _contactPointsBiasAcc, _connectPointsBiasAcc;
    Vec _actuatorsDampingForce, _jointsSpringForce;
};


#endif //TASKSPACECONTROL_ROBOTWRAPPER_H
