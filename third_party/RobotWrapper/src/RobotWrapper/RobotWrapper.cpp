//
// Created by nimapng on 6/25/21.
//

#include "RobotWrapper/RobotWrapper.h"

RobotWrapper::RobotWrapper(string urdf_file, bool isFixedBase) : _urdf_file(urdf_file),
                                                                 _isFixedBase(isFixedBase) {
    if (isFixedBase) {
        pin::urdf::buildModel(urdf_file, _model);
    } else {
        pin::urdf::buildModel(urdf_file, pin::JointModelFreeFlyer(), _model);
    }
    _data = pin::Data(_model);
    for (pin::JointIndex joint_id = 0; joint_id < (pin::JointIndex) _model.njoints; ++joint_id)
        std::cout << std::setw(24) << std::left
                  << _model.names[joint_id] << endl;
}

ConstRefVec RobotWrapper::actuatorsEffortLimit() {
    return ConstRefVec(_model.effortLimit.tail(na()));
}

void RobotWrapper::computeAllData(ConstRefVec qpos, ConstRefVec qvel, const VecXi &mask) {
    _qpos = qpos;
    _qvel = qvel;
    pin::normalize(_model, _qpos);
    setContactMask(mask);

    pinocchio::computeAllTerms(_model, _data, _qpos, _qvel);
    _data.M.triangularView<Eigen::StrictlyLower>()
            = _data.M.transpose().triangularView<Eigen::StrictlyLower>();
    pin::computeMinverse(_model, _data, _qpos);
    _data.Minv.triangularView<Eigen::StrictlyLower>() = _data.Minv.transpose().triangularView<Eigen::StrictlyLower>();

    pinocchio::updateFramePlacements(_model, _data);
    pinocchio::centerOfMass(_model, _data, _qpos, _qvel, Eigen::VectorXd::Zero(_model.nv));
    pinocchio::ccrba(_model, _data, _qpos, _qvel);

    computeContactJacobia();
    computeActiveContactPointBiasAcc();
}

Vec3 RobotWrapper::CoM_pos() {
    return _data.com[0];
}

Vec3 RobotWrapper::CoM_vel() {
    return _data.vcom[0];
}

Vec3 RobotWrapper::CoM_acc() {
    return _data.acom[0];
}

const Mat3x &RobotWrapper::Jacobia_CoM() {
    return _data.Jcom;
}

void RobotWrapper::Jacobia_local(string frame_name, Mat6x &J) {
    if (J.size() != 6 * _model.nv) {
        J.resize(6, _model.nv);
    }
    J.setZero();
    pin::getFrameJacobian(_model, _data, getFrameID(frame_name), pin::LOCAL, J);
}

void RobotWrapper::Jacobia_world(string frame_name, Mat6x &J) {
    if (J.size() != 6 * _model.nv) {
        J.resize(6, _model.nv);
    }
    J.setZero();
    pin::getFrameJacobian(_model, _data, getFrameID(frame_name), pin::WORLD, J);
}

void RobotWrapper::analyticalJacobia(string frame_name, Mat6x &J) {
    if (J.size() != 6 * _model.nv) {
        J.resize(6, _model.nv);
    }
    J.setZero();
    pin::getFrameJacobian(_model, _data, getFrameID(frame_name), pin::LOCAL_WORLD_ALIGNED, J);
}

pin::FrameIndex RobotWrapper::getFrameID(string frame_name) {
    assert(_model.existFrame(frame_name));
    return _model.getFrameId(frame_name);
}

ConstRefMat RobotWrapper::M() {
    return ConstRefMat(_data.M);
}

ConstRefMat RobotWrapper::Minv() {
    return ConstRefMat(_data.Minv);
}

ConstRefVec RobotWrapper::nonLinearEffects() {
    return ConstRefVec(_data.nle);
}

const pin::SE3 &RobotWrapper::frame_pose(string frame_name) {
    return _data.oMf[getFrameID(frame_name)];
}

pin::Motion RobotWrapper::frame_6dVel_local(string frame_name) {

    return pin::getFrameVelocity(_model, _data, getFrameID(frame_name), pin::LOCAL);
}

pin::Motion RobotWrapper::frame_6dAcc_local(string frame_name) {
    return pin::getFrameAcceleration(_model, _data, getFrameID(frame_name), pin::LOCAL);
}

pin::Motion RobotWrapper::frame_6dClassicalAcc_local(string frame_name) {
    return pin::getFrameClassicalAcceleration(_model, _data, getFrameID(frame_name), pin::LOCAL);
}

pin::Motion RobotWrapper::frame_6dClassicalAcc_world(string frame_name) {
    return pin::getFrameClassicalAcceleration(_model, _data, getFrameID(frame_name), pin::WORLD);
}

int RobotWrapper::nq() {
    return _model.nq;
}

int RobotWrapper::nv() {
    return _model.nv;
}

int RobotWrapper::na() {
    if (_isFixedBase) return _model.nv;
    else return _model.nv - 6;
}

int RobotWrapper::nc() {
    return _mask.cwiseEqual(1).cast<int>().sum();
}

void RobotWrapper::setContactVirtualLink(const vector<string> &names) {
    _contactPoint_virtual_link = names;
}

void RobotWrapper::setContactMask(const VecXi &mask) {
    assert(mask.size() == _contactPoint_virtual_link.size());
    _mask = mask;
}

void RobotWrapper::computeContactJacobia() {
    if (_Jc.rows() != 3 * nc() || _Jc.cols() != nv()) {
        _Jc.resize(3 * nc(), nv());
    }
    Mat6x Jci;
    int ci = 0;
    for (int i = 0; i < _mask.size(); i++) {
        if (_mask(i) == 1) {
            analyticalJacobia(_contactPoint_virtual_link[i], Jci);
            _Jc.middleRows(3 * ci, 3) = Jci.topRows(3);
//            cout << "---------------" << i << "------------------" << endl;
//            cout << Jci << endl;
//            analyticalJacobia("l_foot", Jci);
//            cout << "---------------l_foot------------------" << endl;
//            cout << Jci << endl;
            ci++;
        }
    }
}

ConstRefMat RobotWrapper::contactJacobia() {
    assert(_mask.size() == _contactPoint_virtual_link.size());
    return ConstRefMat(_Jc);
}

const VecXi &RobotWrapper::contactMask() {
    return _mask;
}

void RobotWrapper::computeActiveContactPointBiasAcc() {
    if (_contactPointsBiasAcc.size() != 3 * nc())
        _contactPointsBiasAcc.resize(3 * nc());
    int ci = 0;
    for (int i = 0; i < _mask.size(); i++) {
        if (_mask(i) == 1) {
            auto acc = frame_6dClassicalAcc_world(_contactPoint_virtual_link[i]);
            _contactPointsBiasAcc.segment(3 * ci, 3) = acc.linear();
            if (acc.linear().hasNaN())
                throw runtime_error(_contactPoint_virtual_link[i] + " get wrong bias acc");
            ci++;
        }
    }
}

ConstRefVec RobotWrapper::activeContactPointBiasAcc() {
    return ConstRefVec(_contactPointsBiasAcc);
}

ConstRefVec RobotWrapper::qpos() {
    return ConstRefVec(_qpos);
}

ConstRefVec RobotWrapper::qvel() {
    return ConstRefVec(_qvel);
}

const Mat6x &RobotWrapper::momentumJacobia() {
    return _data.Ag;
}

Vec6 RobotWrapper::momentumTimeVariation() {
    return pinocchio::computeCentroidalMomentumTimeVariation(_model, _data).toVector();
}










