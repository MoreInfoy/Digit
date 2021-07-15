//
// Created by nimapng on 6/25/21.
//

#include "RobotWrapper/RobotWrapper.h"

RobotWrapper::RobotWrapper(string urdf_file, bool isFixedBase) : _urdf_file(urdf_file),
                                                                 has_srdf(false),
                                                                 _isFixedBase(isFixedBase) {
    if (isFixedBase) {
        pin::urdf::buildModel(urdf_file, _model);
    } else {
        pin::urdf::buildModel(urdf_file, pin::JointModelFreeFlyer(), _model);
    }
    _data = pin::Data(_model);
    for (pin::JointIndex joint_id = 0; joint_id < (pin::JointIndex) _model.njoints; ++joint_id)
        std::cout << std::setw(24) << std::left
                  << _model.names[joint_id] << ":" << _model.getJointId(_model.names[joint_id]) << endl;
    cout << "Joint damping: " << _model.damping.transpose() << endl;
    cout << "Joint friction: " << _model.friction.transpose() << endl;

    _qpos = Vec::Zero(nq());
    _qvel = Vec::Zero(nv());
    _jointsSpringForce = Vec::Zero(na());
    _actuatorsDampingForce = Vec::Zero(na());
}

RobotWrapper::RobotWrapper(string urdf_file, string srdf_file, bool isFixedBase) : _urdf_file(urdf_file),
                                                                                   has_srdf(true),
                                                                                   _srdf_file(srdf_file),
                                                                                   _isFixedBase(isFixedBase) {
    if (isFixedBase) {
        pin::urdf::buildModel(urdf_file, _model);
    } else {
        pin::urdf::buildModel(urdf_file, pin::JointModelFreeFlyer(), _model);
    }
    pin::srdf::loadReferenceConfigurations(_model, srdf_file, false);
    pin::srdf::loadRotorParameters(_model, srdf_file, false);
    _data = pin::Data(_model);
    for (pin::JointIndex joint_id = 0; joint_id < (pin::JointIndex) _model.njoints; ++joint_id)
        std::cout << std::setw(24) << std::left
                  << _model.names[joint_id] << ":" << _model.getJointId(_model.names[joint_id]) << endl;
    cout << "Joint damping: " << _model.damping.transpose() << endl;
    cout << "Joint friction: " << _model.friction.transpose() << endl;
    cout << "rotor gear_ratio: " << _model.rotorGearRatio.transpose() << endl;
    cout << "rotor inertia: " << _model.rotorInertia.transpose() << endl;

    _qpos = Vec::Zero(nq());
    _qvel = Vec::Zero(nv());
    _jointsSpringForce = Vec::Zero(na());
    _actuatorsDampingForce = Vec::Zero(na());
}

ConstRefVec RobotWrapper::homeConfigurations() {
    if (has_srdf) {
        return _model.referenceConfigurations["home"];
    } else {
        throw runtime_error("no SRDF file has been given for home configuration");
    }
}

ConstRefVec RobotWrapper::gear_ratio() {
    if (has_srdf) {
        return _model.rotorGearRatio;
    } else {
        throw runtime_error("no SRDF file has been given for rotorGearRatio");
    }
}

ConstRefVec RobotWrapper::rotorInertia() {
    if (has_srdf) {
        return _model.rotorInertia;
    } else {
        throw runtime_error("no SRDF file has been given for rotorInertia");
    }
}

ConstRefVec RobotWrapper::actuatorsEffortLimit() {
    return ConstRefVec(_model.effortLimit.tail(na()));
}

ConstRefVec RobotWrapper::actuatorsDamping() {
    return ConstRefVec(_model.damping.tail(na()));
}

ConstRefVec RobotWrapper::actuatorsFriction() {
    return ConstRefVec(_model.friction.tail(na()));
}

void RobotWrapper::setSpringJoints(const vector<pair<string, Scalar>> spring_joints) {
    _spring_joints = spring_joints;
}

ConstRefVec RobotWrapper::actuatorsDampingForce() {
    return _actuatorsDampingForce;
}


ConstRefVec RobotWrapper::jointsSpringForce() {
    return _jointsSpringForce;
}

void RobotWrapper::computeAllData(ConstRefVec qpos, ConstRefVec qvel, const VecXi &mask) {
    _qpos = qpos;
    _qvel = qvel;
    pin::normalize(_model, _qpos);
    setContactMask(mask);

    pinocchio::computeAllTerms(_model, _data, _qpos, _qvel);
    _data.M.triangularView<Eigen::StrictlyLower>()
            = _data.M.transpose().triangularView<Eigen::StrictlyLower>();
#ifdef CONSIDER_ROTOR_INERTIA
    _data.M.diagonal() += _model.rotorInertia;
#endif
    pin::computeMinverse(_model, _data, _qpos);
    _data.Minv.triangularView<Eigen::StrictlyLower>() = _data.Minv.transpose().triangularView<Eigen::StrictlyLower>();

    pinocchio::updateFramePlacements(_model, _data);
    pinocchio::centerOfMass(_model, _data, _qpos, _qvel, Eigen::VectorXd::Zero(_model.nv));
    pinocchio::ccrba(_model, _data, _qpos, _qvel);

    computeContactJacobia();
    computeActiveContactPointBiasAcc();

    if (_connect_point_pairs.size() > 0) {
        if (_Jps.rows() != 6 * _connect_point_pairs.size()) {
            _Jps.resize(6 * _connect_point_pairs.size(), _model.nv);
        }
        Mat6x Js, Jp;
        for (int i = 0; i < _connect_point_pairs.size(); i++) {
            analyticalJacobia(_connect_point_pairs[i].first, Jp);
            analyticalJacobia(_connect_point_pairs[i].second, Js);
            _Jps.middleRows(6 * i, 6) = Jp - Js;
        }
    }
    if (!_spring_joints.empty()) {
        if (_isFixedBase) {
            for (auto &sj: _spring_joints) {
                pin::JointIndex id = _model.getJointId(sj.first) - 1;
                _jointsSpringForce(id) = -sj.second * _qpos(id);
            }
        } else {
            for (auto &sj: _spring_joints) {
                pin::JointIndex id = _model.getJointId(sj.first) - 2;
                _jointsSpringForce(id) = -sj.second * _qpos(id + 7);
            }
        }
    }
    _actuatorsDampingForce = -actuatorsDamping().cwiseProduct(_qvel.tail(na()));
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

pin::Motion RobotWrapper::frame_6dVel_localWorldAligned(string frame_name) {
    return pin::getFrameVelocity(_model, _data, getFrameID(frame_name), pin::LOCAL_WORLD_ALIGNED);
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
            _contactPointsBiasAcc.segment(3 * ci, 3) = acc.linear()
                                                       + 0.0 * frame_6dVel_localWorldAligned(
                    _contactPoint_virtual_link[i]).linear();
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

bool RobotWrapper::isFixedBase() {
    return _isFixedBase;
}

void RobotWrapper::setConnectVirtualLink(const vector<pair<string, string>> &link_pairs) {
    _connect_point_pairs = link_pairs;
}

void RobotWrapper::setConstraintForceSubspace(ConstRefMat T, ConstRefMat T_dot) {
    _T = T;
    _T_dot = T_dot;
}

void RobotWrapper::computeConstraintForceJacobia() {
    /*if (_connect_point_pairs.size() > 0) {
        Mat X(6, 6);
        X.setIdentity();
        for (int i = 0; i < _connect_point_pairs.size(); i++) {
            X.topLeftCorner<3, 3>() = frame_pose(_connect_point_pairs[i].first).rotation();
            X.bottomRightCorner<3, 3>() = X.topLeftCorner<3, 3>();
            _T.middleRows(6 * i, 6) = X * _T.middleRows(6 * i, 6);
        }
        _K.noalias() = _T.transpose() * _Jps;
    }*/
    _K.noalias() = _T.transpose() * _Jps;
}

void RobotWrapper::computeConnectPointBiasAcc() {
    if (_connect_point_pairs.size() > 0) {
        Vec acc_ps(6 * _connect_point_pairs.size());

        for (int i = 0; i < _connect_point_pairs.size(); i++) {
            auto acc_p = frame_6dClassicalAcc_world(_connect_point_pairs[i].first);
            auto acc_s = frame_6dClassicalAcc_world(_connect_point_pairs[i].second);
            acc_ps.segment(6 * i, 6) = acc_p.toVector() - acc_s.toVector();
        }
        _connectPointsBiasAcc = _T.transpose() * acc_ps + _T_dot.transpose() * _Jps * _qvel;
//        cout << "_connectPointsBiasAcc: " << _connectPointsBiasAcc.transpose() << endl;
    }
}

ConstRefMat RobotWrapper::constraintForceJacobia() {
    return RobotWrapperMath::ConstRefMat(_K);
}

ConstRefVec RobotWrapper::connectPointBiasAcc() {
    return RobotWrapperMath::ConstRefVec(_connectPointsBiasAcc);
}

int RobotWrapper::ncf() {
    return (_T.cols() > 0 ? _T.cols() : 0);
}

ConstRefMat RobotWrapper::connectPointRelativeJacobia() {
    return RobotWrapperMath::ConstRefMat(_Jps);
}

void RobotWrapper::computeClosedChainTerm() {
    if (6 * _connect_point_pairs.size() != _T.rows() || 6 * _connect_point_pairs.size() != _T_dot.rows()) {
        throw runtime_error("constraint-force subspace matrix (T, Tdot) is not compatible with connect-point pairs");
    }
    computeConstraintForceJacobia();
    computeConnectPointBiasAcc();
}











