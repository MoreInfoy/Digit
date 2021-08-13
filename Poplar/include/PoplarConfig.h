//
// Created by nimpng on 7/24/21.
//

#ifndef POPLARDIGIT_POPLARCONFIG_H
#define POPLARDIGIT_POPLARCONFIG_H

#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <vector>
#include <shared_mutex>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/math/quaternion.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define NeedsToAlign false

using namespace std;
using namespace Eigen;
namespace pin = pinocchio;

namespace Poplar {
    typedef double Scalar;
    typedef std::size_t Index;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vec;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Mat;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Mat_R;
    typedef Eigen::Matrix<Index, Eigen::Dynamic, Eigen::Dynamic> MatInt;
    typedef Eigen::Matrix<Index, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatInt_R;
    typedef Eigen::Matrix<Index, Eigen::Dynamic, 1> VecXi;
    typedef Eigen::Matrix<Index, 4, 1> Vec4i;
    typedef Eigen::Array<Index, 4, 1> Array4i;
    typedef Eigen::Matrix<Index, 2, 1> Vec2i;
    typedef Eigen::Array<Index, 2, 1> Array2i;
    typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VecXb;
    typedef Eigen::Matrix<Scalar, 2, 1> Vec2;
    typedef Eigen::Matrix<Scalar, 3, 1> Vec3;
    typedef Eigen::Matrix<Scalar, 4, 1> Vec4;
    typedef Eigen::Matrix<Scalar, 6, 1> Vec6;
    typedef Eigen::Matrix<Scalar, 12, 1> Vec12;
    typedef Eigen::Matrix<Scalar, 13, 1> Vec13;
    typedef Eigen::Matrix<Scalar, 3, 3> Mat3;
    typedef Eigen::Matrix<Scalar, 6, 6> Mat6;
    typedef Eigen::Matrix<Scalar, 12, 12> Mat12;
    typedef Eigen::Matrix<Scalar, 13, 13> Mat13;
    typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Mat3x;
    typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic> Mat6x;

    typedef Eigen::Ref<Vec3> Vec3Ref;
    typedef Eigen::Ref<Vec6> Vec6Ref;
    typedef Eigen::Ref<Vec> VecRef;
    typedef Eigen::Ref<Mat3> Mat3Ref;
    typedef Eigen::Ref<Mat6> Mat6Ref;
    typedef Eigen::Ref<Mat> MatRef;
    typedef Eigen::Ref<MatInt> MatIntRef;
    typedef Eigen::Ref<VecXi> VecXiRef;
    typedef const Eigen::Ref<const Vec3> ConstVec3Ref;
    typedef const Eigen::Ref<const Vec6> ConstVec6Ref;
    typedef const Eigen::Ref<const Vec> ConstVecRef;
    typedef const Eigen::Ref<const Mat> ConstMatRef;
    typedef const Eigen::Ref<const VecXi> ConstVecXiRef;
    typedef const Eigen::Ref<const MatInt> ConstMatIntRef;

}

#endif //POPLARDIGIT_POPLARCONFIG_H
