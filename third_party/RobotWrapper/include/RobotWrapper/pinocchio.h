//
// Created by nimapng on 6/25/21.
//

#ifndef TASKSPACECONTROL_PINOCCHIO_H
#define TASKSPACECONTROL_PINOCCHIO_H

#include <vector>
#include <map>
#include <iostream>
#include <string>
#include <vector>
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

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;
namespace pin = pinocchio;

namespace RobotWrapperMath {
    typedef double Scalar;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vec;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Mat;
    typedef Eigen::VectorXi VecXi;
    typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VecXb;

    typedef Eigen::Matrix<Scalar, 3, 1> Vec3;
    typedef Eigen::Matrix<Scalar, 6, 1> Vec6;
    typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Mat3x;
    typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic> Mat6x;

    typedef Eigen::Ref<Vec3> RefVec3;
    typedef const Eigen::Ref<const Vec3> ConstRefVec3;

    typedef Eigen::Ref<Vec> RefVec;
    typedef const Eigen::Ref<const Vec> ConstRefVec;

    typedef Eigen::Ref<Mat> RefMat;
    typedef const Eigen::Ref<const Mat> ConstRefMat;

    typedef std::size_t Index;
}

#endif //TASKSPACECONTROL_PINOCCHIO_H
