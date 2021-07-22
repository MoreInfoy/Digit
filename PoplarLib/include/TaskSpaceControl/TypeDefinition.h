//
// Created by nimapng on 6/9/21.
//

#ifndef TASK_SPACE_CONTROL_TYPEDEFINITION_H
#define TASK_SPACE_CONTROL_TYPEDEFINITION_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "ConfigurationTSC.h"

namespace TSC {

    using Vec = typename Eigen::Matrix<RealNum, Eigen::Dynamic, 1>;

    using VecInt = typename Eigen::Matrix<int, Eigen::Dynamic, 1>;

    using Vec3 = typename Eigen::Matrix<RealNum, 3, 1>;

    using Vec6 = typename Eigen::Matrix<RealNum, 6, 1>;

    using Mat = typename Eigen::Matrix<RealNum, Eigen::Dynamic, Eigen::Dynamic>;

    using Mat6x = typename Eigen::Matrix<RealNum, 6, Eigen::Dynamic>;

    using Mat6 = typename Eigen::Matrix<RealNum, 6, 6>;

    using Mat3 = typename Eigen::Matrix<RealNum, 3, 3>;

    typedef const Eigen::Ref<const Vec> ConstVecRef;

    typedef const Eigen::Ref<const VecInt> ConstVecIntRef;

    typedef const Eigen::Ref<const Mat> ConstMatRef;

    typedef Eigen::Ref<Vec> VecRef;

    typedef Eigen::Ref<VecInt> VecIntRef;

    typedef Eigen::Ref<Vec6> Vec6Ref;

    typedef Eigen::Ref<Vec3> Vec3Ref;

    typedef Eigen::Ref<Mat6> Mat6Ref;

    typedef Eigen::Ref<Mat3> Mat3Ref;

    typedef Eigen::Ref<Mat> MatRef;

}

#endif //TASK_SPACE_CONTROL_TYPEDEFINITION_H
