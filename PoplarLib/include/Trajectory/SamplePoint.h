//
// Created by nimapng on 7/17/21.
//

#ifndef TRAJECTORY_SAMPLEPOINT_H
#define TRAJECTORY_SAMPLEPOINT_H

#include "Trajectory/config.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <iostream>

using namespace std;

using namespace Eigen;


class SamplePoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum class POINTTYPE {
        ENDPOINT,
        NONE
    };

    SamplePoint(size_t dims, POINTTYPE pointType = POINTTYPE::NONE);

    Ref<Matrix<RealNum, Dynamic, 1>> data();

    RealNum &t();

    Ref<Matrix<RealNum, Dynamic, 1>> derivative();

    POINTTYPE &pointType();

    void print();

private:
    Matrix<RealNum, Dynamic, 1> _data;
    Matrix<RealNum, Dynamic, 1> _derivative;
    POINTTYPE _pointType;
    RealNum _t;
};


#endif //TRAJECTORY_SAMPLEPOINT_H
