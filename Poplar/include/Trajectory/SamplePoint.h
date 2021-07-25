//
// Created by nimapng on 7/17/21.
//

#ifndef TRAJECTORY_SAMPLEPOINT_H
#define TRAJECTORY_SAMPLEPOINT_H

#include "PoplarConfig.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#include <iostream>

using namespace Poplar;

class SamplePoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum class POINTTYPE {
        ENDPOINT,
        NONE
    };

    SamplePoint(size_t dims, POINTTYPE pointType = POINTTYPE::NONE);

    VecRef data();

    Scalar &t();

    VecRef first_derivative();

    VecRef second_derivative();

    POINTTYPE &pointType();

    void print();

private:
    Vec _data;
    Vec _first_derivative, _second_derivative;
    POINTTYPE _pointType;
    Scalar _t;
};


#endif //TRAJECTORY_SAMPLEPOINT_H
