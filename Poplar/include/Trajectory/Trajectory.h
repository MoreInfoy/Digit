//
// Created by nimapng on 7/17/21.
//

#ifndef TRAJECTORY_TRAJECTORY_H
#define TRAJECTORY_TRAJECTORY_H

#include "Trajectory/SamplePoint.h"

#include <vector>

using namespace std;

class Trajectory {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Trajectory(size_t dims);

    virtual SamplePoint operator()(Scalar t) = 0;

    virtual SamplePoint operator()(Scalar t, size_t dim_num) = 0;

    virtual bool setSamplePoints(vector<SamplePoint> &samplePoints) = 0;

protected:
    vector<SamplePoint> _samplePoints;
    size_t _dims;
};


#endif //TRAJECTORY_TRAJECTORY_H
