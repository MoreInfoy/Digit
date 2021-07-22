//
// Created by nimapng on 7/17/21.
//

#ifndef TRAJECTORY_TRAJECTORY_H
#define TRAJECTORY_TRAJECTORY_H

#include "Trajectory/SamplePoint.h"
#include "Trajectory/config.h"

#include <vector>

using namespace std;

class Trajectory {
public:
    Trajectory(size_t dims);

    virtual SamplePoint operator()(RealNum t) = 0;

    virtual RealNum operator()(RealNum t, size_t dim_num) = 0;

    virtual bool setSamplePoints(vector<SamplePoint> &samplePoints) = 0;

protected:
    vector<SamplePoint> _samplePoints;
    size_t _dims;
};


#endif //TRAJECTORY_TRAJECTORY_H
