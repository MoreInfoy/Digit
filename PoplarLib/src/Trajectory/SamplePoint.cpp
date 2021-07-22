//
// Created by nimapng on 7/17/21.
//

#include "Trajectory/SamplePoint.h"


SamplePoint::SamplePoint(size_t dims, POINTTYPE pointType) : _data(dims), _derivative(dims), _pointType(pointType),
                                                             _t(0) {
    _data.setZero();
    _derivative.setZero();
}


Ref<Matrix<RealNum, Dynamic, 1>> SamplePoint::data() {
    return _data;
}

RealNum &SamplePoint::t() {
    return _t;
}

Ref<Matrix<RealNum, Dynamic, 1>> SamplePoint::derivative() {
    return _derivative;
}

SamplePoint::POINTTYPE &SamplePoint::pointType() {
    return std::ref(_pointType);
}

void SamplePoint::print() {
    cout << "t = " << _t << ", data = [" << _data.transpose() << "]\n";
}

