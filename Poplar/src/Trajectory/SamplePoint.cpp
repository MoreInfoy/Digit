//
// Created by nimapng on 7/17/21.
//

#include "Trajectory/SamplePoint.h"


SamplePoint::SamplePoint(size_t dims, POINTTYPE pointType) : _data(dims), _first_derivative(dims),
                                                             _second_derivative(dims),
                                                             _pointType(pointType),
                                                             _t(0) {
    _data.setZero();
    _first_derivative.setZero();
    _first_derivative.setZero();
}


VecRef SamplePoint::data() {
    return _data;
}

Scalar &SamplePoint::t() {
    return _t;
}

VecRef SamplePoint::first_derivative() {
    return _first_derivative;
}

VecRef SamplePoint::second_derivative() {
    return _second_derivative;
}

SamplePoint::POINTTYPE &SamplePoint::pointType() {
    return std::ref(_pointType);
}

void SamplePoint::print() {
    cout << "t = " << _t << ", data = [" << _data.transpose() << "]\n";
    cout << "t = " << _t << ", first_derivative = [" << _first_derivative.transpose() << "]\n";
    cout << "t = " << _t << ", second_derivative = [" << _second_derivative.transpose() << "]\n";
}

