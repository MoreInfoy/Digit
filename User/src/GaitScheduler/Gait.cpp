//
// Created by nimapng on 8/5/21.
//

#include "GaitScheduler/Gait.h"


// Offset - Duration Gait
Gait::Gait(size_t nSegment, Vec2i offsets, Vec2i durations, const std::string &name) :
        _nSegment(nSegment),
        _offsets(offsets.array()),
        _durations(durations.array()),
        _iteration(0),
        _phase(0) {

    _name = name;

    _offsetsDouble = offsets.cast<double>() / (double) nSegment;
    _durationsDouble = durations.cast<double>() / (double) nSegment;

    swingPhase.setZero();
    stancePhase.setZero();
}


Gait::~Gait() {
}

Vec2 Gait::getContactState() {
    return stancePhase.matrix();
}

Vec2 Gait::getSwingState() {
    return swingPhase.matrix();
}


ConstMatIntRef Gait::getContactTable(size_t n_pre, size_t n_per) {
    _contact_table.resize(2, n_pre);
    for (int i = 0; i < n_pre; i++) {
        size_t iter = (i * n_per + _iteration + 1) % _nSegment;
        Poplar::Array2i progress = iter - _offsets;
        for (int j = 0; j < 2; j++) {
            if (progress[j] < 0) progress[j] += _nSegment;
            if (progress[j] < _durations[j])
                _contact_table(j, i) = 1;
            else
                _contact_table(j, i) = 0;
        }
    }
    return _contact_table;
}


void Gait::run(size_t iter) {
    _iteration = iter % _nSegment;
    _phase = double(_iteration) / double(_nSegment);

    Array2d leg_process = _phase - _offsetsDouble;
    for (int i(0); i < 2; i++) {
        if (leg_process[i] < 0.)
            leg_process[i] += 1;

        if (leg_process[i] <= _durationsDouble[i]) {
            stancePhase[i] = leg_process[i] / _durationsDouble[i];
            swingPhase[i] = 0.;
        } else {
            swingPhase[i] = (leg_process[i] - _durationsDouble[i]) / (1 - _durationsDouble[i]);
            stancePhase[i] = 0.;
        }
    }
}

size_t Gait::getCurrentGaitPhase() const {
    return _iteration;
}


Vec2 Gait::getSwingTime(double dt) const {
    return dt * (_nSegment - _durations).cast<double>();
}


Vec2 Gait::getStanceTime(double dt) const {
    return dt * _durations.cast<double>();
}

Scalar Gait::eps() {
    return 1e-6;
}
