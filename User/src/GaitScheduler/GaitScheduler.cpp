//
// Created by nimpng on 6/16/21.
//

#include "GaitScheduler/GaitScheduler.h"

GaitScheduler::GaitScheduler(Scalar dt) : _dt(dt),
                                          standing(500, Vec2i(0, 0), Vec2i(500, 500), "standing"),
                                          walking(2000, Vec2i(0, 1000), Vec2i(1000, 1000), "standing") {
    _gaitData.zero();
    gait = &standing;
}

void GaitScheduler::run(size_t iter, const RobotState &state) {
    gait = &walking;
    if (gait != nullptr) {
        gait->run(iter);
        Vec2 swPhase = gait->getSwingState();
        Vec2 stPhase = gait->getContactState();
        _gaitData.swingTime = gait->getSwingTime(_dt);
        _gaitData.stanceTime = gait->getStanceTime(_dt);

        for (int i(0); i < 2; i++) {
            if (swPhase[i] < gait->eps() && stPhase[i] >= gait->eps()) { // leg in stance
                _gaitData.stanceTimeRemain[i] = _gaitData.stanceTime[i] * (1 - stPhase[i]);
                if (_gaitData.stanceTimeRemain[i] == 0.) // stance finished, swing begin
                    _gaitData.swingTimeRemain[i] = _gaitData.swingTime[i];
                else
                    _gaitData.swingTimeRemain[i] = 0.;
            } else { // leg in swing
                _gaitData.swingTimeRemain[i] = _gaitData.swingTime[i] * (1 - swPhase[i]);
                _gaitData.stanceTimeRemain[i] = 0.;
            }
        }
    }

}

const GaitData &GaitScheduler::data() {
    return _gaitData;
}

ConstMatIntRef GaitScheduler::contactTable(size_t n_pre, size_t n_per) {
    return gait->getContactTable(n_pre, n_per);
}
