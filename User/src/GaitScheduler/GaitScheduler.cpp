//
// Created by nimpng on 6/16/21.
//

#include "GaitScheduler/GaitScheduler.h"

GaitScheduler::GaitScheduler(Scalar dt) : _dt(dt),
                                          standing(5000, Vec2i(0, 0), Vec2i(5000, 5000), "standing"),
                                          walking(300, Vec2i(0, 150), Vec2i(150, 150), "standing") {
    _gaitData.zero();
    gait = &standing;
    gaitType = GAIT_TYPE::STANCE;
}

void GaitScheduler::run(size_t iter, const RobotState &state, RobotWrapper &robot) {
    fsm.run(GAIT_TYPE::WALK);
    switch (fsm.current()) {
        case GAIT_TYPE::STANCE:
            gait = &standing;
            break;
        case GAIT_TYPE::WALK:
            gait = &walking;
            break;
        case GAIT_TYPE::RUN:
            break;
        default:
            gait = &standing;
            break;
    }

    if (gait != nullptr) {
        gait->run(fsm.iterations());
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

void GaitScheduler::updateContactTable(size_t n_pre, size_t n_per) {
    _gaitData.contactTable = MatInt::Ones(2, n_pre);
    auto gaitArray = fsm.future(n_pre * n_per);
    Gait *nextGait = gait;
    GAIT_TYPE nextGaitType = fsm.current();
    Poplar::Index start = 0;
    for (size_t i = 0; i < gaitArray.size(); i += n_per) {
        if (gaitArray[i] != fsm.current() && nextGaitType != gaitArray[i]) {
            nextGaitType = gaitArray[i];

            nextGait->run(fsm.iterations() + start);
            _gaitData.contactTable.middleCols(start / n_per, (i - start) / n_per)
                    = nextGait->getContactTable((i - start) / n_per, n_per);

            switch (nextGaitType) {
                case GAIT_TYPE::STANCE:
                    nextGait = &standing;
                    break;
                case GAIT_TYPE::WALK:
                    nextGait = &walking;
                    break;
                case GAIT_TYPE::RUN:
                    break;
                default:
                    nextGait = &standing;
                    break;
            }
            start = i;
        }
    }
    nextGait->run(fsm.iterations() + start);
    _gaitData.contactTable.middleCols(start / n_per, (gaitArray.size() - start) / n_per)
            = nextGait->getContactTable((gaitArray.size() - start) / n_per, n_per);
    cout << "---------------contact table-----------------" << endl
         << _gaitData.contactTable << endl;
//    getchar();
}

GAIT_TYPE GaitScheduler::gait_type() {
    return fsm.current();
}
