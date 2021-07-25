//
// Created by nimpng on 6/16/21.
//

#include "GaitScheduler/GaitScheduler.h"

GaitScheduler::GaitScheduler() {
    _gaitData.zero();
}

void GaitScheduler::run(size_t iter, const RobotState &state) {

}

const GaitData &GaitScheduler::data() {
    return _gaitData;
}
