//
// Created by nimpng on 7/25/21.
//

#include "Planner/FloatingBasePlanner.h"

FloatingBasePlanner::FloatingBasePlanner() {

}

void FloatingBasePlanner::plan(size_t iter, const RobotState &state, const GaitData &gaitData, Tasks &tasks) {
    tasks.floatingBaseTask.pos(2) = 0.892442 + 0.08 * sin(0.006 * iter);
}
