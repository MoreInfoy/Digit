//
// Created by nimapng on 8/5/21.
//

#ifndef POPLARDIGIT_GAIT_H
#define POPLARDIGIT_GAIT_H

#include "PoplarConfig.h"

using namespace Poplar;

enum GAIT_TYPE {
    STANCE,
    WALK,
    RUN
};

class Gait {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Gait(size_t nSegment, Vec2i offset, Vec2i durations, const std::string &name);

    ~Gait();

    Vec2 getContactState();

    Vec2 getSwingState();

    ConstMatIntRef getContactTable(size_t n_pre, size_t n_per = 1);

    void run(size_t iter);

    Vec2 getStanceTime(double dtMPC) const;

    Vec2 getSwingTime(double dtMPC) const;

    size_t getCurrentGaitPhase() const;

    Scalar eps();

private:
    size_t _nSegment;
    Poplar::Array2i _offsets;   // offset in mpc segments
    Poplar::Array2i _durations; // duration of step in mpc segments
    std::string _name;

    Array2d _offsetsDouble; // offsets in phase (0 to 1)
    Array2d _durationsDouble; // durations in phase (0 to 1)

    MatInt _contact_table;
    size_t _iteration;
    Scalar _phase;

    Array2d swingPhase;
    Array2d stancePhase;
};


#endif //POPLARDIGIT_GAIT_H
