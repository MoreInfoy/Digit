//
// Created by nimapng on 6/9/21.
//

#ifndef TASK_SPACE_CONTROL_TASKSPACECONTROL_H
#define TASK_SPACE_CONTROL_TASKSPACECONTROL_H

#include "Task/Task.h"
#include "Constraints/LinearConstraints.h"
#include "RobotWrapper/RobotWrapper.h"
#include "qpOASES.hpp"
#include "eiquadprog/eiquadprog-fast.hpp"

// #define USE_QPOASES

//#define PRINT_ERR

using namespace Poplar;

namespace TSC {
    class TaskSpaceControl {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit TaskSpaceControl(RobotWrapper &robot);

        void addTask(shared_ptr<Task> task_ptr);

        void removeTask(string name);

        bool existTask(string name);

        void addLinearConstraint(shared_ptr<LinearConstraints> constraints_ptr);

        bool existLinearConstraint(string name);

        void removeLinearConstraint(string name);

        size_t getInputDims();

        void solve();

        ConstVecRef getOptimalQacc(); // [qacc;f]

        ConstVecRef getOptimalTorque(); // tau

        ConstVecRef getOptimalContactForce(); // f

        void saveAllData(string file_name);

        void printCstrsErr();

    private:
        RobotWrapper &_robot;
        vector<shared_ptr<Task>> _tasks;
        vector<shared_ptr<LinearConstraints>> _linearConstraints;
#ifdef USE_QPOASES
        Mat_R H, C, Ce;
#else
        Mat H, C, Ce;
#endif
        Vec g, c_lb, c_ub, ce, optimal_u, optimal_tau;
        int _u_dims;

#ifndef USE_QPOASES
        eiquadprog::solvers::EiquadprogFast_status solver_state;
#else
        shared_ptr<qpOASES::QProblem> solver;
#endif
    };
}

#endif //TASK_SPACE_CONTROL_TASKSPACECONTROL_H
