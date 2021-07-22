//
// Created by nimapng on 6/9/21.
//

#include "TaskSpaceControl/TaskSpaceControl.h"
#include <fstream>

using namespace TSC;

TSC::TaskSpaceControl::TaskSpaceControl(RobotWrapper &robot) : _robot(robot), _u_dims(0) {
}

void TSC::TaskSpaceControl::addTask(TSC::Task *task) {
    _tasks.push_back(task);
}

void TaskSpaceControl::addLinearConstraint(LinearConstraints *constraints) {
    _linearConstraints.push_back(constraints);
}

void TaskSpaceControl::solve() {
/*    FullPivLU<Mat> rank_check(_robot.contactJacobia().topRows(12));
    cout << "rank of Jc: " << rank_check.rank() << endl;*/

    _u_dims = _robot.nv() + 3 * _robot.nc() + _robot.ncf();

    if (_u_dims == 0) {
        throw runtime_error("[TaskSpaceControl::getOptimalControl] decision variable dimensions is 0");
    }

    H.resize(_u_dims, _u_dims);
    g.resize(_u_dims);
    H.setZero();
    g.setZero();
    for (auto &task: _tasks) {
        task->update();
        if (task->H().rows() != _u_dims || task->H().cols() != _u_dims || task->g().size() != _u_dims) {
            throw runtime_error(task->name() + " task  matrix (H,g) dimension is wrong");
        }
        H += task->H();
        g += task->g();
    }

    /*FullPivLU<Mat> rank_check(H);
    cout << "rank of H: " << rank_check.rank() << endl;*/

    size_t nDims_cstrs = 0;
    size_t nDims_cstrs_eq = 0;
    for (auto &cstr: _linearConstraints) {
        cstr->update();
        if (cstr->isEqual()) {
            assert(cstr->c_lb() == cstr->c_ub());
            if (cstr->C().cols() != _u_dims || cstr->C().rows() != cstr->c_ub().size() ||
                cstr->C().rows() != cstr->c_lb().size()) {
                printf("_u_dims = %d, C().rows() = %ld, C().cols() = %ld, c_ub().size() = %ld, cstr->c_lb().size() = %ld\n",
                       _u_dims, cstr->C().rows(), cstr->C().cols(), cstr->c_ub().size(), cstr->c_lb().size());
                throw runtime_error(cstr->name() + " constraints matrix (C, c_lb, c_ub) dimension is wrong");
            }
            nDims_cstrs_eq += cstr->C().rows();
        } else {
            if (cstr->C().cols() != _u_dims || cstr->C().rows() != cstr->c_ub().size() ||
                cstr->C().rows() != cstr->c_lb().size()) {
                printf("_u_dims = %d, C().rows() = %ld, C().cols() = %ld, c_ub().size() = %ld, cstr->c_lb().size() = %ld\n",
                       _u_dims, cstr->C().rows(), cstr->C().cols(), cstr->c_ub().size(), cstr->c_lb().size());
                throw runtime_error(cstr->name() + " constraints matrix (C, c_lb, c_ub) dimension is wrong");
            }
            nDims_cstrs += cstr->C().rows();
        }
    }
    C.resize(nDims_cstrs, _u_dims);
    C.setZero();
    c_lb.resize(nDims_cstrs);
    c_lb.setZero();
    c_ub.resize(nDims_cstrs);
    c_ub.setZero();

    if (_robot.isFixedBase()) {
        Ce.resize(nDims_cstrs_eq, _u_dims);
        Ce.setZero();
        ce.resize(nDims_cstrs_eq);
        ce.setZero();
    } else {
        Ce.resize(nDims_cstrs_eq + 6, _u_dims);
        Ce.setZero();
        ce.resize(nDims_cstrs_eq + 6);
        ce.setZero();
        if (_robot.ncf() > 0) {
            Ce.bottomRows(6) << _robot.M().topRows(6), -_robot.contactJacobia().leftCols(
                    6).transpose(), -_robot.constraintForceJacobia().leftCols(6).transpose();
        } else {
            Ce.bottomRows(6) << _robot.M().topRows(6), -_robot.contactJacobia().leftCols(6).transpose();
        }
        ce.tail(6) = -_robot.nonLinearEffects().head(6);
    }

    size_t start_row = 0;
    size_t start_row_eq = 0;
    for (int i = 0; i < _linearConstraints.size(); i++) {
        if (_linearConstraints[i]->isEqual()) {
            Ce.middleRows(start_row_eq, _linearConstraints[i]->C().rows()) = _linearConstraints[i]->C();
            ce.segment(start_row_eq, _linearConstraints[i]->c_lb().size()) = _linearConstraints[i]->c_lb();
            start_row_eq += _linearConstraints[i]->C().rows();
        } else {
            C.middleRows(start_row, _linearConstraints[i]->C().rows()) = _linearConstraints[i]->C();
            c_lb.segment(start_row, _linearConstraints[i]->c_lb().size()) = _linearConstraints[i]->c_lb();
            c_ub.segment(start_row, _linearConstraints[i]->c_ub().size()) = _linearConstraints[i]->c_ub();
            start_row += _linearConstraints[i]->C().rows();
        }
    }


#ifdef USE_QPOASES
    int add_DoF = 6;
    if (_robot.isFixedBase()) {
        add_DoF = 0;
    }
    Eigen::Matrix<RealNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Cin(nDims_cstrs + nDims_cstrs_eq + add_DoF,
                                                                                _u_dims);
    Vec clb(nDims_cstrs + nDims_cstrs_eq + add_DoF);
    Vec cub(nDims_cstrs + nDims_cstrs_eq + add_DoF);
    Cin << C, Ce;
    clb << c_lb, ce;
    cub << c_ub, ce;

/*    Mat Par(Cin.cols() + 2, Cin.rows());
    Par << Cin.transpose(), clb.transpose(), cub.transpose();
    FullPivLU<Mat> rank_check(Par);
    Mat ParN = rank_check.image(Par).transpose();
    Eigen::Matrix<RealNum, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> _Cin = ParN.leftCols(Cin.cols());
    Vec _clb = -ParN.col(ParN.cols() - 2);
    Vec _cub = ParN.col(ParN.cols() - 1);*/

    qpOASES::QProblem *solver = new qpOASES::QProblem(_u_dims, Cin.rows(), qpOASES::HST_INDEF);
    qpOASES::Options opt;
    opt.setToMPC();
    opt.enableEqualities = qpOASES::BT_TRUE;
    opt.enableRegularisation = qpOASES::BT_TRUE;
    opt.numRegularisationSteps = 1000;
    opt.printLevel = qpOASES::PL_NONE;
    solver->setOptions(opt);

    qpOASES::int_t nWSR = 5000;
    solver->init(H.data(), g.data(), Cin.data(), NULL, NULL, clb.data(), cub.data(), nWSR);
    optimal_u.resize(_u_dims);
    if (solver->isSolved()) {
        solver->getPrimalSolution(optimal_u.data());
    } else {
        saveAllData("qp_failed.txt");
        throw runtime_error("TaskSpaceControl::solve() qp failed, related data has been saved in qp_failed.txt");
    }
    delete solver;
#else
    eiquadprog_solver.reset(_u_dims, ce.size(), 2 * c_lb.size());
    Mat Cin(C.rows() * 2, _u_dims);
    Cin << C, -C;
    Vec cin(C.rows() * 2), ce0;
    cin << -c_lb, c_ub;
    ce0 = -ce;
    auto state = eiquadprog_solver.solve_quadprog(H, g, Ce, ce0, Cin, cin, optimal_u);
    printf("solver state: %d\n", state);
    if(state != eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL) {
        saveAllData("qp_failed.txt");
        throw runtime_error("TaskSpaceControl::solve() qp failed, related data has been saved in qp_failed.txt");
    }
#endif

#ifdef PRINT_ERR
    printCstrsErr();
#endif
}

ConstVecRef TaskSpaceControl::getOptimalQacc() {
    return TSC::ConstVecRef(optimal_u.head(_robot.nv()));
}

size_t TaskSpaceControl::getInputDims() {
    _u_dims = _robot.nv() + 3 * _robot.nc() + _robot.ncf();
    return _u_dims;
}

void TaskSpaceControl::saveAllData(string file_name) {
    ofstream outfile(file_name);
    if (!outfile.is_open()) {
        throw std::runtime_error("[LinearMPC::outputAllDataToFile] The file can not be opened");
    }
    outfile << "-----------------------H------------------------" << endl
            << H << endl
            << "-----------------------g------------------------" << endl
            << g.transpose() << endl
            << "-----------------------Ce------------------------" << endl
            << Ce << endl
            << "-----------------------c_e------------------------" << endl
            << ce.transpose() << endl
            << "-----------------------EqualityCstrs Err------------------------" << endl
            << (Ce * optimal_u).transpose() - ce.transpose() << endl
            << "-----------------------C------------------------" << endl
            << C << endl
            << "-----------------------c_lb------------------------" << endl
            << c_lb.transpose() << endl
            << "-----------------------c_ub------------------------" << endl
            << c_ub.transpose() << endl
            << "-----------------------sol------------------------" << endl
            << optimal_u.transpose() << endl
            << "-----------------------optimal_torque------------------------" << endl
            << getOptimalTorque().transpose() << endl
            << "-----------------------optimal_force------------------------" << endl
            << getOptimalContactForce().transpose() << endl
            << "-----------------------spring force------------------------" << endl
            << _robot.jointsSpringForce().transpose() << endl;
    outfile.close();
}

void TaskSpaceControl::removeTask(string name) {
    for (auto it = _tasks.begin(); it != _tasks.end(); it++) {
        if ((*it)->name() == name) {
            _tasks.erase(it);
            break;
        }
    }
}

void TaskSpaceControl::removeLinearConstraint(string name) {
    for (auto it = _linearConstraints.begin(); it != _linearConstraints.end(); it++) {
        if ((*it)->name() == name) {
            _linearConstraints.erase(it);
            break;
        }
    }
}

ConstVecRef TaskSpaceControl::getOptimalTorque() {
    ConstVecRef qacc = optimal_u.head(_robot.nv());
    /*cout << "damping: " << _robot.actuatorsDamping().transpose() << endl;
cout << "qJ_vel: " << _robot.qvel().tail(_robot.na()).transpose() << endl;*/
#ifdef DAMPING_TERM
    if (_robot.ncf() > 0) {
        optimal_tau = _robot.M() * qacc + _robot.nonLinearEffects()
                      - _robot.contactJacobia().transpose() * optimal_u.segment(_robot.nv(), _robot.nc() * 3)
                      - _robot.constraintForceJacobia().transpose() * optimal_u.tail(_robot.ncf());
        optimal_tau.tail(_robot.na()) -= _robot.actuatorsDampingForce();
        optimal_tau.tail(_robot.na()) -= _robot.jointsSpringForce();
    } else {
        optimal_tau = _robot.M() * qacc + _robot.nonLinearEffects()
                      - _robot.contactJacobia().transpose() * optimal_u.segment(_robot.nv(), _robot.nc() * 3);
        optimal_tau.tail(_robot.na()) -= _robot.actuatorsDampingForce();
        optimal_tau.tail(_robot.na()) -= _robot.jointsSpringForce();
    }
#else
    if (_robot.ncf() > 0) {
        optimal_tau = _robot.M() * qacc + _robot.nonLinearEffects()
                      - _robot.contactJacobia().transpose() * optimal_u.segment(_robot.nv(), _robot.nc() * 3)
                      - _robot.constraintForceJacobia().transpose() * optimal_u.tail(_robot.ncf());
        optimal_tau.tail(_robot.na()) -= _robot.jointsSpringForce();

    } else {
        optimal_tau = _robot.M() * qacc + _robot.nonLinearEffects()
                      - _robot.contactJacobia().transpose() * optimal_u.segment(_robot.nv(), _robot.nc() * 3);
        optimal_tau.tail(_robot.na()) -= _robot.jointsSpringForce();
    }
#endif
    return ConstVecRef(optimal_tau.tail(_robot.na()));
}

ConstVecRef TaskSpaceControl::getOptimalContactForce() {
    return ConstVecRef(optimal_u.segment(_robot.nv(), _robot.nc() * 3));
}

void TaskSpaceControl::printCstrsErr() {
    for (auto &cstr: _linearConstraints) {
        cstr->errPrint(optimal_u);
    }
}



