//
// Created by nimpng on 6/13/21.
//

#ifndef POPLARALIENGO_TRAJECTORY_H
#define POPLARALIENGO_TRAJECTORY_H

#include <vector>

using namespace std;

template<typename DataType>
class Trajectory {
public:
    explicit Trajectory(size_t n_entries);

    void clear();

    void next();

    void push_back(DataType sample);

    const vector<DataType> &traj();

private:
    vector<DataType> _traj;
    size_t _index;
};

template<typename DataType>
Trajectory<DataType>::Trajectory(size_t n_entries) : _traj(n_entries) {

}

template<typename DataType>
void Trajectory<DataType>::clear() {
    _traj.clear();
}

template<typename DataType>
void Trajectory<DataType>::push_back(DataType sample) {
    _traj.push_back(sample);
}

template<typename DataType>
const vector<DataType> &Trajectory<DataType>::traj() {
    return ref(_traj);
}


#endif //POPLARALIENGO_TRAJECTORY_H
