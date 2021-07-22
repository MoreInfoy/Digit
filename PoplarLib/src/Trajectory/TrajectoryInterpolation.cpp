//
// Created by nimapng on 7/17/21.
//

#include "Trajectory/TrajectoryInterpolation.h"

using namespace tk;

TrajectoryInterpolation::TrajectoryInterpolation(size_t dims, Spline::spline_type line_type) : Trajectory(dims),
                                                                                               _line_type(line_type) {

}

SamplePoint TrajectoryInterpolation::operator()(double t) {
    int i_piece;
    for (i_piece = 0; i_piece < end_t.size(); i_piece++) {
        if (t < end_t[i_piece]) break;
    }
    i_piece--;
    SamplePoint samplePoint(_dims);
    samplePoint.t() = t;
    for (int i = 0; i < _dims; i++) {
        samplePoint.data()(i) = n_lines[i_piece][i](t);
        samplePoint.derivative()(i) = n_lines[i_piece][i].deriv(Spline::first_deriv, t);
    }
    return samplePoint;
}

RealNum TrajectoryInterpolation::operator()(double t, size_t dim_num) {
    int i_piece = 0;
    for (i_piece = 0; i_piece < end_t.size(); i_piece++) {
        if (t < end_t[i_piece]) break;
    }
    i_piece--;
    return n_lines[i_piece][dim_num](t);
}

bool TrajectoryInterpolation::setSamplePoints(vector<SamplePoint> &samplePoints) {
    _samplePoints = samplePoints;
    _samplePoints[0].pointType() = SamplePoint::POINTTYPE::ENDPOINT;
    _samplePoints.end()->pointType() = SamplePoint::POINTTYPE::ENDPOINT;
    n_lines.clear();
    end_t.clear();
    vector<vector<RealNum>> data(_dims);
    vector<RealNum> t;
    vector<vector<RealNum>> boundary_der(_dims);
    bool first_point = true;

    for (auto &point: samplePoints) {
        if (point.data().size() != _dims) {
            throw std::runtime_error("the Dofs of sample point does not coincide with predefined Dofs");
        }

        for (int i = 0; i < _dims; i++) {
            data[i].push_back(point.data()(i));
        }
        t.push_back(point.t());


        if (point.pointType() == SamplePoint::POINTTYPE::ENDPOINT) {
            for (int i = 0; i < _dims; i++) {
                boundary_der[i].push_back(point.derivative()(i));
            }
            if (first_point) {
                first_point = false;
            } else {
                vector<Spline> _spline_n;
                for (int i = 0; i < _dims; i++) {
                    switch (_line_type) {
                        case Spline::spline_type::linear:
                            _spline_n.push_back(Spline(t, data[i], Spline::linear, false, Spline::first_deriv,
                                                       *(boundary_der[i].end() - 1), Spline::first_deriv,
                                                       *boundary_der[i].end()));
                            break;
                        case Spline::spline_type::cspline_hermite:
                            _spline_n.push_back(Spline(t, data[i], Spline::cspline_hermite, true, Spline::first_deriv,
                                                       *(boundary_der[i].end() - 1), Spline::first_deriv,
                                                       *boundary_der[i].end()));
                            break;
                        default:
                            throw runtime_error(
                                    "[TrajectoryInterpolation::setSamplePoints] line type is not supported");
                    }
                    data[i].clear();
                    data[i].push_back(point.data()(i));
                }
                t.clear();
                t.push_back(point.t());
                n_lines.push_back(_spline_n);
            }
            end_t.push_back(point.t());
        }
    }
    return true;
}
