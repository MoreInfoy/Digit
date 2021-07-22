//
// Created by nimapng on 7/17/21.
//

#include "Trajectory/TrajectoryInterpolation.h"

int main() {
    TrajectoryInterpolation cubicBSpline(3, Spline::spline_type::cspline_hermite);
    vector<SamplePoint> points;
    for (int i = 0; i < 13; i++) {
        SamplePoint samplePoint(3);
        samplePoint.t() = 0.5 * i;
        samplePoint.data() << pow(-1, i) * 0.4, sin(1.57 * i), 0.0;
        if (i % 12 == 0)
            samplePoint.pointType() = SamplePoint::POINTTYPE::ENDPOINT;
        if (4 < i && i < 8) {
            samplePoint.derivative() << 0, 0, 0;
            samplePoint.data()(2) = 2.0;
        }
        points.push_back(samplePoint);
        printf("%f,", samplePoint.data()(2));
    }
    printf("\n");
    cubicBSpline.setSamplePoints(points);
    for (int i = 0; i < 500; i++) {
        printf("%f,", cubicBSpline(0.01 * i, 2));
    }
}
