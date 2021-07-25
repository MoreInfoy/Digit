//
// Created by nimapng on 7/23/21.
//



/*
 * Spline.h
 *
 * simple cubic Spline interpolation library without external
 * dependencies
 *
 * ---------------------------------------------------------------------
 * Copyright (C) 2011, 2014, 2016, 2021 Tino Kluge (ttk448 at gmail.com)
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------
 *
 */


#ifndef POPLARLIB_SPLINE_H
#define POPLARLIB_SPLINE_H

#include <cstdio>
#include <cassert>
#include <cmath>
#include <vector>
#include <algorithm>
#include "PoplarConfig.h"

using namespace Poplar;

#ifdef HAVE_SSTREAM
#include <sstream>
#include <string>
#endif // HAVE_SSTREAM



// unnamed namespace only because the implementation is in this
// header file and we don't want to export symbols to the obj files

namespace tk {

// Spline interpolation
    class Spline {
    public:
        // Spline types
        enum spline_type {
            linear = 10,            // linear interpolation
            cspline = 30,           // cubic splines (classical C^2)
            cspline_hermite = 31    // cubic hermite splines (local, only C^1)
        };

        // boundary condition type for the Spline end-points
        enum bd_type {
            first_deriv = 1,
            second_deriv = 2,
            not_a_knot = 3
        };

    protected:
        std::vector<Scalar> m_x, m_y;            // x,y coordinates of points
        // interpolation parameters
        // f(x) = a_i + b_i*(x-x_i) + c_i*(x-x_i)^2 + d_i*(x-x_i)^3
        // where a_i = y_i, or else it won't go through grid points
        std::vector<Scalar> m_b, m_c, m_d;        // Spline coefficients
        Scalar m_c0;                            // for left extrapolation
        spline_type m_type;
        bd_type m_left, m_right;
        Scalar m_left_value, m_right_value;
        bool m_made_monotonic;

        void set_coeffs_from_b();               // calculate c_i, d_i from b_i
        size_t find_closest(Scalar x) const;    // closest idx so that m_x[idx]<=x

    public:
        // default constructor: set boundary condition to be zero curvature
        // at both ends, i.e. natural splines
        Spline();

        Spline(const std::vector<Scalar> &X, const std::vector<Scalar> &Y,
               spline_type type = cspline,
               bool make_monotonic = false,
               bd_type left = second_deriv, Scalar left_value = 0.0,
               bd_type right = second_deriv, Scalar right_value = 0.0
        );


        // modify boundary conditions: if called it must be before set_points()
        void set_boundary(bd_type left, Scalar left_value,
                          bd_type right, Scalar right_value);

        // set all data points (cubic_spline=false means linear interpolation)
        void set_points(const std::vector<Scalar> &x,
                        const std::vector<Scalar> &y,
                        spline_type type = cspline);

        // adjust coefficients so that the Spline becomes piecewise monotonic
        // where possible
        //   this is done by adjusting slopes at grid points by a non-negative
        //   factor and this will break C^2
        //   this can also break boundary conditions if adjustments need to
        //   be made at the boundary points
        // returns false if no adjustments have been made, true otherwise
        bool make_monotonic();

        // evaluates the Spline at point x
        Scalar operator()(Scalar x) const;

        Scalar deriv(int order, Scalar x) const;

        // solves for all x so that: Spline(x) = y
        std::vector<Scalar> solve(Scalar y, bool ignore_extrapolation = true) const;

        // returns the input data points
        std::vector<Scalar> get_x() const;

        std::vector<Scalar> get_y() const;

        Scalar get_x_min() const;

        Scalar get_x_max() const;

#ifdef HAVE_SSTREAM
        // Spline info string, i.e. Spline type, boundary conditions etc.
std::string info() const;
#endif // HAVE_SSTREAM

    };


    namespace internal {

// band matrix solver
        class band_matrix {
        private:
            std::vector<std::vector<Scalar> > m_upper;  // upper band
            std::vector<std::vector<Scalar> > m_lower;  // lower band
        public:
            band_matrix();                            // constructor
            band_matrix(int dim, int n_u, int n_l);       // constructor
            ~band_matrix();                        // destructor
            void resize(int dim, int n_u, int n_l);      // init with dim,n_u,n_l
            int dim() const;                             // matrix dimension
            int num_upper() const;

            int num_lower() const;
            // access operator
            Scalar &operator()(int i, int j);            // write
            Scalar operator()(int i, int j) const;      // read
            // we can store an additional diagonal (in m_lower)
            Scalar &saved_diag(int i);

            Scalar saved_diag(int i) const;

            void lu_decompose();

            std::vector<Scalar> r_solve(const std::vector<Scalar> &b) const;

            std::vector<Scalar> l_solve(const std::vector<Scalar> &b) const;

            std::vector<Scalar> lu_solve(const std::vector<Scalar> &b,
                                          bool is_lu_decomposed = false);

        };

        Scalar get_eps();

        std::vector<Scalar> solve_cubic(Scalar a, Scalar b, Scalar c, Scalar d,
                                         int newton_iter = 0);
    } // namespace internal
}
#endif //POPLARLIB_SPLINE_H
