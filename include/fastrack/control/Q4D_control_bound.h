/*
 * Copyright (c) 2018, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Box bound for quadrotor controls.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef Q4D_CONTROL_BOUND_H
#define Q4D_CONTROL_BOUND_H

#include <fastrack/control/control_bound.h>
#include <fastrack/control/scalar_bound_interval.h>

#include <math.h>

namespace fastrack
{
    namespace control
    {

        class Q4DControlBoundBox : public ControlBound<Vector2d>
        {
        public:
            ~Q4DControlBoundBox() {}
            explicit Q4DControlBoundBox() : acceration_interval_(0, 0),
                                            turn_rate_interval_(0, 0) {}
            explicit Q4DControlBoundBox(const Vector2d &min,
                                        const Vector2d &max)
                : acceration_interval_(min(0), max(0)),
                  turn_rate_interval_(min(1), max(1)) {}

            // Assume params are laid out as follows:
            // [min a, min turn_rate, max a, max turn_rate]
            explicit Q4DControlBoundBox(const std::vector<double> &params)
                : acceration_interval_(params[0], params[2]),
                  turn_rate_interval_(params[1], params[3]) {}

            // Custom definition of copy-assign operator.
            Q4DControlBoundBox &operator=(const Q4DControlBoundBox &other)
            {
                if (&other == this)
                    return *this;

                acceration_interval_ = other.acceration_interval_;
                turn_rate_interval_ = other.turn_rate_interval_;
                return *this;
            }

            // Derived classes must be able to check whether a query is inside the
            // bound.
            inline bool Contains(const Vector2d &query) const
            {
                return acceration_interval_.Contains(query(0)) &&
                       turn_rate_interval_.Contains(query(1));
            }

            // Derived classes must be able to compute the projection of a vector
            // (represented as the templated type) onto the surface of the bound.
            // NOTE: this is basically solving an LP with the bound as the feasible
            // set and the query as the coefficients.
            inline Vector2d ProjectToSurface(
                const Vector2d &query) const
            {
                return Vector2d(acceration_interval_.ProjectToSurface(query(0)),
                                turn_rate_interval_.ProjectToSurface(query(1)));
            }

        private:
            // ScalarBoundIntervals for each control variable.
            ScalarBoundInterval acceration_interval_;
            ScalarBoundInterval turn_rate_interval_;
        }; //\class ControlBound

    } // namespace control
} // namespace fastrack

#endif
