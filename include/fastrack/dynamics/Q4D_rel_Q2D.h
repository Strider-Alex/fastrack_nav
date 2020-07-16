#ifndef Q4D_REL_Q2D_H
#define Q4D_REL_Q2D_H

#include <fastrack/dynamics/relative_dynamics.h>
#include <fastrack/state/Q4D_rel_Q2D_state.h>

#include <math.h>

using fastrack::state::Q4DRelQ2DState;

namespace fastrack
{
    namespace dynamics
    {
        class Q4DRelQ2D
            : public RelativeDynamics<Vector4d, Vector2d,
                                      Vector2d, Vector2d>
        {
        public:
            ~Q4DRelQ2D() {}
            explicit Q4DRelQ2D()
                : RelativeDynamics<Vector4d, Vector2d, Vector2d,
                                   Vector2d>() {}

            // Derived classes must be able to give the time derivative of relative state
            // as a function of current state and control of each system.
            std::unique_ptr<RelativeState<Vector4d, Vector2d>>
            Evaluate(const Vector4d &tracker_x, const Vector2d &tracker_u,
                     const Vector2d &planner_x, const Vector2d &planner_u) const;

            // Derived classes must be able to compute an optimal control given
            // the gradient of the value function at the relative state specified
            // by the given system states, provided abstract control bounds.
            Vector2d OptimalControl(
                const Vector4d &tracker_x, const Vector2d &planner_x,
                const RelativeState<Vector4d, Vector2d> &value_gradient,
                const ControlBound<Vector2d> &tracker_u_bound,
                const ControlBound<Vector2d> &planner_u_bound) const;
        }; //\class Q4DRelQ2D

        Vector2d Q4DRelQ2D::OptimalControl(
                const Vector4d &tracker_x, const Vector2d &planner_x,
                const RelativeState<Vector4d, Vector2d> &value_gradient,
                const ControlBound<Vector2d> &tracker_u_bound,
                const ControlBound<Vector2d> &planner_u_bound) const
        {
            ROS_INFO_STREAM("########################START Q4DRelQ2D::OptimalControl " << std::endl);
            VectorXd grad = value_gradient.ToVector();
            Vector2d control = tracker_u_bound.ProjectToSurface(-Vector2d(grad(2),grad(3)));
            return control;
        }

        std::unique_ptr<RelativeState<Vector4d, Vector2d>>
            Q4DRelQ2D::Evaluate(const Vector4d &tracker_x, const Vector2d &tracker_u,
                     const Vector2d &planner_x, const Vector2d &planner_u) const
        {
            return std::unique_ptr<Q4DRelQ2DState>(
                new Q4DRelQ2DState(Vector4d()));

        }

    } // namespace dynamics
} // namespace fastrack

#endif