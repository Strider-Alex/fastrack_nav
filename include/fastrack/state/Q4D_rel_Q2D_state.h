#ifndef Q4D_REL_Q2D_STATE_H
#define Q4D_REL_Q2D_STATE_H

#include <fastrack/utils/types.h>

namespace fastrack
{
    namespace state
    {

        class Q4DRelQ2DState : public RelativeState<Vector4d, Vector2d>
        {
        public:
            virtual ~Q4DRelQ2DState() {}\
            Q4DRelQ2DState(const VectorXd x)
            {
                relative_state_ = x;
            }
            Q4DRelQ2DState(const Vector4d tracker_state, const Vector2d planner_state)
            {
                relative_state_ = Vector4d(tracker_state(0)-planner_state(0),
                    tracker_state(1)-planner_state(1),
                    tracker_state(2),
                    tracker_state(3));
            }

            // Convert from/to VectorXd.
            void FromVector(const VectorXd &x){
                relative_state_ = x;
            }
            VectorXd ToVector() const {
                return relative_state_;
            }

        private:
            Vector4d relative_state_;
        }; //\class RelativeState

    } // namespace state
} // namespace fastrack

#endif