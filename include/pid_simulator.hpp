/*!
 * @brief: Simulate a PID controller on a motion system
 * @author: Sarv Parteek Singh
 * @date: July-06-2020
 */

#ifndef CONTROLS_PID_SIMULATOR_HPP
#define CONTROLS_PID_SIMULATOR_HPP

#include <algorithm>
#include "pid.hpp"

namespace pid
{
    double updatePosition(double const &pos, double const &vel, double const &dt)
    {
        return (pos + vel * dt); // kinematic model
    }

    double getClampedControlEffort(double const &q_dot_effort, double const &q_dot_effort_last,
                                   double const &q_ddot_max,   double const &q_ddot_min,
                                   double const &q_dot_max,    double const &q_dot_min,
                                   double const &dt)
    {
        /* If computed q_dot will result in an acceleration greater than the max allowed q_ddot
         * over the next dt, then clamp the computed x_dot.
         * 1) Compute commanded q_ddot
         * 2) Clamp commanded q_ddot to respect acceleration limits
         * 3) Update velocity effort using clamped acceleration limits
         * 4) Update velocity effort using velocity limits
         */

        /* Step 1 */
        double const q_ddot_effort = (q_dot_effort - q_dot_effort_last) / dt;

        /* Step 2 */
        double const q_ddot_effort_clamped = std::clamp(q_ddot_effort, q_ddot_min, q_ddot_max);
        double q_dot_effort_clamped_acc    = q_dot_effort;

        /* Step 3 */
        if (std::abs(q_ddot_effort) > std::abs(q_ddot_effort_clamped))
        {
            q_dot_effort_clamped_acc = q_dot_effort_last + q_ddot_effort_clamped * dt;
        }

        /* Step 4 */
        double const q_dot_effort_clamped = std::clamp(q_dot_effort_clamped_acc, q_dot_min, q_dot_max);

        return q_dot_effort_clamped;
    }

    template <typename T>
    void simulateController(PID<T> & controller)
    {
        double dt             = 1e-3;
        double t_max          = 10;
        double t              =  0;
        double reference      = 10; // 10 m target
        double feedback       =  0;
        double x_dot_max      =  2; // 2 m/s max velocity
        double x_dot_min      = -2; // -2 m/s min velocity
        double x_ddot_max     =  2; // 2 m/s^2 max accel
        double x_ddot_min     = -2; // -2 m/s^2 min accel
        double control_effort =  0;
        while (t < t_max)
        {
            control_effort  = getClampedControlEffort(controller.getControlEffort(reference, feedback, dt),
                                                      control_effort, x_ddot_max, x_ddot_min, x_dot_max, x_dot_min, dt);
            feedback        = updatePosition(feedback, control_effort, dt); // perfect tracking and estimation
            t              += dt;
            // TODO : Write control effort to csv file
        }
    }

    void testControllers()
    {
        std::cout << std::endl << __PRETTY_FUNCTION__ << std::endl;
        PID<unsigned int> p_controller(Gains<unsigned int>(1.5, 0, 0, 0, 0));

    }

}

#endif //CONTROLS_PID_SIMULATOR_HPP