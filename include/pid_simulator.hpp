/*!
 * @brief: Simulate a PID controller on a motion system
 * @author: Sarv Parteek Singh
 * @date: July-06-2020
 */

#ifndef CONTROLS_PID_SIMULATOR_HPP
#define CONTROLS_PID_SIMULATOR_HPP

#include <algorithm>
#include <fstream>
#include <cmath>
#include <vector>
#include "pid.hpp"

namespace pid
{
    /*!
     * @brief Updates position by integrating velocity
     * @param[in] pos Previous position
     * @param[in] vel Current velocity command
     * @param[in] dt Loop period (over which velocity command can be assumed constant)
     * @param[in] lag_factor Resistive factor which stops position from moving forward.
     *                         1   => no resistance (nominal motion)
     *                         0   => full resistance (no motion)
     *                       (0,1) => some resistance
     * @return Update position
     */
    double updatePosition(double const &pos, double const &vel, double const &dt, double const &lag_factor)
    {
        return (pos + vel * dt * lag_factor); // kinematic model
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

    void logMotion(double const &ref, double const &fdbk, double const &vel_cmd, double const &time, std::ofstream &file)
    {
        file << ref      << ","
             << fdbk     << ","
             << vel_cmd  << ","
             << time     << std::endl;
    }

    template <typename T>
    void simulateController(PID<T> & controller, std::ofstream &file, double const &lag_factor = 1,
                            double const & bias_vel = 0, double const &t_ref_jump_percent = -1)
    {
        double dt             = 1e-3;
        double t_max          = 20;
        double t              =  0;
        double reference      = 10; // 10 m target
        double feedback       =  0;
        double x_dot_max      =  2; // 2 m/s max velocity
        double x_dot_min      = -2; // -2 m/s min velocity
        double x_ddot_max     =  2; // 2 m/s^2 max accel
        double x_ddot_min     = -2; // -2 m/s^2 min accel
        double control_effort =  0;
        double l_t_ref_jump = t_ref_jump_percent * t_max;

        while (t < t_max)
        {
            /* When time approaches the jump point, have the reference jump up to a higher value */
            if (l_t_ref_jump > 0 && std::abs(t - l_t_ref_jump) < 2 * dt)
            {
                reference      += 5;
                l_t_ref_jump  = -1; // Do not jump again
            }
            control_effort  = getClampedControlEffort(controller.getControlEffort(reference, feedback, dt) + bias_vel,
                                                      control_effort, x_ddot_max, x_ddot_min, x_dot_max, x_dot_min, dt);
            feedback        = updatePosition(feedback, control_effort, dt, lag_factor);
            t              += dt;
            logMotion(reference, feedback, control_effort, t, file);
            // TODO : Simulate effect of a discrete jump in reference/feedback
        }
    }

    void testControllers()
    {
        std::cout << std::endl << __PRETTY_FUNCTION__ << std::endl;
        std::string output_folder = "C:/Users/sarvp/CLionProjects/Controls/data";

        for (int i = 1; i <= 3; ++i)
        {
            PID<double> p_controller(Gains<double>(2 * std::pow(10,i-1), 0, 0, 0, 0));
            std::ofstream file_controller(output_folder + "/controller_" + std::to_string(i) + ".csv");
            simulateController(p_controller, file_controller); // use
                                                                      // t_ref_jump_percent = 0.028 for P gain of 2
                                                                      // to get a jump at 5.6 (settling happens at
                                                                      // 6 otherwise)
            file_controller.close();
        }


        std::vector<double> i_gains{0.9999, 1.0, 100};
        int idx_base = 4;
        for (int i = 4; i <= 6; ++i)
        {
            PID<double> pi_controller(Gains<double>(1.5, i_gains.at(i-idx_base), 0, 0, 0));
            std::ofstream file_controller(output_folder + "/controller_" + std::to_string(i) + ".csv");
            simulateController(pi_controller, file_controller);
            file_controller.close();
            // TODO: Implement anti-windup control. Better option: clamp control effort in class PID, then call
            // anti-windup
        }
    }

}

#endif //CONTROLS_PID_SIMULATOR_HPP
