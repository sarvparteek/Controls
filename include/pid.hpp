/*!
 * @brief Implements a PID controller
 * @author Sarv Parteek Singh
 * @date July-01-2020
 */

#ifndef CONTROLS_PID_HPP
#define CONTROLS_PID_HPP

#include <iostream>
#include <utility> // for move
#include <cassert>
#include <algorithm>

#include "is_numeric.hpp"
#include "math/signum.hpp"

namespace pid
{
    template <typename T>
    class Gains
    {
    public:
        Gains() = default;

        /* Constructor */
        Gains(T const &p, T const &i, T const &d, T const &vel_ff, T const &accel_ff): m_p(p),
                                                                                       m_i(i),
                                                                                       m_d(d),
                                                                                       m_vel_ff(vel_ff),
                                                                                       m_accel_ff(accel_ff)
        {
            assert(isNumeric<T>::value && "Non-numeric type not supported");
        }

        /* Copy constructor */
        Gains(Gains const &g): m_p(g.m_p),
                               m_i(g.m_i),
                               m_d(g.m_d),
                               m_vel_ff(g.m_vel_ff),
                               m_accel_ff(g.m_accel_ff)
        {}

        /* Move constructor */
        Gains(Gains const &&g): m_p(std::move(g.m_p)),
                                m_i(std::move(g.m_i)),
                                m_d(std::move(g.m_d)),
                                m_vel_ff(std::move(g.m_vel_ff)),
                                m_accel_ff(std::move(g.m_accel_ff))
        {}

        /* Copy-assignment instructor */
        Gains& operator=(Gains const &g)
        {
            m_p        = g.m_p;
            m_i        = g.m_i;
            m_d        = g.m_d;
            m_vel_ff   = g.m_vel_ff;
            m_accel_ff = g.m_accel_ff;

            return *this;
        }

        /* Move-assignment instructor */
        Gains& operator=(Gains const &&g)
        {
            m_p        = std::move(g.m_p);
            m_i        = std::move(g.m_i);
            m_d        = std::move(g.m_d);
            m_vel_ff   = std::move(g.m_vel_ff);
            m_accel_ff = std::move(g.m_accel_ff);
        }

/* ---------------- Setters --------------- */
        void setPGain(T const &p)
        {
            m_p = p;
        }

        void setIGain(T const &i)
        {
            m_i = i;
        }

        void setDGain(T const &d)
        {
            m_d = d;
        }

        void setVelFfGain(T const &vel_ff)
        {
            m_vel_ff = vel_ff;
        }

        void setAccelFfGain(T const &accel_ff)
        {
            m_accel_ff = accel_ff;
        }

/* ---------------- Getters ------------------------ */

        T getPGain() const
        {
            return m_p;
        }

        T getIGain() const
        {
            return m_i;
        }

        T getDGain() const
        {
            return m_d;
        }

        T getVelFfGain() const
        {
            return m_vel_ff;
        }

        T getAccelFfGain() const
        {
            return m_accel_ff;
        }

    private:
        T m_p;
        T m_i;
        T m_d;
        T m_vel_ff;
        T m_accel_ff;
    };

    template <typename T>
    class PID
    {
    public:
        PID(Gains<T> const &g) : m_gains(g),
                                 m_err(0.0),
                                 m_err_sum(0.0),
                                 m_err_deriv(0.0),
                                 m_err_deriv_filt(0.0),
                                 m_control_effort(0.0),
                                 m_max_control_effort(2.0),
                                 m_windup(1.0)
        {}

        double getControlEffort(double const &reference, double const &feedback, double const &dt,
                                double const &q_dot_cmd = 0, double const &q_ddot_cmd = 0)
        {
            auto m_prev_err  = m_err;
            m_err            = reference - feedback;
            m_err_sum       += m_err * dt;
            m_err_deriv      = (m_err - m_prev_err) / dt;
            m_err_deriv_filt = m_err_deriv; // TODO - Add filtering
            m_control_effort =   m_gains.getPGain()       * m_err
                               + m_gains.getIGain()       * m_err_sum
                               + m_gains.getDGain()       * m_err_deriv_filt
                               + m_gains.getVelFfGain()   * q_dot_cmd
                               + m_gains.getAccelFfGain() * q_ddot_cmd;
            //applyAntiWindup(m_control_effort);
            return m_control_effort;
        }

        void applyAntiWindup(double const &control_effort)
        {
            /* C  = Ki * err_sum + f(e, e_dot)
             * where C = control effort
             * So, f(e, e_dot) = C - Ki * err_sum
             * */
            auto l_f_e_edot           = control_effort - m_gains.getIGain() * m_err_sum;

            /* if (|C| > |C_max|)
             *      Ki * err_sum = sgn(C).C_max - f(e,e_dot)
             *                     sgn(C).C_max - f(e,e_dot)
             *           err_sum = --------------------------
             *                                 Ki
             * */
            auto l_max_control_effort = std::max(m_windup, 1.0) * m_max_control_effort;
            if (std::abs(control_effort) > l_max_control_effort)
            {
                m_err_sum = (math::sgn<double>(control_effort, 1e-5) * l_max_control_effort - l_f_e_edot) /
                                                                                                    m_gains.getIGain();
                m_control_effort = m_gains.getIGain() * m_err_sum;
            }
        }

    private:
        Gains<T> m_gains;
        double   m_err;
        double   m_err_sum;
        double   m_err_deriv;
        double   m_err_deriv_filt;
        double   m_control_effort;
        double   m_max_control_effort;
        double   m_windup;
    };

} // pid
#endif //CONTROLS_PID_HPP
