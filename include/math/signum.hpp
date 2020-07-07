/*!
 * @brief Implements a signum function
 * @author Sarv Parteek Singh
 * @date July-07-2020
 */

#ifndef CONTROLS_SIGNUM_HPP
#define CONTROLS_SIGNUM_HPP

namespace math
{
    /*!
 * @brief Computes signum of the input number: 1 for positive numbers, -1 for negative numbers and 0 for 0.
 * @param[in] num Input number.
 * @param[in] epsilon Range around zero where signum would be zero
 * @return Signum of input.
 */
    template <typename T>
    int sgn(T const &num, T const epsilon = T(0))
    {
        return (num > std::abs(epsilon)) - (num < -std::abs(epsilon));
    }

/*!
 * @brief Computes signum of the input number: 1 for positive numbers and 0, -1 for negative numbers.
 * @param[in] num Input number.
 * @param[in] epsilon Range around zero where signum would be 1.
 * @return Non-zero signum of input.
 */
    template <typename T>
    int nonZeroSgn(T const &num, T const epsilon = T(0))
    {
        return (num >= -std::abs(epsilon)) - (num < -std::abs(epsilon));
    }
}
#endif //CONTROLS_SIGNUM_HPP
