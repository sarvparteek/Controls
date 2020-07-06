/*!
 * @brief: Checks whether a given type is numeric (int or floating point type)
 * @author: Sarv Parteek Singh
 */

#ifndef CONTROLS_IS_NUMERIC_HPP
#define CONTROLS_IS_NUMERIC_HPP

#include <type_traits>

/*!
 * @brief Checks whether the template type is floating point or int type
 * @tparam T Template type
 */
template <typename T>
struct isNumeric : std::integral_constant<bool,
                                          std::is_arithmetic<T>::value
                                          && !std::is_same<typename std::remove_cv<T>::type, bool>::value
                                          && !std::is_same<typename std::remove_cv<T>::type, char>::value
                                          && !std::is_same<typename std::remove_cv<T>::type, char16_t>::value
                                          && !std::is_same<typename std::remove_cv<T>::type, char32_t>::value
                                          && !std::is_same<typename std::remove_cv<T>::type, wchar_t >::value
                                          && !std::is_same<typename std::remove_cv<T>::type, unsigned char>::value
                                          && !std::is_same<typename std::remove_cv<T>::type, signed char>::value>
{};


#endif //CONTROLS_IS_NUMERIC_HPP
