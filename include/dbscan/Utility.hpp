/**
 * @file    Utility.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <cmath>
#include <functional>
#include <stdexcept>

namespace clustering
{
template <int POINT_DIMENSION, typename POINT_TYPE> double at(const POINT_TYPE& p, const int axis)
{
    if (axis >= POINT_DIMENSION) {
        throw std::runtime_error("axis out of range");
    }

    return p[axis];
}

/**
 *  @brief l2 norm between two points
 */
template <int POINT_DIMENSION, typename POINT_TYPE>
double distance(const POINT_TYPE& p1, const POINT_TYPE& p2,
                const std::function<double(const POINT_TYPE& p, const int axis)>& valueAtFunc =
                    clustering::at<POINT_DIMENSION, POINT_TYPE>)
{
    double result = 0.0;
    for (int i = 0; i < POINT_DIMENSION; ++i) {
        result += std::pow(valueAtFunc(p1, i) - valueAtFunc(p2, i), 2);
    }
    return std::sqrt(result);
}
}  // namespace clustering
