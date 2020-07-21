/**
 * @file    Utility.hpp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <cmath>

namespace clustering
{
/**
 *  @brief l2 norm between two points
 */
template <int POINT_DIMENSION, typename POINT_TYPE> double distance(const POINT_TYPE& p1, const POINT_TYPE& p2)
{
    double result = 0.0;
    for (int i = 0; i < POINT_DIMENSION; ++i) {
        result += std::pow(p1[i] - p2[i], 2);
    }
    return std::sqrt(result);
}
}  // namespace clustering
