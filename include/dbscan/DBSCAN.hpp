/**
 * @file    DBSCAN.hpp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <algorithm>
#include <deque>
#include <memory>
#include <vector>

#include "KDTree.hpp"
#include "Utility.hpp"

namespace clustering
{
template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE = std::vector<POINT_TYPE>> class DBSCAN
{
 public:
    using PointType = POINT_TYPE;
    using VecPointType = VEC_POINT_TYPE;

    using Ptr = std::shared_ptr<DBSCAN>;

    using ValueAtFunctionType = const std::function<double(const POINT_TYPE& p, const int axis)>;
    using DistanceFunctionType = std::function<double(const PointType& p1, const PointType& p2, ValueAtFunctionType)>;

 public:
    DBSCAN(const double eps, const int minPoints,
           const DistanceFunctionType& distFunc = clustering::distance<POINT_DIMENSION, POINT_TYPE>,
           const ValueAtFunctionType& valueAtFunc = clustering::at<POINT_DIMENSION, POINT_TYPE>);

    ~DBSCAN() = default;

    std::vector<std::vector<int>> estimateClusterIndices(const VecPointType& points);
    std::vector<VecPointType> estimateClusters(const VecPointType& points);

 protected:
    void expandCluster(const VecPointType& points, std::vector<int>& curCluster, const std::vector<int>& neighbors,
                       std::vector<char>& visited, std::vector<char>& isNoise) const;

 private:
    double m_eps;
    int m_minPoints;

    const ValueAtFunctionType m_valueAtFunc;
    const DistanceFunctionType m_distFunc;

    typename KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::Ptr m_kdtree;
};
}  // namespace clustering
#include "impl/DBSCANImpl.ipp"
