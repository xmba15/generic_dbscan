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

 public:
    DBSCAN(const std::function<double(const PointType& p1, const PointType& p2)>& distFunc =
               clustering::distance<POINT_DIMENSION, POINT_TYPE>);
    DBSCAN(const double eps, const int minPoints,
           const std::function<double(const PointType& p1, const PointType& p2)>& distFunc =
               clustering::distance<POINT_DIMENSION, POINT_TYPE>);
    ~DBSCAN();

    void setMinPoints(const int minPoints);
    void setEps(const double eps);

    std::vector<std::vector<int>> estimateClusterIndices(const VecPointType& points);
    std::vector<VecPointType> estimateClusters(const VecPointType& points);

 protected:
    void expandCluster(const VecPointType& points, std::vector<int>& curCluster, const std::vector<int>& neighbors,
                       std::vector<char>& visited, std::vector<char>& isNoise) const;

 public:
    const std::function<double(const PointType& p1, const PointType& p2)> m_distFunc;

 protected:
    double m_eps;
    int m_minPoints;

    typename KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::Ptr m_kdtree;
};
}  // namespace clustering

#include "impl/DBSCANImpl.ipp"
