/**
 * @file    KDTree.hpp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>
#include <utility>
#include <vector>

#include "Utility.hpp"

namespace clustering
{
template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE = std::vector<POINT_TYPE>> class KDTree
{
 public:
    using PointType = POINT_TYPE;
    using VecPointType = VEC_POINT_TYPE;

    using Ptr = std::shared_ptr<KDTree>;

 protected:
    struct KDNode;

 public:
    explicit KDTree(const VecPointType& points,
                    const std::function<double(const PointType& p1, const PointType& p2)>& distFunc =
                        clustering::distance<POINT_DIMENSION, POINT_TYPE>);
    virtual ~KDTree();

    virtual int nearestSearch(const PointType& queryPoint) const;

    virtual std::vector<int> radiusSearch(const PointType& queryPoint, const double radius) const;

    virtual std::vector<int> knnSearch(const PointType& queryPoint, const int k) const;

 protected:
    virtual void buildKDTree();
    virtual void clear();

    // DistanceIndex->(distance from query point to current point, index of the current point)
    using DistanceIndex = std::pair<double, int>;

    // priority queue from max heap based on distance
    using KNNPriorityQueue = std::priority_queue<DistanceIndex, std::vector<DistanceIndex>, std::less<DistanceIndex>>;

    //@{
    /** @brief utility methods to be used recursively
     */
    virtual KDNode* insertRec(std::vector<int>& pointIndices, const int depth);
    virtual void clearRec(KDNode* node);
    virtual void nearestSearchRec(const PointType& queryPoint, const KDNode* node, int& nearestPointIdx,
                                  double& nearestDist) const;
    virtual void radiusSearchRec(const PointType& queryPoint, const KDNode* node, std::vector<int>& indices,
                                 const double radius) const;
    virtual void knnSearchRec(const PointType& queryPoint, const KDNode* node, KNNPriorityQueue& knnMaxHeap,
                              double& maxDistance, const int k) const;
    //@}

 public:
    const int m_pointDimension = POINT_DIMENSION;
    const std::function<double(const PointType& p1, const PointType& p2)> m_distFunc;

 protected:
    KDNode* m_root;

    // only stores reference to points
    const VecPointType& m_points;
};
}  // namespace clustering

#include "impl/KDTreeImpl.ipp"
