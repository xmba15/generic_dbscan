/**
 * @file    KDTree.hpp
 *
 * @author  btran
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

#include <dbscan/Utility.hpp>

namespace clustering
{
template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE = std::vector<POINT_TYPE>> class KDTree
{
 public:
    using PointType = POINT_TYPE;
    using VecPointType = VEC_POINT_TYPE;

    using Ptr = std::shared_ptr<KDTree>;

    using ValueAtFunctionType = const std::function<double(const POINT_TYPE& p, const int axis)>;
    using DistanceFunctionType = std::function<double(const PointType& p1, const PointType& p2, ValueAtFunctionType)>;

 public:
    explicit KDTree(const VecPointType& points,
                    const DistanceFunctionType& distFunc = clustering::distance<POINT_DIMENSION, POINT_TYPE>,
                    const ValueAtFunctionType& valueAtFunc = clustering::at<POINT_DIMENSION, POINT_TYPE>);
    ~KDTree();

    int nearestSearch(const PointType& queryPoint) const;

    std::vector<int> radiusSearch(const PointType& queryPoint, const double radius) const;

    std::vector<int> knnSearch(const PointType& queryPoint, const int k) const;

    auto valueAtFunc() const
    {
        return m_valueAtFunc;
    }

    auto distFunc() const
    {
        return m_distFunc;
    }

 private:
    void buildKDTree();
    void clear();

    // DistanceIndex->(distance from query point to current point, index of the current point)
    using DistanceIndex = std::pair<double, int>;

    // priority queue from max heap based on distance
    using KNNPriorityQueue = std::priority_queue<DistanceIndex, std::vector<DistanceIndex>, std::less<DistanceIndex>>;

    struct KDNode;
    KDNode* insertRec(std::vector<int>& pointIndices, const int depth);
    void clearRec(KDNode* node);
    void nearestSearchRec(const PointType& queryPoint, const KDNode* node, int& nearestPointIdx,
                          double& nearestDist) const;
    void radiusSearchRec(const PointType& queryPoint, const KDNode* node, std::vector<int>& indices,
                         const double radius) const;
    void knnSearchRec(const PointType& queryPoint, const KDNode* node, KNNPriorityQueue& knnMaxHeap,
                      double& maxDistance, const int k) const;

 private:
    KDNode* m_root;

    const VecPointType& m_points;

    ValueAtFunctionType m_valueAtFunc;
    DistanceFunctionType m_distFunc;
};
}  // namespace clustering

#include "impl/KDTreeImpl.ipp"
