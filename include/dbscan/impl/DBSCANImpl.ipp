/**
 * @file    DBSCANImpl.ipp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */

namespace clustering
{
template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
DBSCAN<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::DBSCAN(const double eps, const int minPoints,
                                                            const DistanceFunctionType& distFunc,
                                                            const ValueAtFunctionType& valueAtFunc)
    : m_valueAtFunc(valueAtFunc)
    , m_distFunc(distFunc)
    , m_eps(eps)
    , m_minPoints(minPoints)
    , m_kdtree(nullptr)
{
}

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
std::vector<std::vector<int>>
DBSCAN<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::estimateClusterIndices(const VecPointType& points)
{
    m_kdtree = std::make_shared<KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>>(points, m_distFunc, m_valueAtFunc);

    std::vector<std::vector<int>> clusterIndices;
    std::vector<char> visited(points.size(), false);
    std::vector<char> isNoise(points.size(), false);

    for (std::size_t i = 0; i < points.size(); ++i) {
        if (visited[i]) {
            continue;
        }

        visited[i] = true;
        const PointType& curPoint = points[i];
        const std::vector<int> neighbors = m_kdtree->radiusSearch(curPoint, m_eps);

        if (neighbors.size() < m_minPoints) {
            isNoise[i] = true;
        } else {
            clusterIndices.emplace_back(std::vector<int>{static_cast<int>(i)});
            this->expandCluster(points, clusterIndices.back(), neighbors, visited, isNoise);
        }
    }

    return clusterIndices;
}

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
void DBSCAN<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::expandCluster(const VecPointType& points,
                                                                        std::vector<int>& curCluster,
                                                                        const std::vector<int>& neighbors,
                                                                        std::vector<char>& visited,
                                                                        std::vector<char>& isNoise) const
{
    std::deque<int> neighborDeque(neighbors.begin(), neighbors.end());
    while (!neighborDeque.empty()) {
        int curIdx = neighborDeque.front();
        neighborDeque.pop_front();

        if (isNoise[curIdx]) {
            curCluster.emplace_back(curIdx);
            continue;
        }

        if (!visited[curIdx]) {
            visited[curIdx] = true;
            curCluster.emplace_back(curIdx);

            const PointType& curPoint = points[curIdx];
            std::vector<int> curNeighbors = m_kdtree->radiusSearch(curPoint, m_eps);

            if (curNeighbors.size() < m_minPoints) {
                continue;
            }

            std::copy(curNeighbors.begin(), curNeighbors.end(), std::back_inserter(neighborDeque));
        }
    }
}

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
std::vector<typename DBSCAN<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::VecPointType>
DBSCAN<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::estimateClusters(const VecPointType& points)
{
    std::vector<std::vector<int>> clusterIndices = this->estimateClusterIndices(points);
    std::vector<VecPointType> clusters;
    clusters.reserve(clusterIndices.size());

    for (const auto& curIndices : clusterIndices) {
        VecPointType curCluster;
        curCluster.reserve(curIndices.size());
        for (const int pointIdx : curIndices) {
            curCluster.emplace_back(points[pointIdx]);
        }
        clusters.emplace_back(curCluster);
    }

    return clusters;
}
}  // namespace clustering
