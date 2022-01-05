/**
 * @file    KDTreeImpl.ipp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */

namespace clustering
{
template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::KDTree(const VecPointType& points,
                                                            const DistanceFunctionType& distFunc,
                                                            const ValueAtFunctionType& valueAtFunc)
    : m_points(points)
    , m_valueAtFunc(valueAtFunc)
    , m_distFunc(distFunc)
{
    this->buildKDTree();
}

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::~KDTree()
{
    this->clear();
}

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
void KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::buildKDTree()
{
    std::vector<int> pointIndices(m_points.size());
    std::iota(pointIndices.begin(), pointIndices.end(), 0);
    m_root = this->insertRec(pointIndices, 0);
}

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
void KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::clear()
{
    this->clearRec(m_root);
    m_root = nullptr;
}

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
int KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::nearestSearch(const PointType& queryPoint) const
{
    int nearestPointIdx = -1;

    if (!m_root) {
        return nearestPointIdx;
    }

    double nearestDist = std::numeric_limits<double>::max();
    this->nearestSearchRec(queryPoint, m_root, nearestPointIdx, nearestDist);

    return nearestPointIdx;
}

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
std::vector<int> KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::radiusSearch(const PointType& queryPoint,
                                                                                   const double radius) const
{
    std::vector<int> indices;

    if (!m_root) {
        return indices;
    }

    this->radiusSearchRec(queryPoint, m_root, indices, radius);
    return indices;
}

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
std::vector<int> KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::knnSearch(const PointType& queryPoint,
                                                                                const int k) const
{
    std::vector<int> indices;

    if (!m_root || k < 0) {
        return indices;
    }

    if (k > m_points.size()) {
        indices.resize(m_points.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(), [&queryPoint, this](const int idx1, const int idx2) {
            return m_distFunc(queryPoint, m_points[idx1], m_valueAtFunc) <
                   m_distFunc(queryPoint, m_points[idx2], m_valueAtFunc);
        });

        return indices;
    }

    double maxDistance = std::numeric_limits<double>::max();
    KNNPriorityQueue knnMaxHeap;
    this->knnSearchRec(queryPoint, m_root, knnMaxHeap, maxDistance, k);

    indices.reserve(k);
    while (!knnMaxHeap.empty()) {
        indices.emplace_back(knnMaxHeap.top().second);
        knnMaxHeap.pop();
    }

    return indices;
}

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
typename KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::KDNode*
KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::insertRec(std::vector<int>& pointIndices, const int depth)
{
    if (pointIndices.empty()) {
        return nullptr;
    }

    const int medianIdx = pointIndices.size() / 2;
    const int splitType = depth % POINT_DIMENSION;
    std::nth_element(pointIndices.begin(), pointIndices.begin() + medianIdx, pointIndices.end(),
                     [&](const int lhs, const int rhs) {
                         return (m_valueAtFunc(m_points[lhs], splitType) < m_valueAtFunc(m_points[rhs], splitType));
                     });

    KDNode* curNode = new KDNode();
    curNode->pointIdx = pointIndices[medianIdx];
    curNode->splitType = splitType;

    std::vector<int> leftIndices(pointIndices.begin(), pointIndices.begin() + medianIdx);
    std::vector<int> rightIndices(pointIndices.begin() + medianIdx + 1, pointIndices.end());
    curNode->child[0] = this->insertRec(leftIndices, depth + 1);
    curNode->child[1] = this->insertRec(rightIndices, depth + 1);

    return curNode;
}

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
void KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::clearRec(KDNode* node)
{
    if (!node) {
        return;
    }

    if (node->child[0]) {
        this->clearRec(node->child[0]);
    }

    if (node->child[1]) {
        this->clearRec(node->child[1]);
    }

    delete node;
}

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
void KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::nearestSearchRec(const PointType& queryPoint,
                                                                           const KDNode* node, int& nearestPointIdx,
                                                                           double& nearestDist) const
{
    if (!node) {
        return;
    }

    const PointType& curPoint = m_points[node->pointIdx];
    const double curDist = m_distFunc(queryPoint, curPoint, m_valueAtFunc);
    if (curDist < nearestDist) {
        nearestPointIdx = node->pointIdx;
        nearestDist = curDist;
    }

    const int splitType = node->splitType;
    const int searchDirection = m_valueAtFunc(queryPoint, splitType) < m_valueAtFunc(curPoint, splitType) ? 0 : 1;
    this->nearestSearchRec(queryPoint, node->child[searchDirection], nearestPointIdx, nearestDist);

    const double distToTheRemainingPlane =
        std::fabs(m_valueAtFunc(queryPoint, splitType) - m_valueAtFunc(curPoint, splitType));
    if (distToTheRemainingPlane < nearestDist) {
        this->nearestSearchRec(queryPoint, node->child[!searchDirection], nearestPointIdx, nearestDist);
    }
}

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
void KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::radiusSearchRec(const PointType& queryPoint,
                                                                          const KDNode* node, std::vector<int>& indices,
                                                                          const double radius) const
{
    if (!node) {
        return;
    }

    const PointType& curPoint = m_points[node->pointIdx];
    const double curDist = m_distFunc(queryPoint, curPoint, m_valueAtFunc);
    if (curDist < radius) {
        indices.emplace_back(node->pointIdx);
    }

    const int splitType = node->splitType;
    const int searchDirection = m_valueAtFunc(queryPoint, splitType) < m_valueAtFunc(curPoint, splitType) ? 0 : 1;
    this->radiusSearchRec(queryPoint, node->child[searchDirection], indices, radius);

    const double distToTheRemainingPlane =
        std::fabs(m_valueAtFunc(queryPoint, splitType) - m_valueAtFunc(curPoint, splitType));
    if (distToTheRemainingPlane < radius) {
        this->radiusSearchRec(queryPoint, node->child[!searchDirection], indices, radius);
    }
}

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
void KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::knnSearchRec(const PointType& queryPoint, const KDNode* node,
                                                                       KNNPriorityQueue& knnMaxHeap,
                                                                       double& maxDistance, const int k) const
{
    if (!node) {
        return;
    }

    if (knnMaxHeap.size() == k) {
        maxDistance = knnMaxHeap.top().first;
    }

    const PointType& curPoint = m_points[node->pointIdx];
    const double curDist = m_distFunc(queryPoint, curPoint, m_valueAtFunc);

    if (curDist < maxDistance) {
        while (knnMaxHeap.size() >= k) {
            knnMaxHeap.pop();
        }

        knnMaxHeap.emplace(std::make_pair(curDist, node->pointIdx));
    }

    const int splitType = node->splitType;
    const int searchDirection = m_valueAtFunc(queryPoint, splitType) < m_valueAtFunc(curPoint, splitType) ? 0 : 1;
    this->knnSearchRec(queryPoint, node->child[searchDirection], knnMaxHeap, maxDistance, k);

    const double distToTheRemainingPlane =
        std::fabs(m_valueAtFunc(queryPoint, splitType) - m_valueAtFunc(curPoint, splitType));
    if (knnMaxHeap.size() < k || distToTheRemainingPlane < maxDistance) {
        this->knnSearchRec(queryPoint, node->child[!searchDirection], knnMaxHeap, maxDistance, k);
    }
}

//------------------------------------------------------------------------------
// kdnode data structure
//------------------------------------------------------------------------------

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
struct KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::KDNode {
    KDNode();

    int pointIdx;  // index of the point this node is pointing to
    int splitType;
    KDNode* child[2];  // left , right nodes
};

template <int POINT_DIMENSION, typename POINT_TYPE, typename VEC_POINT_TYPE>
KDTree<POINT_DIMENSION, POINT_TYPE, VEC_POINT_TYPE>::KDNode::KDNode()
    : pointIdx(-1)
    , splitType(-1)
{
    child[0] = nullptr;
    child[1] = nullptr;
}
}  // namespace clustering
