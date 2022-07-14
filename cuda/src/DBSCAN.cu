/**
 * @file    DBSCAN.cu
 *
 * @author  btran
 *
 */

#include <dbscan/CudaUtils.cuh>
#include <dbscan/DBSCAN.cuh>

#include <pcl/point_types.h>

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

#include "Types.cuh"

namespace clustering
{
namespace cuda
{
namespace
{
cudaDeviceProp dProperties;

template <typename PointType>
__global__ void makeGraphStep1Kernel(const PointType* __restrict__ points, Node* nodes, int* nodeDegs, int numPoints,
                                     float eps, int minPoints)
{
    int tid = threadIdx.x + blockIdx.x * blockDim.x;

    while (tid < numPoints) {
        Node* node = &nodes[tid];
        const PointType* point = &points[tid];

        for (int i = 0; i < numPoints; ++i) {
            if (i == tid) {
                continue;
            }

            const PointType* curPoint = &points[i];
            float diffx = point->x - curPoint->x;
            float diffy = point->y - curPoint->y;
            float diffz = point->z - curPoint->z;

            float sqrDist = diffx * diffx + diffy * diffy + diffz * diffz;

            if (sqrDist < eps * eps) {
                node->numNeighbors++;
            }
        }

        if (node->numNeighbors >= minPoints) {
            node->type = NodeType::CORE;
        }

        nodeDegs[tid] = node->numNeighbors;

        tid += blockDim.x * gridDim.x;
    }
}

template <typename PointType>
__global__ void makeGraphStep2Kernel(const PointType* __restrict__ points, const int* __restrict__ neighborStartIndices,
                                     int* adjList, int numPoints, float eps)
{
    int tid = threadIdx.x + blockIdx.x * blockDim.x;

    while (tid < numPoints) {
        const PointType* point = &points[tid];

        int startIdx = neighborStartIndices[tid];

        int countNeighbors = 0;
        for (int i = 0; i < numPoints; ++i) {
            if (i == tid) {
                continue;
            }

            const PointType* curPoint = &points[i];
            float diffx = point->x - curPoint->x;
            float diffy = point->y - curPoint->y;
            float diffz = point->z - curPoint->z;

            float sqrDist = diffx * diffx + diffy * diffy + diffz * diffz;

            if (sqrDist < eps * eps) {
                adjList[startIdx + countNeighbors++] = i;
            }
        }

        tid += blockDim.x * gridDim.x;
    }
}

__global__ void BFSKernel(const Node* __restrict__ nodes, const int* __restrict__ adjList,
                          const int* __restrict__ neighborStartIndices, char* Fa, char* Xa, int numPoints)
{
    int tid = threadIdx.x + blockIdx.x * blockDim.x;

    while (tid < numPoints) {
        if (Fa[tid]) {
            Fa[tid] = false;
            Xa[tid] = true;

            int startIdx = neighborStartIndices[tid];

            for (int i = 0; i < nodes[tid].numNeighbors; ++i) {
                int nIdx = adjList[startIdx + i];
                Fa[nIdx] = 1 - Xa[nIdx];
            }
        }

        tid += blockDim.x * gridDim.x;
    }
}

void BFS(Node* hNodes, Node* dNodes, const int* adjList, const int* neighborStartIndices, int v, int numPoints,
         std::vector<int>& curCluster)
{
    thrust::device_vector<char> dXa(numPoints, false);
    thrust::device_vector<char> dFa(numPoints, false);
    dFa[v] = true;

    int numThreads = 256;
    int numBlocks = std::min(dProperties.maxGridSize[0], (numPoints + numThreads - 1) / numThreads);

    int countFa = 1;
    while (countFa > 0) {
        BFSKernel<<<numBlocks, numThreads>>>(dNodes, adjList, neighborStartIndices,
                                             thrust::raw_pointer_cast(dFa.data()), thrust::raw_pointer_cast(dXa.data()),
                                             numPoints);
        countFa = thrust::count(thrust::device, dFa.begin(), dFa.end(), true);
    }

    thrust::host_vector<char> hXa = dXa;

    for (int i = 0; i < numPoints; ++i) {
        if (hXa[i]) {
            hNodes[i].visited = true;
            curCluster.emplace_back(i);
        }
    }
}

template <typename PointType> Graph makeGraph(const PointType* points, int numPoints, float eps, int minPoints)
{
    Graph graph;

    graph.nodes.resize(numPoints);
    graph.neighborStartIndices.resize(numPoints);

    thrust::device_vector<int> dNodeDegs(numPoints);

    int numThreads = 256;
    int numBlocks = std::min(dProperties.maxGridSize[0], (numPoints + numThreads - 1) / numThreads);

    makeGraphStep1Kernel<PointType><<<numBlocks, numThreads>>>(points, thrust::raw_pointer_cast(graph.nodes.data()),
                                                               thrust::raw_pointer_cast(dNodeDegs.data()), numPoints,
                                                               eps, minPoints);

    thrust::exclusive_scan(dNodeDegs.begin(), dNodeDegs.end(), graph.neighborStartIndices.begin());

    int totalEdges = dNodeDegs.back() + graph.neighborStartIndices.back();
    graph.adjList.resize(totalEdges);

    makeGraphStep2Kernel<PointType>
        <<<numBlocks, numThreads>>>(points, thrust::raw_pointer_cast(graph.neighborStartIndices.data()),
                                    thrust::raw_pointer_cast(graph.adjList.data()), numPoints, eps);

    return graph;
}
}  // namespace

template <typename PointType>
DBSCAN<PointType>::DBSCAN(const Param& param)
    : m_param(param)
    , m_dPoints(nullptr)
    , m_allocatedSize(-1)
{
    HANDLE_ERROR(cudaGetDeviceProperties(&dProperties, 0));
}

template <typename PointType> DBSCAN<PointType>::~DBSCAN()
{
    if (m_dPoints) {
        HANDLE_ERROR(cudaFree(m_dPoints));
        m_dPoints = nullptr;
    }
}

template <typename PointType>
std::vector<std::vector<int>> DBSCAN<PointType>::run(const PointType* points, int numPoints) const
{
    if (numPoints <= 0) {
        throw std::runtime_error("number of points must be more than 0");
    }

    if (m_allocatedSize < numPoints) {
        m_allocatedSize = numPoints;
        if (m_dPoints) {
            HANDLE_ERROR(cudaFree(m_dPoints));
        }
        HANDLE_ERROR(cudaMalloc((void**)&m_dPoints, numPoints * sizeof(PointType)));
    }
    HANDLE_ERROR(cudaMemcpy(m_dPoints, points, numPoints * sizeof(PointType), cudaMemcpyHostToDevice));

    auto graph = makeGraph<PointType>(m_dPoints, numPoints, m_param.eps, m_param.minPoints);

    thrust::host_vector<Node> hNodes = graph.nodes;

    std::vector<std::vector<int>> clusterIndices;
    for (int i = 0; i < numPoints; ++i) {
        auto& curHNode = hNodes[i];
        if (curHNode.visited || curHNode.type != NodeType::CORE) {
            continue;
        }

        std::vector<int> curCluster;
        curCluster.emplace_back(i);
        curHNode.visited = true;

        BFS(hNodes.data(), thrust::raw_pointer_cast(graph.nodes.data()), thrust::raw_pointer_cast(graph.adjList.data()),
            thrust::raw_pointer_cast(graph.neighborStartIndices.data()), i, numPoints, curCluster);

        clusterIndices.emplace_back(std::move(curCluster));
    }

    return clusterIndices;
}

#undef INSTANTIATE_TEMPLATE
#define INSTANTIATE_TEMPLATE(DATA_TYPE) template class DBSCAN<DATA_TYPE>;

INSTANTIATE_TEMPLATE(pcl::PointXYZ);
INSTANTIATE_TEMPLATE(pcl::PointXYZI);
INSTANTIATE_TEMPLATE(pcl::PointXYZRGB);
INSTANTIATE_TEMPLATE(pcl::PointNormal);
INSTANTIATE_TEMPLATE(pcl::PointXYZRGBNormal);
INSTANTIATE_TEMPLATE(pcl::PointXYZINormal);

#undef INSTANTIATE_TEMPLATE
}  // namespace cuda
}  // namespace clustering
