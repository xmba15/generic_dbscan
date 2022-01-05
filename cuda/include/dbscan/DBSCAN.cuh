/**
 * @file    DBSCAN.cuh
 *
 * @author  btran
 *
 */

#pragma once

#include <vector>

namespace clustering
{
namespace cuda
{
template <typename PointType> class DBSCAN
{
 public:
    struct Param {
        int pointDimension;
        double eps;
        int minPoints;
    };

    explicit DBSCAN(const Param& param);
    ~DBSCAN();

    std::vector<std::vector<int>> run(const PointType*, int numPoints) const;

 private:
    Param m_param;

    mutable PointType* m_dPoints;
    mutable int m_allocatedSize;
};
}  // namespace cuda
}  // namespace clustering
