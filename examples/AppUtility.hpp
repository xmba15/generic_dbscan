/**
 * @file    AppUtility.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <utility>
#include <vector>

namespace
{
inline pcl::PointIndices::Ptr toPclPointIndices(const std::vector<int>& indices)
{
    pcl::PointIndices::Ptr pclIndices(new pcl::PointIndices);
    pclIndices->indices = indices;

    return pclIndices;
}

template <typename POINT_CLOUD_TYPE>
inline std::vector<typename pcl::PointCloud<POINT_CLOUD_TYPE>::Ptr>
toClusters(const std::vector<std::vector<int>>& clusterIndices,
           const typename pcl::PointCloud<POINT_CLOUD_TYPE>::Ptr& inPcl)
{
    std::vector<typename pcl::PointCloud<POINT_CLOUD_TYPE>::Ptr> clusters;
    clusters.reserve(clusterIndices.size());

    for (const auto& curIndices : clusterIndices) {
        typename pcl::PointCloud<POINT_CLOUD_TYPE>::Ptr curPcl(new pcl::PointCloud<POINT_CLOUD_TYPE>);
        pcl::copyPointCloud(*inPcl, *toPclPointIndices(curIndices), *curPcl);
        clusters.emplace_back(curPcl);
    }

    return clusters;
}

inline std::vector<std::array<std::uint8_t, 3>> generateColorCharts(const uint16_t numSources,
                                                                    const std::uint16_t seed = 2020)
{
    std::srand(seed);

    std::vector<std::array<std::uint8_t, 3>> colors;
    colors.reserve(numSources);

    for (std::uint16_t i = 0; i < numSources; ++i) {
        colors.emplace_back(std::array<uint8_t, 3>{static_cast<uint8_t>(std::rand() % 256),
                                                   static_cast<uint8_t>(std::rand() % 256),
                                                   static_cast<uint8_t>(std::rand() % 256)});
    }

    return colors;
}

template <typename POINT_CLOUD_TYPE>
inline std::pair<std::vector<std::array<std::uint8_t, 3>>,
                 std::vector<pcl::visualization::PointCloudColorHandlerCustom<POINT_CLOUD_TYPE>>>
initPclColorHandlers(const std::vector<typename pcl::PointCloud<POINT_CLOUD_TYPE>::Ptr>& inPcls)
{
    using PointCloudType = POINT_CLOUD_TYPE;
    using PointCloud = pcl::PointCloud<PointCloudType>;

    std::vector<pcl::visualization::PointCloudColorHandlerCustom<PointCloudType>> pclColorHandlers;
    pclColorHandlers.reserve(inPcls.size());
    auto colorCharts = ::generateColorCharts(inPcls.size());

    for (int i = 0; i < inPcls.size(); ++i) {
        const auto& curColor = colorCharts[i];
        pclColorHandlers.emplace_back(pcl::visualization::PointCloudColorHandlerCustom<PointCloudType>(
            inPcls[i], curColor[0], curColor[1], curColor[2]));
    }

    return std::make_pair(colorCharts, pclColorHandlers);
}

pcl::visualization::PCLVisualizer::Ptr initializeViewer()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::PointXYZ o(0.1, 0, 0);
    viewer->addSphere(o, 0.1, "sphere", 0);
    viewer->setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer->addCoordinateSystem(0.5);
    viewer->setCameraPosition(-26, 0, 3, 10, -1, 0.5, 0, 0, 1);

    return viewer;
}
}  // namespace
