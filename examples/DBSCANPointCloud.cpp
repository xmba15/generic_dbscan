/**
 * @file    DBSCANPointCloud.cpp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */

#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <dbscan/dbscan.hpp>

namespace
{
using PointCloudType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointCloudType>;
using PointCloudPtr = PointCloud::Ptr;

class CustomPoint3D : public std::array<double, 3>
{
 public:
    CustomPoint3D(const double x, const double y, const double z)
    {
        (*this)[0] = x;
        (*this)[1] = y;
        (*this)[2] = y;
    }
};

using DBSCAN = clustering::DBSCAN<3, CustomPoint3D>;

DBSCAN::VecPointType toVecPointType(const PointCloudPtr& inPcl);
pcl::PointIndices::Ptr toPclPointIndices(const std::vector<int>& indices);
std::vector<PointCloudPtr> toClusters(const std::vector<std::vector<int>>& clusterIndices, const PointCloudPtr& inPcl);

inline std::vector<std::array<std::uint8_t, 3>> generateColorCharts(const uint16_t numSources,
                                                                    const uint16_t seed = 255);
template <typename POINT_CLOUD_TYPE>
inline std::pair<std::vector<std::array<std::uint8_t, 3>>,
                 std::vector<pcl::visualization::PointCloudColorHandlerCustom<POINT_CLOUD_TYPE>>>
initPclColorHandlers(const std::vector<typename pcl::PointCloud<POINT_CLOUD_TYPE>::Ptr>& inPcls);
pcl::visualization::PCLVisualizer::Ptr initializeViewer();
}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 4) {
        std::cerr << "Usage: [app] [path/to/pcl/file] [eps] [min/points]\n";
        return EXIT_FAILURE;
    }

    const std::string pclFilePath = argv[1];
    const double eps = std::atof(argv[2]);
    const double minPoints = std::atoi(argv[3]);

    PointCloudPtr inputPcl(new PointCloud);
    if (pcl::io::loadPCDFile(pclFilePath, *inputPcl) == -1) {
        std::cerr << "Failed to load pcl file\n";
        return EXIT_FAILURE;
    }

    DBSCAN::Ptr dbscan = std::make_shared<DBSCAN>(eps, minPoints);
    DBSCAN::VecPointType customPoints = ::toVecPointType(inputPcl);
    std::vector<std::vector<int>> clusterIndices = dbscan->estimateClusterIndices(customPoints);
    std::vector<PointCloudPtr> clusters = ::toClusters(clusterIndices, inputPcl);

    auto [colors, colorHandlers] = ::initPclColorHandlers<PointCloudType>(clusters);
    auto pclViewer = ::initializeViewer();

    std::size_t countElem = 0;
    for (const auto& cluster : clusters) {
        pclViewer->addPointCloud<PointCloudType>(cluster, colorHandlers[countElem], std::to_string(countElem));
        pclViewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                                    std::to_string(countElem));
        countElem++;
    }

    while (!pclViewer->wasStopped()) {
        pclViewer->spinOnce();
    }

    return EXIT_SUCCESS;
}

namespace
{
DBSCAN::VecPointType toVecPointType(const PointCloudPtr& inPcl)
{
    DBSCAN::VecPointType customPoints;
    customPoints.reserve(inPcl->points.size());

    for (const auto& point : inPcl->points) {
        customPoints.emplace_back(DBSCAN::PointType(point.x, point.y, point.z));
    }

    return customPoints;
}

pcl::PointIndices::Ptr toPclPointIndices(const std::vector<int>& indices)
{
    pcl::PointIndices::Ptr pclIndices(new pcl::PointIndices);
    std::copy(indices.begin(), indices.end(), std::back_inserter(pclIndices->indices));

    return pclIndices;
}

std::vector<PointCloudPtr> toClusters(const std::vector<std::vector<int>>& clusterIndices, const PointCloudPtr& inPcl)
{
    std::vector<PointCloudPtr> clusters;
    clusters.reserve(clusterIndices.size());

    for (const auto& curIndices : clusterIndices) {
        PointCloudPtr curPcl(new PointCloud);
        pcl::copyPointCloud(*inPcl, *toPclPointIndices(curIndices), *curPcl);
        clusters.emplace_back(curPcl);
    }

    return clusters;
}

std::vector<std::array<std::uint8_t, 3>> generateColorCharts(const uint16_t numSources, const uint16_t seed)
{
    std::srand(seed);

    std::vector<std::array<std::uint8_t, 3>> colors;
    colors.reserve(numSources);

    for (uint16_t i = 0; i < numSources; ++i) {
        colors.emplace_back(std::array<uint8_t, 3>{static_cast<uint8_t>(std::rand() % 255),
                                                   static_cast<uint8_t>(std::rand() % 255),
                                                   static_cast<uint8_t>(std::rand() % 255)});
    }

    return colors;
}

template <typename POINT_CLOUD_TYPE>
std::pair<std::vector<std::array<std::uint8_t, 3>>,
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
