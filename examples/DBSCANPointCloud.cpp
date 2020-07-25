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

template <typename POINT_CLOUD_TYPE> double at(const POINT_CLOUD_TYPE& p, const int axis)
{
    switch (axis) {
        case 0: {
            return p.x;
            break;
        }
        case 1: {
            return p.y;
            break;
        }
        case 2: {
            return p.z;
            break;
        }
        default: {
            throw std::runtime_error("axis out of range\n");
        }
    }
}

template <typename POINT_CLOUD_TYPE>
double
distance(const POINT_CLOUD_TYPE& p1, const POINT_CLOUD_TYPE& p2,
         const std::function<double(const POINT_CLOUD_TYPE& p, const int axis)>& valueAtFunc = ::at<POINT_CLOUD_TYPE>)
{
    double result = 0.0;
    for (int i = 0; i < 3; ++i) {
        result += std::pow(valueAtFunc(p1, i) - valueAtFunc(p2, i), 2);
    }
    return std::sqrt(result);
}

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

    using DBSCAN = clustering::DBSCAN<3, PointCloudType, decltype(inputPcl->points)>;
    DBSCAN::Ptr dbscan = std::make_shared<DBSCAN>(eps, minPoints, ::distance<PointCloudType>, ::at<PointCloudType>);
    std::vector<std::vector<int>> clusterIndices = dbscan->estimateClusterIndices(inputPcl->points);
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
