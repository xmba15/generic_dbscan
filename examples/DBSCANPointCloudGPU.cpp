/**
 * @file    DBSCANPointCloudGPU.cpp
 *
 * @author  btran
 *
 */

#include <iostream>

#include <pcl/common/time.h>

#include "AppUtility.hpp"
#include <dbscan/CudaUtils.cuh>
#include <dbscan/DBSCAN.cuh>

namespace
{
using PointCloudType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointCloudType>;
using PointCloudPtr = PointCloud::Ptr;
using DBSCAN = clustering::cuda::DBSCAN<PointCloudType>;

auto pclViewer = initializeViewer();
auto timer = pcl::StopWatch();
int NUM_TEST = 10;
}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 4) {
        std::cerr << "Usage: [app] [path/to/pcl/file] [eps] [min/points]\n";
        return EXIT_FAILURE;
    }

    const std::string pclFilePath = argv[1];
    const double eps = std::atof(argv[2]);
    const int minPoints = std::atoi(argv[3]);

    PointCloudPtr inCloud(new PointCloud);
    if (pcl::io::loadPCDFile(pclFilePath, *inCloud) == -1) {
        std::cerr << "Failed to load pcl file\n";
        return EXIT_FAILURE;
    }
    std::cout << "number of points: " << inCloud->size() << "\n";

    cuda::utils::warmUpGPU();

    DBSCAN::Param param{.pointDimension = 3, .eps = eps, .minPoints = minPoints};
    DBSCAN dbscanHandler(param);
    std::vector<std::vector<int>> clusterIndices;

    timer.reset();
    for (int i = 0; i < NUM_TEST; ++i) {
        clusterIndices = dbscanHandler.run(inCloud->points.data(), inCloud->size());
    }

    std::cout << "number of clusters: " << clusterIndices.size() << "\n";
    std::cout << "processing time (gpu): " << timer.getTime() / NUM_TEST << "[ms]\n";

    std::vector<PointCloudPtr> clusters = ::toClusters<PointCloudType>(clusterIndices, inCloud);

    auto [colors, colorHandlers] = ::initPclColorHandlers<PointCloudType>(clusters);

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
