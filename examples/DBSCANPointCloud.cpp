/**
 * @file    DBSCANPointCloud.cpp
 *
 * @author  btran
 *
 */

#include <pcl/common/time.h>

#include <dbscan/dbscan.hpp>

#include "AppUtility.hpp"

namespace
{
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
            throw std::runtime_error("axis out of range");
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

using PointCloudType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointCloudType>;
using PointCloudPtr = PointCloud::Ptr;

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

    PointCloudPtr inputPcl(new PointCloud);
    if (pcl::io::loadPCDFile(pclFilePath, *inputPcl) == -1) {
        std::cerr << "Failed to load pcl file\n";
        return EXIT_FAILURE;
    }
    std::cout << "number of points: " << inputPcl->size() << "\n";

    using DBSCAN = clustering::DBSCAN<3, PointCloudType, decltype(inputPcl->points)>;
    DBSCAN::Ptr dbscan = std::make_shared<DBSCAN>(eps, minPoints, ::distance<PointCloudType>, ::at<PointCloudType>);

    std::vector<std::vector<int>> clusterIndices;
    timer.reset();
    for (int i = 0; i < NUM_TEST; ++i) {
        clusterIndices = dbscan->estimateClusterIndices(inputPcl->points);
    }

    std::cout << "number of clusters: " << clusterIndices.size() << "\n";
    std::cout << "processing time (cpu): " << timer.getTime() / NUM_TEST << "[ms]\n";

    std::vector<PointCloudPtr> clusters = ::toClusters<PointCloudType>(clusterIndices, inputPcl);

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
