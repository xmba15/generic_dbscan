/**
 * @file    TestDBSCAN.cpp
 *
 * @author  btran
 *
 * Copyright (c) organization
 *
 */

#include <functional>
#include <random>

#include <gtest/gtest.h>

#include <dbscan/dbscan.hpp>

namespace
{
class RandDouble
{
 public:
    RandDouble(const double low, const double high)
        : m_genFunc(std::bind(std::uniform_real_distribution<>(low, high), std::default_random_engine()))
    {
    }

    double operator()() const
    {
        return m_genFunc();
    }

 private:
    std::function<double()> m_genFunc;
};

class Point3D : public std::array<double, 3>
{
 public:
    Point3D(const double x, const double y, const double z)
    {
        (*this)[0] = x;
        (*this)[1] = y;
        (*this)[2] = y;
    }
};

using DBSCAN = clustering::DBSCAN<3, Point3D>;
}  // namespace

TEST(TestDBSCAN, TestInitialization)
{
    const double eps = 0.2;
    const int minPoints = 3;

    ASSERT_NO_THROW(::DBSCAN dbscan(eps, minPoints));
}

TEST(TestDBSCAN, TestClustering)
{
    const double eps = 5;
    const int minPoints = 3;

    const int numPoints = 1000;
    ::DBSCAN::VecPointType points;
    points.reserve(numPoints);
    {
        auto doubleGenerator = ::RandDouble(0.0, 200.0);
        for (int i = 0; i < numPoints; ++i) {
            points.emplace_back(doubleGenerator(), doubleGenerator(), doubleGenerator());
        }
    }

    ::DBSCAN dbscan(eps, minPoints);
    ASSERT_NO_THROW(dbscan.estimateClusterIndices(points));
    ASSERT_NO_THROW(dbscan.estimateClusters(points));
}
