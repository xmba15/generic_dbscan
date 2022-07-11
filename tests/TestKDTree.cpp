/**
 * @file    TestKDTree.cpp
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

using KDTree = clustering::KDTree<3, Point3D>;
}  // namespace

TEST(TestKDTree, TestInitialization)
{
    const int numPoints = 1000;
    KDTree::VecPointType points;
    points.reserve(numPoints);
    {
        auto doubleGenerator = ::RandDouble(0.0, 200.0);
        for (int i = 0; i < numPoints; ++i) {
            points.emplace_back(doubleGenerator(), doubleGenerator(), doubleGenerator());
        }
    }

    {
        ASSERT_NO_THROW(::KDTree kdTree(points););
    }

    {
        ::KDTree::ValueAtFunctionType valueAtFunc = [](const ::KDTree::PointType& p, const int axis) {
            if (axis >= 3) {
                throw std::runtime_error("axis out of range\n");
            }

            return p[axis];
        };

        const auto distFunc = [](const ::KDTree::PointType& p1, const ::KDTree::PointType& p2,
                                 const ::KDTree::ValueAtFunctionType& valueAtFunc) {
            double result = 0.0;
            for (int i = 0; i < 3; ++i) {
                result += std::fabs(valueAtFunc(p1, i) - valueAtFunc(p2, i));
            }
            return std::sqrt(result);
        };

        ASSERT_NO_THROW(::KDTree kdTree(points, distFunc, valueAtFunc););
    }
}

TEST(TestKDTree, TestNearestSearch)
{
    {
        const int numPoints = 10000;
        KDTree::VecPointType points;
        points.reserve(numPoints);
        {
            auto doubleGenerator = ::RandDouble(0.0, 200.0);
            for (int i = 0; i < numPoints; ++i) {
                points.emplace_back(doubleGenerator(), doubleGenerator(), doubleGenerator());
            }
        }

        ::KDTree kdTree(points);

        KDTree::PointType queryPoint(50.0, 40.0, 40.6);

        int expectedNearestIdx = kdTree.nearestSearch(queryPoint);

        int bruteForcedNearestIdx = -1;
        double bruteForcedMinDist = std::numeric_limits<double>::max();
        for (int i = 0; i < numPoints; ++i) {
            const auto& curPoint = points[i];
            const double curDist = kdTree.distFunc()(queryPoint, curPoint, kdTree.valueAtFunc());
            if (curDist < bruteForcedMinDist) {
                bruteForcedNearestIdx = i;
                bruteForcedMinDist = curDist;
            }
        }

        EXPECT_EQ(expectedNearestIdx, bruteForcedNearestIdx);
    }
}

TEST(TestKDTree, TestRadiusSearch)
{
    {
        const int numPoints = 10000;
        KDTree::VecPointType points;
        points.reserve(numPoints);
        {
            auto doubleGenerator = ::RandDouble(0.0, 200.0);
            for (int i = 0; i < numPoints; ++i) {
                points.emplace_back(doubleGenerator(), doubleGenerator(), doubleGenerator());
            }
        }

        ::KDTree kdTree(points);

        KDTree::PointType queryPoint(50.0, 40.0, 40.6);
        const double radius = 10;

        std::vector<int> expectedIndices = kdTree.radiusSearch(queryPoint, radius);

        std::vector<int> bruteForcedIndices;
        for (int i = 0; i < numPoints; ++i) {
            const auto& curPoint = points[i];
            const double curDist = kdTree.distFunc()(queryPoint, curPoint, kdTree.valueAtFunc());
            if (curDist < radius) {
                bruteForcedIndices.emplace_back(i);
            }
        }

        EXPECT_EQ(expectedIndices.size(), bruteForcedIndices.size());

        std::sort(expectedIndices.begin(), expectedIndices.end());
        std::sort(bruteForcedIndices.begin(), bruteForcedIndices.end());
        std::vector<int> idxIndices(expectedIndices.size());
        std::iota(idxIndices.begin(), idxIndices.end(), 0);
        EXPECT_TRUE(std::all_of(idxIndices.begin(), idxIndices.end(), [&](const int idxIdx) {
            return expectedIndices[idxIdx] == bruteForcedIndices[idxIdx];
        }));
    }
}

TEST(TestKDTree, TestKNNSearch)
{
    {
        const int numPoints = 10000;
        KDTree::VecPointType points;
        points.reserve(numPoints);
        {
            auto doubleGenerator = ::RandDouble(0.0, 200.0);
            for (int i = 0; i < numPoints; ++i) {
                points.emplace_back(doubleGenerator(), doubleGenerator(), doubleGenerator());
            }
        }

        ::KDTree kdTree(points);

        KDTree::PointType queryPoint(50.0, 40.0, 40.6);
        const int k = 10;

        std::vector<int> expectedIndices = kdTree.knnSearch(queryPoint, k);

        std::vector<int> allIndices(numPoints);
        std::iota(allIndices.begin(), allIndices.end(), 0);
        std::sort(allIndices.begin(), allIndices.end(), [&](const int idx1, const int idx2) {
            return kdTree.distFunc()(queryPoint, points[idx1], kdTree.valueAtFunc()) <
                   kdTree.distFunc()(queryPoint, points[idx2], kdTree.valueAtFunc());
        });

        std::vector<int> bruteForcedIndices(allIndices.begin(), allIndices.begin() + k);

        EXPECT_EQ(expectedIndices.size(), bruteForcedIndices.size());

        std::sort(expectedIndices.begin(), expectedIndices.end());
        std::sort(bruteForcedIndices.begin(), bruteForcedIndices.end());
        std::vector<int> idxIndices(expectedIndices.size());
        std::iota(idxIndices.begin(), idxIndices.end(), 0);
        EXPECT_TRUE(std::all_of(idxIndices.begin(), idxIndices.end(), [&](const int idxIdx) {
            return expectedIndices[idxIdx] == bruteForcedIndices[idxIdx];
        }));
    }
}
