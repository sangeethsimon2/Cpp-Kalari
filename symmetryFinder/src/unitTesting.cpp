#include </home/cfd/simon/.local/include/gtest/gtest.h>

#include<cmath>
#include<iostream>

#include "Point.hpp"
#include "Kernels.hpp"

// Test for checking if the Hash function returns same key for exactly similar points
TEST(Point2DHashFunctionTest, SimilarPointsHaveSameKey){
    Point2D point1(1, 1.0000000000, 2.0000000000);
    Point2D point2(2, 1.0000000000, 2.0000000000);

    Kernels::hashFunc hasher;
    size_t hash1 = hasher(point1);
    size_t hash2 = hasher(point2);

    EXPECT_EQ(hash1, hash2);
}

// Test for checking if the Hash function returns different keys for dissimilar points
TEST(Point2DHashFunctionTest, DissimilarPointsHaveDifferentKey){

    Point2D point1(1, 1.0, 2.0);
    Point2D point2(2, 3.0, 4.0);

    Kernels::hashFunc hasher;
    size_t hash1 = hasher(point1);
    size_t hash2 = hasher(point2);

    EXPECT_NE(hash1, hash2);
}

// Test for checking if the reflection code's accuracy for reflection about x axis
TEST(Point2DReflectionTest, ReflectionOnXAxis) {
    Point2D point(1, 1.0, 2.0);
    const Point2D expectedReflectedPoint(1, 1.0, -2.0);
    Point2D reflectedPoint = Kernels::constructReflection(point, 0, 1, 0); // Line y = 0

    EXPECT_EQ(reflectedPoint, expectedReflectedPoint);
}

// Test for checking if the reflection code's accuracy for reflection about y axis
TEST(Point2DReflectionTest, ReflectionOnYAxis) {
    Point2D point(2, 1.0, 2.0);
    const Point2D expectedReflectedPoint(2, -1.0, 2.0);
    Point2D reflectedPoint = Kernels::constructReflection(point, 1, 0, 0); // Line x = 0

    EXPECT_EQ(reflectedPoint, expectedReflectedPoint);
}

//Test for checking if a known pair of points are symmetrical wrt a known symmetry line
TEST(Point2DCloudSymmetryTest, 2PointsSymmetryAboutAKnownSymmetryLine){

    //Define the point cloud on a plane
    std::vector<Point2D> points{
      {1, 4.0, 2.0},
      {2, 1.0, 8.0},
    };
    double a = 2.; double b = -4.; double c = 15.;
    bool isSetSymmetrical = Kernels::IsSymmetricSet(points, a, b, c);

    EXPECT_TRUE(isSetSymmetrical);
}

//Test for checking if a known group of points are NOT symmetrical wrt a line
TEST(Point2DCloudSymmetryTest, 3PointsNonSymmetryAboutALine){

    //Define the point cloud on a plane
    std::vector<Point2D> points{
      {1, 4.0, 2.0},
      {2, 1.0, 8.0},
      {3, 2.0, 9.0}
    };
    double a = 2.; double b = -4.; double c = 15.;
    bool isSetSymmetrical = Kernels::IsSymmetricSet(points, a, b, c);

    EXPECT_FALSE(isSetSymmetrical);
}

//Test to check for symmetry within a point cloud containing duplicate points about a known symmetry line
TEST(Point2DCloudSymmetryTest, duplicityType1InSymmetryAboutASymmetryLine){

    //Define the point cloud on a plane
    std::vector<Point2D> points{
      {1, 4.0, 2.0},
      {2, 1.0, 8.0},
      {2, 1.0, 8.0},
      {2, 1.0, 8.0}
    };
    double a = 2.; double b = -4.; double c = 15.;
    bool isSetSymmetrical = Kernels::IsSymmetricSet(points, a, b, c);

    EXPECT_TRUE(isSetSymmetrical);
}

//Test to check for symmetry within a point cloud containing duplicate points about a known symmetry line
TEST(Point2DCloudSymmetryTest, duplicityType2InSymmetryAboutASymmetryLine){

    //Define the point cloud on a plane
    std::vector<Point2D> points{
      {1, 4.0, 2.0},
      {1, 4.0, 2.0},
      {1, 4.0, 2.0},
      {1, 4.0, 2.0},
      {2, 1.0, 8.0}
    };
    double a = 2.; double b = -4.; double c = 15.;
    bool isSetSymmetrical = Kernels::IsSymmetricSet(points, a, b, c);

    EXPECT_TRUE(isSetSymmetrical);
}

TEST(LargePoint2DCloudSymmetryTest, LargePointCloudSymmetryAboutDiagonalOfACircle){
    size_t N = 10;
    double R = 1.0;
    bool isSetSymmetrical = true;
    double a=0.; double b=0.; double c=0.;

    std::vector<Point2D> points;
    points.reserve(N); // Reserve space for N points

    const double PI = 3.14159265358979323846;
    for (int i = 0; i < N; ++i) {
        double theta = 2.0 * PI * i / N; // Angle in radians
        double x = R * std::cos(theta);
        double y = R * std::sin(theta);
        points.emplace_back(i, x, y); // Create a Point2D and add to vector
    }
    isSetSymmetrical = Kernels::checkForSymmetryInPointCloud(points, a, b, c);

    EXPECT_FALSE(isSetSymmetrical);
}

//Driver to initiate tests
int main(int argc, char** argv){
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}