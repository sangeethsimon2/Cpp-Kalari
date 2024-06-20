#include </home/cfd/simon/.local/include/gtest/gtest.h>

#include<cmath>
#include<iostream>
#include <cstdlib>
#include <ctime>

#include "Point.hpp"
#include "Kernels.hpp"

//#########################################[HASH FUNCTION TEST SUITE]################
// Test for checking if the hashFunc_float returns same key for exactly similar points
TEST(Point2DHashFunctionTest, SimilarPointsHaveSameKey_hashFunc_float){
    Point2D point1(1, 1.0000000000, 2.0000000000);
    Point2D point2(2, 1.0000000000, 2.0000000000);

    Kernels::hashFunc_floatkey hasher;
    size_t hash1 = hasher(point1);
    size_t hash2 = hasher(point2);

    EXPECT_EQ(hash1, hash2);
}

// Test for checking if the Hash function hashFunc_longlongintKey returns same key for exactly similar points
TEST(Point2DHashFunctionTest, SimilarPointsHaveSameKey_hashFunc_longlongint){
    Point2D point1(1, 1.0000000000, 2.0000000000);
    Point2D point2(2, 1.0000000000, 2.0000000000);

    Kernels::hashFunc_longlongintkey hasher;
    size_t hash1 = hasher(point1);
    size_t hash2 = hasher(point2);

    EXPECT_EQ(hash1, hash2);
}



// Test for checking if the hashFunc_float returns different keys for dissimilar points
TEST(Point2DHashFunctionTest, DissimilarPointsHaveDifferentKey_hashFunc_float){

    Point2D point1(1, 1.0, 2.0);
    Point2D point2(2, 3.0, 4.0);

    Kernels::hashFunc_floatkey hasher;
    size_t hash1 = hasher(point1);
    size_t hash2 = hasher(point2);

    EXPECT_NE(hash1, hash2);
}

// Test for checking if the hashFunc_longlongint returns different keys for dissimilar points
TEST(Point2DHashFunctionTest, DissimilarPointsHaveDifferentKey_hashFunc_longlongint){

    Point2D point1(1, 1.0, 2.0);
    Point2D point2(2, 3.0, 4.0);

    Kernels::hashFunc_longlongintkey hasher;
    size_t hash1 = hasher(point1);
    size_t hash2 = hasher(point2);

    EXPECT_NE(hash1, hash2);
}

//##################################################################################################

//#########################################[REFLECTION METHODS TEST SUITE]################

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

//##################################################################################################

//#########################################[SMALL POINT CLOUD SYMMETRY TEST SUITE]################

//Test for checking if a known pair of points are symmetrical wrt a known symmetry line
TEST(SmallPoint2DCloudSymmetryTest, 2PointsSymmetryAboutXaxis){

    //Define the point cloud on a plane
    std::vector<Point2D> points{
      {1, 1.0, 1.0},
      {2, 1.0, -1.0},
    };
    double a = 0.; double b = -1.; double c = 0.;
    bool isSetSymmetrical = Kernels::checkForSymmetryInPointCloud(points, a, b, c);
    EXPECT_TRUE(isSetSymmetrical);
}

//Test for checking if a known pair of points are symmetrical wrt a known symmetry line
TEST(SmallPoint2DCloudSymmetryTest, 2PointsSymmetryAboutAKnownSymmetryLine){

    //Define the point cloud on a plane
    std::vector<Point2D> points{
      {1, 4.0, 2.0},
      {2, 1.0, 8.0},
    };
    double a = 0.447214; double b = -0.894427; double c = 3.3541;
    bool isSetSymmetrical = Kernels::checkForSymmetryInPointCloud(points, a, b, c);
    EXPECT_TRUE(isSetSymmetrical);
}

//Test to check for symmetry within a point cloud containing duplicate points about a known symmetry line
TEST(SmallPoint2DCloudSymmetryTest, duplicityType1InSymmetryAboutAKnownSymmetryLine){

    //Define the point cloud on a plane
    std::vector<Point2D> points{
      {1, 4.0, 2.0},
      {2, 1.0, 8.0},
      {2, 1.0, 8.0},
    };
    double a = 0.447214; double b = -0.894427; double c = 3.3541;
    bool isSetSymmetrical = Kernels::checkForSymmetryInPointCloud(points, a, b, c);

    EXPECT_TRUE(isSetSymmetrical);
}


//Test for checking if a known group of points are NOT symmetrical wrt a line
TEST(SmallPoint2DCloudSymmetryTest, 3PointsNonSymmetryAboutALine){

    //Define the point cloud on a plane
    std::vector<Point2D> points{
      {1, 4.0, 2.0},
      {2, 1.0, 8.0},
      {3, 2.0, 9.0}
    };
    double a = 2.; double b = -4.; double c = 15.;
    bool isSetSymmetrical = Kernels::checkForSymmetryInPointCloud(points, a, b, c);

    EXPECT_FALSE(isSetSymmetrical);
}


//Test to check for symmetry within a point cloud containing duplicate points about a known symmetry line
TEST(SmallPoint2DCloudSymmetryTest, duplicityType2InSymmetryAboutAKnownSymmetryLine){

    //Define the point cloud on a plane
    std::vector<Point2D> points{
      {1, 4.0, 2.0},
      {1, 4.0, 2.0},
      {1, 4.0, 2.0},
      {1, 4.0, 2.0},
      {2, 1.0, 8.0}
    };
    double a = 2.; double b = -4.; double c = 15.;
    //bool isSetSymmetrical = Kernels::IsSymmetricSet(points, a, b, c);
    bool isSetSymmetrical = Kernels::checkForSymmetryInPointCloud(points, a, b, c);

    EXPECT_TRUE(isSetSymmetrical);
}
//##################################################################################################

//#########################################[LARGE POINT CLOUD SYMMETRY TEST SUITE]################

//Test to check for symmetry of a curated set of points on a circle - intended to pass.
//This test also features points lying on the symmetry line and serve as a check on the quality of hashing
TEST(LargePoint2DCloudSymmetryTest, LargePointCloudSymmetryAboutDiagonalOfACircle_symmetricalPoints){
    size_t N = 2;
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

    EXPECT_TRUE(isSetSymmetrical);
}

//Test to check for symmetry of random set of points on a circle - intended to fail in its current config.
TEST(LargePoint2DCloudSymmetryTest, LargePointCloudSymmetryAboutDiagonalOfACircle_randomPoints){
    size_t N = 3;
    double R = 1.;
    bool isSetSymmetrical = true;
    double a=0.; double b=0.; double c=0.;

    std::vector<Point2D> points;
    points.reserve(N); // Reserve space for N points

    const double PI = 3.14159265358979323846;
    std::srand(std::time(0));

    for (int i = 0; i < N; ++i) {
        //double theta = 2.0 * PI * i / N; // Angle in radians
        double x = static_cast<double>(std::rand()) / RAND_MAX - 0.5;
        double y = sqrt(R*R - x*x);
        points.emplace_back(i, x, y); // Create a Point2D and add to vector
    }

    isSetSymmetrical = Kernels::checkForSymmetryInPointCloud(points, a, b, c);

    EXPECT_FALSE(isSetSymmetrical);
}

//Test to check for symmetry of random set of points on a unit square centered at 0,0 about its horizontal axis
TEST(LargePoint2DCloudSymmetryTest, LargePointCloudOnAnUnitSquareSymmetryAboutHorizontalAxis){
    size_t N = 5000;
    bool isSetSymmetrical = true;
    double a=0.; double b=0.; double c=0.;

    std::vector<Point2D> points;
    points.reserve(N); // Reserve space for N points

    std::srand(std::time(0));

    for (size_t i = 0; i < N / 2; ++i) {
        double x = static_cast<double>(std::rand()) / RAND_MAX - 0.5; // Random x in [-0.5, 0.5]
        double y = static_cast<double>(std::rand()) / RAND_MAX / 2 - 0.25; // Random y in [-0.25, 0.25)

        // Add point and its reflection
        points.emplace_back(i, x, y);
        points.emplace_back(-i, x, -y);
    }
    isSetSymmetrical = Kernels::checkForSymmetryInPointCloud(points, a, b, c);

    EXPECT_TRUE(isSetSymmetrical);
}
//##################################################################################################

//Driver to initiate tests
int main(int argc, char** argv){
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}