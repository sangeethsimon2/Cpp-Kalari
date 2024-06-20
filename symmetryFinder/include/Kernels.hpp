#ifndef KERNELSHEADER
#define KERNELSHEADER

#include "Point.hpp"
#include <../include/Eigen/Dense>
#include <../include/Eigen/SVD>
#include <unordered_map>
#include <tuple>

namespace Kernels{

   //Custom hashfunction that maps 2 coordinates to a size_t value
   struct hashFunc_floatkey{
     size_t operator()(const Point2D& otherPoint) const{
       size_t h1 = std::hash<double>()(otherPoint.m_x);
       size_t h2 = std::hash<double>()(otherPoint.m_y);
       return (h1 ^ (h2 << 1));
     }
   };

   // Custom hash function that converts the double valued coord pair into in valued pair
   struct hashFunc_longlongintkey {
     size_t operator()(const Point2D& otherPoint) const {
       //  the coordinates
       long long _intEqX = static_cast<long long>(std::round(otherPoint.m_x / scalingFactor));
       long long _intEqY = static_cast<long long>(std::round(otherPoint.m_y / scalingFactor));

       // Hash the quantized coordinates
       size_t h1 = std::hash<long long>()(_intEqX);
       size_t h2 = std::hash<long long>()(_intEqY);
       return (h1 ^ (h2 << 1));
     }
   };

   //Custom equality function to allow key comparisons
   struct equalFunc{
     bool operator() ( const Point2D& firstPoint, const Point2D& secondPoint) const{
       return ( (std::fabs(firstPoint.m_x-secondPoint.m_x)<ComparisonEpsilon) && (std::fabs(firstPoint.m_y-secondPoint.m_y)<ComparisonEpsilon));
     }
   };

   // Function to center the points
   void centerPointsToOriginAndStoreCentroid(std::vector<Point2D>& points, double& x_centroid, double& y_centroid) {
     double sumX = 0.0;
     double sumY = 0.0;
     int n = points.size();
     // Calculate the mean of the points
     for (const auto& point : points) {
       sumX += point.m_x;
       sumY += point.m_y;
     }
     double meanX = sumX / n;
     double meanY = sumY / n;
     x_centroid = meanX;
     y_centroid = meanY;
     // Center each point to the origin
     for (auto& point : points) {
       point.m_x -= meanX;
       point.m_y -= meanY;
     }
   }

   //Routine to construct a reflection of a given point about a given line
   Point2D constructReflection(const Point2D& inputPoint, const double a, const double b, const double c){
     double x1 = inputPoint.m_x;
     double y1 = inputPoint.m_y;
     double denom = a * a + b * b;
     double x2 = x1 - 2 * a * (a * x1 + b * y1 + c) / denom;
     double y2 = y1 - 2 * b * (a * x1 + b * y1 + c) / denom;
     Point2D reflectedPoint(-inputPoint.m_Index, x2, y2);
     return reflectedPoint;
   }

   //Method to check that takes a set of point, and a line and checks for symmetry about this line
   //Returns false even when the first unsymmetrical point in encountered
   //Returns true when all the points are symmetrical about this line
   bool IsSymmetricSet(const std::vector<Point2D>& points, const double a, const double b, const double c){
     //Declare a bool that acts as a flag to indicate if the point cloud is symmetric
     //Default: assume that the set is symmetric
     bool b_isSymmetricSet = true;
     //Declare an unordered_map that has key: Point2D and value: number of times the point is encountered
     std::unordered_map<Point2D, int, hashFunc_longlongintkey, equalFunc> Point2DSet;

     //For each new point encountered from the container, increment a counter to indicate its presence
     for(const auto& eachPoint: points)
       Point2DSet[eachPoint]++;

     //Loop over each point and check for symmetry about the potential line passed as argument
     for (const auto& eachPoint : points){
       //For each point encountered from the container, Call routine to construct a symmetrical point
       Point2D reflectedPoint = constructReflection( eachPoint, a, b, c);

       //Check if this reflected point does not exist in the given set
       if (Point2DSet.find(reflectedPoint) == Point2DSet.end()) {
         b_isSymmetricSet = false;
         return b_isSymmetricSet;// Indicates that the set of points is not symmetrical wrt the given line
       }
       //If it does exist,
       else {
         if(Point2DSet[eachPoint]>0)
           Point2DSet[eachPoint]--;
         if(Point2DSet[eachPoint]==0 && Point2DSet[reflectedPoint]==0)
           Point2DSet.erase(reflectedPoint);
       }
     }
     return b_isSymmetricSet;
   }

   //Method to print the details of the SVD decomposition
   void printSVDDetails(const Eigen::MatrixXd& pointMatrix, const Eigen::Vector2d& singularValues, const Eigen::MatrixXd& U, Eigen::MatrixXd& V){
     std::cout << "Singular values (in decreasing order):" << std::endl;
     std::cout << singularValues << std::endl;
     std::cout << "Orthonormal eigenvectors (U matrix, in corresponding order):" << std::endl;
     std::cout << U << std::endl;
     std::cout << "V matrix (for completeness, in corresponding order):" << std::endl;
     std::cout << V << std::endl;
     std::cout << "A * V = \n" << pointMatrix * V << "\n\n";
     std::cout << "Principle Directions = \n";

     for(int i=0; i< U.cols(); i++){
       std::cout<<"Direction "<<i<<"\n";
       std::cout << (pointMatrix * V).col(i).normalized()<<"\n";
     }
   }

   //Method to check for symmetry in a poind cloud and return the line of symmetry if found.
   //Input: vector of points and double values to hold the found symmetry if any
   bool checkForSymmetryInPointCloud(std::vector<Point2D>& points, double& a, double& b, double& c){
     //Assume that the point cloud is not symmetrical
     bool isSetSymmetrical = false;

     //Declare a Variable to store the centroid points
     double x_centroid = 0.; double y_centroid = 0.;

     //Declare variables to store the coefficients of the potential lines of symmetry
     double temp_a =0.; double temp_b =0.; double temp_c=0.;

     //Keep a Copy of the original points
     const std::vector<Point2D> originalPoints = points;

     //Center the points to origin and rewrite them
     Kernels::centerPointsToOriginAndStoreCentroid(points, x_centroid, y_centroid);

     //Construct an Eigen matrix of dimension 2(x,y coords) X N(=number of points)
     Eigen::MatrixXd pointMatrix(2, points.size());

     //Fill the matrix with the point coordinates
     for (int i = 0; i < points.size(); ++i) {
       pointMatrix(0, i) = points[i].m_x;  // x coordinate
       pointMatrix(1, i) = points[i].m_y;  // y coordinate
     }

     // Perform Singular Value Decomposition
     Eigen::JacobiSVD<Eigen::MatrixXd> svd(pointMatrix, Eigen::ComputeFullU | Eigen::ComputeFullV);

     //Store the eigenvalues
     Eigen::VectorXd singularValues = svd.singularValues();

     //Store the U ( left eigenvector matrix)
     Eigen::MatrixXd U = svd.matrixU();

     //Store the V (right eigenvector matrix)
     Eigen::MatrixXd V = svd.matrixV();
 #ifdef DEBUG
     printSVDDetails(pointMatrix, singularValues, U, V);
 #endif
     //Construct potential lines of symmetry through the centroid of the poind cloud

     //The first candidate line would have a direction vector defined by V[:1] = [vx, vy]
     //For a line equation ax+by+c=0; a = vy, b = -vx, c = -(a*x_cent+b*y_cent)
     temp_a = U(1,1);
     temp_b = -U(0,1);
     temp_c = -(temp_a*x_centroid+temp_b*y_centroid);

     if(isSetSymmetrical=Kernels::IsSymmetricSet(originalPoints,  temp_a,  temp_b,  temp_c)){
       a=temp_a; b=temp_b;c=temp_c;
       return isSetSymmetrical;
     }
     else{
       //The second candidate line would have a direction vector defined by V[:1] = [vx, vy]
       //For a line equation ax+by+c=0; a = vy, b = -vx, c = -(a*x_cent+b*y_cent)
       temp_a = U(1,0);
       temp_b = -U(0,0);
       temp_c = -(temp_a*x_centroid+temp_b*y_centroid);

       if(isSetSymmetrical=Kernels::IsSymmetricSet(originalPoints,  temp_a,  temp_b,  temp_c)){
         a=temp_a; b=temp_b;c=temp_c;
         return isSetSymmetrical;
       }
     }
     return isSetSymmetrical;
   }
}
#endif