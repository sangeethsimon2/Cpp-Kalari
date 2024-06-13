#ifndef KERNELSHEADER
#define KERNELSHEADER

#include "Point.hpp"
#include <unordered_map>

namespace Kernels{

   //Custom hashfunction that maps 2 coordinates to a size_t value
    struct hashFunc{
        size_t operator()(const Point2D& otherPoint) const{
        size_t h1 = std::hash<double>()(otherPoint.m_x);
        size_t h2 = std::hash<double>()(otherPoint.m_y);
        return (h1 ^ (h2 << 1));
        }
    };

   //Custom equality function to allow key comparisons
    struct equalFunc{
        bool operator() ( const Point2D& firstPoint, const Point2D& secondPoint) const{
          return ( (std::fabs(firstPoint.m_x-secondPoint.m_x)<epsilon) && (std::fabs(firstPoint.m_y-secondPoint.m_y)<epsilon));
        }
    };

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


   bool IsSymmetricSet(const std::vector<Point2D>& points, const double a, const double b, const double c){
      //Declare a bool that acts as a flag to indicate if the point cloud is symmetric
      //Default: assume that the set is symmetric
      bool isSymmetricSet = true;

      //Declare an unordered_map that has key: Point2D and value: number of times the point is encountered
      std::unordered_map<Point2D, int, hashFunc, equalFunc> Point2DSet;

      //For each new point encountered from the container, increment a counter to indicate its presence
      for(const auto& eachPoint: points)
        Point2DSet[eachPoint]++;


      for (const auto& eachPoint : points){
        //For each point encountered from the container, Call routine to construct a symmetrical point
        Point2D reflectedPoint = constructReflection( eachPoint, a, b, c);
        //Check if this reflected point does not exist in the given set
        if (Point2DSet.find(reflectedPoint) == Point2DSet.end()) {
            isSymmetricSet = false;
            return isSymmetricSet;// Indicates that the set of points is not symmetrical wrt the given line
        }
        //If it does exist,
        else {
              if(Point2DSet[eachPoint]>0)
                Point2DSet[eachPoint]--;
              if(Point2DSet[eachPoint]==0 && Point2DSet[reflectedPoint]==0)
                Point2DSet.erase(reflectedPoint);
        }
      }
      return isSymmetricSet;
    }

    bool checkForSymmetryInPointCloud(const std::vector<Point2D>& points, double& a, double& b, double& c){
      //Assume that the set is not symmetrical to begin with
      bool isSetSymmetrical = false;

      //Logic: Loop over every pair of non-identical points, construct a perpendicular bisector to a line joining them,
      //For each point, construct a symmetrical point about this line and check for it's existence in the given set
      //Even if one point does not have a matching
      //Loop over pair of points
      for (int i ={}; i<points.size(); i++){
        for (int j=i+1; j<points.size(); j++){
          //Go to next iteration if the points are same.
          if(points[i]==points[j]) continue;
          //Compute the sum of x coords of every pair
          double sumOfXCoords = points[i].m_x + points[j].m_x;
          //Compute the sum of y coords of every pair
          double sumOfYCoords = points[i].m_y + points[j].m_y;
          //Compute the difference of x coords of every pair
          double diffOfXCoords = points[j].m_x - points[i].m_x;
          //Compute the difference of y coords of every pair
          double diffOfYCoords = points[j].m_y - points[i].m_y;

          //Compute coefficients of the perpendicular bisector line
          // Line form ax + by + c = 0
          a = 2.0*(diffOfXCoords);
          b = 2.0*(diffOfYCoords);
          c = -1.0*(sumOfYCoords*diffOfYCoords + sumOfXCoords*diffOfXCoords);

          //Check for symmetry; break when the first symmetry line is discovered
          //Continue searching if the current line is not a symmetry line
          if(isSetSymmetrical = IsSymmetricSet(points, a, b, c))
            break;
        }
      }
    return isSetSymmetrical;
  }

    //Custom sign function
    template <typename T> int Sign(T val) {
       return (T(0) < val) - (val < T(0));
    }
}
#endif