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
          return ( ((firstPoint.m_x-secondPoint.m_x)<epsilon) && ((firstPoint.m_y-secondPoint.m_y)<epsilon));
        }
    };

    //Routine to construct a reflection of a given point about a given line
    Point2D constructReflection(const Point2D& inputPoint, const int a, const int b, const int c){
        double x1 = inputPoint.m_x;
        double y1 = inputPoint.m_y;
        double denom = a * a + b * b;
        double x2 = x1 - 2 * a * (a * x1 + b * y1 + c) / denom;
        double y2 = y1 - 2 * b * (a * x1 + b * y1 + c) / denom;
        Point2D reflectedPoint(-inputPoint.m_Index, x2, y2);
        return reflectedPoint;
    }


   bool IsSymmetricSet(const std::vector<Point2D>& points, const int a, const int b, const int c){
      //Declare an unordered_map that has key: Point2D and value: number of symmetry associations
      std::unordered_map<Point2D, int, hashFunc, equalFunc> Point2DSet;

      //For each new point encountered from the container, increment a counter to indicate that its reflection needs to be searched for
      for(const auto& eachPoint: points)
        Point2DSet[eachPoint]++;


      for (const auto& eachPoint : points){
        //For each point encountered from the container, Call routine to construct a symmetrical point
        Point2D reflectedPoint = constructReflection( eachPoint, a, b, c);
        //Check if this reflected point does not exist in the given set
        if (Point2DSet.find(reflectedPoint) == Point2DSet.end()) {
            return false;// Indicates that the set of points is not symmetrical wrt the given line
        }
        //If it does exist,
        else {
            Point2DSet[reflectedPoint]--;
            if (Point2DSet[reflectedPoint] == 0) {
              Point2DSet.erase(reflectedPoint);
            }
        }
      }
      return true;
    }

    //Custom sign function
    template <typename T> int Sign(T val) {
       return (T(0) < val) - (val < T(0));
    }


}
#endif