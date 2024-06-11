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
        bool operator()(const Point2D& firstPoint, const Point2D& secondPoint){
          return ( ((firstPoint.m_x-secondPoint.m_x)<epsilon) && ((firstPoint.my-secondPoint.my)<epsilon));
        }
    };


   bool IsSymmetricSet(const std::vector<Point2D>& points, const int a, const int b, const int c){
      //Declare an unordered_map that has key: Point2D and value: number of symmetry associations
      std::unordered_map<Point2D, int, hashFunc, equalFunc> Point2DSet;

    }

}
#endif