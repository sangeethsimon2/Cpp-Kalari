#ifndef KERNELSHEADER
#define KERNELSHEADER

#include "Point.hpp"
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
      bool b_isSymmetricSet = true;
      //Declare an unordered_map that has key: Point2D and value: number of times the point is encountered
      std::unordered_map<Point2D, int, hashFunc_longlongintkey, equalFunc> Point2DSet;

      //For each new point encountered from the container, increment a counter to indicate its presence
      for(const auto& eachPoint: points)
        Point2DSet[eachPoint]++;


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

    bool checkForSymmetryInPointCloud(const std::vector<Point2D>& points, double& a, double& b, double& c){
      //Assume that the set is not symmetrical to begin with
      bool isSetSymmetrical = false;

      //Map to store the mid points of all pairs of points;
      //key=hash(Point2D), value=(count, xcoord, ycoord)
      //Count expresses a ranking system that signifies how probable a midpoint lies on a symmetry line for this set
      std::unordered_map<Point2D, std::tuple<int, Point2D>, hashFunc_longlongintkey, equalFunc> midPointMap;

      //Map to store the coefficient triplet of perpendicular bisector line of every pair of points
      //key = hash(Point2D), value=(a, b c)
      std::unordered_map<Point2D, std::tuple<double, double, double>, hashFunc_longlongintkey, equalFunc> perpBisecCoefficientsMap;

      //Logic for Step 1:
      //Loop over every pair of non-identical points,
      //Construct its mid point and store its (x,y) coord along with the number of times it was encountered
      //Construct a perpendicular bisector to a line joining them and store the coefficients of this line

      //Loop over pair of points
      for (int i ={}; i<points.size(); i++){
        for (int j=i+1; j<points.size(); j++){
          //Go to next iteration if the points are same.
          if(points[i]==points[j]) continue;

          //Compute mid point of every pair
          double midPoint_xCoord = 0.5*(points[i].m_x + points[j].m_x);
          double midPoint_yCoord  = 0.5*(points[i].m_y + points[j].m_y);
          double slopeOfPerpendicularBisector = - ((points[j].m_x - points[i].m_x)/(points[j].m_y - points[i].m_y));
          Point2D midPoint(i*200+j*300, midPoint_xCoord, midPoint_yCoord);

          //Add the midPoint to the midPointMap by storing its occurrence, and its Point2D struct.
          if (midPointMap.find(midPoint) != midPointMap.end()) {
            // Increment the count if midpoint already exists.
            std::get<0>(midPointMap[midPoint])++;
          } else {
            // Add new entry to the map.
            midPointMap[midPoint] = std::make_tuple(1, midPoint);
          }

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
          double m_a = 2.0*(diffOfXCoords);
          double m_b = 2.0*(diffOfYCoords);
          double m_c = -1.0*(sumOfYCoords*diffOfYCoords + sumOfXCoords*diffOfXCoords);
          //Add the midPoint to the perpBisecCoefficientMap by storing the coefficients of a
          //perpendicular bisector line passing through it
          if (perpBisecCoefficientsMap.find(midPoint) == perpBisecCoefficientsMap.end()) {
              perpBisecCoefficientsMap[midPoint] = std::make_tuple(m_a, m_b, m_c);
          }//If it exists, do not repeat storing it since its redundant
        }
      }

      //Logic for Step 2
      //Loop over set of lineCoefficients corresponding to each midPoint in the lineCoefficientsMap
      //Loop over the each midPoint within the midPointMap
      //Check if the midPoint lies on the line, if yes, increment the midPoint counter again
      for (auto& eachMidPoint : midPointMap) {
        auto [counter,midPoint] = eachMidPoint.second;
        auto [m_a, m_b, m_c] = perpBisecCoefficientsMap[midPoint];
        if((m_a*midPoint.m_x+m_b*midPoint.m_y+m_c)<1e-15)
            std::get<0>(midPointMap[midPoint])++;
      }

      //Intermediate pruning step
      //Remove all elements whose counter is < 2
      for (auto& eachMidPoint : midPointMap) {
        auto [counter,midPoint] = eachMidPoint.second;
        if(counter<2)
          midPointMap.erase(midPoint);
      }

      //Logic for Step 3
      //Loop over each midPoint within the midPointMap
      //Check if its count value is > 2
      //If it is, then check for symmetry of the pointcloud about its perpendicular bisector line
      for (auto& eachMidPoint : midPointMap) {
        auto [counter,midPoint] = eachMidPoint.second;
        if(counter>=2){
          auto& [m_a, m_b, m_c] = perpBisecCoefficientsMap[midPoint];
          //Check for symmetry; break when the first symmetry line is discovered
          //Continue searching if the current line is not a symmetry line
          if(isSetSymmetrical = IsSymmetricSet(points, m_a, m_b, m_c)){
            a=m_a; b=m_b;c=m_c;
            break;
          }
        }
      }
    return isSetSymmetrical;
    }
}
#endif