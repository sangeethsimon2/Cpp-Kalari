#include <iostream>
#include <vector>
#include <unordered_map>


#include "Point.hpp"
#include "Kernels.hpp"




int main(){

    //Define the point cloud on a plane
    std::vector<Point2D> points{
      {1, 4.0, 2.0},
      {1, 4.0, 2.0},
      {2, 1.0, 8.0},
      {3, 1.0, 8.0}
    };

    //Ensure that the point cloud is not empty
    if(points.size()==0){
      throw std::runtime_error("The point cloud is empty! Please retry again!");
    }

    //Logic: Loop over every pair of non-identical points, construct a perpendicular bisector to a line joining them,
    //For each point, construct a symmetrical point about this line and check for it's existence in the given set
    //Even if one point does not have a matching
    //Loop over pair of points
    for (int i ={}; i<points.size(); i++){
      for (int j=i+1; j<points.size(); j++){
        if(points[i]==points[j]) continue;
        //Compute mid point of every pair
        double midPoint_xCoord = 0.5*(points[i].m_x + points[j].m_x);
        double midPoint_yCoord  = 0.5*(points[i].m_y + points[j].m_y);

        //Compute slope of perpendicular bisector of line joining every pair (= negative inverse of slope of line joining every pair)
        double slopeOfPerpendicularBisector = - ((points[j].m_x - points[i].m_x)/(points[j].m_y - points[i].m_y));

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
        double a = 2.0*(diffOfXCoords);
        double b = 2.0*(diffOfYCoords);
        double c = -1.0*(sumOfYCoords*diffOfYCoords + sumOfXCoords*diffOfXCoords);

        //Check for symmetry
        bool isSetSymmetrical = Kernels::IsSymmetricSet(points, a, b, c);
        if(isSetSymmetrical)
         std::cout<<" The given points on the plane are symmetrical wrt the line "<<a<<"x(+)"<<b<<"y(+)"<<c<<"\n";
       else
         std::cout<<" Unable to find a line of symmetry for the given point cloud \n";
     }
    }
    return 0;

}

