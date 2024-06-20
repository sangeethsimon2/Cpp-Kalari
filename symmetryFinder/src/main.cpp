#include <iostream>
#include <vector>
#include <unordered_map>
#include <iomanip>

#include "Point.hpp"
#include "Kernels.hpp"

int main(){
    //Declare flag variable that indicates whether is the set is symmetrical (default assumption: false)
    bool isSetSymmetrical = true;
    //Declare variables to store the coefficients of the line (ax+by+c=0)
    double a=0.; double b=0.; double c=0.;
    //Declare a Variable to store the centroid points
    double x_centroid = 0.; double y_centroid = 0.;

    //Define a set of point to check for symmetry
    std::vector<Point2D> points{
      {1, 1., 1.},
      {2, 1., -1.},
    };

    //Ensure that the point cloud is not empty
    if(points.size()==0){
      throw std::runtime_error("The point cloud is empty! Please retry again!");
    }
    //Call a improved method to check for symmetry in the given point cloud
    isSetSymmetrical = Kernels::checkForSymmetryInPointCloud(points, a, b, c);

    //Print to console the final verdit on the symmetry of the pointcloud
    if(isSetSymmetrical)
      std::cout<<" The given points on the plane are symmetrical wrt the line "<<a<<"x(+)"<<b<<"y(+)"<<c<<"\n";
    else
      std::cout<<" Unable to find a line of symmetry for the given point cloud \n";

    return 0;
}

