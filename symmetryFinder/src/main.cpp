#include <iostream>
#include <vector>
#include <unordered_map>

#include "Point.hpp"
#include "Kernels.hpp"

#include <../include/Eigen/Dense>
#include <../include/Eigen/SVD>


int main(){
    //Declare flag variable that indicates whether is the set is symmetrical (default assumption: false)
    bool isSetSymmetrical = false;
    //Declare variables to store the coefficients of the line (ax+by+c=0)
    double a=0.; double b=0.; double c=0.;

    //Define a sample set of points constituting a point cloud on a plane
    std::vector<Point2D> points{
      {1, 4.0, 2.0},
      {2, 1.0, 8.0},
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

