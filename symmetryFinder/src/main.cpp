#include <iostream>
#include <vector>
#include <unordered_map>


#include "Point.hpp"
#include "Kernels.hpp"




int main(){

    //Define the point cloud on a plane
    std::vector<Point> points{
      {1, 4.0, 2.0},
      {2, 1.0, 8.0}
    };

    //Define the arbitrary line against which symmetry must be checked
    //Line form: ax + by + c = 0
    double a = 2.; double b = -4.; double c = 15.;

    //Check for symmetry
    Kernels::IsSymmetricSet(points, a, b, c);


    return 0;

}

