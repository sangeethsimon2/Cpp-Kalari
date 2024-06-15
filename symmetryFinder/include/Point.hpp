#ifndef POINTHEADER
#define POINTHEADER

#include <cmath>
#define scalingFactor 1e-6
#define epsilon  1e-10

struct Point2D{
    //Default ctor
    Point2D() = default;  // Default constructor

    //CTOR
    Point2D(int index, double x, double y ): m_Index(index), m_x(x), m_y(y) {}

    //Overload '==' for equality between points
    bool operator == (const Point2D& otherPoint)const{
     return (std::fabs(m_x - otherPoint.m_x) < epsilon && std::fabs(m_y - otherPoint.m_y) < epsilon);
     }
    //Members
    int m_Index;
    double m_x=0.; double m_y=0.;
};

#endif