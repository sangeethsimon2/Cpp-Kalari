#ifndef POINTHEADER
#define POINTHEADER

#include <cmath>

#define epsilon  1e-10

struct Point2D{
    //CTOR
    Point2D(int index, double x, double y ): m_Index(index), m_x(x), m_y(y) {}

    //Overload '==' for equality between points
    bool operator == (const Point2D& otherPoint)const{
     return (std::fabs(m_x - otherPoint.m_x) < epsilon && std::fabs(m_y - otherPoint.m_y) < epsilon);
     }
    //Members
    int m_Index;
    double m_x, m_y;
};

#endif