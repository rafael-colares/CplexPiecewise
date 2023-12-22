#include "linearFunction.hpp"

LinearFunction::LinearFunction(const double a, const Point &p)
{
    setA(a);
    setB(p.second - a*p.first);     // b = y - ax
}

LinearFunction::LinearFunction(const Point &p1, const Point &p2)
{
    double deltaY = (p2.second - p1.second);
    double deltaX = (p2.first - p1.first);
    double slope = deltaY/deltaX;
    setA(slope);
    setB(p1.second - (slope * p1.first));
}

Point LinearFunction::getIntersectionPoint(const LinearFunction &f)
{
    assert(this->getA() != f.getA());
    double x = (f.getB() - this->getB())/(this->getA() - f.getA());
    double y = this->getValueAt(x);
    return Point(x,y);
}
