#include "segment.hpp"


const double Segment::getValueAt(double x) const
{
    if (!contains(x)){
        throw std::invalid_argument("Trying to get value outside interval bounds." );
    }

    return f.getValueAt(x);
}

const Point Segment::getInf() const
{
    double x = interval.getInf();
    return Point(x, getValueAt(x));
}

const Point Segment::getSup() const
{
    double x = interval.getSup();
    return Point(x, getValueAt(x));
}
