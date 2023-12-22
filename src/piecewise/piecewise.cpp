#include "piecewise.hpp"

void Piecewise::addSegment(const Segment &seg)
{
    segments.push_back(seg);
}

const bool Piecewise::isContinuous() const
{
    if (segments.front().getInterval().getInf() != -DBL_MAX){
        return false;
    }
    if (segments.back().getInterval().getSup() != DBL_MAX){
        return false;
    }

    for (unsigned int i = 1; i < segments.size(); i++){
        if (segments[i].getInterval().getInf() != segments[i-1].getInterval().getSup()) {
            return false;
        }
    }
    return true;
}

const double Piecewise::getValueAt(double x) const
{
    Segment s = getSegmentAt(x); 
    return s.getValueAt(x);
}

const Segment Piecewise::getSegmentAt(double x) const
{
    for (Segment s : segments){
        if (s.contains(x)){
            return s;
        }
    }

    throw std::invalid_argument("No segment contains value " + std::to_string(x) + ".");
}

const Point Piecewise::getBreakpoint_i(int i) const
{
    assert(i >= 0 && i < getNbBreakpoints());
    return getSegment_i(i).getSup();
}

void Piecewise::setFunction(const std::vector<Point> &breakpointList)
{
    /* Clear current function */
    segments.clear();

    /* First segment */
    addSegment(Segment( Point(-DBL_MAX, breakpointList[0].second), breakpointList[0]));

    /* Mid segments */
    for (unsigned int i = 1; i < breakpointList.size(); i++){
        assert(breakpointList[i-1].first < breakpointList[i].first);
        addSegment(Segment(breakpointList[i-1], breakpointList[i]));
    }

    /* Last segment */
    addSegment(Segment( breakpointList.back(), Point(DBL_MAX, breakpointList.back().second)));
}

void Piecewise::displayBreakpoints() const
{
    for (unsigned int i = 1; i < segments.size(); i++){
        std::cout << "(" << segments[i].getInf().first << ", " << segments[i].getInf().second << ")  ";
    }
    std::cout << std::endl;
}

Piecewise Piecewise::buildLogFunction(int nbSegments, double inf, double sup)
{
    assert(inf > 0 && inf < sup && nbSegments >= 1);
    const double STEP_SIZE = (sup - inf)/((double) nbSegments);
    Piecewise logFunc; 
    logFunc.addSegment(Segment( Point(-DBL_MAX, log(inf)), Point(inf, log(inf)) ));
    for (int i = 0; i < nbSegments; i++){
        double x_i = inf + (i)*STEP_SIZE;
        double y_i = log(x_i);
        Point p_i(x_i, y_i);

        double x_j = inf + (i+1)*STEP_SIZE;
        double y_j = log(x_j);
        Point p_j(x_j, y_j);

        logFunc.addSegment(Segment(p_i, p_j));
    }
    logFunc.addSegment(Segment( Point(sup, log(sup)), Point(DBL_MAX, log(sup)) ));
    return logFunc;
}

const std::string Piecewise::toString() const
{
    std::string str = "";
    str += "Piecewise lienar function : \n";
    for (Segment s : segments){
        str += s.toString() + "\n";
    }
    return str;
}
