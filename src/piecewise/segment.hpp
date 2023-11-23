#ifndef __segment__hpp
#define __segment__hpp

#include "linearFunction.hpp"
#include "interval.hpp"

#include <iostream>
#include <string>
/**
 * This class models a segment between points u and v.
 */
class Segment{

    private:
        Interval interval;
        LinearFunction f;

    public:
        /** Constructor. @param i The segment interval. @param f The linear function to be applied within the interval. **/
        Segment(const Interval &i, const LinearFunction &f) : interval(i), f(f) {}

        /** Constructor. Builds a segment from its endpoints. **/
        Segment(const Point &p1, const Point &p2) : interval(p1.first, p2.first), f(p1, p2){}

        /** Returns the segment interval. **/
        const Interval getInterval() const { return this->interval; }

        /** Returns the linear function. **/
        const LinearFunction getLinearFunction() const { return this->f; }
        
        /** Returns the function value at x. **/
        const bool contains(double x) const { return interval.contains(x); }

        /** Returns the function value at x. **/
        const double getValueAt(double x) const;

        /** Returns the function value at x. **/
        const Point getInf() const;
        /** Returns the function value at x. **/
        const Point getSup() const;
};

#endif