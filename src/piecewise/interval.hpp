#ifndef __interval__hpp
#define __interval__hpp

#include <float.h>
#include <assert.h>
#include <utility>

/**
 * This class models an interval in the real line. The interval is defined as [inf, sup].
 */
class Interval{

    private:
        std::pair<double, double> interval;

    public:
        /** Constructor. @param a The slope. @param b The independent term.**/
        Interval(const double inf = -DBL_MAX, const double sup = DBL_MAX);

        /** Copy constructor.**/
        Interval(const Interval &i): interval(i.getInf(), i.getSup()){}

        /** Returns the infimum endpoint. **/
        const double getInf() const { return interval.first; }

        /** Returns the supremum endpoint. **/
        const double getSup() const { return interval.second; }

        /** Sets the infimum endpoint. **/
        void setInf(double val) { interval.first = val; }

        /** Sets the supremum endpoint. **/
        void setSup(double val) { interval.second = val; }

        /** Checks if the interval contains a given point.**/
        const bool contains(double x) const { return (x >= getInf() && x <= getSup()); }

};

#endif