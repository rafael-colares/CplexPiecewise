#ifndef __piecewise__hpp
#define __piecewise__hpp

#include "segment.hpp"


#include <iostream>
#include <string>
#include <vector>
#include <numeric>
#include <math.h>       /* log */

typedef std::pair<double, double> Point;

/**
 * This class models a (continuous) piecewise linear function.
 */
class Piecewise{

    private:
		std::vector<Segment> segments; 		/** The ordered vector of points where the piecewise linear function changes its slope. */

    public:
        /** Default constructor. **/
        Piecewise(){}

        /** Add a segment to the piecewise linear function. */
        void addSegment(const Segment &seg);

        const bool isContinuous() const;

        const int getNbSegments() const { return (int)segments.size(); }

        const int getNbBreakpoints() const { return (int)segments.size() - 1; }

        /** Returns the function value at x. */
        const double getValueAt(double x) const; 

        /** Returns the segment that contains x. */
        const Segment getSegmentAt(double x) const;

        /** Returns the i-th segment. */
        const Segment getSegment_i(int i) const { return segments[i]; }

        /** Returns the x coordinate of the first breakpoint. */
        const double getInf() const { return segments.front().getSup().first; }

        /** Returns the x coordinate of the last breakpoint. */
        const double getSup() const { return segments.back().getInf().first; }

        /** Returns the i-th breakpoint. */
        const Point getBreakpoint_i(int i) const;

        /** Sets the piecewise linear function from a list of breakpoints */
        void setFunction(const std::vector<Point> &breakpointList);

        /** Displays the list of breakpoints */
        void displayBreakpoints() const;

        /** Builds a piecewise linear function that approximates the logarithm function from below. The breakpoints are evenly spaced in the interval required. @param nbSegments number of segments to be applied within the interval. @param inf lower interval boundary. @param sup upper interval boundary. */
        static Piecewise buildLogFunction(int nbSegments, double inf, double sup);

        const std::string toString() const;
};

#endif