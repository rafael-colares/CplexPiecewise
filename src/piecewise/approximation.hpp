#ifndef __approximation__hpp
#define __approximation__hpp

#include "piecewise.hpp"

/*** LEMON Libraries ***/     
#include <lemon/list_graph.h>
#include "lemon/dijkstra.h"

typedef lemon::ListDigraph 	Graph;
/* Maps */
typedef Graph::NodeMap<int> NodeMap;
typedef Graph::ArcMap<double> CostMap;

/**
 * This class models a (continuous) piecewise linear function.
 */
class Approximation{

    public:
        enum class Direction{
            DIRECTION_FROM_ABOVE,
            DIRECTION_FROM_BELOW
        };

    private:
		Piecewise original; 		    /** The original piecewise linear function. */
		Piecewise approx; 		        /** The approximated piecewise linear function. */
        Graph graph;
        NodeMap breakpointId; 
        const Direction DIR;

    public:
        /** Constructor. **/
        Approximation(const Piecewise &orig, const Direction &d);

        /** Sets the original function that will be approximated. */
        void setOriginalFunction(const Piecewise &f) { original = f; }
        
        /** Sets the approximated function. */
        void setApproximatedFunction(const Piecewise &f) { approx = f; }
        
        /** Returns the distance between a given y and approx(x) for a given point (x,y). */
        double getApproximationErrorFrom(const Point &p) const;

        /** Returns the total error between the original and approximated functions. It roughly corresponds to the area between the two curves. */
        double getTotalError() const;
        
        /** Returns the max error between the original and approximated functions. It roughly corresponds to the maximal difference of values between the two functions. */
        double getMaxError() const;

        /** Builds the best approximated function with a given number of breakpoints.*/
        void buildApproximation(const int TARGET_NB_BREAKPOINTS);
        
        /** Builds the approximated function while penalizing the use of a breakpoints with a coefficient alpha (between 0 and 1) and the total error with (1 - alpha). */
        void approximate(const double alpha);

        /** Returns the reversed shortest path from s to t. */
        std::vector<Graph::Node> getPath(const lemon::Dijkstra<Graph, CostMap> &shortestPath, const Graph::Node &s, const Graph::Node &t) const;

        /** Displays the s-t path. */
        void displayPath(const std::vector<Graph::Node> &path, const Graph::Node &s, const Graph::Node &t) const;

        /** Returns the approximation error obtained from ignoring the original breakpoints between the i-th and j-th breakpoints. TODO: modify taking into consideration the approx from above*/
        double getApproximationError(int i, int j) const;
        
        /** Returns the list of breakpoints of the approximated function based on the shortest path from s to t. TODO: modify taking into consideration the approx from above*/
        std::vector<Point> getListOfBreakpoints(const std::vector<Graph::Node> &path, const Graph::Node &s, const Graph::Node &t);
};

#endif