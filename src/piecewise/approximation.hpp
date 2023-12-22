#ifndef __approximation__hpp
#define __approximation__hpp

#include "piecewise.hpp"

#include <algorithm>
#include <math.h>

/*** LEMON Libraries ***/     
#include <lemon/list_graph.h>
#include "lemon/dijkstra.h"

typedef lemon::ListDigraph 	Graph;
/* Maps */
typedef Graph::NodeMap<int> NodeMap;
typedef Graph::ArcMap<double> CostMap;

#define DBL_EPS 1e+12

/**
 * This class models a (continuous) piecewise linear function.
 */
class Approximation{

    public:
        enum class Direction{
            DIRECTION_FROM_ABOVE,
            DIRECTION_FROM_BELOW
        };
        enum class Method{
            METHOD_SHORTEST_PATH,
            METHOD_BILLONNET,
            METHOD_EQUIDISTANT
        };

    private:
		Piecewise original; 		    /** The original piecewise linear function. */
		Piecewise approx; 		        /** The approximated piecewise linear function. */
        Graph graph;
        NodeMap breakpointId; 
        const Direction DIR;
        const Method METHOD;
        int TARGET_NB_BREAKPOINTS;
        int TARGET_NB_TOUCHES;

    public:
        /** Constructor. **/
        Approximation(const Piecewise &orig, const Direction &d, const Method &m, const int TARGET_NB_TOUCHES);

        /** Builds the approximated function. */
        void buildApproximation();

        /** Builds the best approximated function using the equidistant's method : the breakpoints are evenly sparsed. */
        void buildEquidistantApproximation();
        void buildEquidistantApproximationFromBelow();
        void buildEquidistantApproximationFromAbove();

        /** Builds the best approximated function using the Billonnet's method : the ratio between consecutive slopes is constant.  */
        void buildBillonnetApproximation();
        void buildBillonnetApproximationFromBelow();
        void buildBillonnetApproximationFromAbove();

        /** Builds the best approximated function using the shortest path approximation. */
        void buildShortestPathApproximation();
        
        /** Builds the approximated function while penalizing the use of a breakpoints with a coefficient alpha (between 0 and 1) and the total error with (1 - alpha). */
        void approximateWithShortestPath(const double alpha);

        /****************************************************************************************/
        /*										   Setters  									*/
        /****************************************************************************************/

        /** Sets the original function that will be approximated. */
        void setOriginalFunction(const Piecewise &f) { original = f; }
        
        /** Sets the approximated function. */
        void setApproximatedFunction(const Piecewise &f) { approx = f; }
        

        /****************************************************************************************/
        /*										   Getters  									*/
        /****************************************************************************************/

        /** Returns the list of breakpoints of the approximated function from above based on the list of points that touches the log function. */
        std::vector<Point> getListOfBreakpointsFromTangents(const std::vector<Point> &touches);

        /** Returns the distance between a given y and approx(x) for a given point (x,y). */
        double getApproximationErrorFrom(const Point &p) const;

        /** Returns the total error between the original and approximated functions. It roughly corresponds to the area between the two curves. */
        double getTotalError() const;
        
        /** Returns the average error between the original and approximated functions. */
        double getAvgError() const;
        
        /** Returns the max error between the original and approximated functions. It roughly corresponds to the maximal difference of values between the two functions. */
        double getMaxError() const;

        /** Returns the reversed shortest path from s to t. */
        std::vector<Graph::Node> getPath(const lemon::Dijkstra<Graph, CostMap> &shortestPath, const Graph::Node &s, const Graph::Node &t) const;

        /** Returns the approximation error obtained from ignoring the original breakpoints between the i-th and j-th breakpoints. */
        double getApproximationError(int i, int j) const;
        
        /** Returns the list of breakpoints of the approximated function based on the shortest path. */
        std::vector<Point> getListOfBreakpoints(const std::vector<Graph::Node> &path);
        
        /** Returns the list of breakpoints of the approximated function based on the chosen breakpoints from the original function. */
        std::vector<Point> getListOfBreakpoints(const std::vector<Point> &originalBreakpoints);


        /****************************************************************************************/
        /*										   Display  									*/
        /****************************************************************************************/
        /** Displays the s-t path. */
        void displayPath(const std::vector<Graph::Node> &path, const Graph::Node &s, const Graph::Node &t) const;
        /** Displays the approximated function properties. */
        void displayApprox() const;

};

#endif