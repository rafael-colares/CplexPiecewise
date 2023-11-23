#include "approximation.hpp"
using namespace lemon;

double Approximation::getTotalError() const
{
    double totalError = 0;
    for (int i = 0; i < original.getNbBreakpoints(); i++) {
        Point breakpoint = original.getBreakpoint_i(i);
        totalError += getApproximationErrorFrom(breakpoint);
    }
    return totalError;
}

Approximation::Approximation(const Piecewise &orig, const Direction &d) : original(orig), breakpointId(graph), DIR(d)
{ 
	/* Define nodes */
	for (int i = 0; i < original.getNbBreakpoints(); i++){
        Graph::Node n = graph.addNode();
        breakpointId[n] = i;
    }

	/* Define arcs */
    for (Graph::NodeIt u(graph); u != lemon::INVALID; ++u){
        for (Graph::NodeIt v(graph); v != lemon::INVALID; ++v){
            if (breakpointId[u] < breakpointId[v]){
                graph.addArc(u, v);
            }
        }
    }
}

std::vector<Point> Approximation::getListOfBreakpoints(const std::vector<Graph::Node> &path, const Graph::Node &s, const Graph::Node &t)
{
    std::vector<Point> breakpoints;
    for (auto p = path.rbegin(); p != path.rend(); ++p)
        breakpoints.push_back(original.getBreakpoint_i(breakpointId[*p]));
    return breakpoints;
}

double Approximation::getApproximationErrorFrom(const Point &p) const
{
    double error = p.second - approx.getValueAt(p.first);
    assert(error >= -0.00001);
    return error;
}

double Approximation::getMaxError() const
{
    double maxError = 0;
    for (int i = 0; i < original.getNbBreakpoints(); i++) {
        Point breakpoint = original.getBreakpoint_i(i);
        double error = getApproximationErrorFrom(breakpoint);

        if (error > maxError){
            maxError = error;
        }
    }
    return maxError;
}

void Approximation::buildApproximation(const int TARGET_NB_BREAKPOINTS)
{
    assert(TARGET_NB_BREAKPOINTS >= 2);
    double alpha = 1;
    double step_size = 0.5;
    while (approx.getNbBreakpoints() != TARGET_NB_BREAKPOINTS){
        if (approx.getNbBreakpoints() < TARGET_NB_BREAKPOINTS){
            alpha -= step_size;
        }
        else{
            alpha += step_size;
        }
        step_size = step_size/2.0;
        approximate(alpha);

        std::cout << "Obtained approximation has " << approx.getNbBreakpoints() << " breakpoints." << std::endl;
        std::cout << "Total error : " << getTotalError() << std::endl;
        std::cout << "Max error : " << getMaxError() << std::endl;
    }
    std::cout << "Final alpha value : " << alpha << std::endl;
}

void Approximation::approximate(const double ALPHA)
{   
    assert(ALPHA >= 0 && ALPHA <= 1);

    const double BETA = 1.0 - ALPHA;
    const int NB_BREAKPOINTS = original.getNbBreakpoints();

	/* Allocation of cost */
    CostMap cost(graph);
    for (Graph::ArcIt a(graph); a != lemon::INVALID; ++a){
        Graph::Node u = graph.source(a);
        Graph::Node v = graph.target(a);
        cost[a] = ALPHA + BETA*getApproximationError(breakpointId[u], breakpointId[v]);
    }
    
	/* Shortest path computation */
    lemon::Dijkstra<Graph, CostMap> shortestPath(graph, cost);
    Graph::Node start = graph.nodeFromId(0);
    Graph::Node end = graph.nodeFromId(NB_BREAKPOINTS-1);
    assert(breakpointId[start] == 0);
    assert(breakpointId[end] == NB_BREAKPOINTS-1);
    shortestPath.run(start, end);

    /* Results */
    std::vector<Graph::Node> path = getPath(shortestPath, start, end);
    double pathCost = shortestPath.dist(end);
    displayPath(path, start, end);
    std::cout << "Total cost for the shortest path is: " << pathCost << std::endl;
    approx.setFunction(getListOfBreakpoints(path, start, end));
}

std::vector<Graph::Node> Approximation::getPath(const lemon::Dijkstra<Graph, CostMap> &shortestPath, const Graph::Node &s, const Graph::Node &t) const{
    std::vector<Graph::Node> path;
    for (Graph::Node v = t; v != s; v = shortestPath.predNode(v))
    {
        if (v != lemon::INVALID && shortestPath.reached(v)) //special LEMON node constant
        {
            path.push_back(v);
        }
    }
    path.push_back(s);
    return path;
}

void Approximation::displayPath(const std::vector<Graph::Node> &path, const Graph::Node &s, const Graph::Node &t) const
{
    std::cout << "Path from " << breakpointId[s] << " to " << breakpointId[t] << " is: " << std::endl; 
    for (auto p = path.rbegin(); p != path.rend(); ++p)
        std::cout << breakpointId[*p] <<  " ";
    std::cout << std::endl;
}

double Approximation::getApproximationError(int bp1, int bp2) const {
    Point p1 = original.getBreakpoint_i(bp1);
    Point p2 = original.getBreakpoint_i(bp2);
    LinearFunction f(p1, p2);
    double totalError = 0.0;
    for (int bp = bp1 + 1; bp < bp2; bp++){
        Point x = original.getBreakpoint_i(bp);
        double error = x.second - f.getValueAt(x.first);
        assert(error >= 0);
        totalError += error;
    }

    return totalError;
}