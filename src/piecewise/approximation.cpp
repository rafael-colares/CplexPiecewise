#include "approximation.hpp"
using namespace lemon;

Approximation::Approximation(const Piecewise &orig, const Direction &d, const Method &m, const int nb_bp) : original(orig), breakpointId(graph), DIR(d), METHOD(m)
{ 
	/* Define nodes */
    if (METHOD == Method::METHOD_SHORTEST_PATH){
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
    TARGET_NB_TOUCHES = nb_bp;
    TARGET_NB_BREAKPOINTS = (DIR == Direction::DIRECTION_FROM_BELOW) ? TARGET_NB_TOUCHES : TARGET_NB_TOUCHES + 1;
    assert(TARGET_NB_TOUCHES >= 2);
    buildApproximation();
    displayApprox();
}

void Approximation::buildApproximation()
{
    if (METHOD == Method::METHOD_EQUIDISTANT){
        return buildEquidistantApproximation();
    }
    if (METHOD == Method::METHOD_BILLONNET){
        return buildBillonnetApproximation();
    }
    if (METHOD == Method::METHOD_SHORTEST_PATH){
        return buildShortestPathApproximation();
    }
    // should never arrive to this point.
    assert(false);
}

void Approximation::buildEquidistantApproximation()
{
    if (DIR == Direction::DIRECTION_FROM_BELOW){
        buildEquidistantApproximationFromBelow();
    }
    else {
        buildEquidistantApproximationFromAbove();
    }
}

void Approximation::buildEquidistantApproximationFromBelow()
{
    approx = Piecewise::buildLogFunction(TARGET_NB_BREAKPOINTS-1, original.getInf(), original.getSup());
    std::cout << "Built approx" << std::endl;
}

void Approximation::buildEquidistantApproximationFromAbove()
{
    std::vector<Point> touches;
    const double STEP_SIZE = (original.getSup() - original.getInf())/((double) TARGET_NB_TOUCHES - 1);
    std::cout << "Touches : " ;
    for (int i = 0; i < TARGET_NB_TOUCHES; i++){
        double x_i = original.getInf() + (i)*STEP_SIZE;
        double y_i = log(x_i);
        Point u_i(x_i, y_i);
        touches.push_back(u_i);
        std::cout << "(" << u_i.first << ", " << u_i.second << "), ";
    }
    std::cout << std::endl;
    std::vector<Point> breakpoints = getListOfBreakpointsFromTangents(touches);
    approx.setFunction(breakpoints);
}

void Approximation::buildBillonnetApproximation()
{
    if (DIR == Direction::DIRECTION_FROM_BELOW){
        buildBillonnetApproximationFromBelow();
    }
    else {
        buildBillonnetApproximationFromAbove();
    }
}

void Approximation::buildBillonnetApproximationFromBelow()
{
    const double APPROX_EPSILON = 1.0e-8;
    std::vector<Point> touches;
    const int NB_SEGMENTS = TARGET_NB_TOUCHES - 1;
    const double lb = original.getInf();
    const double ub = original.getSup();
    const double RATIO = pow((lb / ub), (1.0 / ((double) NB_SEGMENTS + 1)));
    double s_0 = 1.0 / lb;
    std::vector<double> slope;
    std::vector<Point> breakpoints;
    
    std::cout << "Inf: " << lb << std::endl;
    std::cout << "Sup: " << ub << std::endl;
    std::cout << "Ratio: " << RATIO << std::endl;

    breakpoints.push_back(Point(lb, log(lb)));
    for (int i = 1; i < TARGET_NB_TOUCHES-1; i++){
        double s_i = s_0 * (pow(RATIO, (double) i));
        slope.push_back(s_i);
        LinearFunction f_i(s_i, breakpoints[i-1]);
        double guess = 1.0/s_i;
        while (log(guess) - f_i.getValueAt(guess) >= 0){
            guess += APPROX_EPSILON;
        }
        guess -= APPROX_EPSILON;
        breakpoints.push_back(Point(guess, log(guess)));
        //bp_i = f_i.
    }
    slope.push_back(slope.back()*RATIO);
    breakpoints.push_back(Point(ub, log(ub)));
    std::cout << "Forecasted slopes: " << std::endl;
    for (double s : slope){
        std::cout << "(" << s << ") ";
    }
    std::cout << std::endl;
    std::cout << "Actual slopes: " << std::endl;
    for (unsigned int i = 1; i < breakpoints.size(); i++){
        double deltaX = breakpoints[i].first - breakpoints[i-1].first;
        double deltaY = breakpoints[i].second - breakpoints[i-1].second;
        std::cout << "(" << deltaY/deltaX << ") ";
    }
    std::cout << std::endl;
    
    for (unsigned int i = 1; i < breakpoints.size() - 1; i++){
        std::cout << "Ratio between slopes " << i << " and " << i-1 << " : ";
        double deltaX = breakpoints[i].first - breakpoints[i-1].first;
        double deltaY = breakpoints[i].second - breakpoints[i-1].second;
        double s_i = deltaY/deltaX;
        
        deltaX = breakpoints[i+1].first - breakpoints[i].first;
        deltaY = breakpoints[i+1].second - breakpoints[i].second;
        double s_j = deltaY/deltaX;
        
        std::cout << s_j/s_i << std::endl;
    }
    std::cout << std::endl;
    approx.setFunction(breakpoints);
    approx.displayBreakpoints();
}

void Approximation::buildBillonnetApproximationFromAbove()
{
    std::vector<Point> touches;
    const double RATIO = pow((original.getInf() / original.getSup()), (1.0 / ((double) TARGET_NB_TOUCHES - 1)));
    
    for (int i = 0; i < TARGET_NB_TOUCHES; i++){
        double s_0 = 1.0 / original.getInf();
        double s_i = s_0 * (pow(RATIO, (double) i));
        double x_i = 1.0/s_i;
        Point u_i(x_i, log(x_i));
        touches.push_back(u_i);
    }
    std::cout << std::endl;

    std::vector<Point> breakpoints = getListOfBreakpointsFromTangents(touches);
    approx.setFunction(breakpoints);
}

void Approximation::buildShortestPathApproximation()
{
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
        approximateWithShortestPath(alpha);

        //std::cout << "Obtained approximation has " << approx.getNbBreakpoints() << " breakpoints." << std::endl;
        //std::cout << "Total error : " << getTotalError() << std::endl;
        //std::cout << "Max error : " << getMaxError() << std::endl;
    }
    //std::cout << "Final alpha value : " << alpha << std::endl;
}



void Approximation::approximateWithShortestPath(const double ALPHA)
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
    std::vector<Point> touches = getListOfBreakpoints(path);
    std::vector<Point> breakpoints;
    if (DIR == Direction::DIRECTION_FROM_ABOVE) {
        breakpoints = getListOfBreakpointsFromTangents(touches);
    }
    if (DIR == Direction::DIRECTION_FROM_BELOW) {
        breakpoints = touches;
    }
    approx.setFunction(breakpoints);
    
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
    std::reverse(path.begin(), path.end());
    return path;
}

void Approximation::displayPath(const std::vector<Graph::Node> &path, const Graph::Node &s, const Graph::Node &t) const
{
    std::cout << "Path from " << breakpointId[s] << " to " << breakpointId[t] << " is: " << std::endl; 
    for (auto node : path)
        std::cout << breakpointId[node] <<  " ";
    std::cout << std::endl;
}

void Approximation::displayApprox() const
{
    std::cout << "Approximated function has " << approx.getNbBreakpoints() << " breakpoints." << std::endl;
    assert(approx.getNbBreakpoints() == TARGET_NB_BREAKPOINTS);
    std::cout << "Breakpoints : " ;
    approx.displayBreakpoints();
    std::cout << "Avg error : " << getAvgError() << std::endl;
    std::cout << "Max error : " << getMaxError() << std::endl;
    //std::cout << approx.toString() << std::endl;
}

double Approximation::getApproximationError(int bp1, int bp2) const {
    Point p1 = original.getBreakpoint_i(bp1);
    Point p2 = original.getBreakpoint_i(bp2);
    double totalError = 0.0;
    if (DIR == Direction::DIRECTION_FROM_BELOW){
        LinearFunction f(p1, p2); // line passing through p1 and p2
        for (int bp = bp1 + 1; bp < bp2; bp++){
            Point x = original.getBreakpoint_i(bp);
            double error = x.second - f.getValueAt(x.first);
            assert(error > -DBL_EPS);
            totalError += error;
        }
    }
    
    if (DIR == Direction::DIRECTION_FROM_ABOVE){
        LinearFunction f1(1.0/p1.first, p1); // tangent line of log(x) at p1
        LinearFunction f2(1.0/p2.first, p2); // tangent line of log(x) at p1
        for (int bp = bp1 + 1; bp < bp2; bp++){
            Point x = original.getBreakpoint_i(bp);
            double error = std::min(f1.getValueAt(x.first), f2.getValueAt(x.first)) - x.second;
            assert(error > -DBL_EPS);
            totalError += error;
        }
    }
    return totalError;
}


double Approximation::getTotalError() const
{
    double totalError = 0;
    for (int i = 0; i < original.getNbBreakpoints(); i++) {
        Point breakpoint = original.getBreakpoint_i(i);
        totalError += getApproximationErrorFrom(breakpoint);
    }
    return totalError;
}

double Approximation::getAvgError() const
{
    return getTotalError() / original.getNbBreakpoints();
}

std::vector<Point> Approximation::getListOfBreakpoints(const std::vector<Graph::Node> &path)
{
    std::vector<Point> breakpoints;
    for (auto p = path.begin(); p != path.end(); ++p)
        breakpoints.push_back(original.getBreakpoint_i(breakpointId[*p]));

    return breakpoints;
}

std::vector<Point> Approximation::getListOfBreakpoints(const std::vector<Point> &originalBreakpoints)
{
    if (DIR == Direction::DIRECTION_FROM_BELOW){
        return originalBreakpoints;
    }

    std::vector<Point> breakpoints;
    if (DIR == Direction::DIRECTION_FROM_ABOVE){
        for (unsigned int p = 0; p < originalBreakpoints.size() - 1; ++p){
            Point p1 = originalBreakpoints[p];
            Point p2 = originalBreakpoints[p+1];
            
            LinearFunction f1(1.0/p1.first, p1);    // tangent line of log(x) at p1
            LinearFunction f2(1.0/p2.first, p2);    // tangent line of log(x) at p2

            Point bp = f1.getIntersectionPoint(f2);
            breakpoints.push_back(bp);
        }
    }

    return breakpoints;
}

std::vector<Point> Approximation::getListOfBreakpointsFromTangents(const std::vector<Point> &touches)
{
    std::vector<Point> breakpoints;
    breakpoints.push_back(touches.front());
    for (unsigned int i = 0; i < touches.size() - 1; ++i){
        Point u_i = touches[i];
        Point u_j = touches[i+1];
        
        LinearFunction f1(1.0/u_i.first, u_i);    // tangent line of log(x) at p1
        LinearFunction f2(1.0/u_j.first, u_j);    // tangent line of log(x) at p2

        Point p = f1.getIntersectionPoint(f2);
        breakpoints.push_back(p);
    }
    breakpoints.push_back(touches.back());
    return breakpoints;
}

double Approximation::getApproximationErrorFrom(const Point &p) const
{
    double error = p.second - approx.getValueAt(p.first);
    if (DIR == Direction::DIRECTION_FROM_ABOVE) {
        error = -error;
    }
    if (error < 0.0) std::cout << "Negative error at x = " << p.first << " : log(x) = " << log(p.first) << ", bp is at " << p.second << ", and approx is " << approx.getValueAt(p.first) << ". Error = " << error << std::endl;
    return std::max(0.0, error);
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