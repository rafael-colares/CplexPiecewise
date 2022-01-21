#include "model.hpp"

void printVector(const std::vector<double> &v, std::string name){
    std::cout << name << " :" << std::endl;
    for (unsigned int k = 0; k < v.size(); k++){
        std::cout << k << " : " << v[k] << std::endl;
    }
    std::cout << std::endl;
}
/* Constructor */
Model::Model(const IloEnv& env_, const Data& data_) : 
                env(env_), model(env), cplex(model), data(data_), 
                obj(env), constraints(env)
{

    std::cout << "=> Building model ... " << std::endl;
    
    buildPiecewiseLinearApproximation();
    setVariables();
    setObjective();  
    setConstraints();  
    setCplexParameters();

    std::cout << "\t Model was correctly built ! " << std::endl;                 
}
// breaks[0] = 0 and breaks[last] = 1.0
/* Returns g(y) where g(y) is a function approximating log(y) from above */
// double Model::approx_log_from_above(double y, const vector<IloNum> &breaks, const vector<IloNum> &u){
//     int nbBreaks = breaks.size();
//     int last = nbBreaks - 1; 
//     if (y < 0 || y > 1){
//         std::cerr << "ERROR: approximation is only done between 0 and 1" << std::endl;
// 		exit(EXIT_FAILURE);
//     }
//     if (y >= 0 && y <= breaks[1]){
//         return (((1.0 / u[0])*y) + std::log(u[0]) - 1.0);
//     }
//     if (y >= breaks[last-1] && y <= 1.0){
//         return (y - 1.0);
//     }
//     for (unsigned int k = 1; k < breaks.size()-2; k++){
//         if (y >= breaks[k] && y <= breaks[k+1]){
//             return (((1.0 / u[k])*y) + std::log(u[k]) - 1.0);
//         }
//     }
    
//     std::cerr << "ERROR: approximation is undefined" << std::endl;
//     exit(EXIT_FAILURE);
// }

/* Returns g(val) where g(val) is a function approximating log(val) for a given demand k*/
double Model::approx_log_from_below(double val, const IloNumVector &breakpoints){
    const int NB_BREAKS = breakpoints.size();
    double slope = 0.0;
    double ref = 0.0;
    if (val < breakpoints[0] || val > breakpoints[NB_BREAKS-1]){
        std::cerr << "ERROR: weird stuff happening in approximation" << std::endl;
        return 1;
    }

    for (int i = 0; i < NB_BREAKS-1; i++){
        if (val >= breakpoints[i] && val <= breakpoints[i+1]){
            ref = breakpoints[i];
            slope = (std::log(breakpoints[i+1]) - std::log(breakpoints[i])) / (breakpoints[i+1] - breakpoints[i]);
            double delta_X = val - ref;
            return (std::log(ref) + delta_X*slope);
        }
    }
    
    std::cerr << "ERROR: approximation is undefined" << std::endl;
    exit(EXIT_FAILURE);
}
/* Set up the vector u for approximating log(avail). */
void Model::buildPiecewiseLinearApproximation(){
    if (data.getInput().isBasic() == false){
        buildAvailVector_u();
        buildAvailBreakpoints();
        buildUnavailVector_u();
        buildUnavailBreakpoints();
    }
    // std::cout << "\t >>> PLOTTING <<< " << std::endl;     
    // std::string plot_file = "plot.txt";
	// std::ofstream fileReport(plot_file); // File report
    // if(!fileReport)
    // {
    //     std::cerr << "ERROR: Unable to access plot file." << std::endl;
    //     exit(EXIT_FAILURE);
    // }
    // int NB_BREAKS = avail_breakpoints[0].size();
    
    // fileReport << "avail:" << std::endl;
    // for (double x = avail_breakpoints[0][0]; x <= avail_breakpoints[0][NB_BREAKS-1]; x+=0.0001){
    //     double approx = approx_log_from_below(x, avail_breakpoints[0]);
    //     fileReport  << x << ","
    //                 << approx << ","
    //                 << std::log(x) << ","
    //                 << std::exp(approx) << ","
    //                 << x - std::exp(approx) << std::endl;
    // }

    // fileReport << "unavail:" << std::endl;
    // NB_BREAKS = unavail_breakpoints[0].size();
    // for (double x = unavail_breakpoints[0][0]; x <= unavail_breakpoints[0][NB_BREAKS-1]; x+=0.0001){
    //     double approx = approx_log_from_below(x, unavail_breakpoints[0]);
    //     fileReport  << x << ","
    //                 << approx << ","
    //                 << std::log(x) << ","
    //                 << std::exp(approx) << ","
    //                 << x - std::exp(approx) << std::endl;
    // }
    // fileReport.close();
}
/* Set up the vector u for approximating log(avail). */
void Model::buildAvailVector_u(){
    const int PATH_NB_BREAKS    = data.getInput().getNbBreakpoints();
    const int NB_DEMANDS        = data.getNbDemands();
    int PATH_NB_TOUCHS          = PATH_NB_BREAKS;

    if (data.getInput().getApproximationType() ==  Input::APPROXIMATION_TYPE_RESTRICTION){
        PATH_NB_TOUCHS = PATH_NB_BREAKS + 1;
    }

    avail_touch.resize(NB_DEMANDS);
    for (int q = 0; q < NB_DEMANDS; q++){
        double leastAvailPath = data.getChainAvailability(data.getNLeastAvailableNodes(data.getDemand(q).getNbVNFs()));
        double mostAvailPath = data.getChainAvailability(data.getNMostAvailableNodes(1));
    
        for (int k = 1; k <= PATH_NB_TOUCHS; k++){
            double expo = ((double) (PATH_NB_TOUCHS - k)) / (PATH_NB_TOUCHS - 1);
            double u = mostAvailPath * std::pow((leastAvailPath/mostAvailPath), expo);
            avail_touch[q].push_back(u);
        }
        printVector(avail_touch[q], "avail touch");
    }
}

/* Set up the breakpoints for approximating log(avail). */
void Model::buildAvailBreakpoints(){
    const int NB_DEMANDS = data.getNbDemands();
    avail_breakpoints.resize(NB_DEMANDS);
    if (data.getInput().getApproximationType() == Input::APPROXIMATION_TYPE_RESTRICTION){
        for (int q = 0; q < NB_DEMANDS; q++){
            for (unsigned int k = 1; k < avail_touch[q].size(); k++){
                double log_u_k1 = std::log(avail_touch[q][k]);
                double log_u_k = std::log(avail_touch[q][k-1]);
                double inv_u_k1 = 1.0/avail_touch[q][k];
                double inv_u_k = 1.0/avail_touch[q][k-1];
                
                avail_breakpoints[q].push_back( (log_u_k1 - log_u_k) / (inv_u_k - inv_u_k1) );
            }
            printVector(avail_breakpoints[q], "avail breakpoints");
        }
    }
    else{
        for (int q = 0; q < NB_DEMANDS; q++){
            for (unsigned int k = 0; k < avail_touch[q].size(); k++){
                avail_breakpoints[q].push_back(avail_touch[q][k]);
            }
            printVector(avail_breakpoints[q], "avail breakpoints");
        }
    }
}

/* Set up the vector u for approximating log(unavail). */
void Model::buildUnavailVector_u(){
    const int NB_DEMANDS        = data.getNbDemands();
    unavail_touch.resize(NB_DEMANDS);

    for (int q = 0; q < NB_DEMANDS; q++){
        const int PATH_NB_TOUCHS    = (int)avail_touch[q].size();
        //std::cout << (int)avail_touch[q].size() << std::endl;
        //const int PATH_NB_TOUCHS    = 12;
        double leastAvailPath = data.getChainAvailability(data.getNLeastAvailableNodes(data.getDemand(q).getNbVNFs()));
        double mostAvailPath = data.getChainAvailability(data.getNMostAvailableNodes(1));
        double UB = 1.0 - leastAvailPath;
        double LB = 1.0 - mostAvailPath;
        for (int k = 1; k <= PATH_NB_TOUCHS; k++){
            double expo = ((double) (PATH_NB_TOUCHS - k)) / (PATH_NB_TOUCHS - 1);
            double u = (UB) * std::pow((LB/UB), expo);
            unavail_touch[q].push_back(u);
        }
        unavail_touch[q].push_back(1.0);
        printVector(unavail_touch[q], "config vector u");
    }
}

/* Set up the breakpoints for approximating log(unavail). */
void Model::buildUnavailBreakpoints(){
    const int NB_DEMANDS = data.getNbDemands();
    unavail_breakpoints.resize(NB_DEMANDS);
    
    if (data.getInput().getApproximationType() == Input::APPROXIMATION_TYPE_RESTRICTION){
        for (int q = 0; q < NB_DEMANDS; q++){
            for (unsigned int k = 1; k <= unavail_touch[q].size()-1; k++){
                double log_u_k1 = std::log(unavail_touch[q][k]);
                double log_u_k = std::log(unavail_touch[q][k-1]);
                double inv_u_k1 = 1.0/unavail_touch[q][k];
                double inv_u_k = 1.0/unavail_touch[q][k-1];
                unavail_breakpoints[q].push_back( (log_u_k1 - log_u_k) / (inv_u_k - inv_u_k1) );
            }
            printVector(unavail_breakpoints[q], "unavail breakpoints");
        }
    }
    else{
        for (int q = 0; q < NB_DEMANDS; q++){
            for (unsigned int k = 0; k < unavail_touch[q].size(); k++){
                unavail_breakpoints[q].push_back(unavail_touch[q][k]);
            }
            printVector(unavail_breakpoints[q], "unavail breakpoints");
        }
    }
}

/** Set up the Cplex parameters. **/
void Model::setCplexParameters(){
    /** Build callback **/
    callback = new Callback(env, data, y, x, alpha);

    /* Define contexts under which the callback will be executed */
    CPXLONG chosenContext = 0;
	chosenContext |= IloCplex::Callback::Context::Id::Candidate;
	//chosenContext |= IloCplex::Callback::Context::Id::Relaxation;

    /* Use callback within the defined contexts */
    if (data.getInput().getApproximationType() == Input::APPROXIMATION_TYPE_RELAXATION || data.getInput().isBasic()){
	    cplex.use(callback, chosenContext);
    }

    /** Time limit definition **/
    cplex.setParam(IloCplex::Param::TimeLimit, data.getInput().getTimeLimit());    // Execution time limited
	
    // cplex.setParam(IloCplex::Param::Simplex::Tolerances::Feasibility, 1e-9);
    // cplex.setParam(IloCplex::Param::Simplex::Tolerances::Optimality, 1e-9);
    // cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 1e-12); 
    // cplex.setParam(IloCplex::Param::MIP::Tolerances::AbsMIPGap, 1e-12); 
    // // cplex.setParam(IloCplex::Param::MIP::Tolerances::Linearization, 1e-12); 
    // // cplex.setParam(IloCplex::Param::MIP::Tolerances::LowerCutoff, 1e-6); 
    // cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 1e-12); 
    // cplex.setParam(IloCplex::Param::MIP::Tolerances::ObjDifference, 1e-12);  
    // cplex.setParam(IloCplex::Param::MIP::Tolerances::RelObjDifference, 1e-12);  
    // // cplex.setParam(IloCplex::Param::MIP::Tolerances::UpperCutoff, 1e-6);    // Execution time limited
    // //cplex.setParam(IloCplex::Param::Threads, 1); // Treads limited
}

/* Set up variables */
void Model::setVariables(){
    const int NB_NODES   = lemon::countNodes(data.getGraph());
    const int NB_ARCS    = lemon::countArcs(data.getGraph());
    const int NB_DEMANDS = data.getNbDemands();
    const int NB_VNFS    = data.getNbVnfs();

    std::cout << "\t Setting up variables... " << std::endl;

    /* Flow variables: f[k][a][i][p] = 1 if arc a is used for routing section i of path p from demand k. */
    std::cout << "\t > Setting up flow variables. " << std::endl;
    f.resize(NB_DEMANDS);
    for (int k = 0; k < NB_DEMANDS; k++){
        const int NB_PATHS   = data.getNbPaths(k);
        f[k].resize(NB_ARCS);
        for (ArcIt it(data.getGraph()); it != lemon::INVALID; ++it){
            int a = data.getArcId(it);
            const int NB_SECTIONS = data.getDemand(k).getNbVNFs() + 1;
            f[k][a].resize(NB_SECTIONS);
            for (int i = 0; i < NB_SECTIONS; i++){
                f[k][a][i].resize(NB_PATHS);
                for (int p = 0; p < NB_PATHS; p++){
                    std::string name = "f(" + std::to_string(k) + "," + std::to_string(a) + "," + std::to_string(i) + "," + std::to_string(p) + ")";
                    if (data.getInput().isRelaxation()){
                        f[k][a][i][p] = IloNumVar(env, 0.0, 1.0, ILOFLOAT, name.c_str());
                    }
                    else{
                        f[k][a][i][p] = IloNumVar(env, 0.0, 1.0, ILOINT, name.c_str());
                    }
                    model.add(f[k][a][i][p]);
                }
            }
        }
    }

    /* VNF section placement variables: x[k][v][i][p] = 1 if there is the i-th VNF of demand k is placed on node v within its path p. */
    std::cout << "\t > Setting up VNF section placement variables. " << std::endl;
    x.resize(NB_DEMANDS);
    for (int k = 0; k < NB_DEMANDS; k++){
        const int NB_PATHS = data.getNbPaths(k);
        x[k].resize(NB_NODES);
        for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
            int v = data.getNodeId(n);
            x[k][v].resize(data.getDemand(k).getNbVNFs());
            for (int i = 0; i < data.getDemand(k).getNbVNFs(); i++){
                x[k][v][i].resize(NB_PATHS);
                for (int p = 0; p < NB_PATHS; p++){
                    std::string name = "x(" + std::to_string(k) + "," + std::to_string(v) + "," + std::to_string(i) + "," + std::to_string(p) + ")";
                    if (data.getInput().isRelaxation()){
                        x[k][v][i][p] = IloNumVar(env, 0.0, 1.0, ILOFLOAT, name.c_str());
                    }
                    else{
                        x[k][v][i][p] = IloNumVar(env, 0.0, 1.0, ILOINT, name.c_str());
                    }
                    model.add(x[k][v][i][p]);
                }
            }
        }
    }

    /* VNF path placement variables: y[k][v][p] = 1 if there is a VNF placed on node v within the path p of demand k. */
    std::cout << "\t > Setting up VNF path placement variables. " << std::endl;
    y.resize(NB_DEMANDS);
    for (int k = 0; k < NB_DEMANDS; k++){
        const int NB_PATHS = data.getNbPaths(k);
        y[k].resize(NB_NODES);
        for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
            int v = data.getNodeId(n);
            y[k][v].resize(NB_PATHS);
            for (int p = 0; p < NB_PATHS; p++){
                std::string name = "y(" + std::to_string(k) + "," + std::to_string(v) + "," + std::to_string(p) + ")";
                if (data.getInput().isRelaxation()){
                    y[k][v][p] = IloNumVar(env, 0.0, 1.0, ILOFLOAT, name.c_str());
                }
                else{
                    y[k][v][p] = IloNumVar(env, 0.0, 1.0, ILOINT, name.c_str());
                }
                model.add(y[k][v][p]);
            }
        }
    }

    /* VNF global placement variables: z[v][f] = 1 if vnf f is placed on node v */
    std::cout << "\t > Setting up VNF global placement variables. " << std::endl;
    z.resize(NB_NODES);
    for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
        int v = data.getNodeId(n);
        z[v].resize(NB_VNFS);
        for (int f = 0; f < NB_VNFS; f++){
            int vnf = data.getVnf(f).getId();
            std::string name = "z(" + std::to_string(v) + "," + std::to_string(vnf) + ")";
            if (data.getInput().isRelaxation()){
                z[v][vnf] = IloNumVar(env, 0.0, 1.0, ILOFLOAT, name.c_str());
            }
            else{
                z[v][vnf] = IloNumVar(env, 0.0, 1.0, ILOINT, name.c_str());
            }
            model.add(z[v][vnf]);
        }
    }

    /* Path activation variables: alpha[k][p] = 1 if path p is active for demand k */
    std::cout << "\t > Setting up path activation variables. " << std::endl;
    alpha.resize(NB_DEMANDS);
    for (int k = 0; k < NB_DEMANDS; k++){
        const int NB_PATHS = data.getNbPaths(k);
        alpha[k].resize(NB_PATHS);
        double REQUIRED_AVAIL = data.getDemand(k).getAvailability();
        int MIN_NB_PATHS = data.getMinNbNodes(REQUIRED_AVAIL);
        std::cout << "Min/max nb paths for demand " << k << ": " << MIN_NB_PATHS << ", " << NB_PATHS << std::endl;
        for (int p = 0; p < NB_PATHS; p++){
            std::string name = "alpha(" + std::to_string(k) + "," + std::to_string(p) + ")";
            double lb = 0.0;
            if (p < MIN_NB_PATHS && data.getInput().getNbPathsLowerBound() == Input::NB_PATHS_LOWER_BOUND_ON){
                lb = 1.0;
            }
            if (data.getInput().isRelaxation()){
                alpha[k][p] = IloNumVar(env, lb, 1.0, ILOFLOAT, name.c_str());
            }
            else{
                alpha[k][p] = IloNumVar(env, lb, 1.0, ILOINT, name.c_str());
            }
            model.add(alpha[k][p]);
        }
    }

    if(data.getInput().isRelaxation() == false){
        /* Path availability variables: avail[k][p] */
        std::cout << "\t > Setting up path availability variables. " << std::endl;
        avail.resize(NB_DEMANDS);
        for (int k = 0; k < NB_DEMANDS; k++){
            const int NB_PATHS = data.getNbPaths(k);
            avail[k].resize(NB_PATHS);
            for (int p = 0; p < NB_PATHS; p++){
                std::string name = "avail(" + std::to_string(k) + "," + std::to_string(p) + ")";
                avail[k][p] = IloNumVar(env, 0.0, 1.0, ILOFLOAT, name.c_str());
                model.add(avail[k][p]);
            }
        }


        /* Approximated path availability variables: approx_log_avail[k][p] */
        std::cout << "\t > Setting up path availability variables. " << std::endl;
        approx_log_avail.resize(NB_DEMANDS);
        for (int k = 0; k < NB_DEMANDS; k++){
            const int NB_PATHS = data.getNbPaths(k);
            approx_log_avail[k].resize(NB_PATHS);
            for (int p = 0; p < NB_PATHS; p++){
                std::string name = "approx_log_avail(" + std::to_string(k) + "," + std::to_string(p) + ")";
                approx_log_avail[k][p] = IloNumVar(env, -IloInfinity, 0.0, ILOFLOAT, name.c_str());
                model.add(approx_log_avail[k][p]);
            }
        }
        /* Path unavailability variables: unavail[k][p] */
        std::cout << "\t > Setting up path unavailability variables. " << std::endl;
        unavail.resize(NB_DEMANDS);
        for (int k = 0; k < NB_DEMANDS; k++){
            const int NB_PATHS = data.getNbPaths(k);
            unavail[k].resize(NB_PATHS);
            for (int p = 0; p < NB_PATHS; p++){
                std::string name = "unavail(" + std::to_string(k) + "," + std::to_string(p) + ")";
                unavail[k][p] = IloNumVar(env, 0.0, 1.0, ILOFLOAT, name.c_str());
                model.add(unavail[k][p]);
            }
        }

        /* Approximated config availability variables: approx_log_unavail[k][p] */
        std::cout << "\t > Setting up path availability variables. " << std::endl;
        approx_log_unavail.resize(NB_DEMANDS);
        for (int k = 0; k < NB_DEMANDS; k++){
            const int NB_PATHS = data.getNbPaths(k);
            approx_log_unavail[k].resize(NB_PATHS);
            for (int p = 0; p < NB_PATHS; p++){
                std::string name = "approx_log_unavail(" + std::to_string(k) + "," + std::to_string(p) + ")";
                approx_log_unavail[k][p] = IloNumVar(env, -IloInfinity, 0.0, ILOFLOAT, name.c_str());
                model.add(approx_log_unavail[k][p]);
            }
        }
    }
}

/* Set up objective function. */
void Model::setObjective(){

    std::cout << "\t Setting up objective function... " << std::endl;

	IloExpr exp(env);
  	for(NodeIt n(data.getGraph()); n != lemon::INVALID; ++n) {
        int v = data.getNodeId(n);
        for (int i = 0; i < data.getNbVnfs(); i++){
            int f = data.getVnf(i).getId();
            double cost = data.getPlacementCost(data.getNode(v), data.getVnf(f));
            exp += ( cost*z[v][f] ); 
        }
    }
    // for (int k = 0; k < data.getNbDemands(); k++){
    //     const int NB_PATHS = data.getNbPaths(k);
    //     for (int p = 0; p < NB_PATHS; p++){
    //         exp -= (avail[k][p]); 
    //     }
    // }
	obj.setExpr(exp);
	obj.setSense(IloObjective::Minimize);
    //obj.setSense(IloObjective::Maximize);
    model.add(obj);
	exp.clear();
    exp.end();
}

/* Set up constraints. */
void Model::setConstraints(){

    std::cout << "\t Setting up constraints... " << std::endl;

    setRoutingConstraints();
    setNodeCapacityConstraints();
    setArcCapacityConstraints();
    setLatencyConstraints();
    setLinkingConstraints();
    setDisjunctionConstraints();
    setDegreeConstraints();

    if (data.getInput().isBasic() == false){
        setSymmetryBreakingConstraints();

            /* Add up the approximated path availability constraints. */
        setPathAvailApproxConstraints();
            /* Add up the approximated configuration availability constraints. */
        setConfigAvailApproxConstraints();
    }
/*
    if (data.getInput().getStrongNodeCapacity() == Input::STRONG_NODE_CAPACITY_ON){
        setStrongNodeCapacityConstraints();
    }
*/
    model.add(constraints);

}

/* Add up the routing constraints: flow conservation constraints among sections */
void Model::setRoutingConstraints()
{
    std::cout << "\t > Setting up routing constraints. " << std::endl;

    for (int k = 0; k < data.getNbDemands(); k++){
        const int NB_PATHS = data.getNbPaths(k);
        int o_k = data.getDemand(k).getSource();
        int d_k = data.getDemand(k).getTarget();
        for (int p = 0; p < NB_PATHS; p++){
            const int NB_SECTIONS = data.getDemand(k).getNbVNFs() + 1;
            for (int i = 0; i < NB_SECTIONS; i++){
                for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
                    int v = data.getNodeId(n);
                    
                    IloExpr exp(env);
                    for (Graph::OutArcIt it(data.getGraph(), n); it != lemon::INVALID; ++it){
                        int a = data.getArcId(it);
                        exp += f[k][a][i][p];
                    }
                    for (Graph::InArcIt it(data.getGraph(), n); it != lemon::INVALID; ++it){
                        int a = data.getArcId(it);
                        exp -= f[k][a][i][p];
                    }
                    
                    /* First section; */
                    if (i == 0){
                        /* Source node; */
                        if (v == o_k){
                            exp -= alpha[k][p];
                            exp += x[k][o_k][i][p];
                        }
                        /* Not source */
                        else{
                            exp += x[k][v][i][p];
                        }
                    }
                    else{
                        /* Last section */
                        if (i == NB_SECTIONS-1){
                            /* Target node; */
                            if (v == d_k){
                                exp -= x[k][d_k][i-1][p];
                                exp += alpha[k][p];
                            }
                            /* Not target */
                            else{
                                exp -= x[k][v][i-1][p];
                            }      
                        }
                        /* other sections */
                        else{
                            exp += x[k][v][i][p];
                            exp -= x[k][v][i-1][p];
                        }
                    }
                    std::string name = "Routing(" + std::to_string(k) + "," + std::to_string(p) + "," + std::to_string(i) + "," + std::to_string(v) + ")";
                    constraints.add(IloRange(env, 0, exp, 0, name.c_str()));
                    exp.clear();
                    exp.end();
                }
            }
        }
    }
}


/* Add up the degree constraints */
void Model::setDegreeConstraints()
{
    std::cout << "\t > Setting up degree constraints. " << std::endl;

    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            const int NB_SECTIONS = data.getDemand(k).getNbVNFs() + 1;
            for (int i = 0; i < NB_SECTIONS; i++){
                for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
                    int v = data.getNodeId(n);
                    
                    IloExpr exp(env);
                    for (Graph::OutArcIt it(data.getGraph(), n); it != lemon::INVALID; ++it){
                        int a = data.getArcId(it);
                        exp += f[k][a][i][p];
                    }

                    std::string name = "Degree(" + std::to_string(k) + "," + std::to_string(p) + "," + std::to_string(i) + "," + std::to_string(v) + ")";
                    constraints.add(IloRange(env, 0, exp, 1, name.c_str()));
                    exp.clear();
                    exp.end();
                }
            }
        }
    }
}

/* Add up the node capacity constraints: the bandwidth treated in a node must respect its capacity. */
void Model::setNodeCapacityConstraints(){
    std::cout << "\t > Setting up node capacity constraints " << std::endl;
    for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
        int v = data.getNodeId(n);
        IloExpr exp(env);
        double capacity = data.getNode(v).getCapacity();
        for (int k = 0; k < data.getNbDemands(); k++){
            for (int i = 0; i < data.getDemand(k).getNbVNFs(); i++){
                for (int p = 0; p < data.getNbPaths(k); p++){
                    int vnf = data.getDemand(k).getVNF_i(i);
                    double coeff = data.getDemand(k).getBandwidth() * data.getVnf(vnf).getConsumption();
                    exp += (coeff * x[k][v][i][p]);
                }
            }
        }
        std::string name = "Node_Capacity(" + std::to_string(v) + ")";
        constraints.add(IloRange(env, 0, exp, capacity, name.c_str()));
        exp.clear();
        exp.end();
    }
}

/* Add up the arc capacity constraints: the bandwidth routed within an arc must respect its capacity. */
void Model::setArcCapacityConstraints(){
    std::cout << "\t > Setting up arc capacity constraints. " << std::endl;
    for (ArcIt it(data.getGraph()); it != lemon::INVALID; ++it){
        int a = data.getArcId(it);
        IloExpr exp(env);
        double capacity = data.getLink(a).getBandwidth();
        for (int k = 0; k < data.getNbDemands(); k++){
            for (int i = 0; i < data.getDemand(k).getNbVNFs(); i++){
                for (int p = 0; p < data.getNbPaths(k); p++){
                    double coeff = data.getDemand(k).getBandwidth();
                    exp += (coeff * f[k][a][i][p]);
                }
            }
        }
        std::string name = "Arc_Capacity(" + std::to_string(a) + ")";
        constraints.add(IloRange(env, 0, exp, capacity, name.c_str()));
        exp.clear();
        exp.end();
    }
}


/* Add up the latency constraints: the path lengths must respect latency. */
void Model::setLatencyConstraints(){
    std::cout << "\t > Setting up latency constraints. " << std::endl;
    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            double latency = data.getDemand(k).getMaxLatency();
            for (ArcIt it(data.getGraph()); it != lemon::INVALID; ++it){
                int a = data.getArcId(it);
                for (int i = 0; i < data.getDemand(k).getNbVNFs()+1; i++){
                    double coeff = data.getLink(a).getDelay();
                    exp += (coeff * f[k][a][i][p]);
                }
            }
            std::string name = "Latency(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, 0, exp, latency, name.c_str()));
            exp.clear();
            exp.end();
        }
    }
}

/* Add up the linking constraints. */
void Model::setLinkingConstraints(){
    std::cout << "\t > Setting up linking constraints. " << std::endl;

    /* linking variables alpha-x */
    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            for (int i = 0; i < data.getDemand(k).getNbVNFs(); i++){
                IloExpr exp(env);
                for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
                    int v = data.getNodeId(n);
                    exp += x[k][v][i][p];
                }
                exp -= alpha[k][p];
                std::string name = "Linking_alpha(" + std::to_string(k) + "," + std::to_string(p) + "," + std::to_string(i) + ")";
                constraints.add(IloRange(env, 0, exp, 0, name.c_str()));
                exp.clear();
                exp.end();
            }
        }
    }

    
    /* linking variables x-y */
    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            for (int i = 0; i < data.getDemand(k).getNbVNFs(); i++){
                for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
                    int v = data.getNodeId(n);
                    IloExpr exp(env);
                    exp += x[k][v][i][p];
                    exp -= y[k][v][p];
                    std::string name = "Linking_x(" + std::to_string(k) + "," + std::to_string(p) + "," + std::to_string(i) +"," + std::to_string(v) + ")";
                    constraints.add(IloRange(env, -1, exp, 0, name.c_str()));
                    exp.clear();
                    exp.end();
                }
            }
        }
    }

    /* linking variables x-z */
    for (int k = 0; k < data.getNbDemands(); k++){
        for (int i = 0; i < data.getDemand(k).getNbVNFs(); i++){
            int f = data.getDemand(k).getVNF_i(i);
            for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
                int v = data.getNodeId(n);
                IloExpr exp(env);
                
                for (int p = 0; p < data.getNbPaths(k); p++){
                    exp += x[k][v][i][p];
                }
                exp -= z[v][f];
                std::string name = "Linking_z(" + std::to_string(k) + + "," + std::to_string(i) +"," + std::to_string(v) + ")";
                constraints.add(IloRange(env, -1, exp, 0, name.c_str()));
                exp.clear();
                exp.end();
            }
        }
    }
}

/* Add up the vnf disjunction constraints: a node cannot host more than one VNF for the same demand. */
void Model::setDisjunctionConstraints(){
    std::cout << "\t > Setting up vnf disjunction constraints. " << std::endl;
    
    for (int k = 0; k < data.getNbDemands(); k++){
        for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
            int v = data.getNodeId(n);
            IloExpr exp(env);
            for (int p = 0; p < data.getNbPaths(k); p++){
                exp += y[k][v][p];
            }
            std::string name = "Disjuction(" + std::to_string(k) + "," + std::to_string(v) + ")";
            constraints.add(IloRange(env, 0, exp, 1, name.c_str()));
            exp.clear();
            exp.end();
        }
    }
}


/*  */
void Model::setSymmetryBreakingConstraints(){
    std::cout << "\t > Setting up symmetry breaking constraints. " << std::endl;
    
    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k)-1; p++){
            IloExpr exp(env);
            exp += alpha[k][p+1];
            exp -= alpha[k][p];
            std::string name = "Symm(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, -1, exp, 0, name.c_str()));
            exp.clear();
            exp.end();
        }
    }
}


/* Add up the strong node capacity constraints. */
void Model::setStrongNodeCapacityConstraints(){
    for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
        int v = data.getNodeId(n);
        double capacity = data.getNode(v).getCapacity();
        for (int f = 0; f < data.getNbVnfs(); f++){
            IloExpr exp(env);
            for (int k = 0; k < data.getNbDemands(); k++){
                for (int i = 0; i < data.getDemand(k).getNbVNFs(); i++){
                    int vnf = data.getDemand(k).getVNF_i(i);
                    if (vnf == f){
                        for (int p = 0; p < data.getNbPaths(k); p++){
                            double coeff = data.getDemand(k).getBandwidth() * data.getVnf(vnf).getConsumption();
                            exp += (coeff * x[k][v][i][p]);
                        }
                    }
                }
            }
            exp -= ( capacity * z[v][f]);
            std::string name = "Strong_Node_Capacity(" + std::to_string(v) + "," + std::to_string(f) + ")";
            constraints.add(IloRange(env, -IloInfinity, exp, 0, name.c_str()));
            exp.clear();
            exp.end();
        }
    }
}


void Model::buildApproximationFunctionAvail(int k, IloNumArray &breakpoints, IloNumArray &slopes){
    for (unsigned int i = 0; i < avail_breakpoints[k].size(); i++){
        breakpoints.add(avail_breakpoints[k][i]);
    }
    switch (data.getInput().getApproximationType()){
        case Input::APPROXIMATION_TYPE_RESTRICTION:
            for (unsigned int i = 0; i < avail_touch[k].size(); i++){
                double tangent = 1.0/avail_touch[k][i];
                slopes.add(tangent);
            }
            break;
        case Input::APPROXIMATION_TYPE_RELAXATION:
            slopes.add( 1.0/avail_touch[k][0] );
            for (unsigned int i = 0; i < avail_touch[k].size()-1; i++){
                double delta_X = avail_touch[k][i+1] - avail_touch[k][i];
                double delta_Y = std::log(avail_touch[k][i+1]) - std::log(avail_touch[k][i]);
                slopes.add( delta_Y/delta_X );
            }
            slopes.add( 0 );
            break;
        default:
            throw IloCplex::Exception(-1, "ERROR: Unexpected availability approx !");
    }
}

void Model::setPathAvailApproxConstraints(){
    std::cout << "\t > Setting up approximated path availability constraints. " << std::endl;
    
    for (int k = 0; k < data.getNbDemands(); k++){
        IloNumArray breakpoints(env);
        IloNumArray slopes(env);
        buildApproximationFunctionAvail(k, breakpoints, slopes);
        // /** TEST WITH ANOTHER PIECEWISE DESCRIPTION **/
        // IloNumArray coord_X(env);
        // IloNumArray coord_Y(env);
        // for (unsigned int i = 0; i < avail_breakpoints[k].size(); i++){
        //     coord_X.add(avail_breakpoints[k][i]);
        //     coord_Y.add(std::log(avail_breakpoints[k][i]));
        // }
        // double firstSlope = (1.0/avail_breakpoints[k][0]);
        // double lastSlope = (1.0/avail_breakpoints[k][avail_breakpoints[k].size()-1]);
        /***********************************************/
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            exp += approx_log_avail[k][p];
            exp -= IloPiecewiseLinear(avail[k][p], breakpoints, slopes, avail_touch[k][0], std::log(avail_touch[k][0]));
            // exp -= IloPiecewiseLinear(avail[k][p], firstSlope, coord_X, coord_Y, lastSlope);
            std::string name = "approx_avail(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, 0, exp, 0, name.c_str()));
            exp.clear();
            exp.end();
        }
    }

    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            exp += approx_log_avail[k][p];
            for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
                int v = data.getNodeId(n);
                exp -= std::log(data.getNode(v).getAvailability()) * y[k][v][p];
            }
            std::string name = "avail(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, -IloInfinity, exp, 0, name.c_str()));
            exp.clear();
            exp.end();
        }
    }

    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            double avail_ub = data.getChainAvailability(data.getNMostAvailableNodes(1));
            exp += avail[k][p];
            exp -= avail_ub*alpha[k][p];
            std::string name = "availImposition(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, -IloInfinity, exp, 0, name.c_str()));
            exp.clear();
            exp.end();
        }
    }
    
    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            double avail_lb = data.getChainAvailability(data.getNLeastAvailableNodes(data.getDemand(k).getNbVNFs()));
            exp += avail[k][p];
            exp -= avail_lb*alpha[k][p];
            std::string name = "avail_lb(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, 0, exp, IloInfinity, name.c_str()));
            exp.clear();
            exp.end();
        }
    }
}

void Model::buildApproximationFunctionUnavail(int k, IloNumArray &breakpoints, IloNumArray &slopes){
    for (unsigned int i = 0; i < unavail_breakpoints[k].size(); i++){
        breakpoints.add(unavail_breakpoints[k][i]);
    }
    switch (data.getInput().getApproximationType()){
        case Input::APPROXIMATION_TYPE_RESTRICTION:
            for (unsigned int i = 0; i < unavail_touch[k].size(); i++){
                double tangent = 1.0/unavail_touch[k][i];
                slopes.add(tangent);
            }
            break;
        case Input::APPROXIMATION_TYPE_RELAXATION:
            slopes.add(1.0/unavail_breakpoints[k][0]);
            for (unsigned int i = 0; i < unavail_breakpoints[k].size()-1; i++){
                double delta_X = unavail_breakpoints[k][i+1] - unavail_breakpoints[k][i];
                double delta_Y = std::log(unavail_breakpoints[k][i+1]) - std::log(unavail_breakpoints[k][i]);
                slopes.add( delta_Y/delta_X );
            }
            slopes.add(0);
            break;
        default:
            throw IloCplex::Exception(-1, "ERROR: Unexpected availability approx !");
    }
}

void Model::setConfigAvailApproxConstraints(){
    /** avail[k][p] = 1 - unavail[k][p] ***/
    std::cout << "\t > Setting up approximated config availability constraints. " << std::endl;
    
    for (int k = 0; k < data.getNbDemands(); k++){
        IloNumArray breakpoints(env);
        IloNumArray slopes(env);
        buildApproximationFunctionUnavail(k, breakpoints, slopes);
        // /** TEST WITH ANOTHER PIECEWISE DESCRIPTION **/
        // IloNumArray coord_X(env);
        // IloNumArray coord_Y(env);
        // for (unsigned int i = 0; i < unavail_breakpoints[k].size(); i++){
        //     coord_X.add(unavail_breakpoints[k][i]);
        //     coord_Y.add(std::log(unavail_breakpoints[k][i]));
        // }
        // double firstSlope = (1.0/unavail_breakpoints[k][0]);
        // double lastSlope = (1.0/unavail_breakpoints[k][unavail_breakpoints[k].size()-1]);
        /***********************************************/
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            exp += approx_log_unavail[k][p];
            exp -= IloPiecewiseLinear(unavail[k][p], breakpoints, slopes, 1, 0);
            // exp -= IloPiecewiseLinear(unavail[k][p], firstSlope, coord_X, coord_Y, lastSlope);
            std::string name = "approx_unavail(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, 0, exp, 0, name.c_str()));
            exp.clear();
            exp.end();
        }
    }

    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            exp += unavail[k][p];
            exp += avail[k][p];
            std::string name = "unavail(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, 1, exp, 1, name.c_str()));
            exp.clear();
            exp.end();
        }
    }

    for (int k = 0; k < data.getNbDemands(); k++){
        IloExpr exp(env);
        for (int p = 0; p < data.getNbPaths(k); p++){
            exp += approx_log_unavail[k][p];
        }
        std::string name = "ReqAvail(" + std::to_string(k) + ")";
        double rhs = std::log(1.0 - data.getDemand(k).getAvailability());
        constraints.add(IloRange(env, -IloInfinity, exp, rhs, name.c_str()));
        exp.clear();
        exp.end();
    }
}

void Model::run()
{
    cplex.exportModel("mip.lp");
    time = cplex.getCplexTime();
	cplex.solve();

	/* Get final execution time */
	time = cplex.getCplexTime() - time;
}

void Model::printResult(){
    
    const int NB_DEMANDS = data.getNbDemands();
    std::cout << "=> Printing solution ..." << std::endl;
    for (int k = 0; k < NB_DEMANDS; k++) {
        std::cout << std::endl << "----------------------------------------------------" << std::endl << std::endl;
        std::cout << "k=" << k+1 << " : From " << data.getDemand(k).getSource() << " to " << data.getDemand(k).getTarget() << std::endl;
        double placementAv = getPlacementAvailability(k);
        std::cout << "\t Placement Avail: " << placementAv << std::endl;
        std::cout << "\t Required Avail : " << data.getDemand(k).getAvailability() << std::endl;
        if (placementAv < data.getDemand(k).getAvailability()){
            std::cout << "\t ==> UNFEASIBLE BY " << data.getDemand(k).getAvailability() - placementAv << std::endl;
        }
        for (int p = 0; p < data.getNbPaths(k); p++){
            if (cplex.getValue(alpha[k][p]) > 1 - EPS){
                std::cout << "\t Path " << p+1 << std::endl;
                double path_availability = 1.0;
                double log_path_availability = 0.0;
                for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
                    int v = data.getNodeId(n);
                    if (cplex.getValue(y[k][v][p]) > 1 - EPS){
                        path_availability *= data.getNode(v).getAvailability();
                        log_path_availability += std::log(data.getNode(v).getAvailability());
                    }
                }
                std::cout << "\t\t Real Path avail : " << std::setprecision(9) << path_availability << std::endl;
                std::cout << "\t\t Var avail       : " << std::setprecision(9) << cplex.getValue(avail[k][p]) << std::endl;
                if (path_availability - cplex.getValue(avail[k][p]) < 0.0){
                    std::cout << "\t\t (N)" << std::endl;
                }
                else{
                    std::cout << "\t\t (P)" << std::endl; 
                }
                
                std::cout << "\t\t log avail      : " << std::setprecision(9) << std::log(path_availability) << std::endl;
                std::cout << "\t\t approx(avail)  : " << std::setprecision(9) << cplex.getValue(approx_log_avail[k][p])  << std::endl;
                // std::cout << "  Var unavail : " << cplex.getValue(unavail[k][p]) << std::endl;
                // std::cout << "  Real unavail : " << 1.0 - path_availability << std::endl;
                // for (int r = 0; r < CONFIG_NB_BREAKS; r++){
                //     if (cplex.getValue(c_lambda[k][p][r]) >  EPS){
                //         std::cout << "  > b_" << r << ": " << unavail_breakpoints[r] << ", c_lambda = " << cplex.getValue(c_lambda[k][p][r]) << std::endl;
                //     }
                // }
                std::cout << "\t\t Path description : " << std::endl;
                for (int i = 0; i < data.getDemand(k).getNbVNFs() + 1; i++){
                    std::cout << "\t\t\t Section " << i+1 << " : ";
                    printSectionPath(k, p, i);
                }
            }
            else{
                std::cout << "\t\t Path " << p+1 << " not used." << std::endl;
                std::cout << "\t\t Var avail : " << cplex.getValue(avail[k][p]) << std::endl;
                // std::cout << "  Var unavail : " << cplex.getValue(unavail[k][p]) << std::endl;
                // for (int r = 0; r < CONFIG_NB_BREAKS; r++){
                //     if (cplex.getValue(c_lambda[k][p][r]) >  EPS){
                //         std::cout << "  > b_" << r << ": " << unavail_breakpoints[r] << ", c_lambda = " << cplex.getValue(c_lambda[k][p][r]) << std::endl;
                //     }
                // }
            }
        }
    }


    std::cout << "Objective value: " << cplex.getValue(obj) << std::endl;
    std::cout << "Nodes evaluated: " << cplex.getNnodes() << std::endl;
    std::cout << "User cuts added: " << callback->getNbUserCuts() << std::endl;
    std::cout << "Lazy constraints added: " << callback->getNbLazyConstraints() << std::endl;
    std::cout << "Time on cuts: " << callback->getTime() << std::endl;
    std::cout << "Total time: " << time << std::endl << std::endl;
    //testRelaxationAvail();
}

/* Returns the path availability induced by CPLEX's solution */
double Model::getPathAvailability(int k, int p){
     if (cplex.getValue(alpha[k][p]) > 1 - EPS){
        double path_availability = 1.0;
        for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
            int v = data.getNodeId(n);
            if (cplex.getValue(y[k][v][p]) > 1 - EPS){
                path_availability *= data.getNode(v).getAvailability();
            }
        }
        return path_availability;
     }
     return 0;
}

/* Returns the placement availability induced by CPLEX's solution */
double Model::getPlacementAvailability(int k){
    double prob_all_paths_fail = 1.0;
    for (unsigned int p = 0; p < alpha[k].size(); p++){
        double path_failure = 1.0;
        if (cplex.getValue(alpha[k][p]) > 1 - EPS){
            double path_availability = getPathAvailability(k, p);
            path_failure = 1.0 - path_availability;
        }
        prob_all_paths_fail *= path_failure;
    }
    return (1.0 - prob_all_paths_fail);
}

void Model::printSectionPath(const int k, const int p, const int i){
    /*identify source and destinations*/
    Graph::Node sourceNode = lemon::INVALID;
    Graph::Node targetNode = lemon::INVALID;
    if (i == 0){
        for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
            if (data.getNodeId(n) == data.getDemand(k).getSource()){
                sourceNode = n;
            }
        }
    }
    else{
        for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
            int v = data.getNodeId(n);
            if (cplex.getValue(x[k][v][i-1][p]) > 1 - EPS){
                sourceNode = n;
            }
        }
    }
    
    if (i == data.getDemand(k).getNbVNFs()){
        for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
            if (data.getNodeId(n) == data.getDemand(k).getTarget()){
                targetNode = n;
            }
        }
    }
    else{
        for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
            int v = data.getNodeId(n);
            if (cplex.getValue(x[k][v][i][p]) > 1 - EPS){
                targetNode = n;
            }
        }
    }

    /* build path */
    std::cout << data.getNodeId(sourceNode) << " -- ";
    Graph::Node v = sourceNode;
    while( data.getGraph().id(v) != data.getGraph().id(targetNode)){
        for (Graph::OutArcIt it(data.getGraph(), v); it != lemon::INVALID; ++it){
            int a = data.getArcId(it);
            if (cplex.getValue(f[k][a][i][p]) > 1 - EPS){
                v = data.getGraph().target(it);
                if (data.getGraph().id(v) != data.getGraph().id(targetNode)){
                    std::cout << data.getNodeId(v) << " -- ";
                }
                break;
            }
        }
    }
    std::cout << data.getNodeId(targetNode) << "." << std::endl; 
}

void Model::output(){
    std::string output_file = data.getInput().getOutputFile();
    if (output_file.empty()){
        return;
    }

	std::ofstream fileReport(output_file, std::ios_base::app); // File report
    // If file_output can't be opened 
    if(!fileReport)
    {
        std::cerr << "ERROR: Unable to access output file." << std::endl;
        exit(EXIT_FAILURE);
    }

    std::string demand_name = getInBetweenString(data.getInput().getDemandFile(), "/", ".");
    std::string node_name = getInBetweenString(data.getInput().getNodeFile(), "/", ".");
    std::string instance_name = node_name + "_" + demand_name;

    fileReport << instance_name << ";"
    		   << time << ";"
    		   << cplex.getObjValue() << ";"
    		   << cplex.getBestObjValue() << ";"
    		   << cplex.getMIPRelativeGap()*100 << ";"
    		   << cplex.getNnodes() << ";"
    		   << cplex.getNnodesLeft()*0.001 << ";" 
               << callback->getNbLazyConstraints() << ";" 
    		   << callback->getNbUserCuts() << ";" 
    		   << callback->getTime() << ";" 
               << std::endl;
    		   
    // Finalization ***
    fileReport.close();

}

/****************************************************************************************/
/*										   Tests  										*/
/****************************************************************************************/

/** Checks if output value of variables are consistent. **/
void Model::testRelaxationAvail(){
    const int NB_DEMANDS = data.getNbDemands();
    std::cout << std::endl << std::endl;
    std::cout << "==========================================================================" << std::endl;
    std::cout << "  Testing consistency of path availabilities in the relaxation case ..." << std::endl;
    std::cout << "==========================================================================" << std::endl << std::endl;
    for (int k = 0; k < NB_DEMANDS; k++) {
        std::cout << "Demand " << k+1 << " : From " << data.getDemand(k).getSource() << " to " << data.getDemand(k).getTarget() << std::endl;
        for (int p = 0; p < data.getNbPaths(k); p++){
            if (cplex.getValue(alpha[k][p]) > 1 - EPS){
                const double PATH_AVAIL    = getPathAvailability(k, p);
                const double VAR_AVAIL     = cplex.getValue(avail[k][p]);
                std::cout << "\t Path " << p+1 << std::endl;
                std::cout << "\t\t Avail induced by y variables : " << std::setprecision(9) << PATH_AVAIL << std::endl;
                std::cout << "\t\t Value of variable avail :      " << std::setprecision(9) << VAR_AVAIL  << std::endl;
                if (PATH_AVAIL >= VAR_AVAIL + 0.00000001){
                    std::cout << "\t\t\t ====> ERROR : " << std::setprecision(9) << PATH_AVAIL - cplex.getValue(avail[k][p]) << ". Negative approx !!!" << std::endl; 
                }
            }
            else{
                std::cout << "\t Path " << p+1 << " not used." << std::endl;
                std::cout << "\t\t Value of variable avail : " << cplex.getValue(avail[k][p]) << std::endl;
            }
        }
    }
}
/****************************************************************************************/
/*										Destructors 									*/
/****************************************************************************************/
Model::~Model(){
    delete callback;
}