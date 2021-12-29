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
    
    buildAvailBreakpoints();
    setVariables();
    setObjective();  
    setConstraints();  
    setCplexParameters();

    std::cout << "\t Model was correctly built ! " << std::endl;                 
}
// breaks[0] = 0 and breaks[last] = 1.0
/* Returns g(y) where g(y) is a function approximating log(y) from above */
double Model::approx_log_from_above(double y, const vector<IloNum> &breaks, const vector<IloNum> &u){
    int nbBreaks = breaks.size();
    int last = nbBreaks - 1; 
    if (y < 0 || y > 1){
        std::cerr << "ERROR: approximation is only done between 0 and 1" << std::endl;
		exit(EXIT_FAILURE);
    }
    if (y >= 0 && y <= breaks[1]){
        return (((1.0 / u[0])*y) + std::log(u[0]) - 1.0);
    }
    if (y >= breaks[last-1] && y <= 1.0){
        return (y - 1.0);
    }
    for (unsigned int k = 1; k < breaks.size()-2; k++){
        if (y >= breaks[k] && y <= breaks[k+1]){
            return (((1.0 / u[k])*y) + std::log(u[k]) - 1.0);
        }
    }
    
    std::cerr << "ERROR: approximation is undefined" << std::endl;
    exit(EXIT_FAILURE);
}
/* Set up the breakpoints for approximating log(avail). */
void Model::buildAvailBreakpoints(){
    PATH_NB_BREAKS = 5;
    const int NB_DEMANDS = data.getNbDemands();
    //breakpoint.resize(NB_BREAKS);
    path_vector_u.resize(NB_DEMANDS);
    path_breakpoint.resize(NB_DEMANDS);
    config_breakpoint.resize(NB_DEMANDS);
    config_vector_u.resize(NB_DEMANDS);

    for (int q = 0; q < NB_DEMANDS; q++){
        double leastAvailPath = data.getChainAvailability(data.getNLeastAvailableNodes(data.getDemand(q).getNbVNFs()));
        double mostAvailPath = data.getChainAvailability(data.getNMostAvailableNodes(1));
    
        path_vector_u[q].push_back(leastAvailPath);
        for (int k = 1; k < PATH_NB_BREAKS; k++){
            double expo = ((double) (PATH_NB_BREAKS - 1 - k)) / (PATH_NB_BREAKS - 1);
            double u = mostAvailPath * std::pow((leastAvailPath/mostAvailPath), expo);
            path_vector_u[q].push_back(u);
        }
        printVector(path_vector_u[q], "path vector u");
    
        path_breakpoint[q].push_back(0.0);
        for (int k = 0; k < PATH_NB_BREAKS-1; k++){
            double log_u_k1 = std::log(path_vector_u[q][k+1]);
            double log_u_k = std::log(path_vector_u[q][k]);
            double inv_u_k1 = 1.0/path_vector_u[q][k+1];
            double inv_u_k = 1.0/path_vector_u[q][k];
            path_breakpoint[q].push_back( (log_u_k1 - log_u_k) / (inv_u_k - inv_u_k1) );
        }
        path_breakpoint[q].push_back(1.0);
        printVector(path_breakpoint[q], "path breakpoints");
    

        config_vector_u[q].push_back(1.0- mostAvailPath);
        for (int k = 1; k < PATH_NB_BREAKS; k++){
            double expo = ((double) (PATH_NB_BREAKS - 1 - k)) / (PATH_NB_BREAKS - 1);
            double u = (1.0 - leastAvailPath) * std::pow(((1.0 - mostAvailPath)/(1.0 - leastAvailPath)), expo);
            config_vector_u[q].push_back(u);
        }
        printVector(config_vector_u[q], "config vector u");

        config_breakpoint[q].push_back(0.0);
        for (int k = 0; k < config_vector_u[q].size()-1; k++){
            double log_u_k1 = std::log(config_vector_u[q][k+1]);
            double log_u_k = std::log(config_vector_u[q][k]);
            double inv_u_k1 = 1.0/config_vector_u[q][k+1];
            double inv_u_k = 1.0/config_vector_u[q][k];
            config_breakpoint[q].push_back( (log_u_k1 - log_u_k) / (inv_u_k - inv_u_k1) );
        }
        config_breakpoint[q].push_back(1.0);
        //path_breakpoint.push_back(1.0);
        printVector(config_breakpoint[q], "config breakpoints");
    }
    // CONFIG_NB_BREAKS = 25;
    // std::cout << "vector u: " << std::endl;
    // for (int k = 0; k < CONFIG_NB_BREAKS; k++){
    //     double expo = ((double) (CONFIG_NB_BREAKS - 1 - k)) / (CONFIG_NB_BREAKS - 1);
    //     config_vector_u.push_back(std::pow(0.00000001, expo));
    //     std::cout << k << ": " << config_vector_u[k] << std::endl;
    // }

    // std::cout << "config_breakpoints: " << std::endl;
    // for (int k = 0; k < CONFIG_NB_BREAKS-1; k++){
    //     double log_u_k1 = std::log(config_vector_u[k+1]);
    //     double log_u_k = std::log(config_vector_u[k]);
    //     double inv_u_k1 = 1.0/config_vector_u[k+1];
    //     double inv_u_k = 1.0/config_vector_u[k];
    //     config_breakpoint.push_back( (log_u_k1 - log_u_k) / (inv_u_k - inv_u_k1) );
    //     std::cout << k << ": " << config_breakpoint[k] << std::endl;
    // }
    // config_breakpoint.push_back(1.0);

/*
    std::string plot_file = "plot.txt";
	std::ofstream fileReport(plot_file); // File report
    if(!fileReport)
    {
        std::cerr << "ERROR: Unable to access plot file." << std::endl;
        exit(EXIT_FAILURE);
    }

    for (double x = 0.001; x <= 1; x+=0.001){
        fileReport  << x << ","
                    << approx_log_from_above(x, path_breakpoint, path_vector_u) << ","
                    << std::log(x) << std::endl;
    }		   
    fileReport.close();
    */
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
	// cplex.use(callback, chosenContext);

    /** Time limit definition **/
    cplex.setParam(IloCplex::Param::TimeLimit, data.getInput().getTimeLimit());    // Execution time limited
	
    cplex.setParam(IloCplex::Param::Simplex::Tolerances::Feasibility, 1e-9);
    cplex.setParam(IloCplex::Param::Simplex::Tolerances::Optimality, 1e-9);
    cplex.setParam(IloCplex::Param::MIP::Tolerances::Integrality, 1e-12); 
    cplex.setParam(IloCplex::Param::MIP::Tolerances::AbsMIPGap, 1e-12); 
    // cplex.setParam(IloCplex::Param::MIP::Tolerances::Linearization, 1e-12); 
    // cplex.setParam(IloCplex::Param::MIP::Tolerances::LowerCutoff, 1e-6); 
    cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 1e-12); 
    cplex.setParam(IloCplex::Param::MIP::Tolerances::ObjDifference, 1e-12);  
    cplex.setParam(IloCplex::Param::MIP::Tolerances::RelObjDifference, 1e-12);  
    // cplex.setParam(IloCplex::Param::MIP::Tolerances::UpperCutoff, 1e-6);    // Execution time limited
    //cplex.setParam(IloCplex::Param::Threads, 1); // Treads limited
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

    /* lambda variables for piecewise linear approx: lambda[k][p][r] */
    std::cout << "\t > Setting up lambda variables for piecewise linear approx. " << std::endl;
    lambda.resize(NB_DEMANDS);
    for (int k = 0; k < NB_DEMANDS; k++){
        const int NB_PATHS = data.getNbPaths(k);
        lambda[k].resize(NB_PATHS);
        for (int p = 0; p < NB_PATHS; p++){
            lambda[k][p].resize(path_breakpoint[k].size());
            for (unsigned int r = 0; r < path_breakpoint[k].size(); r++){
                std::string name = "lambda(" + std::to_string(k) + "," + std::to_string(p) + "," + std::to_string(r) + ")";
                lambda[k][p][r] = IloNumVar(env, 0.0, 1.0, ILOFLOAT, name.c_str());
                model.add(lambda[k][p][r]);
            }
        }
    }

    /* interval lambda variables for piecewise linear approx: interval[k][p][r] */
    std::cout << "\t > Setting up interval variables for piecewise linear approx. " << std::endl;
    interval.resize(NB_DEMANDS);
    for (int k = 0; k < NB_DEMANDS; k++){
        const int NB_PATHS = data.getNbPaths(k);
        interval[k].resize(NB_PATHS);
        for (int p = 0; p < NB_PATHS; p++){
            interval[k][p].resize(path_breakpoint[k].size()-1);
            for (unsigned int r = 0; r < path_breakpoint[k].size()-1; r++){
                std::string name = "interval(" + std::to_string(k) + "," + std::to_string(p) + "," + std::to_string(r) + ")";
                interval[k][p][r] = IloNumVar(env, 0.0, 1.0, ILOINT, name.c_str());
                model.add(interval[k][p][r]);
            }
        }
    }

    /* c_lambda variables for piecewise linear approx: lambda[k][p][r] */
    std::cout << "\t > Setting up clambda variables for piecewise linear approx. " << std::endl;
    c_lambda.resize(NB_DEMANDS);
    for (int k = 0; k < NB_DEMANDS; k++){
        const int NB_PATHS = data.getNbPaths(k);
        c_lambda[k].resize(NB_PATHS);
        for (int p = 0; p < NB_PATHS; p++){
            c_lambda[k][p].resize(config_breakpoint[k].size());
            for (unsigned int r = 0; r < config_breakpoint[k].size(); r++){
                std::string name = "c_lambda(" + std::to_string(k) + "," + std::to_string(p) + "," + std::to_string(r) + ")";
                c_lambda[k][p][r] = IloNumVar(env, 0.0, 1.0, ILOFLOAT, name.c_str());
                model.add(c_lambda[k][p][r]);
            }
        }
    }

    /* interval lambda variables for piecewise linear approx: interval[k][p][r] */
    std::cout << "\t > Setting up cinterval variables for piecewise linear approx. " << std::endl;
    c_interval.resize(NB_DEMANDS);
    for (int k = 0; k < NB_DEMANDS; k++){
        const int NB_PATHS = data.getNbPaths(k);
        c_interval[k].resize(NB_PATHS);
        for (int p = 0; p < NB_PATHS; p++){
            c_interval[k][p].resize(config_breakpoint[k].size()-1);
            for (unsigned int r = 0; r < config_breakpoint[k].size()-1; r++){
                std::string name = "c_interval(" + std::to_string(k) + "," + std::to_string(p) + "," + std::to_string(r) + ")";
                c_interval[k][p][r] = IloNumVar(env, 0.0, 1.0, ILOINT, name.c_str());
                model.add(c_interval[k][p][r]);
            }
        }
    }
}

/* Set up objective function. */
void Model::setObjective(){

    std::cout << "\t Setting up objective function... " << std::endl;

	IloExpr exp(env);
    //TODO: change obj
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

    setSymmetryBreakingConstraints();

		/* Add up the approximated path availability constraints. */
	 setPathAvailApproxConstraints();
		/* Add up the approximated configuration availability constraints. */
	 setConfigAvailApproxConstraints();
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



void Model::setPathAvailApproxConstraints(){
    std::cout << "\t > Setting up approximated path availability constraints. " << std::endl;
    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            for (int r = 0; r < path_breakpoint[k].size(); r++){
                exp += (path_breakpoint[k][r] * lambda[k][p][r]);
            }
            exp -= avail[k][p];
            std::string name = "avail(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, 0, exp, 0, name.c_str()));
            exp.clear();
            exp.end();
        }
    }

    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            for(int r = 0; r < path_breakpoint[k].size(); r++){
                exp += lambda[k][p][r];
            }
            std::string name = "lambda(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, 1, exp, 1, name.c_str()));
            exp.clear();
            exp.end();
        }
    }

    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            for(int r = 0; r < path_breakpoint[k].size(); r++){
                // coeff = g(bp[r])
                double coeff = approx_log_from_above(path_breakpoint[k][r], path_breakpoint[k], path_vector_u[k]);
                exp += coeff * lambda[k][p][r];
            }

            for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
                int v = data.getNodeId(n);
                exp -= std::log(data.getNode(v).getAvailability()) * y[k][v][p];
            }
            std::string name = "logAv(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, -IloInfinity, exp, 0, name.c_str()));
            exp.clear();
            exp.end();
        }
    }

    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            exp += avail[k][p];
            exp -= alpha[k][p];
            std::string name = "availImposition(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, -IloInfinity, exp, 0, name.c_str()));
            exp.clear();
            exp.end();
        }
    }

    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            for (unsigned int i = 0; i < interval[k][p].size(); i++){
                exp += interval[k][p][i];
            }
            std::string name = "interval_choice(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, 1, exp, 1, name.c_str()));
            exp.clear();
            exp.end();
        }
    }
    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            exp += lambda[k][p][0];
            exp -= interval[k][p][0];
            std::string name = "interval_1st(" + std::to_string(k) + "," + std::to_string(p) + "," + std::to_string(0) + ")";
            constraints.add(IloRange(env, -IloInfinity, exp, 0, name.c_str()));
            exp.clear();
            exp.end();
        }
    }

    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            for (unsigned int i = 1; i < lambda[k][p].size()-1; i++){
                IloExpr exp(env);
                exp += lambda[k][p][i];
                exp -= interval[k][p][i];
                exp -= interval[k][p][i-1];
            
                std::string name = "interval_imp(" + std::to_string(k) + "," + std::to_string(p) + "," + std::to_string(i) + ")";
                constraints.add(IloRange(env, -IloInfinity, exp, 0, name.c_str()));
                exp.clear();
                exp.end();
            }
        }
    }
    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            unsigned int last = lambda[k][p].size()-1;
            IloExpr expr(env);
            expr += lambda[k][p][last];
            expr -= interval[k][p][last-1];
        
            std::string name = "interval_last(" + std::to_string(k) + "," + std::to_string(p) + "," + std::to_string(last) + ")";
            constraints.add(IloRange(env, -IloInfinity, expr, 0, name.c_str()));
            expr.clear();
            expr.end();
        }
    }
}

void Model::setConfigAvailApproxConstraints(){
    /** avail[k][p] = 1 - unavail[k][p] ***/
    std::cout << "\t > Setting up approximated config availability constraints. " << std::endl;
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
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            for (int r = 0; r < config_breakpoint[k].size(); r++){
                exp += (config_breakpoint[k][r] * c_lambda[k][p][r]);
            }
            exp -= unavail[k][p];
            std::string name = "unavail2(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, 0, exp, 0, name.c_str()));
            exp.clear();
            exp.end();
        }
    }

    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            for(int r = 0; r < config_breakpoint[k].size(); r++){
                exp += c_lambda[k][p][r];
            }
            std::string name = "clambda(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, 1, exp, 1, name.c_str()));
            exp.clear();
            exp.end();
        }
    }

    

    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            for (unsigned int i = 0; i < c_interval[k][p].size(); i++){
                exp += c_interval[k][p][i];
            }
            std::string name = "cinterval_choice(" + std::to_string(k) + "," + std::to_string(p) + ")";
            constraints.add(IloRange(env, 1, exp, 1, name.c_str()));
            exp.clear();
            exp.end();
        }
    }
    



    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            IloExpr exp(env);
            exp += c_lambda[k][p][0];
            exp -= c_interval[k][p][0];
            std::string name = "c_interval_1st(" + std::to_string(k) + "," + std::to_string(p) + "," + std::to_string(0) + ")";
            constraints.add(IloRange(env, -IloInfinity, exp, 0, name.c_str()));
            exp.clear();
            exp.end();
        }
    }

    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            for (unsigned int i = 1; i < c_lambda[k][p].size()-1; i++){
                IloExpr exp(env);
                exp += c_lambda[k][p][i];
                exp -= c_interval[k][p][i];
                exp -= c_interval[k][p][i-1];
            
                std::string name = "c_interval_imp(" + std::to_string(k) + "," + std::to_string(p) + "," + std::to_string(i) + ")";
                constraints.add(IloRange(env, -IloInfinity, exp, 0, name.c_str()));
                exp.clear();
                exp.end();
            }
        }
    }
    for (int k = 0; k < data.getNbDemands(); k++){
        for (int p = 0; p < data.getNbPaths(k); p++){
            unsigned int last = c_lambda[k][p].size()-1;
            IloExpr expr(env);
            expr += c_lambda[k][p][last];
            expr -= c_interval[k][p][last-1];
        
            std::string name = "c_interval_last(" + std::to_string(k) + "," + std::to_string(p) + "," + std::to_string(last) + ")";
            constraints.add(IloRange(env, -IloInfinity, expr, 0, name.c_str()));
            expr.clear();
            expr.end();
        }
    }

    for (int k = 0; k < data.getNbDemands(); k++){
        IloExpr exp(env);
        for (int p = 0; p < data.getNbPaths(k); p++){
            for(int r = 0; r < config_breakpoint[k].size(); r++){
                // coeff = g(bp[r])
                double coeff = approx_log_from_above(config_breakpoint[k][r], config_breakpoint[k], config_vector_u[k]);
                exp += coeff * c_lambda[k][p][r];
            }
        }
        std::string name = "Avail(" + std::to_string(k) + ")";
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
        std::cout << "k=" << k+1 << " : From " << data.getDemand(k).getSource() << " to " << data.getDemand(k).getTarget() << std::endl;
        double placementAv = getPlacementAvailability(k);
        std::cout << "PLACEMENT AVAIL: " << placementAv << std::endl;
        std::cout << "REQUIRED AVAIL: " << data.getDemand(k).getAvailability() << std::endl;
        if (placementAv < data.getDemand(k).getAvailability()){
            std::cout << "====> Unfeasible by " << data.getDemand(k).getAvailability() - placementAv << std::endl;
        }
        for (int p = 0; p < data.getNbPaths(k); p++){
            if (cplex.getValue(alpha[k][p]) > 1 - EPS){
                std::cout << "  Path " << p+1 << std::endl;
                double path_availability = 1.0;
                double log_path_availability = 0.0;
                for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
                    int v = data.getNodeId(n);
                    if (cplex.getValue(y[k][v][p]) > 1 - EPS){
                        path_availability *= data.getNode(v).getAvailability();
                        log_path_availability += std::log(data.getNode(v).getAvailability());
                    }
                }
                std::cout << "  Path avail : " << std::setprecision(9) << path_availability << ", \t";
                std::cout << "  Var avail : " << std::setprecision(9) << cplex.getValue(avail[k][p]) << std::endl;
                if (path_availability - cplex.getValue(avail[k][p]) < 0.0){
                    std::cout << " >> ERROR : " << std::setprecision(9) << path_availability - cplex.getValue(avail[k][p]) << ". Negative approx !!!" << std::endl; 
                }
                else{
                    std::cout << " Diff : " << std::setprecision(9) << path_availability - cplex.getValue(avail[k][p]) << std::endl; 
                }
                double computed_avail = 0;
                double computed_var_log = 0;
                for (int r = 0; r < PATH_NB_BREAKS; r++){
                    if (cplex.getValue(lambda[k][p][r]) >  EPSILON){
                        computed_avail += path_breakpoint[k][r]* cplex.getValue(lambda[k][p][r]);
                        computed_var_log += std::log(path_breakpoint[k][r])* cplex.getValue(lambda[k][p][r]);
                        std::cout << "  > u" << r << ": " << path_breakpoint[k][r] << ", lambda = " << cplex.getValue(lambda[k][p][r]) << std::endl;
                    }
                }
                std::cout << "  log avail : " << std::setprecision(9) << std::log(path_availability) << std::endl;
                std::cout << "  g(avail) : " << std::setprecision(9) << computed_var_log << std::endl;
                // std::cout << "  Var unavail : " << cplex.getValue(unavail[k][p]) << std::endl;
                // std::cout << "  Real unavail : " << 1.0 - path_availability << std::endl;
                // for (int r = 0; r < CONFIG_NB_BREAKS; r++){
                //     if (cplex.getValue(c_lambda[k][p][r]) >  EPS){
                //         std::cout << "  > b_" << r << ": " << config_breakpoint[r] << ", c_lambda = " << cplex.getValue(c_lambda[k][p][r]) << std::endl;
                //     }
                // }
                for (int i = 0; i < data.getDemand(k).getNbVNFs() + 1; i++){
                    std::cout << "    Section " << i+1 << " : ";
                    printSectionPath(k, p, i);
                }
            }
            else{
                std::cout << "  Path " << p+1 << " not used." << std::endl;
                std::cout << "  Var avail : " << cplex.getValue(avail[k][p]) << std::endl;
                for (int r = 0; r < lambda[k][p].size(); r++){
                    if (cplex.getValue(lambda[k][p][r]) >  EPS){
                        std::cout << "  > u" << r << ": " << path_breakpoint[k][r] << ", lambda = " << cplex.getValue(lambda[k][p][r]) << std::endl;
                    }
                }
                // std::cout << "  Var unavail : " << cplex.getValue(unavail[k][p]) << std::endl;
                // for (int r = 0; r < CONFIG_NB_BREAKS; r++){
                //     if (cplex.getValue(c_lambda[k][p][r]) >  EPS){
                //         std::cout << "  > b_" << r << ": " << config_breakpoint[r] << ", c_lambda = " << cplex.getValue(c_lambda[k][p][r]) << std::endl;
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

}

double Model::getPlacementAvailability(int k){
    
    /* Compute placement availability */
    double prob_all_paths_fail = 1.0;
    for (unsigned int p = 0; p < alpha[k].size(); p++){
        double path_failure = 1.0;
        if (cplex.getValue(alpha[k][p]) > 1 - EPS){
            double path_availability = 1.0;
            for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
                int v = data.getNodeId(n);
                if (cplex.getValue(y[k][v][p]) > 1 - EPS){
                    path_availability *= data.getNode(v).getAvailability();
                }
            }
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
/*										Destructors 									*/
/****************************************************************************************/
Model::~Model(){
    delete callback;
}