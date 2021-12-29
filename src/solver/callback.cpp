#include "callback.hpp"

/****************************************************************************************/
/*										CONSTRUCTOR										*/
/****************************************************************************************/

Callback::Callback(const IloEnv& env_, const Data& data_, const IloNumVar3DMatrix& y_, const IloNumVar4DMatrix& x_, const IloNumVarMatrix& alpha_) :
	                env(env_), data(data_),	y(y_), x(x_), alpha(alpha_), cutPool(env)
{	
	/*** Control ***/
    thread_flag.lock();
	nb_cuts_avail_heuristic = 0;
    nbLazyConstraints = 0;
    nbUserCuts = 0;
    nbCuts = 0;
	timeAll = 0.0;
    setCutPool();
    thread_flag.unlock();
}

/****************************************************************************************/
/*										 INVOKE        									*/
/****************************************************************************************/
void Callback::invoke(const Context& context)
{
    IloNum time_start = context.getDoubleInfo(IloCplex::Callback::Context::Info::Time);
    switch (context.getId()){
        /* Fractional solution */
        case Context::Id::Relaxation:
            addUserCuts(context);
            break;

        /* Integer solution */
        case Context::Id::Candidate:
			if (context.isCandidatePoint()) {
	    		addLazyConstraints(context);
			}
            break;

        /* Not an option for callback */
		default:
			throw IloCplex::Exception(-1, "ERROR: Unexpected context ID !");
    }
    IloNum time_spent = context.getDoubleInfo(IloCplex::Callback::Context::Info::Time) - time_start;
    incrementTime(time_spent);
}


/****************************************************************************************/
/*										 CUT POOL        								*/
/****************************************************************************************/
/** Sets up the cut pool that is checked on relaxation context. On this pool only cuts appearing in a polynomial number are added. **/
void Callback::setCutPool()
{
    std::cout << "---- Setting cut pool ----" << std::endl;
    cutPool.clear();
}

/* Checks whether the current solution satisfies all cuts in the pool and add the unsatisfied one. */
bool Callback::checkCutPool(const Context &context)
{
    bool found_violated_cut = false;
    for (IloInt i = 0; i < cutPool.getSize(); ++i) {
        const IloRange& cut = cutPool[i];
        IloNum const lhs = context.getRelaxationValue(cut.getExpr());
        if (lhs < cut.getLB() - EPS || lhs > cut.getUB() + EPS ) {
            //std::cout << "Adding " << cut.getName() << std::endl;
            context.addUserCut(cut, IloCplex::UseCutForce, IloFalse);
            incrementUsercuts();
            found_violated_cut = true;
            /* Uncomment next line to add only one violated cut at a time. */
            // return true;
        }
    }
    return found_violated_cut;
}

/****************************************************************************************/
/*										 USER CUTS        								*/
/****************************************************************************************/
/* Separation routine applied on fractional solutions */
void Callback::addUserCuts(const Context &context)
{
    /* Mandatory clauses */
    if (context.getId() != Context::Id::Relaxation){
        throw IloCplex::Exception(-1, "ERROR: Trying to access fractional solution while not in relaxation context !");
    }

    /* Add section failure cuts */
    if (data.getInput().getSectionFailureCuts() == Input::SECTION_FAILURE_CUTS_ON){
        sectionFailureSeparation(context);
    }
    
}

/* Solves the separation problem associated with Section Failure inequalities. */
void Callback::sectionFailureSeparation(const Context &context)
{
    try {
        for (int k = 0; k < data.getNbDemands(); k++){
            const double REQUIRED_AVAIL      = data.getDemand(k).getAvailability(); 
            const int    NB_PATHS            = data.getNbPaths(k);
            const double RHS                 = -std::log(1.0 - REQUIRED_AVAIL);

            std::vector<int> selected_sections;
            selected_sections.resize(NB_PATHS);

            double lhs = 0.0;

            for (int p = 0; p < NB_PATHS; p++){
                lhs += getMostUnavailableSection(context, k, p, selected_sections[p]);
            }

            /* Build inequality */
            if (lhs < RHS - BIG_EPSILON){
                IloExpr exp(env);
                for (int p = 0; p < NB_PATHS; p++){
                    for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
                        int v = data.getNodeId(n);
                        int section = selected_sections[p];
                        double coeff = -std::log(1.0 - data.getNode(v).getAvailability());
                        exp += coeff * x[k][v][section][p];
                    }
                }
                
                /* Add the cut: exp >= rhs */
                IloRange cut(env, RHS, exp, IloInfinity);
                context.addUserCut(cut, IloCplex::UseCutForce, IloFalse);
                //std::cout << "Adding user cut: " << cut << std::endl;
                exp.end();
                incrementUsercuts();
            }
        }
    }
    catch (...) {
        throw;
    }
}
/****************************************************************************************/
/*									LAZY CONSTRAINTS        							*/
/****************************************************************************************/
/* Separation routine applied on integer solutions */
void Callback::addLazyConstraints(const Context &context)
{
    try {
        /* Get current integer solution */
        IloNum3DMatrix ySol = getIntegerSolution(context); 
        /* Check VNF placement availability for each demand */
        for (int k = 0; k < data.getNbDemands(); k++){
            const double REQUIRED_AVAIL      = data.getDemand(k).getAvailability(); 
            const int    NB_PATHS            = data.getNbPaths(k);
            const int    LAST_ACTIVATED_PATH = getLastPathIndex(context, k);
            const double PLACEMENT_AVAIL     = getPlacementAvailability(context, k);
            
            /* If placement availability is not enough, add lazy constraint. */
            if (PLACEMENT_AVAIL < REQUIRED_AVAIL){
                /* Build inequality. */
                IloExpr exp(env);
                int rhs = 1;
                for (int p = 0; p < NB_PATHS; p++){
                    if (context.getCandidatePoint(alpha[k][p]) > 1 - EPS){
                        for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
                            int v = data.getNodeId(n);
                            if (ySol[k][v][p] > 1 - EPS){
                                exp -= y[k][v][p];
                                rhs--;
                            }
                        }
                    }
                }
                if (LAST_ACTIVATED_PATH < NB_PATHS - 1){
                    exp += alpha[k][LAST_ACTIVATED_PATH+1];
                }
                
                /* Add the cut: exp >= rhs */
                IloRange cut(env, rhs, exp, IloInfinity);
                context.rejectCandidate(cut);
                //std::cout << "Adding lazy constraint: " << cut << std::endl;
                exp.end();
                incrementLazyConstraints();
            }
        }
    }
    catch (...) {
        throw;
    }
}

/****************************************************************************************/
/*									QUERY METHODS        								*/
/****************************************************************************************/
/* Returns the last activated path index within a candidate solution. */
const int Callback::getLastPathIndex(const Context &context, const int k) const
{
    /* Mandatory clauses */
    if (context.getId() != Context::Id::Candidate){
        throw IloCplex::Exception(-1, "ERROR: Trying to access integer solution while not in candidate context !");
    }
    if (!context.isCandidatePoint()) {
        throw IloCplex::Exception(-1, "ERROR: Unbounded solution within callback !");
    }

    /* Compute last activated path index */
    int last = 0;
    for (unsigned int p = 0; p < alpha[k].size(); p++){
        if (context.getCandidatePoint(alpha[k][p]) > 1 - EPS){
            last = p;
        }
    }
    return last;
}

/* Returns the SFC availability induced by a candidate solution. */
const double Callback::getPlacementAvailability(const Context &context, const int k) const
{
    /* Mandatory clauses */
    if (context.getId() != Context::Id::Candidate){
        throw IloCplex::Exception(-1, "ERROR: Trying to access integer solution while not in candidate context !");
    }
    if (!context.isCandidatePoint()) {
        throw IloCplex::Exception(-1, "ERROR: Unbounded solution within callback !");
    }

    /* Compute placement availability */
    double prob_all_paths_fail = 1.0;
    for (unsigned int p = 0; p < alpha[k].size(); p++){
        double path_failure = 1.0;
        if (context.getCandidatePoint(alpha[k][p]) > 1 - EPS){
            double path_availability = 1.0;
            for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
                int v = data.getNodeId(n);
                if (context.getCandidatePoint(y[k][v][p]) > 1 - EPS){
                    path_availability *= data.getNode(v).getAvailability();
                }
            }
            path_failure = 1.0 - path_availability;
        }
        prob_all_paths_fail *= path_failure;
    }
    return (1.0 - prob_all_paths_fail);
}


/* Returns the most unavailable section of a path in the form -log(unavailability) given a fractional solution. */
const double Callback::getMostUnavailableSection(const Context &context, const int k, const int p, int &section) const
{
    /* Mandatory clauses */
    if (context.getId() != Context::Id::Relaxation){
        throw IloCplex::Exception(-1, "ERROR: Trying to access fractional solution while not in relaxation context !");
    }

    const int NB_VNFS = data.getDemand(k).getNbVNFs();
    double result = IloInfinity;
    for (int i = 0; i < NB_VNFS; i++){
        double lhs = 0.0;
        for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
            int v = data.getNodeId(n);
            double coeff = -std::log(1.0 - data.getNode(v).getAvailability());
            lhs += (coeff * context.getRelaxationValue(x[k][v][i][p]));
        }
        if (lhs < result){
            result = lhs;
            section = i;
        }
    }
    //std::cout << "The most unavailable section of path " << p << " is " << section << std::endl;
    return result;
}




IloNum3DMatrix Callback::getIntegerSolution(const Context &context) const
{
    /* Mandatory clauses */
    if (context.getId() != Context::Id::Candidate){
        throw IloCplex::Exception(-1, "ERROR: Trying to access integer solution while not in candidate context !");
    }
    if (!context.isCandidatePoint()) {
        throw IloCplex::Exception(-1, "ERROR: Unbounded solution within callback !");
    }

    /* Initialize solution */
    IloNum3DMatrix ySol;
    const int NB_NODES = lemon::countNodes(data.getGraph());
    const int NB_DEMANDS = data.getNbDemands();
    ySol.resize(NB_DEMANDS);
    for (int k = 0; k < data.getNbDemands(); k++){
        ySol[k].resize(NB_NODES);
        for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
            int v = data.getNodeId(n);
            ySol[k][v].resize(data.getNbPaths(k));
        }
    }

    /* Fill solution matrix */
    for (int k = 0; k < NB_DEMANDS; k++){
        const int NB_PATHS = data.getNbPaths(k);
        for (NodeIt n(data.getGraph()); n != lemon::INVALID; ++n){
            int v = data.getNodeId(n);
            for (int p = 0; p < NB_PATHS; p++){
                ySol[k][v][p] = context.getCandidatePoint(y[k][v][p]);
            }
        }
    }
    
    /* Return solution matrix */
    return ySol;
}

/****************************************************************************************/
/*								THREAD PROTECTED METHODS			    				*/
/****************************************************************************************/
void Callback::incrementLazyConstraints()
{
    thread_flag.lock();
    ++nbLazyConstraints;
    ++nbCuts;
    thread_flag.unlock();
}

void Callback::incrementUsercuts()
{
    thread_flag.lock();
    ++nbUserCuts;
    ++nbCuts;
    thread_flag.unlock();
}

void Callback::incrementTime(const IloNum time)
{
    thread_flag.lock();
    timeAll += time;
    thread_flag.unlock();
}
