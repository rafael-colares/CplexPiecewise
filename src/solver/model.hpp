#ifndef __model__hpp
#define __model__hpp


/****************************************************************************************/
/*										LIBRARIES										*/
/****************************************************************************************/

/*** Own Libraries ***/
#include "callback.hpp"

/****************************************************************************************/
/*										TYPEDEFS										*/
/****************************************************************************************/

/*** CPLEX ***/	
typedef std::vector<IloNumVar>          IloNumVarVector;
typedef std::vector<IloNumVarVector>    IloNumVarMatrix;
typedef std::vector<IloNumVarMatrix>    IloNumVar3DMatrix;
typedef std::vector<IloNumVar3DMatrix>  IloNumVar4DMatrix;
typedef std::vector<IloNumVar4DMatrix>  IloNumVar5DMatrix;

typedef std::vector<IloNum>            IloNumVector;
typedef std::vector<IloNumVector>      IloNumMatrix;
typedef std::vector<IloNumMatrix>      IloNum3DMatrix;
typedef std::vector<IloNum3DMatrix>    IloNum4DMatrix;
typedef std::vector<IloNum4DMatrix>    IloNum5DMatrix;


/********************************************************************************************
 * This class models the MIP formulation and solves it using CPLEX. 											
********************************************************************************************/

class Model
{
	private:
		/*** Cplex features ***/
		const IloEnv&   env;    /**< IBM environment **/
	 	IloModel        model;  /**< IBM Model **/
		IloCplex        cplex;  /**< IBM Cplex **/

		/*** Formulation specific ***/
		const Data&     data;   		/**< Data read in data.hpp **/
		IloObjective    obj;            /**< Objective function **/
		IloRangeArray   constraints;    /**< Set of constraints **/
		Callback* 		callback; 		/**< User generic callback **/

		/*** Formulation variables ***/
		IloNumVar4DMatrix 	f;              /**< Flow variables **/
		IloNumVarMatrix 	z;              /**< VNF global placement variables **/
		IloNumVar3DMatrix 	y;              /**< VNF path placement variables **/
		IloNumVar4DMatrix 	x;            	/**< VNF section placement variables **/
		IloNumVarMatrix 	alpha;          /**< Path activation variables **/

		/*** Approx additional variables ***/
		IloNumVarMatrix 	avail;             		/**< availability of path **/
		IloNumVarMatrix 	approx_log_avail;   	/**< approximated log of availability of path**/
		IloNumVarMatrix 	unavail;            	/**< unavailability of path **/
		IloNumVarMatrix 	approx_log_unavail;   	/**< approximated log of unavailability of path**/
		
		/*** Approx additional features ***/
		IloNumMatrix 		avail_touch; 			/**< the vector of points where the availability approximation function touches the approximated function */
		IloNumMatrix 		avail_breakpoints; 		/**< the vector of points where the availability approximation function changes its slope */
		IloNumMatrix 		unavail_touch; 			/**< same as avail_breakpoints but related with the unavailability approx **/
		IloNumMatrix 		unavail_breakpoints; 	/**< same as avail_touch but related with the unavailability approx **/

		/*** Manage execution and control ***/
		IloNum time;

	public:
	/****************************************************************************************/
	/*										Constructors									*/
	/****************************************************************************************/
		/** Constructor. Builds the model (variables, objective function, constraints and further parameters). **/
		Model(const IloEnv& env, const Data& data);
		Model(const IloEnv& env, const Data&&) = delete;
		Model() = delete;

	/****************************************************************************************/
	/*									    Formulation  									*/
	/****************************************************************************************/
        /** Set up the Cplex parameters. **/
        void setCplexParameters();
        /** Set up the variables. **/
        void setVariables();
        /** Set up the objective function. **/
        void setObjective();
        /** Set up the constraints. **/
        void setConstraints();
		
		/** Add up the routing constraints: flow conservation constraints among sections **/
		void setRoutingConstraints();
        /** Add up the node capacity constraints: the bandwidth treated in a node must respect its capacity. **/
        void setNodeCapacityConstraints();
        /** Add up the arc capacity constraints: the bandwidth routed within an arc must respect its capacity. **/
        void setArcCapacityConstraints();
        /** Add up the latency constraints: the path lengths must respect latency. **/
        void setLatencyConstraints();
        /** Add up the linking constraints. **/
        void setLinkingConstraints();
        /** Add up the vnf disjunction constraints. **/
        void setDisjunctionConstraints();

		/* Add up the strong node capacity constraints. */
		void setStrongNodeCapacityConstraints();
		/* Add up the degree constraints. */
		void setDegreeConstraints();
		/* Add up the symmetry breaking constraints. */
		void setSymmetryBreakingConstraints();


		/* Add up the approximated path availability constraints. */
		void setPathAvailApproxConstraints();
		/* Add up the approximated configuration availability constraints. */
		void setConfigAvailApproxConstraints();

		/* Set up the breakpoints for approximating log(avail). */
		void buildAvailVector_u();
    	void buildAvailBreakpoints();
    	void buildUnavailVector_u();
    	void buildUnavailBreakpoints();
		void buildPiecewiseLinearApproximation();
		void buildApproximationFunctionAvail(int k, IloNumArray &breakpoints, IloNumArray &slopes);
		void buildApproximationFunctionUnavail(int k, IloNumArray &breakpoints, IloNumArray &slopes);

		/* Returns the value of approx function on y */
		double approx_log_from_above(double y, const vector<IloNum> &breaks, const vector<IloNum> &u);
		double approx_log_from_below(double x, const IloNumVector &breakpoints);
	/****************************************************************************************/
	/*										   Getters  									*/
	/****************************************************************************************/
		double getPlacementAvailability(int k);
		double getPathAvailability(int k, int p);
	/****************************************************************************************/
	/*										   Methods  									*/
	/****************************************************************************************/
		/** Solves the MIP. **/
		void run();

		/** Displays the obtained results **/
		void printResult();

		/** Displays the routing of demand k through the i-th section of path p **/
		void printSectionPath(const int k, const int p, const int i);

		/** Outputs the obtained results **/
		void output();
	/****************************************************************************************/
	/*										   Tests  										*/
	/****************************************************************************************/
		/** Checks if output value of variables are consistent. **/
		void testRelaxationAvail();
		void testRelaxationUnavail();

	/****************************************************************************************/
	/*										Destructors 									*/
	/****************************************************************************************/
		/** Destructor. Free dynamic allocated memory. **/
		~Model();
};


#endif // MODELS_HPP