#ifndef __callback__hpp
#define __callback__hpp

/****************************************************************************************/
/*										LIBRARIES										*/
/****************************************************************************************/

/*** C++ Libraries ***/
#include <thread>
#include <mutex>

/*** CPLEX Libraries ***/
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

/*** Own Libraries ***/
#include "../instance/data.hpp"
#include "../tools/others.hpp"

/****************************************************************************************/
/*										TYPEDEFS										*/
/****************************************************************************************/

/*** CPLEX ***/	
typedef std::vector<IloNumVar>          IloNumVarVector;
typedef std::vector<IloNumVarVector>    IloNumVarMatrix;
typedef std::vector<IloNumVarMatrix>    IloNumVar3DMatrix;
typedef std::vector<IloNumVar3DMatrix>  IloNumVar4DMatrix;
typedef std::vector<IloNumVar4DMatrix>  IloNumVar5DMatrix;

typedef std::vector<IloNum>             IloNumVector;
typedef std::vector<IloNumVector>       IloNumMatrix;
typedef std::vector<IloNumMatrix>       IloNum3DMatrix;
typedef std::vector<IloNum3DMatrix>     IloNum4DMatrix;
typedef std::vector<IloNum4DMatrix>     IloNum5DMatrix;

typedef IloCplex::Callback::Context     Context;

/****************************************************************************************/
/*										DEFINES			    							*/
/****************************************************************************************/
#define EPS 1e-4 // Tolerance, about float precision
#define EPSILON 1e-6 // Tolerance, about float precision
#define BIG_EPSILON 1e-10 // Tolerance, about float precision




/************************************************************************************
 * This class implements the generic callback interface. It has two main 
 * functions: addUserCuts and addLazyConstraints.
 ************************************************************************************/
class Callback: public IloCplex::Callback::Function {

    
private:
    /*** General variables ***/
    const IloEnv&   env;    /**< IBM environment **/
    const Data&     data;   /**< Data read in data.hpp **/


    /*** LP data ***/
	const IloNumVar3DMatrix&    y;          /**< VNF placement variables **/
	const IloNumVar4DMatrix&    x;          /**< VNF section placement variables **/
	const IloNumVarMatrix&      alpha;      /**< Path activation variables **/
    IloRangeArray               cutPool;    /**< Cutpool to be checked on each node. **/


    /*** Manage execution and control ***/
    std::mutex  thread_flag;                /**< A mutex for synchronizing multi-thread operations. **/
    int         nb_cuts_avail_heuristic;    /**< Number of availability cuts added through heuristic procedure. **/
    int         nbLazyConstraints;          /**< Number of lazy constraints added. **/
    int         nbUserCuts;                 /**< Number of user cuts added. **/
    int         nbCuts;                     /**< Total number of user cuts added. **/
    IloNum      timeAll;                    /**< Total time spent on callback. **/


public:

	/****************************************************************************************/
	/*										Constructors									*/
	/****************************************************************************************/
    /** Constructor. Initializes callback variables. **/
	Callback(const IloEnv& env_, const Data& data_, const IloNumVar3DMatrix& y_, const IloNumVar4DMatrix& x_, const IloNumVarMatrix& alpha_);


	/****************************************************************************************/
	/*									Main operations  									*/
	/****************************************************************************************/
    /** CPLEX will call this method during the solution process at the places that we asked for. @param context Defines on which places the method is called and we use it do define what to do in each case.**/
    void            invoke                  (const Context& context);

    /** Solves the separation problems for a given fractional solution. @note Should only be called within relaxation context.**/
	void            addUserCuts             (const Context& context); 
    
    /** Solves the separation problems for a given integer solution. @note Should only be called within candidate context.**/
    void            addLazyConstraints      (const Context& context);

    /** Returns the current integer y-solution. @note Should only be called within candidate context. **/ 
    IloNum3DMatrix  getIntegerSolution      (const Context &context) const;
    
    /** Returns the current fractional solution. @note Should only be called within relaxation context. **/ 
    IloNum3DMatrix  getFractionalSolution   (const Context &context) const;

    /** Checks whether the current solution satisfies all cuts in the pool and add the unsatisfied one. **/
    bool            checkCutPool            (const Context &context);

	/****************************************************************************************/
	/*							Cut Pool Definition Methods  							    */
	/****************************************************************************************/
    /** Sets up the cut pool that is checked on relaxation context. @note On this pool, only cuts appearing in a polynomial number are added. **/
    void setCutPool();

	/****************************************************************************************/
	/*								      Query Methods	    	    	    				*/
	/****************************************************************************************/
    /** Returns the number of user cuts added so far. **/ 
    const int    getNbUserCuts()           const{ return nbCuts; }
    /** Returns the number of lazy constraints added so far. **/ 
    const int    getNbLazyConstraints()    const{ return nbLazyConstraints; }
    /** Returns the total time spent on callback so far. **/ 
    const IloNum getTime()                 const{ return timeAll; }
    /** Checks if all placement variables of a given SFC demand are inetegers. @param k The demand id. @param xSol The current solution. **/
    const bool   isIntegerAssignment(const int& k, const IloNum3DMatrix& xSol) const;
    /** Returns the last activated path index for a given SFC within a candidate solution. @note Should only be called within candidate context. **/
    const int    getLastPathIndex(const Context &context, const int k) const;
    /** Returns the SFC availability induced by a candidate solution. @note Should only be called within candidate context. **/
    const double getPlacementAvailability(const Context &context, const int k) const;
    /* Returns the most unavailable section of a path in the for log(unavailability) given a fractional solution. @note Should only be called within relaxation context.*/
    const double getMostUnavailableSection(const Context &context, const int k, const int p, int &section) const;


	/****************************************************************************************/
	/*								    Separation Methods	    	    	    			*/
	/****************************************************************************************/
    /** Solves the separation problem associated with Section Failure inequalities. **/
    void sectionFailureSeparation(const Context &context);

	/****************************************************************************************/
	/*								Thread Protected Methods			    				*/
	/****************************************************************************************/
    /** Increase by one the number of lazy constraints added. **/
    void incrementLazyConstraints();
    /** Increase by one the number of user cuts added. **/
    void incrementUsercuts();
    /** Increases the total callback time. @param time The time to be added. **/
    void incrementTime(const IloNum time);

	/****************************************************************************************/
	/*										Destructors			    						*/
	/****************************************************************************************/
    /** Destructor **/
    ~Callback() {}

};

#endif