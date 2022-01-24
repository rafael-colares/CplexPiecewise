#ifndef __input__hpp
#define __input__hpp

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>


/*****************************************************************************************
 * This class stores all the information recovered from the parameter file, that is,
 * input/output file paths, execution and control parameters.						
*****************************************************************************************/
class Input{

public: 
	/** States wheter strong node capacity constraints are activated.**/
	enum Strong_Node_Capacity_Constraints {
		STRONG_NODE_CAPACITY_OFF = 0,  		
		STRONG_NODE_CAPACITY_ON = 1 	        
	};
	/** States wheter upper bounds on the number of paths are activated.**/
	enum Nb_Paths_Upper_Bound {
		NB_PATHS_UPPER_BOUND_OFF = 0,  		
		NB_PATHS_UPPER_BOUND_ON = 1 	        
	};
	/** States wheter lower bounds on the number of paths are activated.**/
	enum Nb_Paths_Lower_Bound {
		NB_PATHS_LOWER_BOUND_OFF = 0,  		
		NB_PATHS_LOWER_BOUND_ON = 1 	        
	};
	/** States wheter section failure cuts are activated.**/
	enum Section_Failure_Cuts {
		SECTION_FAILURE_CUTS_OFF = 0,  		
		SECTION_FAILURE_CUTS_ON = 1 	        
	};
	/** States wheter section failure cuts are activated.**/
	enum Approximation_Type {
		APPROXIMATION_TYPE_RESTRICTION = 0,  		
		APPROXIMATION_TYPE_RELAXATION = 1 	        
	};

private:
    /***** Input file paths *****/
    const std::string   parameters_file;
    std::string         node_file;
    std::string         link_file;
    std::string         demand_file;
    std::string         vnf_file;

    /***** Formulation parameters*****/
    Strong_Node_Capacity_Constraints        strong_node_capacity;           /**< Refers to the activation of strong node capacity constraints. **/
    Section_Failure_Cuts					section_failure_cuts; 			/**< Refers to the activation of section failure cuts. **/
	Nb_Paths_Upper_Bound					nb_paths_ub; 					/**< Refers to the activation of upper bounds on the number of paths. **/
	Nb_Paths_Lower_Bound					nb_paths_lb; 					/**< Refers to the activation of lower bounds on the number of paths. **/
    Approximation_Type                      approx_type;                    /**< Refers to the type of approximation used for modeling availability constraints. **/

	/***** Optimization parameters*****/
    bool                linear_relaxation;
    bool                basic;
    int                 time_limit;
    int                 nb_breakpoints;


    /***** Output file paths *****/
    std::string         output_file;
    
public:
	/********************************************/
	/*				Constructors				*/
	/********************************************/
	/** Constructor initializes the object with the information contained in the parameter file. @param file The address of the parameter file (usually the address of file 'parameters.txt'). **/
    Input(const std::string file);
    /** Constructor always need a parameter file. **/
    Input() = delete;



	/********************************************/
	/*				    Getters	    			*/
	/********************************************/
    /** Returns the parameters file. */
    const std::string& getParameterFile()  const { return this->parameters_file; }
    /** Returns the node file. */
    const std::string& getNodeFile()       const { return this->node_file; }
    /** Returns the link file. */
    const std::string& getLinkFile()       const { return this->link_file; }
    /** Returns the demand file. */
    const std::string& getDemandFile()     const { return this->demand_file; }
    /** Returns the VNF file. */
    const std::string& getVnfFile()        const { return this->vnf_file; }


	/** Returns whether strong node capacity constraints are activated. **/
    const Strong_Node_Capacity_Constraints &        getStrongNodeCapacity()         const { return strong_node_capacity; }
	/** Returns whether section failure cuts are activated. **/ //TODO: To be implemented
    const Section_Failure_Cuts &                    getSectionFailureCuts()         const { return section_failure_cuts; }
	/** Returns whether number of paths upper bounds are activated. **/ 
    const Nb_Paths_Upper_Bound &                    getNbPathsUpperBound()         const { return nb_paths_ub; }
	/** Returns whether number of paths lower bounds are activated. **/ 
    const Nb_Paths_Lower_Bound &                    getNbPathsLowerBound()         const { return nb_paths_lb; }
	/** Returns the type of availability approximation to be used. **/ 
    const Approximation_Type &                      getApproximationType()         const { return approx_type; }
	

    /** Returns true if linear relaxation is to be applied. */
    const bool&        isRelaxation()      const { return this->linear_relaxation; }
    /** Returns true if the basic formulation is being applied. */
    const bool&        isBasic()      const { return this->basic; }
    /** Returns time limit in seconds to be applied. */
    const int&         getTimeLimit()      const { return this->time_limit; }
    /** Returns the number of breakpoints to be used in the log approximation. */
    const int&         getNbBreakpoints() const { return this->nb_breakpoints; }
    /** Returns the output file. */
    const std::string& getOutputFile()     const { return this->output_file; }

	/********************************************/
	/*				    Methods	    			*/
	/********************************************/
    /** Returns the pattern value in the parameters file. */
    std::string getParameterValue(const std::string pattern);

	/********************************************/
	/*				    Output	    			*/
	/********************************************/
    /** Print the parameters stored in the parameter file. */
    void print();

};

#endif