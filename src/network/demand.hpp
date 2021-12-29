#ifndef __demand__hpp
#define __demand__hpp

/****************************************************************************************/
/*										LIBRARIES										*/
/****************************************************************************************/

/*** C++ Libraries ***/
#include <iostream>
#include <string>
#include <vector>


/****************************************************************************************
 * This class models a SFC demand in the network. Each SFC has an id, a name, a source, 
 * a target, a maximum latency and a list of required VNFs
****************************************************************************************/
class Demand{
    private:
        const int 					id;					/**< Demand id. **/
        const std::string 			name;				/**< Demand name. **/
		const int				    source;		        /**< Demand source node id. **/
		const int				    target;		        /**< Demand target node id. **/
		const double        		max_latency;     	/**< Demand maximum latency. **/
		const double 				bandwidth;			/**< Demand requested bandwidth. **/
		const double 				availability;		/**< Demand requested availability. **/
        std::vector<int>            VNF_list;           /**< Demand VNF list. **/
		std::vector<int>	        link_list;	        /**< List of the ids of links routing the demand. **/


    public:
	/****************************************************************************************/
	/*										Constructor										*/
	/****************************************************************************************/

	/** Constructor. @param id_ Demand id. @param name_ Demand name. @param s Demand source node id. @param t Demand target node id. @param l Demand maximum latency. @param b Demand requested bandwidth. @param a Demand requested availability.**/
	Demand(const int id_ = -1, const std::string name_ = "", const int s = -1, const int t = -1, const double l = 0.0, const double b = 0.0, const double a = 0.0);
    

	/****************************************************************************************/
	/*										Getters											*/
	/****************************************************************************************/
    
	/** Returns the demand's id. **/
	const int& 					getId()				const { return this->id; }
	/** Returns the demand's name. **/
	const std::string& 			getName() 			const { return this->name; }
	/** Returns the demand's source node id. **/
	const int& 				    getSource() 	    const { return this->source; }
	/** Returns the demand's target node id. **/
	const int& 				    getTarget() 	    const { return this->target; }
	/** Returns the demand's maximum latency. **/
	const double& 				getMaxLatency() 	const { return this->max_latency; }
	/** Returns the demand's requested bandwidth. **/
	const double& 				getBandwidth() 		const { return this->bandwidth; }
	/** Returns the demand's requested availability. **/
	const double& 				getAvailability() 		const { return this->availability; }
	/** Returns the demand's list of VNFs. **/
	const std::vector<int>& 	getListOfVNFs()     const { return this->VNF_list; }
	/** Returns the number of VNFs requested by the demand. **/
	const int 		            getNbVNFs() 	    const { return (int)this->VNF_list.size(); }
	/** Returns the id of the i-th VNFs of the demand. **/
	const int 		            getVNF_i(int i)     const { return this->VNF_list[i]; }
	/** Returns the list of arcs routing the demand. **/
	const std::vector<int>& 	getListOfLinks()    const { return this->link_list; }
	/** Returns the number of number of hops in demand's path. **/
	const unsigned int 		    getNbHops() 	    const { return this->link_list.size(); }

	/****************************************************************************************/
	/*										Setters											*/
	/****************************************************************************************/

	/** Adds a requested VFN. @param id The VNF's id. **/
	void addVNF(const int id) { this->VNF_list.push_back(id); }

	/****************************************************************************************/
	/*										Display											*/
	/****************************************************************************************/
	/** Displays information about the demand. **/
	void print();
};

#endif