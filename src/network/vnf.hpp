#ifndef __vnf__hpp
#define __vnf__hpp

/****************************************************************************************/
/*										LIBRARIES										*/
/****************************************************************************************/

/*** C++ Libraries ***/
#include <iostream>
#include <string>
#include <vector>


/*************************************************
 * This class models a VNF in the network. 
 * Each VNF has an id, a name and.
*************************************************/
class VNF{
    private:
        const int 					id;					/**< VNF id. **/
        const std::string 			name;				/**< VNF name. **/
		const double				consumption;		/**< VNF resource consumption. **/

    public:
	/****************************************************************************************/
	/*										Constructor										*/
	/****************************************************************************************/

	/** Constructor. @param id_ VNF id. @param name_ VNF name. @param cons VNF resource consumption.**/
	VNF(const int id_ = -1, const std::string name_ = "", const double cons = 0.0);
    
	/****************************************************************************************/
	/*										Getters											*/
	/****************************************************************************************/
    
	/** Returns the vnf's id. **/
	const int& 					getId()				const { return this->id; }
	/** Returns the vnf's name. **/
	const std::string& 			getName() 			const { return this->name; }
	/** Returns the vnf's resource consumption. **/
	const double& 				getConsumption() 	const { return this->consumption; }

	/****************************************************************************************/
	/*										Display											*/
	/****************************************************************************************/
	/** Displays information about the vnf. **/
	void print();
};

#endif