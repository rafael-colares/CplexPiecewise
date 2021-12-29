#ifndef __link__hpp
#define __link__hpp

/****************************************************************************************/
/*										LIBRARIES										*/
/****************************************************************************************/

/*** C++ Libraries ***/
#include <iostream>
#include <string>


/********************************************************
 * This class models a link in the network. 
 * Each link has an id, a name, a source and a target.
********************************************************/
class Link{
    private:
        const int 	 	  id;			/**< Link id. **/				
        const std::string name;			/**< Link name. **/
        const int 		  source_id;	/**< Link source. **/
        const int 		  target_id;	/**< Link target. **/
		const double 	  delay;		/**< Link delay. **/
		const double 	  bandwidth;	/**< Link total bandwidth. **/

    public:
	/****************************************************************************************/
	/*										Constructor										*/
	/****************************************************************************************/
	/** Constructor. @param id_ Link id. @param name_ Link name. @param s Link's source id. @param t Link's target id. @param d Link's delay. @param b Link's total bandwidth. **/
	Link(const int id_ = -1, const std::string name_ = "", const int s = -1, const int t = -1, const double d = 0.0, const double b = 0.0);
    

	/****************************************************************************************/
	/*										Getters											*/
	/****************************************************************************************/
	/** Returns the link's id. **/
	const int& 			getId() 		const { return this->id; }
	/** Returns the link's name. **/
	const std::string& 	getName() 		const { return this->name; }
	/** Returns the link's source id. **/
	const int& 			getSource() 	const { return this->source_id; }
	/** Returns the link's target id. **/
	const int& 			getTarget() 	const { return this->target_id; }
	/** Returns the link's delay. **/
	const double& 		getDelay() 		const { return this->delay; }
	/** Returns the link's total bandwidth. **/
	const double& 		getBandwidth() 	const { return this->bandwidth; }

	/****************************************************************************************/
	/*										Display											*/
	/****************************************************************************************/
	/** Displays information about the link. **/
	void print();
};

#endif