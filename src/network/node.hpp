#ifndef __node__hpp
#define __node__hpp

/************************************/
/*			LIBRARIES				*/
/************************************/

/*** C++ Libraries ***/
#include <iostream>
#include <string>


/*************************************************
 * This class models a node in the network. Each 
 * node has an id, a name, coordinates, a capacity, 
 * an availability, and an unitary cost.
*************************************************/
class Node{
    private:
        const int id;					/**< The node's id. **/
        const std::string name;			/**< The node's name. **/
        const double coordinate_x;		/**< The node's x coordinate. **/
        const double coordinate_y;		/**< The node's y coordinate. **/
		const double capacity;			/**< The node's capacity. **/
		const double availability;		/**< The node's availability. **/
		const double unitary_cost;		/**< The node's unitary cost. **/

    public:
    /************************************/
	/*			Constructor				*/
	/************************************/

	/** Constructor. @param id_ Node id. @param name_ Node name. @param x Node's x coordinate. @param y Node's y coordinate. @param cap Node's capacity. @param avail Node's availability. @param cost Node's unitary cost.**/
	Node(const int id_ = -1, const std::string name_ = "", const double x = 0.0, const double y = 0.0, const double cap = 0.0, const double avail = 0.0, const double cost = 0.0);
    

    /************************************/
	/*			    Getters				*/
	/************************************/
    
	/** Returns the node's id. **/
	const int& 			getId() 		 const { return this->id; }
	/** Returns the node's name. **/
	const std::string& 	getName() 		 const { return this->name; }
	/** Returns the node's x coordinate. **/
	const double& 		getCoordinateX() const { return this->coordinate_x; }
	/** Returns the node's y coordinate. **/
	const double& 		getCoordinateY() const { return this->coordinate_y; }
	/** Returns the node's capacity. **/
	const double& 		getCapacity() 	 const { return this->capacity; }
	/** Returns the node's availability. **/
	const double& 		getAvailability() const { return this->availability; }
	/** Returns the node's cost per resource unit consumed. **/
	const double& 		getUnitaryCost()  const { return this->unitary_cost; }

    /************************************/
	/*			    Display				*/
	/************************************/
	/** Displays information about the node. **/
	void print() const;
};


/** Returns true if node a is more available than node b. **/
bool isMoreAvailable(const Node &a, const Node &b);

#endif