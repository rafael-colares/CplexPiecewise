#ifndef __data__hpp
#define __data__hpp


/****************************************************************************************/
/*										LIBRARIES										*/
/****************************************************************************************/

/*** C++ Libraries ***/
#include <float.h>
#include <unordered_map>
#include <algorithm>
#include <cmath>

/*** LEMON Libraries ***/     
#include <lemon/list_graph.h>

/*** Own Libraries ***/  
#include "input.hpp"
#include "../network/demand.hpp"
#include "../network/node.hpp"
#include "../network/link.hpp"
#include "../network/vnf.hpp"
#include "../tools/reader.hpp"


/****************************************************************************************/
/*										TYPEDEFS										*/
/****************************************************************************************/

/*** LEMON ***/
/* Structures */
typedef lemon::ListDigraph 	Graph;
typedef Graph::Arc 			Arc;

/* Iterators */
typedef Graph::NodeIt 		NodeIt;
typedef Graph::ArcIt 		ArcIt;

/* Maps */
typedef Graph::NodeMap<int> NodeMap;
typedef Graph::ArcMap<int> ArcMap;

/********************************************************************************************
 * This class stores the data needed for modeling an instance of the Resilient SFC routing 
 * and VNF placement problem. This consists of a network graph, 											
********************************************************************************************/
class Data {

private:
	Input 				params;						/**< Input parameters. **/
    std::vector<Node> 	tabNodes;         			/**< Set of nodes. **/
	std::vector<Link> 	tabLinks;					/**< Set of links. **/
	std::vector<VNF> 	tabVnfs;					/**< Set of VNFs. **/
	std::vector<Demand> tabDemands;					/**< Set of demands. **/

	Graph* 				graph;						/**< The network graph. **/
	NodeMap* 			nodeId;						/**< A map storing the nodes' ids. **/
	NodeMap* 			lemonNodeId;				/**< A map storing the nodes' lemon ids. **/
	ArcMap* 			arcId;						/**< A map storing the arcs' ids. **/
	ArcMap* 			lemonArcId;					/**< A map storing the arcs' lemon ids. **/

	std::unordered_map<std::string, int> hashNode; 	/**< A map for locating node id's from its name. **/
	std::unordered_map<std::string, int> hashVnf; 	/**< A map for locating vnf id's from its name. **/

	std::vector<int>	availNodeRank;				/**< A vector containing the ids of nodes in decreasing order of availability. **/
	std::vector<int>	maxPathsPerDemand;			/**< A vector storing an upper bound on the number of paths dedicated to each demand. **/
public:

	/****************************************************************************************/
	/*										Constructor										*/
	/****************************************************************************************/

	/** Constructor initializes the object with the information of an Input. @param parameter_file The parameters file.**/
	Data(const std::string &parameter_file);



	/****************************************************************************************/
	/*										Getters											*/
	/****************************************************************************************/

	const Input& 			 	getInput 		 () const { return params; }		/**< Returns a reference to the data's Input. */
	const Graph& 			 	getGraph     	 () const { return *graph; }		/**< Returns a reference to the data's Graph. */
	const NodeMap& 			 	getNodeIds   	 () const { return *nodeId; }		/**< Returns a reference to the map storing the nodes' ids. */
	const NodeMap& 			 	getLemonNodeIds  () const { return *lemonNodeId; }	/**< Returns a reference to the map storing the nodes' lemon ids. */
	const ArcMap& 			 	getArcIds    	 () const { return *arcId; }		/**< Returns a reference to the map storing the arcs' ids. */
	const ArcMap& 			 	getLemonArcIds   () const { return *lemonArcId; }	/**< Returns a reference to the map storing the arcs' lemon ids. */
	const std::vector<Node>& 	getNodes     	 () const { return tabNodes; }		/**< Returns a reference to the vector of nodes. */
	const std::vector<Link>& 	getLinks     	 () const { return tabLinks; }		/**< Returns a reference to the vector of arcs. */
	const std::vector<VNF>&  	getVnfs     	 () const { return tabVnfs; }		/**< Returns a reference to the vector of vnfs. */
	const std::vector<Demand>&  getDemands     	 () const { return tabDemands; }	/**< Returns a reference to the vector of sfc demands. */
	const std::vector<int>&  	getAvailNodeRank () const { return availNodeRank; }	/**< Returns the ranking of most available nodes identified by their id. */

	const VNF& 		getVnf    (const int i) 		const { return tabVnfs[i]; }	/**< Returns a reference to the i-th vnf. */
	const Demand& 	getDemand (const int i) 		const { return tabDemands[i]; }	/**< Returns a reference to the i-th demand. */
	const Link& 	getLink   (const int i) 		const { return tabLinks[i]; }	/**< Returns a reference to the i-th arc. */
	const Node& 	getNode   (const int i) 		const { return tabNodes[i]; }	/**< Returns a reference to the i-th node. */

	const int  getNbNodes     () 					 const { return (int)tabNodes.size(); }		/**< Returns the number of nodes. */
	const int  getNbVnfs      () 					 const { return (int)tabVnfs.size(); }		/**< Returns the number of vnfs. */
	const int  getNbDemands   () 					 const { return (int)tabDemands.size(); }	/**< Returns the number of sfc demands. */
	const int& getNodeId   	  (const Graph::Node& v) const { return (*nodeId)[v]; }				/**< Returns the id of a given node. */
	const int& getLemonNodeId (const Graph::Node& v) const { return (*lemonNodeId)[v]; }		/**< Returns the lemon id of a given node. */
	const int& getArcId    	  (const Arc& a) 		 const { return (*arcId)[a]; }				/**< Returns the id of a given arc. */
	const int& getLemonArcId  (const Arc& a) 		 const { return (*lemonArcId)[a]; }			/**< Returns the lemon id of a given arc. */

	/** Returns the id from the node with the given name. @param name The node name. **/
	int	 	   getIdFromNodeName(const std::string name) const;

	/** Returns the id from the vnf with the given name. @param name The vnf name. **/
	int	 	   getIdFromVnfName(const std::string name) const;

    /** Returns the probability that a set of nodes fail simoustaneously. @param nodes The set of nodes to fail. **/
    const double getFailureProb(const std::vector<int>& nodes) const;
    
    /** Returns the chain availability based on the availability of each section. @note The chain availability is the product of the availability of its sections. @param sectionAvail The sections availability. **/
    const double getChainAvailability(const std::vector<double>& sectionAvail) const;

    /** Returns the chain availability induced by a set of nodes. @note The chain availability is the product of the availability of its nodes. @param nodes The ids of nodes to be considered. **/
    const double getChainAvailability(const std::vector<int>& nodes) const;

	/** Return the cost of placing a vnf on a node. @param node The node to receive the vnf. @param vnf The vnf to be installed. **/
	const double getPlacementCost(const Node& node, const VNF& vnf) const { return node.getUnitaryCost()*vnf.getConsumption(); }
	
	/** Returns the minimum number of nodes with availability at most B required (in parallel) to ensure a given availability level. @param av The availability requested. @param B Nodes with availability higher than B are not considered. @note Returns -1 if the availability requested cannot be satisfied. **/
	const int getMinNbNodes(double av, double B = 1.0) const;
	
	/** Returns the node position on availability ranking. @param id The node's id. @note Returns -1 if id is not found. **/
	const int getNodeRankPosition (int id) const;
	
	/** Returns the availability obtained from the placement of a set of nodes in parallel. @param nodes The set of nodes**/
	const double getParallelAvailability (const std::vector<int>& nodes) const;

	/** Returns a vector containing the ids of the n most available nodes. @param n The number of nodes to be returned. **/
	const std::vector<int> getNMostAvailableNodes(int n) const;

	/** Returns a vector containing the ids of the n most available nodes among a subset of nodes. @param n The number of nodes to be selected. @param NODES The set of nodes that can be selected. **/
	const std::vector<int> getNMostAvailableNodes(int n, const std::vector<bool> &NODES) const;

	/** Returns a vector containing the ids of the n least available nodes. @param n The number of nodes to be returned. **/
	const std::vector<int> getNLeastAvailableNodes(int n) const;

	/** Returns a vector containing the ids of the n least available nodes among a subset of nodes. @param n The number of nodes to be selected. @param NODES The set of nodes that can be selected. **/
	const std::vector<int> getNLeastAvailableNodes(int n, const std::vector<bool> &NODES) const;

	/** Returns the minimum number of vnfs to be installed for a SFC. @param B The SFC availability requirement. @param nbSections The number of sections to be considered. @note All nodes are considered accessible. **/
	const int getVnfLB(double B, int nbSections) const;

	/** Returns the minimum number of vnfs to be installed among certain nodes over a certain number of sections in order to achieve a certain availability level. @param NODES The set of nodes that can receive a vnf. @param NB_SECTIONS The number of sections to be considered. @param AVAIL_REQUIRED The availabily level required. @note If availability cannot be met, return -1. **/
	const int getVnfLB(const std::vector<bool> &NODES, const int NB_SECTIONS, const double AVAIL_REQUIRED) const;

	/** Returns the highest number of vnfs required by an SFC **/
	const int getLongestSFC() const;

	/* Return an upper bound on the number of vnf-disjoint paths */
	const int getNbPaths() const { return getNbNodes(); }
	const int getNbPaths(int k) const { return maxPathsPerDemand[k]; } 

	/****************************************************************************************/
	/*										Setters											*/
	/****************************************************************************************/
	void setNodeId 		(const Graph::Node& v, const int &id) { (*nodeId)[v] = id; }		/**< Sets the id of a given node on the node map. */
	void setLemonNodeId (const Graph::Node& v, const int &id) { (*lemonNodeId)[v] = id; }	/**< Sets the lemon id of a given node on the node map. */
	void setArcId 		(const Graph::Arc& a, const int &id)  { (*arcId)[a] = id; }			/**< Sets the id of a given arc on the arc map. */
	void setLemonArcId 	(const Graph::Arc& a, const int &id)  { (*lemonArcId)[a] = id; }	/**< Sets the lemon id of a given arc on the arc map. */


	/****************************************************************************************/
	/*										Methods											*/
	/****************************************************************************************/

	/** Reads the node file and fills the set of nodes. @param filename The node file to be read. **/
	void readNodeFile(const std::string filename);

	/** Reads the link file and fills the set of link. @param filename The link file to be read. **/
	void readLinkFile(const std::string filename);

	/** Reads the vnf file and fills the set of vnfs. @param filename The vnf file to be read. **/
	void readVnfFile(const std::string filename);

	/** Reads the demand file and fills the set of demands. @param filename The demand file to be read. **/
	void readDemandFile(const std::string filename);

	/** Builds the network graph from data stored in tabNodes and tabLinks. **/
	void buildGraph();
	
	/** Builds the availability ranking of nodes. @note Highest availabilities first. **/
	void buildNodeRank();

	/** Builds the upper bounds on the number of paths that should route each demand. **/
	void computeMaxPathPerDemand();


	/****************************************************************************************/
	/*										Display											*/
	/****************************************************************************************/
	/** Prints global data information. **/
	void print();
	/** Prints node information. **/
	void printNodes();
	/** Prints link information. **/
	void printLinks();
	/** Prints demand information. **/
	void printDemands();
	/** Prints vnfs information. **/
	void printVnfs();
	/** Prints node information sorted by their availability. **/
	void printNodeRank();


	/****************************************************************************************/
	/*										Destructor										*/
	/****************************************************************************************/
	/** Destructor. Clears the vectors of demands and links. **/
	~Data();
};

#endif