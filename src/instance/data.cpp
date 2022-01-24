#include "data.hpp"

/****************************************************************************************/
/*										Constructor										*/
/****************************************************************************************/

/** Constructor. **/
Data::Data(const std::string &parameter_file) : params(parameter_file)
{
	std::cout << "=> Defining data ..." << std::endl;
	readNodeFile(params.getNodeFile());
	readLinkFile(params.getLinkFile());
	readVnfFile(params.getVnfFile());
	readDemandFile(params.getDemandFile());

	buildGraph();
	buildNodeRank();
	computeMaxPathPerDemand();
	std::cout << "\t Data was correctly constructed !" << std::endl;
	
}

/****************************************************************************************/
/*										Getters 										*/
/****************************************************************************************/
/* Returns the id from the node with the given name. */
int Data::getIdFromNodeName(const std::string name) const
{
	auto search = hashNode.find(name);
    if (search != hashNode.end()) {
        return search->second;
    } 
	else {
        std::cerr << "ERROR: Could not find a node with name '"<< name << "'... Abort." << std::endl;
		exit(EXIT_FAILURE);
    }
	int invalid = -1;
	return invalid;
}
/* Returns the id from the vnf with the given name. */
int Data::getIdFromVnfName(const std::string name) const
{
	for( const auto& it : hashVnf ) {
		if (it.first == name){
			return it.second;
		}
    }
    
	std::cerr << "ERROR: Could not find a vnf with name '"<< name << "'... Abort." << std::endl;
	exit(EXIT_FAILURE);
    
	return -1;
}

/* Returns the probability that all nodes fail simoustaneously. */
const double Data::getFailureProb (const std::vector<int>& nodes) const
{
    double prob_fail = 1.0;
    for (unsigned int j = 0; j < nodes.size(); j++){                    
        int v = nodes[j];
        prob_fail *= (1.0 - getNode(v).getAvailability());
    }
    return prob_fail;
}
/* Returns the chain availability based on the availability of each section. */
const double Data::getChainAvailability (const std::vector<double>& sectionAvail) const
{
    double prob = 1.0;
    for (unsigned int i = 0; i < sectionAvail.size(); i++){
        prob *= sectionAvail[i];
    }
    return prob;
}

/* Returns the chain availability induced by a set of nodes. */
const double Data::getChainAvailability (const std::vector<int>& nodes) const
{
	if (nodes.size() < 1) return 0.0;

    double prob = 1.0;
    for (unsigned int i = 0; i < nodes.size(); i++){
		int v = nodes[i];
        prob *= getNode(v).getAvailability();
    }
    return prob;
}

/****************************************************************************************/
/*										Methods 										*/
/****************************************************************************************/

/* Reads the node file and fills the set of nodes. */
void Data::readNodeFile(const std::string filename)
{
    if (filename.empty()){
		std::cerr << "ERROR: A node file MUST be declared in the parameters file.\n";
		exit(EXIT_FAILURE);
	}
    std::cout << "\t Reading " << filename << " ..."  << std::endl;
	Reader reader(filename);
	/* dataList is a vector of vectors of strings. */
	/* dataList[0] corresponds to the first line of the document and dataList[0][i] to the i-th word.*/
	std::vector<std::vector<std::string> > dataList = reader.getData();
	// skip the first line (headers)
	for (unsigned int i = 1; i < dataList.size(); i++)	{
		int nodeId = (int)i - 1;
		std::string nodeName = dataList[i][0];
		double nodeX = atof(dataList[i][1].c_str());
		double nodeY = atof(dataList[i][2].c_str());
		double capacity = atof(dataList[i][3].c_str());
		double avail = atof(dataList[i][4].c_str());
		double cost = atof(dataList[i][5].c_str());
		this->tabNodes.push_back(Node(nodeId, nodeName, nodeX, nodeY, capacity, avail, cost));
		hashNode.insert({nodeName, nodeId});
	}

}

/* Reads the link file and fills the set of links. */
void Data::readLinkFile(const std::string filename)
{
    if (filename.empty()){
		std::cerr << "ERROR: A link file MUST be declared in the parameters file.\n";
		exit(EXIT_FAILURE);
	}
    std::cout << "\t Reading " << filename << " ..."  << std::endl;
	Reader reader(filename);
	/* dataList is a vector of vectors of strings. */
	/* dataList[0] corresponds to the first line of the document and dataList[0][i] to the i-th word.*/
	std::vector<std::vector<std::string> > dataList = reader.getData();
	// skip the first line (headers)
	for (unsigned int i = 1; i < dataList.size(); i++)	{
		int linkId = (int)i - 1;
		std::string linkName = dataList[i][0];
		int source = getIdFromNodeName(dataList[i][1]);
		int target = getIdFromNodeName(dataList[i][2]);
		double delay = atof(dataList[i][3].c_str());;
		double bandwidth = atof(dataList[i][4].c_str());
		this->tabLinks.push_back(Link(linkId, linkName, source, target, delay, bandwidth));
	}
}



/* Reads the vnf file and fills the set of vnfs. */
void Data::readVnfFile(const std::string filename)
{
    if (filename.empty()){
		std::cerr << "ERROR: A vnf file MUST be declared in the parameters file.\n";
		exit(EXIT_FAILURE);
	}
    std::cout << "\t Reading " << filename << " ..."  << std::endl;
	Reader reader(filename);
	/* dataList is a vector of vectors of strings. */
	/* dataList[0] corresponds to the first line of the document and dataList[0][i] to the i-th word.*/
	std::vector<std::vector<std::string> > dataList = reader.getData();
	// skip the first line (headers)
	for (unsigned int i = 1; i < dataList.size(); i++)	{
		int vnfId = (int)i - 1;
		std::string vnfName = dataList[i][0];
		double resource_consumption = atof(dataList[i][1].c_str());
		this->tabVnfs.push_back(VNF(vnfId, vnfName, resource_consumption));
		hashVnf.insert({vnfName, vnfId});
	}
}

/** Reads the demand file and fills the set of demands. @param filename The demand file to be read. **/
void Data::readDemandFile(const std::string filename)
{
    if (filename.empty()){
		std::cerr << "ERROR: A demand file MUST be declared in the parameters file.\n";
		exit(EXIT_FAILURE);
	}
    std::cout << "\t Reading " << filename << " ..."  << std::endl;
	Reader reader(filename);
	/* dataList is a vector of vectors of strings. */
	/* dataList[0] corresponds to the first line of the document and dataList[0][i] to the i-th word.*/
	std::vector<std::vector<std::string> > dataList = reader.getData();
	// skip the first line (headers)
	for (unsigned int i = 1; i < dataList.size(); i++)	{
		int demandId = (int)i - 1;
		std::string demandName = dataList[i][0];
		int source = getIdFromNodeName(dataList[i][1]);
		int target = getIdFromNodeName(dataList[i][2]);
		double latency = atof(dataList[i][3].c_str());
		double band = atof(dataList[i][4].c_str());
		double availability = atof(dataList[i][5].c_str());
		this->tabDemands.push_back(Demand(demandId, demandName, source, target, latency, band, availability));
		std::vector<std::string> list = split(dataList[i][6], ",");
		for (unsigned int j = 0; j < list.size(); j++){
			if (!list[j].empty()){
				int vnfId = getIdFromVnfName(list[j]);
				tabDemands[demandId].addVNF(vnfId);
			}
		}
	}
}



/* Builds the availability ranking of nodes. */
void Data::buildNodeRank()
{
	availNodeRank.resize(tabNodes.size());
	for (unsigned int i = 0; i < availNodeRank.size(); i++){
		availNodeRank[i] = i;
	}

    // One by one move boundary of unsorted subarray  
    for (int i = 0; i < (int)availNodeRank.size()-1; i++)  {  
        
		// Find the minimum element in unsorted array  
        int min = i;  
        for (int j = i+1; j < (int)availNodeRank.size(); j++){
        	if (isMoreAvailable(getNode(availNodeRank[j]), getNode(availNodeRank[min]))){
				min = j;
			}  
		}

        // Swap the found minimum element with the first element  
        int temp = availNodeRank[min];  
		availNodeRank[min] = availNodeRank[i];  
		availNodeRank[i] = temp;  
    }  
	//printNodeRank();
}

/* Builds the network graph from data stored in tabNodes and tabLinks. */
void Data::buildGraph()
{
	
	std::cout << "\t Creating graph..." << std::endl;
	/* Dymanic allocation of graph */
    graph = new Graph();
	nodeId = new NodeMap(*graph);
	lemonNodeId = new NodeMap(*graph);
	arcId = new ArcMap(*graph);
	lemonArcId = new ArcMap(*graph);
	
	/* Define nodes */
	for (unsigned int i = 0; i < tabNodes.size(); i++){
        Graph::Node n = graph->addNode();
        setNodeId(n, tabNodes[i].getId());
        setLemonNodeId(n, graph->id(n));
    }

	/* Define arcs */
	for (unsigned int i = 0; i < tabLinks.size(); i++){
        int source = tabLinks[i].getSource();
        int target = tabLinks[i].getTarget();
        Graph::Node sourceNode = lemon::INVALID;
        Graph::Node targetNode = lemon::INVALID;
        for (NodeIt v(getGraph()); v != lemon::INVALID; ++v){
            if(getNodeId(v) == source){
                sourceNode = v;
            }
            if(getNodeId(v) == target){
                targetNode = v;
            }
        }
        if (targetNode != lemon::INVALID && sourceNode != lemon::INVALID){
            Arc a = graph->addArc(sourceNode, targetNode);
            setLemonArcId(a, graph->id(a));
            setArcId(a, tabLinks[i].getId());
        }
    }
}

/* Returns the node position on availability ranking. */
const int Data::getNodeRankPosition (int id) const
{
	for (unsigned int i = 0; i < availNodeRank.size(); i++){
		if (availNodeRank[i] == id)	return i;
	}
	return -1;
}

/* Returns the minimum number of nodes with availability at most B required to ensure a given availability level. */
const int Data::getMinNbNodes(double av, double B) const
{
	double fail_prob = 1.0;
	double av_prob = 1.0 - fail_prob;
	int nb = 0;
	int i = 0;
	//std::cout << "Required availability: " << av << std::endl;
	while(av_prob < av && i < (int)availNodeRank.size()){
		if (getNode(availNodeRank[i]).getAvailability() <= B){
			fail_prob *= (1 - getNode(availNodeRank[i]).getAvailability());
			nb++;
			av_prob = 1.0 - fail_prob;
			//std::cout << "Add node " << getNode(availNodeRank[i]).getId() << " with availability " << getNode(availNodeRank[i]).getAvailability() << std::endl;
		}
		i++;
	}
	//std::cout << "STOP including nodes. nb = " << nb << std::endl;
	if (av_prob >= av){
		return nb;
	}
	else{
		std::cerr << "ERROR: There is no feasible solution: required availabilities too high. Abort optimization." << std::endl;
		exit(EXIT_FAILURE);
	}
}



/** Builds the upper bounds on the number of paths that should route each demand. **/
void Data::computeMaxPathPerDemand()
{
	const int NB_DEMANDS = getNbDemands();
	maxPathsPerDemand.resize(NB_DEMANDS);
	for (int k = 0; k < NB_DEMANDS; k++){
		maxPathsPerDemand[k] = getNbNodes();
	}
	if (getInput().getNbPathsUpperBound() == Input::NB_PATHS_UPPER_BOUND_ON){
		for (int k = 0; k < NB_DEMANDS; k++){
			const int NB_VNFS = getDemand(k).getNbVNFs();
			const double REQUIRED_AVAIL = getDemand(k).getAvailability();
			std::vector<int> least_available_nodes = getNLeastAvailableNodes(NB_VNFS);
			const double MIN_PATH_AVAIL = getChainAvailability(least_available_nodes);

			double chain_avail = MIN_PATH_AVAIL;
			//std::cout << "Min path avail"  << ": " << MIN_PATH_AVAIL << std::endl;
			int nbPaths = 1;
			while (chain_avail < REQUIRED_AVAIL){
				nbPaths++;
				chain_avail = 1 - std::pow(1.0 - MIN_PATH_AVAIL, nbPaths);
			}
			maxPathsPerDemand[k] = std::min(maxPathsPerDemand[k], nbPaths);
			//std::cout << "Max nb paths for demand " << k << ": " << nbPaths << ", " << maxPathsPerDemand[k] << std::endl;
		}
	}
}

/* Returns the availability obtained from the placement of a set of nodes in parallel.*/
const double Data::getParallelAvailability(const std::vector<int>& nodes) const
{
	double fail_prob = 1.0;
	for (unsigned int i = 0; i < nodes.size(); i++){
		int id = nodes[i];
		double av = getNode(id).getAvailability();
		fail_prob *= (1.0 - av);
	}
	return (1.0 - fail_prob);
}

/* Returns a vector containing the ids of the n most available nodes. */
const std::vector<int> Data::getNMostAvailableNodes(int n) const
{
	n = std::min(n, (int)availNodeRank.size());
	std::vector<int> nodes;
	nodes.resize(n);
	for (int i = 0; i < n; i++){
		nodes[i] = availNodeRank[i];
	}
	return nodes;
}

/* Returns a vector containing the ids of the n most available nodes among a subset of nodes. */
const std::vector<int> Data::getNMostAvailableNodes(int n, const std::vector<bool> &NODES) const{
	n = std::min(n, (int)std::count(NODES.begin(), NODES.end(), true));
	std::vector<int> selected_nodes;
	selected_nodes.resize(n);
	int nb_selected = 0;
	int i = 0;
	while (nb_selected < n){
		if (NODES[availNodeRank[i]] == true){
			selected_nodes[nb_selected] = availNodeRank[i];
			nb_selected++;
		}
		i++;
	}
	return selected_nodes;
}

/* Returns a vector containing the ids of the n least available nodes. */
const std::vector<int> Data::getNLeastAvailableNodes(int n) const
{
	int size = (int)availNodeRank.size();
	n = std::min(n, size);
	std::vector<int> nodes;
	nodes.resize(n);
	for (int i = 0; i < n; i++){
		nodes[i] = availNodeRank[size-1-i];
	}
	return nodes;
}

/* Returns a vector containing the ids of the n least available nodes among a subset of nodes. */
const std::vector<int> Data::getNLeastAvailableNodes(int n, const std::vector<bool> &NODES) const{
	int size = (int)std::count(NODES.begin(), NODES.end(), true);
	n = std::min(n, size);
	std::vector<int> selected_nodes;
	selected_nodes.resize(n);
	int nb_selected = 0;
	int i = (int)NODES.size() - 1;
	while (nb_selected < n){
		if (NODES[availNodeRank[i]] == true){
			selected_nodes[nb_selected] = availNodeRank[i];
			nb_selected++;
		}
		i--;
	}
	return selected_nodes;
}

/* Returns the minimum number of vnfs to be installed for a SFC. */
const int Data::getVnfLB(double B, int nbSections) const
{
	/* Get max number of most available nodes per section violating the availability requirement */
	int vnfs_per_section = 1;
	while (std::pow(getParallelAvailability(getNMostAvailableNodes(vnfs_per_section)), nbSections) < B){
		vnfs_per_section++;
	}
	int base = std::max(vnfs_per_section - 1, 1);
	int result = base*nbSections;
	/* Try to improve the number of vnfs */
	double section_avail = getParallelAvailability(getNMostAvailableNodes(base));
	double chain_avail = std::pow(section_avail, nbSections);
	double new_section_avail = getParallelAvailability(getNMostAvailableNodes(base+1));
	while (chain_avail < B){
		chain_avail = (chain_avail*new_section_avail)/section_avail;
		result++;
	}

	return result;
}


/* Returns the minimum number of vnfs to be installed among certain nodes over a certain number of sections in order to achieve a certain availability level.*/
const int Data::getVnfLB(const std::vector<bool> &NODES, const int NB_SECTIONS, const double AVAIL_REQUIRED) const{
	/* Get max number of most available nodes per section violating the availability requirement */
	int vnfs_per_section = 1;
	while (std::pow(getParallelAvailability(getNMostAvailableNodes(vnfs_per_section, NODES)), NB_SECTIONS) < AVAIL_REQUIRED){
		vnfs_per_section++;
		/* If availability cannot be met with the given NODES*/
		if (vnfs_per_section > (int)std::count(NODES.begin(), NODES.end(), true)){
			return -1;
		}
	}
	int base = std::max(vnfs_per_section - 1, 1);
	int result = base*NB_SECTIONS;
	/* Try to improve the number of vnfs */
	double section_avail = getParallelAvailability(getNMostAvailableNodes(base, NODES));
	double chain_avail = std::pow(section_avail, NB_SECTIONS);
	double new_section_avail = getParallelAvailability(getNMostAvailableNodes(base+1, NODES));
	while (chain_avail < AVAIL_REQUIRED){
		chain_avail = (chain_avail*new_section_avail)/section_avail;
		result++;
	}
	return result;
}


/** Returns the highest number of vnfs required by an SFC **/
const int Data::getLongestSFC() const{
	const int NB_DEMANDS = getNbDemands();
	int number = 0;
	for (int k = 0; k < NB_DEMANDS; k++){
		const int NB_VNFS = getDemand(k).getNbVNFs();
		if (number < NB_VNFS){
			number = NB_VNFS;
		}
	}
	return number;
}

/****************************************************************************************/
/*										Display											*/
/****************************************************************************************/
void Data::print(){
	printNodes();
	printLinks();
	printVnfs();
	printDemands();
}


void Data::printNodes(){
	for (unsigned int i = 0; i < tabNodes.size(); i++){
        tabNodes[i].print();
    }
	std::cout << std::endl;
}
void Data::printLinks(){
	for (unsigned int i = 0; i < tabLinks.size(); i++){
        tabLinks[i].print();
    }
	std::cout << std::endl;
}

void Data::printVnfs(){
	for (unsigned int i = 0; i < tabVnfs.size(); i++){
        tabVnfs[i].print();
    }
	std::cout << std::endl;
}
void Data::printDemands(){
	for (unsigned int i = 0; i < tabDemands.size(); i++){
        tabDemands[i].print();
    }
	std::cout << std::endl;
}

void Data::printNodeRank(){
	std::cout << "Node ranking: " << std::endl;
	for (unsigned int i = 0; i < availNodeRank.size(); i++){
		getNode(availNodeRank[i]).print();
	}
}


/****************************************************************************************/
/*										Destructor										*/
/****************************************************************************************/

/** Desstructor. **/
Data::~Data()
{
    this->tabLinks.clear();
    this->tabNodes.clear();
	this->hashNode.clear();
	this->tabDemands.clear();
	this->tabVnfs.clear();
	delete nodeId;
	delete lemonNodeId;
	delete arcId;
	delete lemonArcId;
	delete graph;
}
