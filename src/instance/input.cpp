#include "input.hpp"

/****************************************************************************************/
/*										Constructor										*/
/****************************************************************************************/

/** Constructor. **/
Input::Input(const std::string filename) : parameters_file(filename){
    std::cout << "=> Reading parameters file: " << parameters_file << " ..." << std::endl;
    
    node_file   = getParameterValue("nodeFile=");
    link_file   = getParameterValue("linkFile=");
    demand_file = getParameterValue("demandFile=");
    vnf_file    = getParameterValue("vnfFile=");

    strong_node_capacity    = (Strong_Node_Capacity_Constraints)std::stoi(getParameterValue("strong_node_capacity="));
    section_failure_cuts    = (Section_Failure_Cuts)std::stoi(getParameterValue("section_failure="));
    nb_paths_ub             = (Nb_Paths_Upper_Bound)std::stoi(getParameterValue("nb_paths_upper_bound="));
    nb_paths_lb             = (Nb_Paths_Lower_Bound)std::stoi(getParameterValue("nb_paths_lower_bound="));
    approx_type             = (Approximation_Type)std::stoi(getParameterValue("availability_relax="));

    linear_relaxation       = std::stoi(getParameterValue("linearRelaxation="));
    basic                   = std::stoi(getParameterValue("basic="));
    time_limit              = std::stoi(getParameterValue("timeLimit="));
    nb_breakpoints          = std::stoi(getParameterValue("nb_breakpoints="));

    output_file             = getParameterValue("outputFile=");

    print();
}



/* Returns the pattern value in the parameters file. */
std::string Input::getParameterValue(std::string pattern){
    std::string line;
    std::string value = "";
    std::ifstream param_file (parameters_file.c_str());
    if (param_file.is_open()) {
        while ( std::getline (param_file, line) ) {
            std::size_t pos = line.find(pattern);
            if (pos != std::string::npos){
                value = line.substr(pos + pattern.size());
                value.erase(std::remove(value.begin(), value.end(), '\r'), value.end());
                value.erase(std::remove(value.begin(), value.end(), '\n'), value.end());
                if (value.empty()){
                    std::cout << "WARNING: Field '" << pattern << "' is empty." << std::endl; 
                }
                return value;
            }
        }
        param_file.close();
    }
    else {
        std::cerr << "ERROR: Unable to open parameters file '" << parameters_file << "'." << std::endl; 
        exit(EXIT_FAILURE);
    }
    std::cout << "WARNING: Did not found field '" << pattern << "' inside parameters file." << std::endl; 
    return value;
}

void Input::print(){
    std::cout << "\t Node File: " << node_file << std::endl;
    std::cout << "\t Link File: " << link_file << std::endl;
    std::cout << "\t Service Chain Function File: " << demand_file << std::endl;
    std::cout << "\t Virtual Network Function File: " << vnf_file << std::endl;
    std::cout << "\t Linear Relaxation: ";
    if (linear_relaxation){
        std::cout << "TRUE" << std::endl;
    }
    else{
        std::cout << "FALSE" << std::endl;
    }
    std::cout << "Strong capacity: " << strong_node_capacity << std::endl;
    std::cout << "Number of paths upper bound: " << nb_paths_ub << std::endl;
    std::cout << "Number of paths lower bound: " << nb_paths_lb << std::endl;
    std::cout << "\t Number of breakpoints: " << nb_breakpoints << std::endl;
    std::cout << "\t Time Limit: " << time_limit << " seconds" << std::endl;
    std::cout << "\t Output File: " << output_file << std::endl;
}