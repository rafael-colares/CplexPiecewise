#include "others.hpp"
/* Writes a greeting message. */
void greetingMessage(){
    std::cout << "===========================================================================================" << std::endl;
    std::cout << "- This program deals with the Availability-Aware Service Function Chains Routing Problem. -" << std::endl;
    std::cout << "-                                                                                         -" << std::endl;
    std::cout << "-     Author:        Rafael Colares                                                       -" << std::endl;
    std::cout << "-     Latest update: 09 Nov 2023                                                          -" << std::endl;
    std::cout << "===========================================================================================" << std::endl;
}

/* Returns the path to parameter file. */
std::string getParameter(int argc, char *argv[]){
    std::string param;
    if (argc != 2){
		throw std::invalid_argument("A parameter file is required in the arguments.\n Please run the executable file in the following way: ./exec parameterFile.txt\n" );
	}
	else{
		param = argv[1];
        std::cout << "PARAMETER FILE: " << param << std::endl;
	}
    return param;
}

std::vector<int> getSortedIndexes_Asc(const std::vector<double> &vec){
    std::vector<int> sorted(vec.size());
    std::iota(sorted.begin(), sorted.end(), 0);

    std::stable_sort(sorted.begin(), sorted.end(), 
        [&vec](int i1, int i2) { return vec[i1] < vec[i2]; });
    
    return sorted;
}

std::vector<int> getSortedIndexes_Desc(const std::vector<double> &vec){
    std::vector<int> sorted(vec.size());
    std::iota(sorted.begin(), sorted.end(), 0);

    std::stable_sort(sorted.begin(), sorted.end(), 
        [&vec](int i1, int i2) { return vec[i1] > vec[i2]; });
    
    return sorted;
}