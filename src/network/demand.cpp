#include "demand.hpp"

/****************************************************************************************/
/*										Constructor										*/
/****************************************************************************************/

/** Constructor. **/
Demand::Demand(const int id_, const std::string name_, const int s, const int t, const double l, const double b, const double a) : 
            id(id_), name(name_), source(s), target(t), max_latency(l), bandwidth(b), availability(a) {}

/****************************************************************************************/
/*										Display											*/
/****************************************************************************************/
/* Displays information about the demand. */
void Demand::print(){
    std::cout << "Id: " << id << ", "
              << "Name: " << name << ","
              << "Source: " << source << ", "
              << "Target: " << target << ","
              << "Max latency: " << max_latency << ", "
              << "Availability: " << availability << ", "
              << "Bandwidth: " << bandwidth << std::endl;
    std::cout << "\tVNF list: ";
    for (unsigned int i = 0; i < VNF_list.size(); i++){
        std::cout << VNF_list[i] << ", ";
    }
    std::cout << std::endl;
}