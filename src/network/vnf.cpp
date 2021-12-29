#include "vnf.hpp"

/****************************************************************************************/
/*										Constructor										*/
/****************************************************************************************/

/** Constructor. **/
VNF::VNF(const int id_, const std::string name_, const double cons) : 
            id(id_), name(name_), consumption(cons){}

/****************************************************************************************/
/*										Display											*/
/****************************************************************************************/
/* Displays information about the vnf. */
void VNF::print(){
    std::cout << "Id: " << id << ", "
              << "Name: " << name << ", "
              << "Consumption: " << consumption << std::endl;
}