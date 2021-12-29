#include "link.hpp"

/****************************************************************************************/
/*										Constructor										*/
/****************************************************************************************/

/** Constructor. **/
Link::Link(const int id_, const std::string name_, const int s, const int t, const double d, const double b) : 
                id(id_), name(name_), source_id(s), target_id(t), delay(d), bandwidth(b) {}

/****************************************************************************************/
/*										Display											*/
/****************************************************************************************/
/* Displays information about the link. */
void Link::print(){
    std::cout << "Id: " << id << ", "
              << "Name: " << name << ", "
              << "Source: " << source_id << ", "
              << "Target: " << target_id << ", "
              << "Delay: " << delay << ", "
              << "Bandwidth: " << bandwidth << std::endl;
}