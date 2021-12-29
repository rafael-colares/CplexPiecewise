#ifndef __others__hpp
#define __others__hpp

#include <iostream>
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>

/***************************************************
 *  This file hosts the list of auxiliary methods. 
 **************************************************/

/** Writes a greeting message. **/
void        greetingMessage();

/** Returns the path to parameter file. **/
std::string getParameter(int argc, char *argv[]);

/** Returns the indexes of a given vector sorted in ascending order of the vector values. @param vec The vector to be sorted. @note Ex.: [7,3,5] => [1,2,0] **/
std::vector<int> getSortedIndexes_Asc(const std::vector<double> &vec);

/** Returns the indexes of a given vector sorted in descending order of the vector values. @param vec The vector to be sorted. @note Ex.: [7,3,5] => [0,2,1] **/
std::vector<int> getSortedIndexes_Desc(const std::vector<double> &vec);
#endif