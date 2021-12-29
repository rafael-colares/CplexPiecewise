#ifndef __reader__hpp
#define __reader__hpp

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

/**
 * This class implements a reader of .csv files. 
 * It is used for reading the input files.
 */
class Reader{
private:
	const std::string filename; 	/**< The file to be read. **/
	const std::string delimeter;	/**< The delimiter used for separating data. **/
public:
	/** Constructor. @param filepath The path of the file to be read. @param delm The delimiter to be used. **/
	Reader(std::string filepath, std::string delm = ";"): filename(filepath), delimeter(delm){}
  
    /** A file must be provided. **/
    Reader() = delete;


    /** Returns the file to be read. **/
    const std::string& getFilename()  const { return filename; }
    /** Returns the delimiter to be used. **/
    const std::string& getDelimeter() const { return delimeter; }

	/** Function to fetch data from a CSV File. It goes through the .csv file, line by line, and returns the data in a vector of vector of strings. **/
	std::vector<std::vector<std::string> > getData();
};

/****************************************************************
 * These are other useful methods for treating strings.
 * *************************************************************/

/** Returns the substring between the first and last delimiters. @param str The string to be examined. @param firstDelimiter The first delimiter. @param lastDelimiter The last delimiter. **/
std::string getInBetweenString(std::string str, std::string firstDelimiter, std::string lastDelimiter);

/** Splits a given string by a delimiter and returns a vector of strings. @param str The string to split. @param delimiter The delimiter. For instance, "1;2;3" becomes vector {1, 2, 3} if delimiter is ";". **/
std::vector<std::string> split(std::string str, std::string delimiter);


std::string removeSpecialChars(std::string str);
#endif