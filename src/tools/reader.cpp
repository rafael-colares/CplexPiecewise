#include "reader.hpp"

/* Function to fetch data from a CSV File. */
std::vector<std::vector<std::string> > Reader::getData()
{
	std::ifstream file(filename.c_str());
	std::vector<std::vector<std::string> > dataList;
	if (file.is_open()) {
		std::string line = "";
		// Iterate through each line and split the content using delimeter
		while (getline(file, line)){
			std::vector<std::string> splittedVector = split(line, getDelimeter());
			dataList.push_back(splittedVector);
		}
		file.close();
	}
	else {
		std::cerr << "ERROR: Unable to open file " << filename << "." << std::endl;
		exit(EXIT_FAILURE);
	}
	return dataList;
}

/* Returns the substring of str between the first and last delimiters */
std::string getInBetweenString(std::string str, std::string firstDelimiter, std::string lastDelimiter)
{
	size_t first = str.find_last_of(firstDelimiter)+1;
	size_t last = str.find_last_of(lastDelimiter);
	std::string strNew = str.substr(first, last - first);
	return strNew;
}

/* Splits a given string into a vector by a given delimiter. */
std::vector<std::string> split(std::string str, std::string delimiter)
{
	std::vector<std::string> vec;
	size_t pos = 0;
	std::string token;
	std::string s = str;
	while ((pos = s.find(delimiter)) != std::string::npos) {
		token = s.substr(0, pos);
		token = removeSpecialChars(token);
        if (!token.empty()){
            vec.push_back(token);
        }
		s.erase(0, pos + delimiter.length());
	}
	s = removeSpecialChars(s);
    if (!s.empty()){
        vec.push_back(s);
    }
	return vec;
}

std::string removeSpecialChars(std::string str){
	str.erase(std::remove_if(str.begin(), str.end(),
    [](char c) { return std::isspace(c); } ),
    str.end());
	return str;
}
