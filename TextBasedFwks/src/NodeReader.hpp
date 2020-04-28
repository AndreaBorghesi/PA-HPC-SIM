#ifndef Def_NodeReader
#define Def_NodeReader
#include <vector>
#include <iostream>
#include <fstream>
#include "Node.hpp"
#include <string>
#include <sstream>

class NodeReader
{
	private:
		std::string _file;
	public:
		NodeReader(std::string file);
		std::vector<Node> read();
};

inline NodeReader::NodeReader(std::string file)
{
	_file=file;
}

inline std::vector<Node> NodeReader::read()
{
	std::ifstream input(_file.c_str());
	if (!input) 
	{
		std::cout << "ERROR: Could not open file \"" << _file << "\"" << std::endl;
		exit(1);
	}
	
	std::vector<Node> node;
	int i=0;
	std::string line;
	while (std::getline(input, line)){
		std::istringstream iss(line);
		int numNode;
		int numCores;
		int numGPUs;
		int numMICs;
		int totMem;
		int reserved;
		iss >> numNode;
		iss >> numCores;
		iss >> numGPUs;
		iss >> numMICs;
		iss >> totMem;
		iss >> reserved;
		node.push_back(Node(numNode,numCores,numGPUs,numMICs,totMem,(
                        reserved==1?true:false)));
		i++;

	}

	input.close();
	return node;
}
#endif
