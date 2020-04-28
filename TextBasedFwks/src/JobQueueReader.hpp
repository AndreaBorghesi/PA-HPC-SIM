#ifndef Def_QueueReader
#define Def_QueueReader

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "JobQueue.hpp"
#include <stdlib.h>

class JobQueueReader
{
	private:
		std::string _file;
	public:
		JobQueueReader(std::string file);
		std::vector<JobQueue> read();
};

inline JobQueueReader::JobQueueReader(std::string file)
{
	_file=file;
}

inline std::vector<JobQueue> JobQueueReader::read()
{
	std::ifstream inputQs(_file.c_str());
	if (!inputQs) 
	{
		std::cout << "ERROR: Could not open file \"" << _file << "\"" << std::endl;
		exit(1);
	}
	
	std::vector<JobQueue> qs;
	while (!inputQs.eof())
	{
		int w;
		std::string id;
		inputQs >> id;
		inputQs >> w;
		qs.push_back(JobQueue(id,w));
	}
	inputQs.close();
	return qs;
}
#endif
