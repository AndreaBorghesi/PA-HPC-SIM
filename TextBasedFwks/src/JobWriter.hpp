#ifndef Def_JobWriter
#define Def_JobWriter
#include <vector>
#include <iostream>
#include <fstream>
#include <time.h>
#include "Job.hpp"

/* Class to write a job (to output) in the format used for the input instances */
class JobWriter
{
private:
	std::string _file;
public:
	JobWriter(std::string file);
	void write(std::vector<Job> jobs,int refTime);
	void write(std::vector<Job> jobs){write(jobs,0);}
};

inline JobWriter::JobWriter(std::string file)
{
	_file=file;
}


inline void JobWriter::write(std::vector<Job> jobs,int refTime)
{
	std::ofstream output(_file.c_str());
	if (!output) 
	{
		std::cout << "ERROR: Could not open file \"" << _file << "\"" << std::endl;
		exit(1);
	}
	
	for (int i=0;i<jobs.size();i++)
	{
		output<<jobs[i].getJobId()<<";";
		output<<jobs[i].getJobName()<<";";
		output<<jobs[i].getUserName()<<";";
		output<<jobs[i].getQueue()<<";";
		output<<Util::timeToStr(jobs[i].getEnterQueueTime()+refTime)<<"__";
		
		for(int j=0;j<jobs[i].getUsedNodes().size();j++)
		{
			std::vector<int> usedNodes=jobs[i].getUsedNodes();
			while(usedNodes[j]>=1)
			{
				output<<j<<";";
				output<<jobs[i].getNumberOfCores()/jobs[i].getNumberOfNodes()<<";";
				output<<jobs[i].getNumberOfGPU()/jobs[i].getNumberOfNodes()<<";";
				output<<jobs[i].getNumberOfMIC()/jobs[i].getNumberOfNodes()<<";";
				output<<jobs[i].getMemory()/jobs[i].getNumberOfNodes()<<"#";
				
				usedNodes[j]--;
			}
		}
		output<<"__"<<Util::timeToStr(jobs[i].getStartTime()+refTime)<<";";
		output<<Util::timeToStr(jobs[i].getRealDuration()+jobs[i].getStartTime()+refTime)<<";";
		output<<jobs[i].getNumberOfNodes()<<";";
		output<<jobs[i].getNumberOfCores()<<";";
		output<<jobs[i].getMemory()<<";";
		output<<Util::timeHHMMToStr(jobs[i].getEstimatedDuration())<<std::endl;
	}
	output.close();
}
#endif
