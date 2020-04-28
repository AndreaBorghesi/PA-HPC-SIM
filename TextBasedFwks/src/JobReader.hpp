#ifndef DEF_JOBREADER
#define DEF_JOBREADER

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include "Job.hpp"
#include "Util.hpp"

#include <stdlib.h>

class JobReader
{
private:
	std::string _file;
public:
	JobReader(std::string file);
	std::vector<Job> read();
};

inline JobReader::JobReader(std::string file)
{
	_file=file;
}

inline std::vector<Job> JobReader::read()
{
	std::ifstream input(_file.c_str());
	if (!input) 
	{
		std::cout << "ERROR: Could not open file \"" << _file << "\"" << std::endl;
		exit(1);
	}
	
	std::vector<Job> job;
	string line;
	int nread = 0;
	while ( getline (input,line) )
	{
		string id,name,userName,q;
		int enterQueueTime,start,nodes=0,cores=0,gpu=0,
            mic=0,mem=0,estimatedDuration,realDuration;
		std::vector<int> usedNodes(64,0);
		bool ready=true;
		std::vector<string> third = Util::split(line,"__");
		std::vector<string> data1 = Util::split(third[0],";");

		enterQueueTime = Util::strToTime(data1[4]);
		std::vector<string> data3=Util::split(third[2],";");

		bool gpuMicAvail=false;

		start = Util::strToTime(data3[0]);

		realDuration = Util::strToTime(data3[1])-start;
		if(data3[2].compare("--")!=0)
			nodes=atoi(data3[2].c_str());
		else
			nodes=-1;
		if(data3[3].compare("--")!=0)
			cores=atoi(data3[3].c_str());
		else
		{
			ready=false;
		}
		if(data3[4].compare("--")!=0)
		{
			int mul=1;
			if(data3[4][data3[4].size()-2]=='g')
				mul=1024*1024;
			else if (data3[4][data3[4].size()-2]=='m')
				mul=1024;
			mem=atoi(data3[4].c_str())*mul;
		}
		else 
		{
			ready=false;
		}
		estimatedDuration = Util::strHHMMToTime(data3[5]);
		std::vector<string> data2=Util::split(third[1],"#");
		if(nodes==-1)
			nodes=data2.size();

		if(!ready || !gpuMicAvail)
		{
			nodes=data2.size(),cores=0,gpu=0,mic=0,mem=0;
			for(int i=0;i<data2.size();i++)
			{
				if(data2[i].size()>0)
				{
					std::vector<string> subInfo=Util::split(data2[i],";");
					int index=std::atoi(subInfo[0].c_str());
					if(index >= 0)
						usedNodes[index]=1;
			
					cores+=atoi(subInfo[1].c_str());
					gpu+=atoi(subInfo[2].c_str());
					mic+=atoi(subInfo[3].c_str());
					mem+=atoi(subInfo[4].c_str());
				}
			}
		}
		if(estimatedDuration==0)
			estimatedDuration=((realDuration/60)+1)*60;
		if(realDuration==0)
			realDuration=1;
		id=data1[0];
		name=data1[1];
		userName=data1[2];
		q = data1[3];
		if(nodes==0)
			nodes=1;
		if(cores==0)
			cores=4;
		if(mem==0)
			mem=200*1024;
        /* The following bounds are imposed in order to reflect 
         * the target supercomputer features, i.e. Euora nodes had maximum
         * 16 cores each or no node could have both a GPU and a MIC */
		if(cores/nodes>16)
		{
			std::cout<<"Too many cores"<<std::endl;
            exit(0);
		}
		if(gpu/nodes>2)
		{
			std::cout<<"Too many GPUs"<<std::endl;
            exit(0);
		}
		if(mic/nodes>2)
		{
			std::cout<<"Too many MICs"<<std::endl;
            exit(0);
		}
		if(gpu/nodes>0 && mic>0)
		{
			std::cout<<"Mixed GPUs and MICs"<<std::endl;
            exit(0);
		}
		if(mem/nodes>33554432)
		{
			std::cout<<"Too much memory"<<std::endl;
            exit(0);
		}
        int application_type = 0;
        if(data3.size() == 10)
            application_type = atoi(data3[9].c_str());
		job.push_back(Job(data1[0],data1[1],data1[2],
                    data1[3],enterQueueTime,nodes,cores
                    ,gpu,mic,mem,estimatedDuration,
                    realDuration,start,application_type,
                    usedNodes));
		nread++;
	}
	input.close();
	return job;
}
#endif

