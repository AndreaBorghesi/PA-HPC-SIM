#ifndef Def_Node
#define Def_Node

class Node 
{
	private:
		long long _numNode;
		long long _numCores;
		long long _numGPUs;
		long long _numMICs;
		long long _totMem;
		bool _reserved;
		double _nominalFrequency;

	public:
		long long getNodeNumber(){return _numNode;}
		long long getCoresNumber(){return _numCores;}
		long long getGPUsNumber(){return _numGPUs;}
		long long getMICsNumber(){return _numMICs;}
		long long getTotalMemory(){return _totMem/(1024);}
		int isReserved(){return _reserved==true?1:0;}
		double getNominalFrequency(){return _nominalFrequency;}
		
		void setNodeNumber(long long n){ _numNode=n;}
		void setCoresNumber(long long n){ _numCores=n;}
		void setGPUsNumber(long long n){ _numGPUs=n;}
		void setMICsNumber(long long n){ _numMICs=n;}
		void setTotalMemory(long long n){ _totMem=n;}
		void setReserved(int r){ _reserved=(r==1?true:false);}
		void setReserved(bool r){ _reserved=r;} 
		void setNFreq(double f){ _nominalFrequency=f;}

		Node(long long nodes,long long cores,long long gpus
                ,long long mics,long long mem,bool reserved){
            setNodeNumber(nodes),setCoresNumber(cores),
                setGPUsNumber(gpus),setMICsNumber(mics),
                setTotalMemory(mem),setReserved(reserved);}
};
#endif
