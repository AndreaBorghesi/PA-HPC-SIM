#ifndef Def_Job
#define Def_Job

#include <ctime>
#include <vector>
#include <string>
#include "Util.hpp"

/* This class is  used to represent a HPC job */

using namespace std;
class Job
{
	private:
		 string _id;
		 string _name;
		 string _userName;
		 string _queue;

         /* When the job enters the system */
		 long long _enterQueueTime;
		 
		 long long _nodes;
		 long long _cores;
		 long long _gpus;
		 long long _mics;
		 long long _mem;
		
         /* Real job duration; obtained since we know the job start time 
          * and end time (from historical data)   */
		 long long _realDuration;
         /* Maximum duration provided by the user. If a job lasts more the 
          * dispatcher kills it */
		 long long _estimatedDuration;

		 long long _start;
		 double _lastMakespan;
		 bool _makespanPresent;

         double _power_prediction;

         /* Specify if the job has already been scheduled */
		 bool _scheduled;

         /* Set of nodes allocated to the job (after its start) */
		 std::vector<int> _usedNodes;
		 int min(int a,int b){return (a<b?a:b);}

         /* This hypothetical field describe the nature of the job (broadly):
          * 0: average application
          * 1: cpu-intensive application
          * 2: memory-intensive application  */
         int _application_type;

	 public:
		 bool isScheduled(){return _scheduled;}
		 void setScheduled(bool s){_scheduled=s;}
		 double getLastMakespan(){return _lastMakespan;}
		 bool isMakespanPresent(){return _makespanPresent;}
		 void setLastMakespan(double lmks){_lastMakespan=lmks;_makespanPresent=true;}
		 string getJobId(){return _id;}
		 string getJobName(){return _name;}
		 string getUserName(){return _userName;}
		 long long getEnterQueueTime(){return _enterQueueTime;}
		 string getQueue(){return _queue;}
		 long long getNumberOfNodes(){return _nodes;}
		 long long getNumberOfCores() const{return _cores;}
		 long long getNumberOfGPU() const{return _gpus;}
		 long long getNumberOfMIC() const{return _mics;}
		 long long getMemory() const{return _mem/(1024);}
		 long long getEstimatedDuration() const{return _estimatedDuration;}
		 long long getRealDuration(){return _realDuration;}
		 long long getStartTime(){return _start;}
         double getPowerPrediction(){return _power_prediction;}
         int getApplicationType(){return _application_type;}
		 std::vector<int> getUsedNodes(){return _usedNodes; }
		 long long getDuration(long long atTime)
		 {
			 if(getStartTime()+min(getEstimatedDuration(),getRealDuration())<= atTime)
			 {
				 return min(getEstimatedDuration(),getRealDuration());
			 }
			 return getEstimatedDuration();
		 }
		 
		 void setJobId(string id){ _id=id;}
		 void setJobName(string name){ _name=name;}
		 void setUserName(string userName){_userName=userName;}
		 void setQueue(string queue){ _queue=queue;}
		 void setEnterQueueTime(long long eqt){_enterQueueTime=eqt;}
		 
		 void setNumberOfNodes(long long n){ _nodes=n;}
		 void setNumberOfCores(long long n){ _cores=n;}
		 void setNumberOfGPU(long long n){ _gpus=n;}
		 void setNumberOfMIC(long long n){ _mics=n;}
		 void setMemory(long long m){ _mem=m;}
		 void setEstimatedDuration(long long d){_estimatedDuration=d;}
		 void setRealDuration(long long d){
			 _realDuration=d;}
			 void setStartTime(long long start){_start=start;}

		 void setPowerPrediction(double pp){_power_prediction=pp;}
         void setApplicationType(int at){_application_type=at;}
		 
		 void allocate(int s, std::vector<int> nodes){_start=s, _usedNodes=nodes;}
		 inline Job(string id,string name,string userName,string queue,long long eqt,
                 long long nodes,long long cores,long long gpu,long long mic,
                 long long mem,long long estDur,long long realDur,long long start,
                 int application_type, std::vector<int> used){
			 
			 setJobId(id);
			 setJobName(name);
			 setUserName(userName);
			 setQueue(queue);
			 setEnterQueueTime(eqt);
			 setNumberOfNodes(nodes);
			 setNumberOfCores(cores);
			 setNumberOfGPU(gpu);
			 setNumberOfMIC(mic);
			 setMemory(mem);
			 setEstimatedDuration(estDur);
			 setRealDuration(realDur);
			 allocate(start,used);
			 _makespanPresent=false;
			 _lastMakespan=-1;
			 _scheduled=false;

             setApplicationType(application_type);
		 }
		 
		 string toString(long long now)
		 {
			 stringstream s("");
			 if(isScheduled()==false)
				 s<<"N ";
 			s<<getJobId()<<" "<< getEnterQueueTime()<<" ["<<getStartTime()<<
                " : "<<Util::timeToStr(getStartTime())<<"->"<<getDuration(now)<<
                "->"<<Util::timeToStr(getDuration(now)+getStartTime())<<"] [";
			for(int j=0;j<getUsedNodes().size();j++)
			{
				s<<getUsedNodes()[j];
			}
			s<<"] ";
			
			return s.str();
		 }

};
#endif
