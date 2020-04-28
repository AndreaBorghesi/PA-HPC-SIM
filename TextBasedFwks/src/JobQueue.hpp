#ifndef Def_Queue
#define Def_Queue
#include <string>

class JobQueue
{
	private:
        /* Each queue is characterized by its name (id) and by the maximum time 
         * one of its job can wait (expressed in seconds)    */
		std::string _id;
		int _waiting;
	public:
		int getMaxMinutesToWait(){return _waiting;}
		std::string getId(){return _id;}
		void setMaxMinutesToWait(int w){_waiting=w;}
		void setId(std::string id){_id=id;}
		JobQueue(std::string id,int w){setId(id),setMaxMinutesToWait(w);}
};
#endif
