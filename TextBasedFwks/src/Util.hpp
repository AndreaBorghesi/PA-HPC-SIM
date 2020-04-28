#ifndef Def_Util
#define Def_Util

#include <vector>
#include <string>
#include <iostream>
#include <ctime>
#include <math.h>
#include <sstream>
#include <time.h> 
#include <stdlib.h>
#include <chrono>

using namespace std;

class Util
{
private:
	static time_t strTimeToTime(string str)
	{
		time_t t=0;
		vector<string> data=split(str,":");
		if(data.size()>3 || data.size()==0)
		{
			std::cout<<"ERROR Util::strHHMMToTime( "<<str<<" ) wrong format:" <<
               " Format accepted HH:MM"<<std::endl;
		} 
		else
		{
			for(int i=data.size()-1;i>=0;i--)
			{
				t+=atoi(data[i].c_str())*pow(60,data.size()-1-i);
			}
		}
		return t;
	}
public:
	
	static int min(int a,int b)
	{
		return (a<b?a:b);
	}
	
	static int max(int a,int b)
	{
		return (a>b?a:b);
	}
	static vector<string> split(string str, string del)
	{
		vector<string> res;
		int end=0;
		while((end=str.find(del,0))>=0)
		{
			res.push_back(str.substr(0,end));
			str=str.substr(end+del.size(),str.size());
		}
		if(str.size()>0)
			res.push_back(str);
		return res;
	}
	
	static string timeHHMMToStr(time_t t)
	{
		stringstream ret("");
		int h=t/(60*60);
		int m=(t-h*60*60)/60;
		ret<<h<<":"<<m;
		return ret.str();
		
	}
	
	static string timeToStr(time_t t)
	{
		std::tm * ptm = std::localtime(&t);
		char buffer[32];
		
		std::strftime(buffer, 32, "%Y-%m-%d %H:%M:%S", ptm);  
		string res(buffer);
		return res;
	}
	static time_t strToTime(string str) 
	{
		struct tm t;
		strptime(str.c_str(), "%Y-%m-%d %H:%M:%S", &t);
		t.tm_isdst = -1;
		return mktime(&t) ;
	}
	
	static time_t strHHMMToTime(string str)
	{
		time_t t=0;
		vector<string> data=split(str,":");
		if(data.size()!=2)
		{
			std::cout<<"ERROR Util::strHHMMToTime( "<<str<<" ) wrong format:" 
               << " Format accepted HH:MM"<<std::endl;
		} 
		else
		{
			t+=atoi(data[0].c_str())*60*60+atoi(data[1].c_str())*60;
		}
		return t;
	}
};

class CustomTimer {
	
private:
	double _elapsed;
	chrono::system_clock::time_point _last;
	bool _running;
	
public:
	CustomTimer() : _elapsed(0), _running(false) {}
	
	virtual ~CustomTimer() {}
	
	void start() {
		if (!_running) {
			_last = chrono::system_clock::now();
			_running = true;
		}
	}
	
	void stop() {
		if (_running) {
			// update elapsed time
			auto now = chrono::system_clock::now();
			auto ms = chrono::duration_cast<std::chrono::microseconds>(now - _last);
			_elapsed += ms.count() / 1000.0;
			_running = false;	
		}
	}
	
	void reset() {
		_elapsed = 0;
		_running = false;
	}
	
	double getVal() {
		if (_running) {
			stop(); // update the elapsed time count
			start(); // keep the time running
		}
		return _elapsed;
	}
};


#endif
