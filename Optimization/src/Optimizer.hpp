/*	Facade class for the all optimizers.

	Some design choices:
	- define everything in a new namespace
	- do not employ any "using namespace" statement
	- use only standard types */

#ifndef OPTIMIZER_H_
#define OPTIMIZER_H_

#include <OptimizerImpl.hpp>
#include <map>

namespace optimization_core {

// =============================================================================
// = Iterator classes                                                          =
// =============================================================================

/* Iterator over the set of 64 bits indices */
class IdxIterator64 {
	// Allow access to private fields from the Optimizer
	friend class Optimizer;
private:
	/* The map over which the iterator is built */
	std::unordered_map<long long, int>& _map;
	/* An STL iterator over a map */
	std::unordered_map<long long, int>::iterator _it; 
	
	/* A simple, private, constructor */
	IdxIterator64(std::unordered_map<long long, int>& map)
		: _map(map), _it(map.begin()) {}
			
public:
	/* True if the end of the collection has not been reached */
	bool ok() { return _it != _map.end(); }
	
	/* Return the current index */
	long long val() { return _it->first; }
	
	/* Move to the next item */
	void next() { ++_it; }
};


/* Iterator over the set of 32 bits indices */
class IdxIterator32 {
	// Allow access to private fields from the Optimizer
	friend class Optimizer;
private:
	/* The map over which the iterator is built */
	std::unordered_map<int, int>& _map;
	/* An STL iterator over a map */
	std::unordered_map<int, int>::iterator _it; 
	
	/* A simple, private, constructor */
	IdxIterator32(std::unordered_map<int, int>& map)
		: _map(map), _it(map.begin()) {}
			
public:
	/* True if the end of the collection has not been reached */
	bool ok() { return _it != _map.end(); }
	
	/* Return the current index */
	int val() { return _it->first; }
	
	/* Move to the next item */
	void next() { ++_it; }
};


// =============================================================================
// = Default implementation of a CompatibilityChecker and IntervalManager      =
// =============================================================================


/* Compatibility checker class. This class defines a single method, that
   is used by the optimizer to check if a specific job can be mapped on a
   specific resource */
class CompatibilityChecker {
public:
	/* Compatibility check method. By default, this always returns true. The
	   default behavior can be altered by subclassing */
	virtual bool ok(long long job_idx, int res_idx) {
		return true;
	}

};



/* Class to deal with special time intervals. The class is capable of:
   - returning the bounds of the interval that overlaps a certain time instant
   - returning the type for the current interval
   - checking if a job is compatible with a certain interval type
   
   The interval start is the first time instant that belongs to the interval,
   while the end is the first time instant that belongs to the next interval */
class IntervalManager {
public:
	/* Return the information about the interval that overlaps with the time
	   instant passed as parameter. By default, we assume there is a single
	   interval. The default behavior can be changed by subclassing. */
	virtual void intervalData(long long time_instant,
								long long& interval_start,
								long long& interval_end,
								int& interval_type) {
		interval_start = std::numeric_limits<long long>::min();
		interval_end = std::numeric_limits<long long>::max();
		interval_type = 0;
	}
	
	/* Check if a job can be executed in a specific interval type. By default,
	   every job can be executed in every interval. The default behavior can
	   be changed by subclassing and overriding the method. */
	virtual bool ok(long long job_idx, int interval_type) {
		return true;
	}
};


// =============================================================================
// = Base class for all dispatching optimizers                                 =
// =============================================================================
	
/* Main interface for all dispatching optimizers. The class provides method to
   configure and solve an optimization problem.
   
   The problem configuration methods are design to build and modify a problem
   incrementally.*/
class Optimizer {
public:
	/*	A simple constructor */
	Optimizer(int ndims,
			CompatibilityChecker* cmp_chk,
			IntervalManager* int_man);
	
	/*	A simple destructor */
	virtual ~Optimizer() {}
	
	/*	Register one more job in the optimizer. All job fields are initialized
		to defaults values. In particular:
		- qet = minimum 64 bits int
		- ewt = minimum 64 bits int
		- start = maximum 64 bits int
		- dur = 0
		- no job units are created
	
		\param job_idx the index of the new job
		\return true if the job index had not been already registered
	*/
	bool addJob_noPowerPred(long long job_idx,
				long long dur,
				long long qet,
				long long ewt,
				const std::vector<std::vector<long long> >& unit_reqs);

	/*	Register one more job in the optimizer. All job fields are initialized
		to defaults values. In particular:
		- qet = minimum 64 bits int
		- ewt = minimum 64 bits int
		- start = maximum 64 bits int
		- dur = 0
        - power prediction = 0
        - current power = 0
		- no job units are created
        - current power and power prediction might differ: 1) current power
          could be the real, measured power consumption; 2) mechanisms 
          such as RAPL or DVFS could change the initial power consumption
	
		\param job_idx the index of the new job
		\return true if the job index had not been already registered
	*/
	bool addJob(long long job_idx,
				long long dur,
                double power_pred,
                double power,
				long long qet,
				long long ewt,
                int application_type,
				const std::vector<std::vector<long long> >& unit_reqs);
	
	/*	Assign a start time to a job */					
	void setStartAndMapping(long long job_idx,
							long long start,
							const std::vector<int>& unit_mapping);
	
	/*	Assign a duration to a job */
	void setDur(long long job_idx, long long dur);

	/*	Assign a predicted power consumption to a job */
	void setPowerPred(long long job_idx, double power_pred);

	/*	Assign a power consumption to a job */
	void setPower(long long job_idx, double power);

	/*	Assign an execution frequency to a job */
	void setFrequency(long long job_idx, int frequency);

	/*	Assign an application type to a job */
	void setApplicationType(long long job_idx, int app_type);
		
	/*	Remove a job */
	void remJob(long long job_idx);
	
		
	/*	Add a new resource
		\param caps The resource capacity over each dimension
		\return the index of the new resource */
	bool addRes(int res_idx, const std::vector<long long>& caps);
	
	/* Remove a resource. The resource data is not lost, however, and the
	   resource can be "recalled" using an ad-hoc method */
	void shutdownRes(long long res_idx);
	
	/* Restore a removed resource */
	void restartRes(long long res_idx);
	
	/* Check the state of a resource */
	bool isShutdown(long long res_idx);
	
	
	/*	Access the number of jobs */
	long long getNJobs();
	
	/*	Access the number of resource dimensions */
	int getNDims();
	
	/*	Access the number of resources */
	int getNRes();
	
	
	/*	Access the QET of a job */
	long long getQET(long long job_idx);
	
	/*	Access the EWT of a job */
	long long getEWT(long long job_idx);
	
	/*	Access the start time of a job */
	long long getStart(long long job_idx);

	/*	Access the duration of a job */
	long long getDur(long long job_idx);

	/*	Access the predicted power consumption of a job */
	double getPowerPred(long long job_idx);

	/*	Access the power consumption of a job */
	double getPower(long long job_idx);

	/*	Access the frequency of a job */
	int getFrequency(long long job_idx);

	/*	Access the application type of a job */
	int getApplicationType(long long job_idx);
	
	/*	Access the requirements of a job unit */
	long long getReq(long long job_idx, int unit_idx, int dim_idx);
	
	/*	Access the cumulative requirements of a job. By cumulative requirement
		we refer to the sum of requirements of all job units for a specific
		dimension */
	long long getReq(long long job_idx, int dim_idx);
	
	
	/*	Access the capacity of a resource */
	long long getCap(int res_idx, int dim_idx);
	
	
	/* Access the cumulative requirements of all (non-removed) jobs */
	long long getReq(int dim_idx);
	
	
	/*	Access the cumulative capacity of all (operating) resources */
	long long getCap(int dim_idx);


	/* Obtain an iterator over the jobs */
	IdxIterator64 jobIterator() { return IdxIterator64(_job_map); }
	
	/* Obtain an iterator over the resources */
	IdxIterator32 resIterator() { return IdxIterator32(_res_map); }
	
	
	/* Obtain the next job to be dispatched. The method returns false if no job
	   ready for dispatching was found.

	   The method requires as a parameter a reference time, so that the solver
	   can distinguish between past, present, and future events.
	
	   The default implementation returns false and is defined only to allow
	   some testing. This method is meant to be overriden in subclasses.
	
	   Whenever this method is called, the current schedule *may* be modified.
	   Hence, all methods that access information about the current schedule
	   status may return modified results 

       Returns also the operational frequency
       - schedulers which don't change the frequency return -1
       - Strong assumption: all units (and related nodes) run at 
         the same frequency    */
	virtual bool nextJob(long long ref_time,
						long long& job_idx,
						std::vector<int>& unit_mapping,
                        int& frequency) {
		return false;
	}

	/* Get the start time of a job in the current proactive schedule. */
	virtual long long getSchedStart(long long job_idx) {
		return -1;
	}
	
	/* Get the end time of a job in the current proactive schedule. */
	virtual long long getSchedEnd(long long job_idx) {
		return -1;
	}
	
	/* Write the current schedule to an output stream */
	virtual void writeSched(std::ostream& out) {
		return;
	}

	/* Return the current problem version (DEBUG purpose) */
	virtual long long getProblemVersion() {
		return -1;
	}

    /* Set the power budget constraint - This method is 
     * useful if we need to change the power cap during online
     * dispatching */
    virtual void setMaxPower(long long maxPower){

    }

     /* Set the jobs perfModels factors.
      *  Method used to synchronize the internal values of
      *  the optimizers with different approach.
      *  (Basically we want DVFS with RAPL to use the same
      *  factors as pure RAPL, in case of different tests with
      *  the same instance)                         */
    void setJobsPerfModelFactors(
            map<long long, double> jobs_perfModel_factors){
    }

protected:
	/* main problem object */
	DispatchingProblem _prb;
	
	/* job index map (external to internal) */
	std::unordered_map<long long, int> _job_map;
	
	/* resource index map (external to internal) */
	std::unordered_map<int, int> _res_map;
	
	/* Pointer to a compatibility checker */
	CompatibilityChecker* _cmp_chk;
	
	/* Pointer to an interval manager */
	IntervalManager* _int_man;	
};

// =============================================================================
// = Method implementations (Optimizer)                                        =
// =============================================================================

Optimizer::Optimizer(int ndims,
					CompatibilityChecker* cmp_chk,
					IntervalManager* int_man)
		: _prb(ndims), _cmp_chk(cmp_chk), _int_man(int_man) {
	assert(cmp_chk);
	assert(int_man);
}


bool Optimizer::addJob_noPowerPred(	long long job_idx,
						long long dur,
						long long qet,
						long long ewt,
						const std::vector<std::vector<long long> >& unit_reqs) {
	assert(unit_reqs.size() > 0);
	// If the job is already registered, then do nothing
	auto it = _job_map.find(job_idx);
	if (it == _job_map.end()) {
		// Otherwise:
		// - add a new job to the problem
		int prb_idx = _prb.addJob(job_idx);
		_job_map[job_idx] = prb_idx;
		// - set the duration
		_prb.setDur(prb_idx, dur);
		// - set the QET
		_prb.setQET(prb_idx, qet);
		// - set the EWT
		_prb.setEWT(prb_idx, ewt);
		// - add all the units
		for (int u = 0; u < unit_reqs.size(); ++u)
			_prb.addUnit(prb_idx, unit_reqs[u]);
		// - return
		return true;
	}

	return false;
}

bool Optimizer::addJob(	long long job_idx,
						long long dur,
                        double power_pred,
                        double power,
						long long qet,
						long long ewt,
                        int application_type,
						const std::vector<std::vector<long long> >& unit_reqs) {
	assert(unit_reqs.size() > 0);
	//cout << _prb.getProblemVersion() << endl;
	auto it = _job_map.find(job_idx);
	if (it == _job_map.end()) {
		// Otherwise:
		// - add a new job to the problem
		int prb_idx = _prb.addJob(job_idx);
		_job_map[job_idx] = prb_idx;
		// - set the duration
		_prb.setDur(prb_idx, dur);
        // - set predicted power consumption
        _prb.setPowerPred(prb_idx, power_pred);
        // - set power consumption
        _prb.setPower(prb_idx, power);

        // - set job frequency
        /* Initially the frequency is not set (-1) since
         * it depends on the node on which the job is mapped
         * After the mapping the frequency must be changed to 
         * its real value */
        _prb.setFrequency(prb_idx, -1);

		// - set the QET
		_prb.setQET(prb_idx, qet);
		// - set the EWT
		_prb.setEWT(prb_idx, ewt);

        // - set the application type
        _prb.setApplicationType(prb_idx, application_type);
		// - add all the units
		for (int u = 0; u < unit_reqs.size(); ++u)
			_prb.addUnit(prb_idx, unit_reqs[u]);
		// - return
		
		//cout << _prb.getProblemVersion() << endl;
		return true;
	}

	//cout << _prb.getProblemVersion() << endl;
	return false;
}

inline void Optimizer::setStartAndMapping(long long job_idx,
				long long start, const std::vector<int>& unit_mapping) {
	assert(_job_map.find(job_idx) != _job_map.end());

	auto it = _job_map.find(job_idx);
	// Assign the start time
	_prb.setStart(it->second, start);
	// Convert the resource indices
	std::vector<int> int_unit_mapping;
	for (int v : unit_mapping) int_unit_mapping.push_back(v);
	// Map the job units
	_prb.mapUnits(it->second, int_unit_mapping);
}

inline void Optimizer::setDur(long long job_idx, long long dur) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	_prb.setDur(it->second, dur);
}

inline void Optimizer::setPowerPred(long long job_idx, double power_pred) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	_prb.setPowerPred(it->second, power_pred);
}

inline void Optimizer::setPower(long long job_idx, double power) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	_prb.setPower(it->second, power);
}

inline void Optimizer::setFrequency(long long job_idx, int frequency) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	_prb.setFrequency(it->second, frequency);
}

inline void Optimizer::setApplicationType(long long job_idx, int app_type) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	_prb.setApplicationType(it->second, app_type);
}

inline void Optimizer::remJob(long long job_idx) {
    //if(_job_map.find(job_idx) != _job_map.end())
    //    cout << "job " << job_idx << endl;
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	_prb.remJob(it->second);
	_job_map.erase(it);

	//cout << _prb.getProblemVersion() << endl;
}


inline bool Optimizer::addRes(int res_idx, const std::vector<long long>& caps) {
	auto it = _res_map.find(res_idx);
	if (it == _res_map.end()) {
		_res_map[res_idx] = _prb.addRes(res_idx, caps);
		return true;
	}
	return false;
}

inline void Optimizer::shutdownRes(long long res_idx) {
	assert(_res_map.find(res_idx) != _res_map.end());
	auto it = _res_map.find(res_idx);
	_prb.remRes(it->second);
}

inline void Optimizer::restartRes(long long res_idx) {
	assert(_res_map.find(res_idx) != _res_map.end());
	auto it = _res_map.find(res_idx);
	_prb.recallRes(it->second);
}

inline bool Optimizer::isShutdown(long long res_idx) {
	assert(_res_map.find(res_idx) != _res_map.end());
	auto it = _res_map.find(res_idx);
	return _prb.isResRemoved(it->second);
}


inline long long Optimizer::getNJobs() {
	return _job_map.size();
}

inline int Optimizer::getNDims() {
	return _prb.getNDims();
}

inline int Optimizer::getNRes() {
	return _res_map.size();
}


inline long long Optimizer::getQET(long long job_idx) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	return _prb.getQET(it->second);
}

inline long long Optimizer::getEWT(long long job_idx) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	return _prb.getEWT(it->second);
}

inline long long Optimizer::getStart(long long job_idx) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	return _prb.getStart(it->second);
}

inline long long Optimizer::getDur(long long job_idx) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	return _prb.getDur(it->second);
}

inline double Optimizer::getPowerPred(long long job_idx) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	return _prb.getPowerPred(it->second);
}

inline double Optimizer::getPower(long long job_idx) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	return _prb.getPower(it->second);
}

inline int Optimizer::getFrequency(long long job_idx) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	return _prb.getFrequency(it->second);
}

inline long long Optimizer::getReq(long long job_idx, int unit_idx, int dim_idx) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	return _prb.getReq(it->second, unit_idx, dim_idx);
}

inline long long Optimizer::getReq(long long job_idx, int dim_idx) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	return _prb.getReq(it->second, dim_idx);
}


inline long long Optimizer::getCap(int res_idx, int dim_idx) {
	assert(_res_map.find(res_idx) != _res_map.end());
	auto it = _res_map.find(res_idx);
	return _prb.getCap(it->second, dim_idx);
}


inline long long Optimizer::getReq(int dim_idx) {
	return _prb.getReq(dim_idx);
}
	
inline long long Optimizer::getCap(int dim_idx) {
	return _prb.getCap(dim_idx);
}


} // end of namespace optimization_core


#endif
