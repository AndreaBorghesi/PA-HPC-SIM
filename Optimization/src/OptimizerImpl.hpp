#ifndef OPTIMIZERIMPL_HPP_
#define OPTIMIZERIMPL_HPP_

#include <cassert>
#include <unordered_map>
#include <vector>
#include <limits>
#include <set>
#include <iostream>

namespace optimization_core {

// =============================================================================
// = Define the debug macros                                                   =
// =============================================================================

// comment/decomment this to enable debug output for this file
#define DEBUG_OPTIMPL_HPP

#ifndef NDEBUG
	#ifdef DEBUG_OPTIMPL_HPP
		#define DBG(X) X
		#define DBGOUT (cerr << "[opt-impl] ")
	#else
		#define DBG(X)
	#endif
#else
	#define DBG(X)
#endif

class DispatchingProblem {
		
public:
	/*	A simple constructor */
	DispatchingProblem(int ndims);
	
	/* A simple destructor */
	virtual ~DispatchingProblem() {}
	
	
	/* Add one job to the problem */
	int addJob(long long ext_idx);
	
	/* Assign the Queue Enter Time */
	void setQET(int job_idx, long long qet);

	/* Assign the Expected Waiting Time */
	void setEWT(int job_idx, long long ewt);

	/* Assign the start time */
	void setStart(int job_idx, long long start);
	
	/* Assign the duration */
	void setDur(int job_idx, long long dur);

	/* Assign the predicted power consumption */
	void setPowerPred(int job_idx, double power_pred);

	/* Assign the current power consumption */
	void setPower(int job_idx, double power);

	/* Assign the current execution frequency */
	void setFrequency(int job_idx, int frequency);

	/* Assign the application type */
	void setApplicationType(int job_idx, int app_type);
	
	/* Register a new unit for a job. This method can be called only on a
	   job that has been just added (i.e. in the "added" collection) */
	void addUnit(int job_idx, const std::vector<long long>& reqs);
	
	/* Force a mapping for all units of a job */
	void mapUnits(int job_idx, const std::vector<int>& unit_mapping);

	/* Remove a job (the job data is not lost) */
	void remJob(int job_idx);
	
	/* Recall a removed (not buried) job. The job is inserted in the "added"
	   collection */
	void recallJob(int job_idx);
	
	/* Commit a job. The job is inserted in the "stable" collection, unless
	   it is removed: in that case, the job data is permanently deleted */
	void commitJob(int job_idx);
	
	//TODO check to code to compute the global statistics
	
	/* Add a new resource
	   \param ext_idx the external index of the resource
	   \param caps The resource capacity over each dimension
	   \return the index of the new resource */
	int addRes(int ext_idx, const std::vector<long long>& caps);
	
	/* Remove a resource */
	void remRes(int res_idx);
	
	/* Recall a removed resource */
	void recallRes(int res_idx);
	
	/* Access added job collection. The collection contains job indices */
	const std::vector<int>& getAddedJobs();
	
	/* Access modified job collection. The collection contains job indices */
	const std::vector<int>& getModifiedJobs();
	
	/* Access removed job collection. The collection contains job indices */
	const std::vector<int>& getRemovedJobs();
	
	/* Access stable job collection. The collection contains job indices */
	const std::vector<int>& getStableJobs();
	
	/* Access stable resource collection. The collection contains indices */
	const std::vector<int>& getStableRes();
	
	/* Access removed resource collection. The collection contains indices */
	const std::vector<int>& getRemovedRes();
	
	/* Access the number of dimensions */
	int getNDims();

	/* Access the Queue Enter Time */
	long long getQET(int job_idx);
	
	/* Check if the job has a user-specified QET */
	bool hasAssignedQET(int job_idx);

	/* Access the Expected Waiting Time */
	long long getEWT(int job_idx);

	/* Access the start time */
	long long getStart(int job_idx);
	
	/* Check if a job has a user-specified start time */
	bool hasAssignedStart(int job_idx);
	
	/* Access the job duration */
	long long getDur(int job_idx);

	/* Access the job predicted power */
	long long getPowerPred(int job_idx);

	/* Access the job consumed power */
	long long getPower(int job_idx);

	/* Access the job frequency */
	long long getFrequency(int job_idx);

	/* Access the job application type */
	long long getApplicationType(int job_idx);
	
	/* Access the external index */
	long long getJobExtIdx(int job_idx);

	/* Access the requirements of a job unit */
	long long getReq(int job_idx, int unit_idx, int dim_idx);

	/* Access the requirements of a job */
	long long getReq(int job_idx, int dim_idx);
	
	/* Access the number of units of a job */
	int getNUnits(int job_idx);
	
	/* Check if the job has already been mapped */
	bool isMapped(int job_idx);
	
	/* Access the unit mapping information */
	int getMapping(int job_idx, int unit_idx);
	
	
	/* Check if the job is in the "added" collection */
	bool isJobAdded(int job_idx);
	
	/* Check if the job is in the "modified" collection */
	bool isJobModified(int job_idx);
	
	/* Check if the job is in the "removed" collection */
	bool isJobRemoved(int job_idx);
	
	/* Check if the job is in the "stable" collection */
	bool isJobStable(int job_idx);
	
	/* Check if a job index is valid */
	bool isJobValid(int job_idx);
	
	
	/* Access the capacity of a resource */
	long long getCap(int res_idx, int dim_idx);
	
	/* Access the external index */
	long long getResExtIdx(int res_idx);
	
	/* Check if a resource is in the "stable" collection */
	bool isResStable(int res_idx);
	
	/* Check if a resource is in the "removed" collection */
	bool isResRemoved(int res_idx);
	
	
	/* Obtain the maximum EWT. All global statistics are computed over the set
	   of non-removed jobs (i.e. added, modified, and stable) */
	long long getMaxEWT();
	
	/* Obtain the total requirement for a certain dimension. All global
	   statistics are computed over the set of non-removed jobs (i.e. added,
	   modified, and stable) */
	long long getReq(int dim_idx);
	
	/* Obtain the total capacity for a certain dimension */
	long long getCap(int dim_idx);
	

	/* Access the number of invalid jobs */
	int getNInvalidJobs();
	
	/* Check if the whole problem is stable */
	bool isProblemStable();
	
	/* Get the problem version */
	long long getProblemVersion();

    /* Set the power budget constraint - This method is 
     * useful if we need to change the power cap during online
     * dispatching */
    void setMaxPower(long long maxPower);
		
private:
	/*	Maximum int64 value */
	static const long long opt_int64max = std::numeric_limits<long long>::max();
	/*	Minimum int64 value */
	static const long long opt_int64min = std::numeric_limits<long long>::min();
	
	/* collection index */
	typedef enum clc_type {
		clc_stb,  // stable
		clc_rem, // removed
		clc_add, // added
		clc_mod, // modified
		clc_invalid // invalid (for internal use only)
	} clc_type;
	
	// data structure to store information about a job
	typedef struct _job_type {
		long long ext_idx; // Job index in the external system
		long long dur; // Job duration
		double power_pred; // Job predicted power consumption
		double power; // Job current power consumption
		int frequency; // Job current execution frequency
		int application_type; // Job application type
		// requirements for each job unit
		std::vector<std::vector<long long > > reqs;
		// sum of requirements for each dimensions
		std::vector<long long> tot_reqs; 
		// mapping for each job unit
		std::vector<int> mapping;
		long long ewt; // Expected waiting time
		long long qet; // Time when the job entered the queue
		long long start; // Start time
		
		clc_type clc; // Main collection for the job
		int pos; // Job position in the collection
		
		// Default constructor
		_job_type() {}
		
		// Main constructor
		_job_type(long long ext_idx, int ndims)
			: ext_idx(ext_idx), dur(0), 
			  ewt(opt_int64min),
			  qet(opt_int64min),
			  start(opt_int64max) {
			tot_reqs.resize(ndims, 0);
		}
	} _job_type;
	
	/* Collection of all jobs in the problem. This may include invalid items */
	std::vector<_job_type> _jobs;
	/* Hole (invalid items) in the global list of jobs */
	std::vector<int> _holes;
	
	/* Notable job collections */
	const int n_job_clc = 4;
	std::vector<std::vector<int> > _job_clcs;
	
	/* negated Expected Waiting Time (for all jobs with non-default EWT) */
	std::multiset<long long> _neg_ewts;
	/* Total requirement for each resource dimension (over all jobs) */
	std::vector<long long> _tot_reqs;
	

	/* Data structure to define a resource */
	typedef struct _res_type {
		int ext_idx; // external index
		std::vector<long long> caps; // Resource capacities
		
		clc_type clc; // collection index
		int pos; // position in the collection
		
		_res_type(int ext_idx) : ext_idx(ext_idx) {}
	} _res_type;
	
	/*	All resources in the problem */
	std::vector<_res_type> _res;
	
	/* Notable resource collections */
	const int n_res_clc = 2;
	std::vector<std::vector<int> > _res_clcs;
	
	/*	Number of dimensions */
	int _ndims;
	
	/* Total capacity for each dimension */
	std::vector<long long> _tot_caps;
	
	/* problem version */
	long long _prb_version;
	
	/* problem stability state */
	bool _prb_stable;
	
private:
	inline void job_to_collection(int job_idx, clc_type clc_idx) {
		assert(job_idx >= 0 && job_idx < _jobs.size());
		assert(_jobs[job_idx].clc != clc_invalid);
		assert(clc_idx >= 0 && clc_idx < _job_clcs.size());
		DBG(int collection_size =_job_clcs[clc_idx].size();)

		/* Get a reference to the job and the relevant collections */
		_job_type& job = _jobs[job_idx];
		std::vector<int>& old_clc = _job_clcs[job.clc];
		std::vector<int>& new_clc = _job_clcs[clc_idx];
		/* Add the job to the new collection */
		new_clc.push_back(job_idx);
		/* Remove the job from the old collection (swap with the last
		   job in the collection) */
		_jobs[old_clc.back()].pos = job.pos;
		old_clc[job.pos] = old_clc.back();
		old_clc.pop_back();
		/* Update the job fields */
		job.clc = clc_idx;
		job.pos = new_clc.size() - 1;
		assert(_job_clcs[clc_idx].size() == collection_size + 1);
	}
	
	inline void res_to_collection(int res_idx, clc_type clc_idx) {
		assert(res_idx >= 0 && res_idx < _res.size());
		assert(clc_idx >= 0 && clc_idx < _res_clcs.size());
		/* Get a reference to the resource and to the relevant collections */
		_res_type& res = _res[res_idx];
		std::vector<int>& old_clc = _res_clcs[res.clc];
		std::vector<int>& new_clc = _res_clcs[clc_idx];
		/* Add the resource to the new collection */
		new_clc.push_back(res_idx);
		/* Remove the resource from the old collection (swap with the last
		   resource in the collection) */
		_res[old_clc.back()].pos = res.pos;
		old_clc[res.pos] = old_clc.back();
		old_clc.pop_back();
		/* Update the res fields */
		res.clc = clc_idx;
		res.pos = new_clc.size() - 1;
	}
};

/*	A simple constructor */
DispatchingProblem::DispatchingProblem(int ndims)
	: _ndims(ndims), _prb_version(0), _prb_stable(true) {
	_tot_reqs.resize(_ndims, 0); // Resize the total requirements
	_tot_caps.resize(_ndims, 0); // Resize the total capacities
	// There are four job collections: stable, added, removed, modified
	_job_clcs.resize(n_job_clc);
	// There are two resource collections: stable, removed
	_res_clcs.resize(n_res_clc);
}

int DispatchingProblem::addJob(long long ext_idx) {
	// Make the problem unstable and advance version
	if (_prb_stable) {
		_prb_stable = false;
		++_prb_version;
	}
	// Obtain the job id (re-use a hole or add a new job)
	int idx = -1;
	if (_holes.size() > 0) {
		idx = _holes.back();
		_holes.pop_back();
		// Store the main data structure for the job
		_jobs[idx] = _job_type(ext_idx, _ndims);
	}
	else {
		idx = _jobs.size();
		// Store the main data structure for the job
		_jobs.push_back(_job_type(ext_idx, _ndims));
	}

	// Store the job index in the "added" collection
	assert(clc_add < _job_clcs.size());
	_jobs[idx].pos = _job_clcs[clc_add].size();
	_jobs[idx].clc = clc_add;
	_job_clcs[clc_add].push_back(idx);
	// Return the index of the job
	return idx;
}

inline void DispatchingProblem::setQET(int job_idx, long long qet) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	// Obtain a reference to the target job
	_job_type& job = _jobs[job_idx];
	if (job.qet != qet) {
		// Make the problem unstable and advance version
		if (_prb_stable) {
			_prb_stable = false;
			++_prb_version;
		}
		// Move the job to the "modified" collection
		if (job.clc == clc_stb)
			job_to_collection(job_idx, clc_mod);
		// Assign the QET
		job.qet = qet;
	}
}

inline void DispatchingProblem::setEWT(int job_idx, long long ewt) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	assert(ewt > opt_int64min);
	// Obtain a reference to the target job
	_job_type& job = _jobs[job_idx];
	if (job.ewt != ewt) {
		// Make the problem unstable and advance version
		if (_prb_stable) {
			_prb_stable = false;
			++_prb_version;
		}
		// Move the job to the "modified" collection
		if (job.clc == clc_stb)
			job_to_collection(job_idx, clc_mod);
		// Assign the EWT
		job.ewt = ewt;
		// Update the _neg_ewt data structure (used to store the maximum EWT)
		_neg_ewts.insert(-ewt);
	}
}

inline void DispatchingProblem::setStart(int job_idx, long long start) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	// Obtain a reference to the target job
	_job_type& job = _jobs[job_idx];
	if (job.start != start) {
		// Make the problem unstable and advance version
		if (_prb_stable) {
			_prb_stable = false;
			++_prb_version;
		}
		// Move the job to the "modified" collection
		if (job.clc == clc_stb)
			job_to_collection(job_idx, clc_mod);
		// Assign the start time
		job.start = start;
	}
}

inline void DispatchingProblem::setDur(int job_idx, long long dur) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	// Obtain a reference to the target job
	_job_type& job = _jobs[job_idx];
	if (job.dur != dur) {
		// Make the problem unstable and advance version
		if (_prb_stable) {
			_prb_stable = false;
			++_prb_version;
		}
		// Move the job to the "modified" collection
		if (job.clc == clc_stb)
			job_to_collection(job_idx, clc_mod);
		// Assign the duration
		job.dur = dur;
	}
}

inline void DispatchingProblem::setPowerPred(int job_idx, double power_pred) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	// Obtain a reference to the target job
	_job_type& job = _jobs[job_idx];
	if (job.power_pred != power_pred) {
		// Make the problem unstable and advance version
		if (_prb_stable) {
			_prb_stable = false;
			++_prb_version;
		}
		// Move the job to the "modified" collection
		if (job.clc == clc_stb)
			job_to_collection(job_idx, clc_mod);
		// Assign the duration
		job.power_pred = power_pred;
	}
}

inline void DispatchingProblem::setPower(int job_idx, double power) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	// Obtain a reference to the target job
	_job_type& job = _jobs[job_idx];
	if (job.power != power) {
		// Make the problem unstable and advance version
		if (_prb_stable) {
			_prb_stable = false;
			++_prb_version;
		}
		// Move the job to the "modified" collection
		if (job.clc == clc_stb)
			job_to_collection(job_idx, clc_mod);
		// Assign the duration
		job.power = power;
	}
}

inline void DispatchingProblem::setFrequency(int job_idx, int frequency) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	// Obtain a reference to the target job
	_job_type& job = _jobs[job_idx];
	if (job.frequency != frequency) {
		// Make the problem unstable and advance version
		if (_prb_stable) {
			_prb_stable = false;
			++_prb_version;
		}
		// Move the job to the "modified" collection
		if (job.clc == clc_stb)
			job_to_collection(job_idx, clc_mod);
		// Assign the duration
		job.frequency = frequency;
	}
}

inline void DispatchingProblem::setApplicationType(int job_idx, int app_type) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	// Obtain a reference to the target job
	_job_type& job = _jobs[job_idx];
	if (job.application_type != app_type) {
		// Make the problem unstable and advance version
		if (_prb_stable) {
			_prb_stable = false;
			++_prb_version;
		}
		// Move the job to the "modified" collection
		if (job.clc == clc_stb)
			job_to_collection(job_idx, clc_mod);
		// Assign the duration
		job.application_type = app_type;
	}
}
				
inline void DispatchingProblem::addUnit(int job_idx,
								const std::vector<long long>& reqs) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(reqs.size() == _ndims);
	assert(_jobs[job_idx].clc == clc_add);
	// Make the problem unstable and advance version
	if (_prb_stable) {
		_prb_stable = false;
		++_prb_version;
	}
	// Obtain a reference to the target job
	_job_type& job = _jobs[job_idx];
	// Store the requirements of the new unit
	job.reqs.push_back(reqs);
	// Update the sum of requirements and the global statistics
	for (int j = 0; j < _ndims; ++j) {
		job.tot_reqs[j] += reqs[j]; // update the local sum
		_tot_reqs[j] += reqs[j]; // update the global sum
	}
}

inline void DispatchingProblem::mapUnits(int job_idx,
						const std::vector<int>& unit_mapping) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(unit_mapping.size() == _jobs[job_idx].reqs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	// Make the problem unstable and advance version
	if (_prb_stable) {
		_prb_stable = false;
		++_prb_version;
	}
	// Obtain a reference to the target job
	_job_type& job = _jobs[job_idx];
	// Store the mapping
	job.mapping = unit_mapping;
	// Move the job to the "modified" collection
	if (job.clc == clc_stb)
		job_to_collection(job_idx, clc_mod);
}

void DispatchingProblem::remJob(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	// Make the problem unstable and advance version
	if (_prb_stable) {
		_prb_stable = false;
		++_prb_version;
	}
	// Obtain a reference to the target job
	// Build a pointer to the job structure
	_job_type& job = _jobs[job_idx];
	// If the job is already removed, then do nothing
	if (job.clc == clc_rem) return; // EARLY EXIT
	// Move the job to the "removed" collection
	if (job.clc != clc_rem)
		job_to_collection(job_idx, clc_rem);
	// Update the total requirement values
	for (int j = 0; j < _ndims; ++j)
		_tot_reqs[j] -= job.tot_reqs[j];
	// Update the maximum EWT
	if (job.ewt != opt_int64min)
		_neg_ewts.erase(_neg_ewts.find(-job.ewt));
}

void DispatchingProblem::recallJob(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc == clc_rem);
	// Make the problem unstable and advance version
	if (_prb_stable) {
		_prb_stable = false;
		++_prb_version;
	}
	// Obtain a reference to the target job
	// Obtain a reference to the job
	_job_type& job = _jobs[job_idx];
	// Move the job to the "added" queue
	job_to_collection(job_idx, clc_add);
	// Update the total requirement values
	for (int j = 0; j < _ndims; ++j)
		_tot_reqs[j] += job.tot_reqs[j];
	// Update the maximum EWT
	if (job.ewt != opt_int64min)
		_neg_ewts.insert(-job.ewt);
}

void DispatchingProblem::commitJob(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	// Obtain a reference to the job
	_job_type& job = _jobs[job_idx];
	// The action depends on the job collection
	if (job.clc == clc_mod || job.clc == clc_add) {
		// If the job was in the "modified" or collection, just move it to the
		// "stable" collection
		job_to_collection(job_idx, clc_stb);
	}
	else if (job.clc == clc_rem) {
		// If the job was in the removed collection, then:
		// - remove the job from the collection
		std::vector<int>& clc = _job_clcs[clc_rem];
		clc[job.pos] = clc.back();
		clc.pop_back();
		// - mark the job as invalid
		job.clc = clc_invalid;
		// - add a new hole
		_holes.push_back(job_idx);
		// - if the number of holes is equalt to the number of overall jobs,
		//   then clear the whole collection
		if (_holes.size() == _jobs.size()) {
			_holes.clear();
			_jobs.clear();
		}
	}
	else{
		exit(0);
	}

	// If the job was already in the "stable" collection, then there is nothing
	// more to do
	// If all jobs and resources are stable, then the problem is stable
	if (_job_clcs[clc_stb].size() + _holes.size() == _jobs.size()) {
		_prb_stable = true;
	}
}


int DispatchingProblem::addRes(int ext_idx, const std::vector<long long>& caps) {
	// Advance the problem version (if the problem is stable)
	if (_prb_stable) ++_prb_version;
	// Obtain the (internal) resource id
	int idx = _res.size();
	// Store the main data structure for the resource
	_res.push_back(_res_type(ext_idx));
	_res[idx].caps = caps;
	// Update the total capacities
	for (int j = 0; j < _ndims; ++j)
		_tot_caps[j] += caps[j];
	// Store the resource index in the "stable" collection
	assert(clc_stb < _res_clcs.size());
	_res[idx].pos = _res_clcs[clc_stb].size();
	_res[idx].clc = clc_stb;
	_res_clcs[clc_stb].push_back(idx);
	// Return the index of the resource
	return idx;
}

void DispatchingProblem::remRes(int res_idx) {
	assert(res_idx >= 0 && res_idx < _res.size());
	// Advance the problem version (if the problem is stable)
	if (_prb_stable) ++_prb_version;
	// Obtain a reference to the resource data
	_res_type& res = _res[res_idx];
	// If the resoruce is already removed, then do nothing
	if (res.clc == clc_rem) return; //EARLY EXIT
	// Move to the "removed" collection
	res_to_collection(res_idx, clc_rem);
	// Update the total capacity
	for (int j = 0; j < _ndims; ++j)
		_tot_caps[j] -= res.caps[j];
}

void DispatchingProblem::recallRes(int res_idx) {
	assert(res_idx >= 0 && res_idx < _res.size());
	// Advance the problem version (if the problem is stable)
	if (_prb_stable) ++_prb_version;
	// Obtain a reference to the resource data
	_res_type& res = _res[res_idx];
	// If the resource is not removed, then do nothing
	if (res.clc != clc_rem) return; //EARLY EXIT
	// Move to the "stable" collection
	res_to_collection(res_idx, clc_stb);
	// Update the total capacity
	for (int j = 0; j < _ndims; ++j)
		_tot_caps[j] += res.caps[j];
}



inline const std::vector<int>& DispatchingProblem::getAddedJobs() {
	return _job_clcs[clc_add];
}

inline const std::vector<int>& DispatchingProblem::getModifiedJobs() {
	return _job_clcs[clc_mod];
}

inline const std::vector<int>& DispatchingProblem::getRemovedJobs() {
	return _job_clcs[clc_rem];
}

inline const std::vector<int>& DispatchingProblem::getStableJobs() {
	return _job_clcs[clc_stb];
}

inline const std::vector<int>& DispatchingProblem::getStableRes() {
	return _res_clcs[clc_stb];
}

//TODO check method inlining

inline const std::vector<int>& DispatchingProblem::getRemovedRes() {
	return _res_clcs[clc_rem];
}



inline int DispatchingProblem::getNDims() {
	return _ndims;
}


inline long long DispatchingProblem::getQET(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].qet;
}

inline bool DispatchingProblem::hasAssignedQET(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].qet != opt_int64min;
}

inline long long DispatchingProblem::getEWT(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].ewt;
}

inline long long DispatchingProblem::getStart(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].start;
}

inline bool DispatchingProblem::hasAssignedStart(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].start != opt_int64max;
}

inline long long DispatchingProblem::getDur(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].dur;
}

inline long long DispatchingProblem::getPowerPred(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].power_pred;
}

inline long long DispatchingProblem::getPower(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].power;
}

inline long long DispatchingProblem::getFrequency(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].frequency;
}

inline long long DispatchingProblem::getApplicationType(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].application_type;
}

inline long long DispatchingProblem::getJobExtIdx(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].ext_idx;
}

inline long long DispatchingProblem::getReq(int job_idx, int unit_idx, int dim_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	assert(unit_idx >= 0 && unit_idx < _jobs[job_idx].reqs.size());
	assert(dim_idx >= 0 && dim_idx < _ndims);
	return _jobs[job_idx].reqs[unit_idx][dim_idx];
}

inline long long DispatchingProblem::getReq(int job_idx, int dim_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	assert(dim_idx >= 0 && dim_idx < _ndims);
	return _jobs[job_idx].tot_reqs[dim_idx];
}

inline int DispatchingProblem::getNUnits(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	return _jobs[job_idx].reqs.size();
}

inline bool DispatchingProblem::isMapped(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	return _jobs[job_idx].mapping.size() > 0;
}

inline int DispatchingProblem::getMapping(int job_idx, int unit_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(unit_idx >= 0 && unit_idx < _jobs[job_idx].reqs.size());
	return _jobs[job_idx].mapping[unit_idx];
}

inline bool DispatchingProblem::isJobAdded(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].clc == clc_add;
}

inline bool DispatchingProblem::isJobModified(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].clc == clc_mod;
}

inline bool DispatchingProblem::isJobRemoved(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].clc == clc_rem;
}

inline bool DispatchingProblem::isJobStable(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	assert(_jobs[job_idx].clc != clc_invalid);
	return _jobs[job_idx].clc == clc_stb;
}

inline bool DispatchingProblem::isJobValid(int job_idx) {
	assert(job_idx >= 0 && job_idx < _jobs.size());
	return _jobs[job_idx].clc != clc_invalid;
}


long long DispatchingProblem::getCap(int res_idx, int dim_idx) {
	assert(res_idx >= 0 && res_idx < _res.size());
	assert(dim_idx >= 0 && dim_idx < _ndims);
	return _res[res_idx].caps[dim_idx];
}

long long DispatchingProblem::getResExtIdx(int res_idx) {
	assert(res_idx >= 0 && res_idx < _res.size());
	return _res[res_idx].ext_idx;
}

bool DispatchingProblem::isResStable(int res_idx) {
	assert(res_idx >= 0 && res_idx < _res.size());
	return _res[res_idx].clc == clc_stb;
}

bool DispatchingProblem::isResRemoved(int res_idx) {
	assert(res_idx >= 0 && res_idx < _res.size());
	return _res[res_idx].clc == clc_rem;
}


inline long long DispatchingProblem::getMaxEWT() {
	return -(*_neg_ewts.begin());
}

inline long long DispatchingProblem::getReq(int dim_idx) {
	assert(dim_idx >= 0 && dim_idx < _ndims);
	return _tot_reqs[dim_idx];
}

inline long long DispatchingProblem::getCap(int dim_idx) {
	assert(dim_idx >= 0 && dim_idx < _ndims);
	return _tot_caps[dim_idx];
}

inline int DispatchingProblem::getNInvalidJobs() {
	return _holes.size();
}

inline bool DispatchingProblem::isProblemStable() {
	return _prb_stable;
}

inline long long DispatchingProblem::getProblemVersion() {
	assert(_job_clcs[clc_add].size() + _job_clcs[clc_mod].size() 
            + _job_clcs[clc_rem].size() + _holes.size() + 
            _job_clcs[clc_stb].size() == _jobs.size());
	return _prb_version;
}

inline void setMaxPower(long long maxPower){

}

} // of the namespace optimization_core

#undef DBG
#undef DBGOUT

#endif
