#ifndef SDVFSSCHEDULER_HPP_
#define SDVFSSCHEDULER_HPP_

#include <algorithm>
#include <map>
#include <Optimizer.hpp>
#include <FixedStartSchedule.hpp>
#include <Rankers.hpp>

using namespace std;

// =============================================================================
// = Define the debug macros                                                   =
// =============================================================================

// comment/decomment this to enable debug output for this file
// #define DEBUG_SDVFSSCHEDULER_HPP

#ifndef NDEBUG
	#ifdef DEBUG_SDVFSSCHEDULER_HPP
		#define DBG(X) X
		#define DBGOUT (cerr << "[sdvfs-solver] ")
	#else
		#define DBG(X)
	#endif
#else
	#define DBG(X)
#endif


namespace optimization_core {

// =============================================================================
// = StaticDVFSingScheduler class                                                =
// =============================================================================

/* Class to solve a dispatching problem via static DVFS scheduling - 
 * 'static' refers to the fact that once the job's frequency has been decided
 * it won't change during its execution */
class StaticDVFSScheduler {

private:	

	// data structure to store information about a job
	typedef struct job_info_ {
        long long job_idx;
		int dur; // Job duration (can be modified)
		int base_dur; // Job original duration (unmodified)
		long long ewt; // Expected waiting time
		long long qet;
        double power_pred;  // Predicted Consumed Power
        double cur_power;  // Current consumed power - may vary
		// requirements for each job unit
		vector< vector<long long > > reqs;
		// sum of requirements for each dimensions
		vector<long long> tot_reqs; 
        // nodes mapping
        vector<int> node_mapping;
        long long start_time;  // when job execution begins
        long long end_time;    // estimated end time (may change)
        long long node_type;   // type of used nodes (we assume no mixed nodes)
        int application_type; // 0: average, 1: cpu-intensive, 2: mem-intensive
        int frequency;
        long long energy;  
        /* execution history is used to keep track of the different power 
         * consumptions that a job can have due to changing frequency.
         * It's a list of (power, duration) which can be summed to 
         * return the total energy consumed by a job. If the list is empty
         * then no frequency change happened, therefore the total energy is
         * simply power * duration. 
         * The final energy of a job can be computed when noticing its 
         * end-time event.      */
         vector< pair<double, int> > exec_hist; // track the execution hist
	} job_info_;

    /* Internal structure to keep track of the problem */
    map<long long, job_info_> internal_prb_;

    /* Assumption: all nodes with higher id are 3.1GHz nodes,
     * lower ids imply 2.1GHz nodes    */
    int _nodeType_threshold;

    /* Create the internal representation of the problem  */
    void init_internal_prb(DispatchingProblem* prb);

	/* Trivial comparison function, included for testing purpose */
	static bool fake_cmp(const int j0, const int j1) {
		return j0 < j1;
	}

    /* Structure used to store the results of 
     * the frequency selection
     * Will be applied when/if the job is actually started */
    typedef struct freq_change_res_{
        int job_idx;
        int frequency;
        double power;
        long long duration;
    } freq_change_res_;

    /* Check if adding a job in the system would exceed the power cap 
     * return true if the job can fit without violating the 
     * constraint, false otherwise  */
    /* This implementation is based not on real
     * job power measures but rather on the estimated
     * power consumption */
    bool checkPower_predJobs(
            DispatchingProblem* prb, 
            int maxPower, 
            vector<int>& running_jobs,
            int job_idx,
            int node_idx, 
            int& hypotetical_sys_power);

    /* Check if adding a job in the system would exceed the power cap 
     * return true if the job can fit without violating the 
     * constraint, false otherwise. This version must be called _only_
     * after a power reduction imposed by DVFS and implemented through a 
     * RAPL-like mechanism   */
    bool checkPower_predJobs_afterDVFS(
            DispatchingProblem* prb, 
            int maxPower, 
            vector<int>& running_jobs,
            int job_idx,
            int node_idx, 
            freq_change_res_ fcr,
            int& hypotetical_sys_power);

    /* This is not exactly DVFS, the frequencies are not directly
     * changed, it's more a RAPL-like implementation, we impose
     * that a certain power budget is not exceeded. 
     * In previous implementations we directly changed the 
     * frequencies, but they were to too tightly related to 
     * our target system Eurora     */
    freq_change_res_ set_frequency_raplLike(
            DispatchingProblem* prb,
            vector<int>& running_jobs, 
            long long cur_time,
            int maxPower,
            int job_idx,
            int node_idx);

    /* RAPL implementation with the power-duration model factor 
     * dependent on the application type. We compute the perfModel
     * factor for each job the first time the function is called.
     * Such values are stored for future calls (the perfModel
     * factor of a job is constant)   */
    map<long long, double> jobs_perfModel_factors_;
    double compute_job_perfModel_factor(
        long long job_idx, int application_type);


    /* Commit the selected frequency, notify
     * everything that needs to be notified  */
    void commit_frequency(
            DispatchingProblem* prb,
            freq_change_res_ fcr);

	/* parameters to compute the consumed energy
     * Based on the power consumption of Eurora supercomputer components 
     * (expressed in Watt)     */

	double CPU_IDLE = 20.0;
	double GPU_IDLE = 12.0;
	double CPU_21 = 20.0;
	double COEFF_ANG = 6.25;
	double GPU_ACTIVE = 50.0;
	double MIC = 115.0;
	double CPU_31 = 20.0;
	double CPU_31_S = 60.0;
	double COEFF_ANG_31 = 10.6;

    /* Minimum duration of jobs considered when computing BSLD.
     * In seconds.
     * Must be equal to the one defined in OnlineDispatcher  */
    int bsld_threshold_ = 60;

    /* Define the aggressiveness of the DVFS.
     * See "Optimizing Job Performance Under a Given Power Constraint
     * In HPC Centers" Etinski et Al. for details  */
    int bsld_condition_th_ = 5;
    int bsld_condition_lower_ = 2;
    int bsld_condition_upper_ = 10;
    double p_lower_ratio_ = 0.25;
    double p_upper_ratio_ = 0.75;

	// custom enumeration to divide jobs in arriving/startin/ending
	typedef enum _job_evt_type {
		job_arriving,
		job_starting,
		job_ending
	} _job_evt_type;

	// custom type to store the jobs that arrive/start/end at certain event
	typedef struct _job_evt {
		int job_idx;
		_job_evt_type type;
	} _job_evt;

	/* map with the ending jobs (this is needed to find quickly the next
	 time point to process). Iterators of the map container stay valid
	 even if a new element is inserted (this is important in the algorithm)
	 NOTE: jobs that already have a start time appear with negative (and
	 shifted) index    */
	map<long long, vector<_job_evt> > events_;

    // length of the backfill queue (PBS backfilling)
    int backfill_q_length_ = 10;

    /* Before scheduling jobs can be sorted.
     * 0: Not sorted (FCFS)
     * 1: Sorted by increasing Wall Time  */
    int sorting_mode_ = 0;

    /* The frequency can change in discrete values (0),
     * or implemented in a RAPL-like mechanism (1).  */
    int frequency_scaling_mode_;

    // with RAPL-like frequency scaling 
    double perfModel_factor_;

    /* Decide the BSLD Threshold dynamically.
     * See "Optimizing Job Performance Under a Given Power Constraint
     * In HPC Centers" Etinski et Al. for details  */
    int decide_bsld(int power_cap, int current_sys_power);

public:
	/* Build a fixed-start schedule using EASY backfilling. */
	void schedule_easy(DispatchingProblem* prb,
					FixedStartSchedule* sched,
                    JobResRanker& jnr,
					ScoreRanker& res_sr,
                    ScoreRanker& j_walltTime_ranker,
					long long ref_time,
					long long maxPower,
                    int frequency_scaling_mode,
                    double perfModel_factor);

	/* Build a fixed-start schedule using PBS backfilling. */
	void schedule_pbs(DispatchingProblem* prb,
					FixedStartSchedule* sched,
                    JobResRanker& jnr,
					ScoreRanker& res_sr,
                    ScoreRanker& j_walltTime_ranker,
					long long ref_time,
					long long maxPower,
                    int frequency_scaling_mode,
                    double perfModel_factor);

    /* Try to map a job on the system */
    bool fitJob(DispatchingProblem* prb,
                FixedStartSchedule* sched,
                vector<int>& ranked_res,
                long long cur_time,
                long long maxPower,
                vector<long long>& usgs,
                vector<int>& running_jobs,
                int job_idx,
                bool apply_change
            );

    /* Print system usgs (Debug)  */
    void printUsgs(DispatchingProblem* prb,
            vector<long long>& usgs,
            int sel_node,
            int sel_res);
};


// =============================================================================
// = Optimizer subclass based on StaticDVFSScheduler                             =
// =============================================================================

class StaticDVFSSchedulingOptimizer: public Optimizer {
public:
	/* A simple constructor */
	StaticDVFSSchedulingOptimizer(int ndims,
			CompatibilityChecker* cmp_chk,
			IntervalManager* int_man,
			long long maxPower,
            int frequency_scaling_mode,
            double perfModel_factor);
	
	/*	A simple destructor */
	virtual ~StaticDVFSSchedulingOptimizer() {}
	
	/* Obtain the next job that should be dispatched */
	virtual bool nextJob(long long ref_time,
						long long& job_idx,
						std::vector<int>& unit_mapping,
                        int& frequency);
				
	/* Get the start time of a job in the current proactive schedule. */
	virtual long long getSchedStart(long long job_idx);

	/* Get the end time of a job in the current proactive schedule. */
	virtual long long getSchedEnd(long long job_idx);
	
	/* Write the current schedule to an output stream */
	virtual void writeSched(ostream& out);

	/* Return the current problem version (DEBUG purpose) */
	virtual long long getProblemVersion();

    /* Set the power budget constraint - This method is 
     * useful if we need to change the power cap during online
     * dispatching */
    void setMaxPower(long long maxPower);

    /* Set the jobs perfModels factors.
     * Method used to synchronize the internal values of
     * the optimizers with different approach.
     * (Basically we want DVFS with RAPL to use the same
     * factors as pure RAPL)     */
    void setJobsPerfModelFactors(
            map<long long, double> jobs_perfModel_factors);
 
private:
    map<long long, double> jobs_perfModel_factors_;

	/* Unscheduled jobs, sorted by increasing start time. */
	map<long long, vector<long long> > _jobs_by_start;
	
	/* Iterator pointing to the next job to be scheduled */
	map<long long, vector<long long> >::iterator _nexttime_it;
	vector<long long>::iterator _nextjob_it;
	
	
	/* reference version of the DispatchingProblem */
	long long _ref_prb_version;

	/* Fixed start schedule */
	FixedStartSchedule _sched;

	/* Power Capping */
	long long _maxPower;

    /* The frequency can change in discrete values (0),
     * or implemented in a RAPL-like mechanism (1).  */
    int frequency_scaling_mode_;
    double perfModel_factor_;

	/* StaticDVFSScheduler instance */
	StaticDVFSScheduler _solver;

    /* Type of backfilling policy.
     * 0: PBS
     * 1: EASY      */
    int backfill_mode_ = 1;
};


// =============================================================================
// = Method implementations (StaticDVFSScheduler)                                =
// =============================================================================

void StaticDVFSScheduler::init_internal_prb(DispatchingProblem* prb){
    
   /* Assumption dependent on node given as input:
    * the nodes with lower ids (first half) are low_freq,
    * nodes ids in the second half are high_freq. 
    * This reflect the Eurora supercomputer, that had half of the nodes
    * (from 1 to 32) equipped with MICs and CPUs operating at 2.1GHz while 
    * the other half of nodes had GPUs and CPUs at 3.1GHz  */
    _nodeType_threshold = prb->getStableRes().size() / 2;

    for(int i : prb->getStableJobs()){
        job_info_ ji;
        int ext_id = prb->getJobExtIdx(i);

        ji.job_idx = i;
        ji.application_type = prb->getApplicationType(i);
        ji.frequency = prb->getFrequency(i);
        ji.ewt = prb->getEWT(i);
        ji.cur_power = prb->getPower(i);
        ji.power_pred = prb->getPowerPred(i);
        ji.qet = prb->getQET(i);

        assert(prb->getDur(i) >= 0);
        ji.dur = prb->getDur(i);
        ji.base_dur = prb->getDur(i);

        internal_prb_[i] = ji;
    }
}

double StaticDVFSScheduler::compute_job_perfModel_factor(
        long long job_idx, int application_type){
   auto it = jobs_perfModel_factors_.find(job_idx);
   // the job has already a perfModel factor
   if(it != jobs_perfModel_factors_.end())
       return it->second;
   // we compute the new perfModel factor
   else{
       /* Assumptions: the factor can range between 0 and 1;
        * the factor is a random value extracted from a normal
        * distribution. Application type has its own distribution 
        * mean (std are the same for all application types)  */
       random_device rd;
       mt19937 e2(rd());
       // normal distributions for each application type
       normal_distribution<double> dist_CPUbound(0.9, 0.2);
       normal_distribution<double> dist_QE(0.7, 0.2);
       normal_distribution<double> dist_MEMbound(0.5, 0.2);
       double perfModel_factor;
       switch(application_type){
           case 0:
               perfModel_factor = dist_QE(e2);
               break;
           case 1:
               perfModel_factor = dist_CPUbound(e2);
               break;
           case 2:
               perfModel_factor = dist_MEMbound(e2);
               break;
           default:
               perfModel_factor = dist_QE(e2);
               break;
       }

       if(perfModel_factor > 1)
           perfModel_factor = 1;
       if(perfModel_factor < 0)
           perfModel_factor = 0;

       jobs_perfModel_factors_[job_idx] = perfModel_factor;

       return perfModel_factor;
   }
}

bool StaticDVFSScheduler::checkPower_predJobs(
        DispatchingProblem* prb, int maxPower,
        vector<int>& running_jobs, int job_idx,
        int node_idx, int& hypotetical_sys_power){
    bool ok = true;

    // compute current system power
    int system_power = 0;
    for(int i : running_jobs)
        /* running jobs with an assigned start
         and their power is (should) be already
         modified depending on the frequency */
        system_power += internal_prb_[i].cur_power;

    /* the considered job has not been started already:
     its power may change depending on the frequency */
    double job_power = internal_prb_[job_idx].power_pred;

    // check if adding the new job would violate the power cap
    if(system_power + job_power > maxPower)
        ok = false;

    // return the hypothetical system power in any case (later uses)
    hypotetical_sys_power = system_power + job_power;

    return ok;
}

bool StaticDVFSScheduler::checkPower_predJobs_afterDVFS(
       DispatchingProblem* prb, int maxPower, 
       vector<int>& running_jobs,
       int job_idx, int node_idx, 
       StaticDVFSScheduler::freq_change_res_ fcr,
       int& hypotetical_sys_power){
    bool ok = true;

    // compute current system power
    int system_power = 0;
    for(int i : running_jobs)
        /* running jobs with an assigned start
         * and their power is (should) be already
         * modified depending on the frequency */
        
        system_power += internal_prb_[i].cur_power;


    /* the considered job has not been started already:
     * its power may change depending on the DVFS (not committed yet) */
    //double job_power = prb->getPower(job_idx);
    double job_power = fcr.power;

    // check if adding the new job would violate the power cap
    if(system_power + job_power > maxPower)
        ok = false;

    // return the hypothetical system power in any case (later uses)
    hypotetical_sys_power = system_power + job_power;

    return ok;
}

StaticDVFSScheduler::freq_change_res_ StaticDVFSScheduler::set_frequency_raplLike(
        DispatchingProblem* prb,
        vector<int>& running_jobs, 
        long long cur_time,
        int maxPower,
        int job_idx,
        int node_idx){
    freq_change_res_ res;

    double job_perfModel_factor;
    // if perfModel_factor_ == -1 this means that each job has its own factor
    if(perfModel_factor_ == -1)
        job_perfModel_factor = compute_job_perfModel_factor(job_idx, 
                internal_prb_[job_idx].application_type);
    else
        job_perfModel_factor = perfModel_factor_;

    double min_power = 1;
    int new_dur, new_power;

    // compute current system power
    int system_power = 0;
    for(int i : running_jobs)
        /* running jobs with an assigned start
         * and their power is (should) be already
         * modified depending on the frequency  */
        system_power += internal_prb_[i].cur_power;

    /* We don't choose a frequency. We try to decrease the power consumption (until
     * we reach a lower bound) and check if the resulting increase duration
     * still satisfies the BSLD condition */
    double job_avail_power = maxPower - system_power;
    double dur_multiplier, power_multiplier;

    power_multiplier = job_avail_power / internal_prb_[job_idx].power_pred;
    if(power_multiplier > 1 || power_multiplier <= 0)
        power_multiplier = 1;

    if(job_perfModel_factor == 0)
        dur_multiplier = 1;
    else
        dur_multiplier = 1 / power_multiplier * job_perfModel_factor;
    if(power_multiplier == 1)
        dur_multiplier = 1;
    if(dur_multiplier < 1)
        dur_multiplier = 1;

    double bsld = 1;

    new_dur = internal_prb_[job_idx].base_dur * dur_multiplier;

    assert(new_dur >= 0);
    new_power = internal_prb_[job_idx].power_pred * power_multiplier;
    if(new_power == 0)
        new_power = 1;
    long long wt = cur_time - internal_prb_[job_idx].qet;

    /* The threshold of the BSLD condition is dynamically computed */
    int dyn_th = decide_bsld(maxPower, system_power + new_power);
    bsld_condition_th_ = dyn_th;

    // bsld with the current frequency
    bsld = max((double)1,
             (double) (wt + new_dur) / 
             max(bsld_threshold_, internal_prb_[job_idx].dur));

    // fake frequency value, RAPL implementation doesn't use discrete freqs
    int sel_freq;

    /* if the predicted BSLD is larger than the threshold the BSLD 
     condition is not satisfied and the job power cannot be modified */
    if(bsld >= bsld_condition_th_){
        sel_freq = internal_prb_[job_idx].frequency;
        new_power = internal_prb_[job_idx].power_pred;
        new_dur = internal_prb_[job_idx].base_dur;
    }

    sel_freq = 1;
    res.frequency = sel_freq;
    res.duration = new_dur;
    res.power = new_power;
    res.job_idx = job_idx;
    return res;
}

int StaticDVFSScheduler::decide_bsld(int power_cap,
        int current_sys_power){
    int new_bsld = 2;

    /* The BSLD condition depends on the current system power consumption */
    // power < than 1/4 power cap, no job will be slowed
    if(current_sys_power < (int) power_cap * p_lower_ratio_)
        new_bsld = 0;
    // power > than 3/4 power cap, many jobs will be slowed
    else if(current_sys_power > (int) power_cap * p_upper_ratio_)
        new_bsld = bsld_condition_upper_;
    // 1/4 p_cap <= power <= 3/4 p_cap, some job will be slowed
    else
        new_bsld = bsld_condition_lower_;

    return new_bsld;
}

void StaticDVFSScheduler::commit_frequency(
        DispatchingProblem* prb,
        StaticDVFSScheduler::freq_change_res_ fcr){

    assert(fcr.duration > 0);
    assert(fcr.frequency > 0);
    assert(fcr.power > 0);

    internal_prb_[fcr.job_idx].frequency = fcr.frequency;
    internal_prb_[fcr.job_idx].dur = fcr.duration;
    internal_prb_[fcr.job_idx].cur_power = fcr.power;
}

bool StaticDVFSScheduler::fitJob(DispatchingProblem* prb,
			FixedStartSchedule* sched,
            vector<int>& ranked_res,
			long long cur_time,
			long long maxPower,
            vector<long long>& usgs,
            vector<int>& running_jobs,
            int job_idx,
            bool apply_change
        ){
    bool ok = true;

	int ndims = prb->getNDims();

	// Data structure to store changes to the resource usage
	typedef struct {
		int res;
		int dim;
		long long val;
	} _updated_usg_type;

    freq_change_res_ fcr;
    
	// check if all job units can be dispatched
	vector<int> unit_mapping; // node where each unit is mapped
	vector<_updated_usg_type> usgs_changes; // usage changes
	for (int u = 0; u < prb->getNUnits(job_idx); ++u) {
		DBG(DBGOUT << "trying to map unit " << u << endl;)
		// loop over all ranked resources
		ok = true;
		for (auto k : ranked_res) {
			// check if there are enough resources available
			ok = true;

			for (int j = 0; j < ndims; ++j) {
				long long cap = prb->getCap(k, j);
				long long req = prb->getReq(job_idx, u, j);
				ok = ok && (usgs[k * ndims + j] + req <= cap);
			}

            int frequency;

            /* First try to fit the job at max frequency */
            int hyp_sys_power = 0;
            bool check_result = checkPower_predJobs(
                    prb, maxPower, running_jobs, 
                    job_idx, k, hyp_sys_power);

            /* If the power check fails we can try to employ DVFS */
            if(!check_result){
                fcr = set_frequency_raplLike(prb, running_jobs, 
                         cur_time, maxPower, job_idx, k);
                frequency = fcr.frequency;

                check_result = checkPower_predJobs_afterDVFS(
                       prb, maxPower, running_jobs, 
                       job_idx, k, fcr, hyp_sys_power);

                /* if even with the (possibly) reduced frequency the power
                 * condition is not met the job won't be started */
                if(!check_result){
                    ok = false;
                }
            }
            // even if the power check was ok still need to update support struct
            else{
                fcr.job_idx = job_idx;
                fcr.frequency = frequency;
                fcr.power = internal_prb_[job_idx].power_pred;
                fcr.duration = internal_prb_[job_idx].dur;
            }

			// if this is the case, then map the unit
			if (ok) {
				DBG(DBGOUT << "check OK, unit mapped on res " << k << endl;)
				// store the unit mapping
				unit_mapping.push_back(k);
				// update the usage
				for (int j = 0; j < ndims; ++j) {
					// Change the usage
					usgs[k * ndims + j] += prb->getReq(job_idx, u, j);
					// Store the change
					usgs_changes.push_back({k, j, prb->getReq(job_idx, u, j)});
					// print debug stuff
				}
				// stop considering the resources
				break;
			}
		}
		// if the unit could not be mapped, then abort the whole
		// job mapping
		if (!ok) {
			// revert the usage to the old values
			for (int cc = 0; cc < usgs_changes.size(); ++cc) {
				int res = usgs_changes[cc].res;
				int dim = usgs_changes[cc].dim;
				long long val = usgs_changes[cc].val;
				usgs[res * ndims + dim] -= val;
			}
			// move to the next job
			break;
		}
	}
	
	// if all job units have been mapped, than start the job
	if (unit_mapping.size() == prb->getNUnits(job_idx) &&
            apply_change) {
        // commit the frequency
        commit_frequency(prb, fcr);

		// store the end event 
		long long end_time = cur_time + internal_prb_[job_idx].dur;
        assert(end_time >= 0);
		auto it = events_.find(end_time);
		if (it != events_.end()) {
			// One more job in the ending job list
			it->second.push_back({job_idx, job_ending});
		}
		else {
			// Build a new ending job list, with only this job
			events_[end_time] = {{job_idx, job_ending}};
		}
	
		// update the schedule: map the job units
		for (int u = 0; u < unit_mapping.size(); ++u) {
			int k = unit_mapping[u];
			sched->mapUnit(job_idx, u, k);
		}
		// update the schedule: assign the start time
		sched->setStartTime(job_idx, cur_time);

        // update the schedule: assign the frequency
        sched->setFrequency(job_idx, fcr.frequency);
	}

    return ok;
}

// Implementation of the schedule function - EASY
void StaticDVFSScheduler::schedule_easy(DispatchingProblem* prb,
			FixedStartSchedule* sched,
            JobResRanker& jnr,
			ScoreRanker& res_sr,
            ScoreRanker& j_walltTime_ranker,
			long long ref_time,
			long long maxPower,
            int frequency_scaling_mode,
            double perfModel_factor) {
	assert(prb != NULL);
	assert(sched != NULL);
	assert(prb->isProblemStable());
	assert(ref_time >= 0);
	assert(maxPower > 0);

    frequency_scaling_mode_ = frequency_scaling_mode;
    perfModel_factor_ = perfModel_factor;

    init_internal_prb(prb);

	// Cache the number of dimensions
	int ndims = prb->getNDims();

    // list of ranked jobs
    vector<int> ranked_jobs;
    
    // list of ranked resources
    vector<int> ranked_res;

    // list of running jobs and their end time
    map<int, long long> running_jobs;
    vector<int> sorted_running_jobs;

    vector<int> extIdx_running_jobs;

    // list of jobs in queue
    vector<int> waiting_jobs;

	// Compute the size of the resource pool
	int respool_size = prb->getRemovedRes().size() + prb->getStableRes().size();

	// current usage level on each resource, for each dimension
	// NOTE This is a flattened matrix
	// NOTE Initially, no resource is used
	vector<long long> usgs(respool_size * ndims, 0);

	// add an event at the reference time
    events_.clear();
	events_[ref_time] = {};
	
	// fill the list of ranked resources
	// NOTE this is a vector copy
	ranked_res = prb->getStableRes();

	// Fill the list of ranked jobs and add the job that already have a
	// start time to the event list
	for (int i : prb->getStableJobs()) {
		if (prb->hasAssignedStart(i)) {
			long long start_time = prb->getStart(i);
			auto it = events_.find(start_time);
			if (it != events_.end())
				it->second.push_back({i, job_starting});
			else events_[start_time] = {{i, job_starting}};
			// Check if an event for the end time exists. If this is the
			// case, register the jobs as "ending" at the event. Otherwise, 
			// build a new job vector for the event containing only "i".
			long long end_time = start_time + internal_prb_[i].dur;
			it = events_.find(end_time);
			if (it != events_.end())
				it->second.push_back({i, job_ending});
			else events_[end_time] = {{i, job_ending}};

            auto itrj = running_jobs.find(i);
            if (itrj == running_jobs.end())
                running_jobs[i] = end_time;
            sorted_running_jobs.push_back(i);
            extIdx_running_jobs.push_back(
                    prb->getJobExtIdx(i));
		}
		else {
			// Jobs that do not yet have a start time must be scheduled
			ranked_jobs.push_back(i);
            waiting_jobs.push_back(i);
		}
	}

    // Sort the resources
    sort(ranked_res.begin(), ranked_res.end(), res_sr);

	// if all jobs have already been scheduled (event-loop termination condition)
	// the current schedule can be derived from the previuos schedule (prb)
	if(ranked_jobs.size() == 0){
		for (int i : prb->getStableJobs()) {
			long long start_time = prb->getStart(i);
			assert(prb->isMapped(i));
				for (int u = 0; u < prb->getNUnits(i); ++u) {
					int k = prb->getMapping(i, u);
					sched->mapUnit(i, u, k);
				}
			sched->setStartTime(i, start_time);
		}
	}

	// number of jobs that will arrive in the future (for the oracle algorithm)
	long long ndelayed = 0;

	long long prev_time = 0;

    // loop until:
    // 1) there are time points to process
    // 2) there are jobs to dispatch
    // In practice the loop should always end because of condition 2
    for (auto evt : events_) {
		if (ranked_jobs.size() + ndelayed == 0) {
			DBG(DBGOUT << "no more jobs to schedule" << endl;)
			break; // EARLY EXIT
		}

		// access the current time
		long long cur_time = evt.first;
		DBG(DBGOUT << "=== processing time " << cur_time << endl;)

		// Update the resource usage to take into account the jobs that
		// end or start at the event time
		for (_job_evt jdesc : evt.second) {
			int job_idx = jdesc.job_idx;
			// If this was a pre-scheduled, starting job, the we need to
			// store the schedulign decision in the output data structure
			if (jdesc.type == job_starting) {
				DBG(DBGOUT << "pre-scheduled job: " << job_idx << endl;)
				// Map the job units
				assert(prb->isMapped(job_idx));
				for (int u = 0; u < prb->getNUnits(job_idx); ++u) {
					int k = prb->getMapping(job_idx, u);
					sched->mapUnit(job_idx, u, k);
				}

				// Store the start time
				assert(prb->getStart(job_idx) == cur_time);
				sched->setStartTime(job_idx, cur_time);

			}
			else if (jdesc.type == job_arriving) {
				DBG(DBGOUT << "arriving job: " << job_idx << endl;)
				// An arriving job is treated as a schedulable job from now on
				ranked_jobs.push_back(job_idx);
				// One less delayed job
				--ndelayed;
			}
			else {
				DBG(DBGOUT << "ending job: " << job_idx << endl;)

                // delete the job from the list of running ones
                running_jobs.erase(job_idx);

                auto it2 = find(sorted_running_jobs.begin(),
                        sorted_running_jobs.end(), job_idx);
                if(it2 != sorted_running_jobs.end())
                    sorted_running_jobs.erase(
                            remove(
                                sorted_running_jobs.begin(), 
                                sorted_running_jobs.end(), job_idx), 
                            sorted_running_jobs.end());
			}
				
			// Update the resource usage
			if (jdesc.type == job_starting || jdesc.type == job_ending) {
				for (int kk = 0; kk < sched->getNUsedRes(job_idx); ++kk) {
					int k = sched->getUsedRes(job_idx, kk);
					for (int j = 0; j < ndims; ++j) {
						long long delta = sched->getCumulativeReq(job_idx, kk, j);
						if (jdesc.type == job_starting)
							usgs[k * ndims + j] += delta;
						else
							usgs[k * ndims + j] -= delta;						
					}
				}	
			}
		}
		
		// Move to the next event if the reference time (i.e. "now") has
		// not yet been reached.
		if (cur_time < ref_time) {
			DBG(DBGOUT << "moving on (this is a past event)" << endl;)
			continue; // EARLY EXIT
		}
		
		if (ranked_jobs.size() == 0) {
			DBG(DBGOUT << "moving on (no waiting jobs, " << 
                    ndelayed << " delayed jobs)" << endl;)
			continue;
		}

		// re-compute the job scores and re-sort the list
		DBG(DBGOUT << "sorting jobs" << endl;)
        switch(sorting_mode_){
            case 0:
                // no sorting
                break;
            case 1:
                // sort by wall time
                sort(ranked_jobs.begin(), 
                        ranked_jobs.end(), j_walltTime_ranker);
                break;
        }

        /* Jobs which need to be scheduled but have not yet
         * been considered in the current scheduling cycle  */
        vector<int> not_considered_jobs;
        not_considered_jobs = ranked_jobs;

		// prepare the new list of ranked jobs
		vector<int> new_ranked_jobs;

        /* maximal end time for backfilled jobs */
        long long max_end_backfilled = -1;

		// try to dispatch each job
		for (int i : ranked_jobs) {
			DBG(DBGOUT << "*** processing job " << i << endl;)
				
			// Skip the job if its QET has not yet been reached (this is
			// possible in off-line tests where knowledge of the future is
			// available)
			if (prb->hasAssignedQET(i) && prb->getQET(i) > cur_time) {
				// Add a new event at the arrival time, to be sure that it will
				// be processed
				long long qet = prb->getQET(i);
				auto it = events_.find(qet);
				if (it == events_.end())
					events_[qet] = {{i, job_arriving}};
				else
					it->second.push_back({i, job_arriving});
				// A job has been delayed
				++ndelayed;
				// move on

                auto itncj = find(not_considered_jobs.begin(),
                        not_considered_jobs.end(), i);
                if(itncj != not_considered_jobs.end())
                    not_considered_jobs.erase(
                            remove(
                                not_considered_jobs.begin(), 
                                not_considered_jobs.end(), i), 
                            not_considered_jobs.end());
				continue;
			}

            bool ok = fitJob(prb, sched, ranked_res, cur_time, maxPower,
                 usgs, sorted_running_jobs, i, true);

            // the job fits
            if(ok){
                long long end_time = cur_time + internal_prb_[i].dur;
                // add the job to running jobs sets
                auto itrj = running_jobs.find(i);
                if (itrj == running_jobs.end())
                    running_jobs[i] = end_time;
                sorted_running_jobs.push_back(i);

                // delete the job from the list of those to be considered yet
                auto itncj = find(not_considered_jobs.begin(),
                        not_considered_jobs.end(), i);
                if(itncj != not_considered_jobs.end())
                    not_considered_jobs.erase(
                            remove(
                                not_considered_jobs.begin(), 
                                not_considered_jobs.end(), i), 
                            not_considered_jobs.end());
            }
		
            /* EASY-backfilling: try to start jobs until they fit
             * when the first one doesn't fit try to backfill  */
			/* if the job could not be dispatched, then it will be
			 * considered in future iterations  */
			else {

				// Store the job in the processing list of the next event
				new_ranked_jobs.push_back(i);

                // delete the job from the list of those to be considered yet
                auto itncj = find(not_considered_jobs.begin(),
                        not_considered_jobs.end(), i);
                if(itncj != not_considered_jobs.end())
                    not_considered_jobs.erase(
                            remove(
                                not_considered_jobs.begin(), 
                                not_considered_jobs.end(), i), 
                            not_considered_jobs.end());

                /* Compute the minimal possible start for the unfitting job */
                // Sort the running jobs by increasing end times
                ScoreRanker rj_et_ranker = 
                    jnr.getJobEndTime_scoreRanker(running_jobs);
                sort(sorted_running_jobs.begin(), sorted_running_jobs.end(),
                        rj_et_ranker);

                vector<long long> future_usgs = usgs;

                vector<int> changing_running_jobs = sorted_running_jobs;
                for(int rj : sorted_running_jobs){
                    // when the jobs finishes can we start the one in queue?
                    
                    auto it2 = find(changing_running_jobs.begin(),
                        changing_running_jobs.end(), rj);
                    if(it2 != changing_running_jobs.end())
                        changing_running_jobs.erase(
                                remove(
                                    changing_running_jobs.begin(), 
                                    changing_running_jobs.end(), rj), 
                                changing_running_jobs.end());

                    // Update the future resource usage (current job rj ends)
                    for (int kk = 0; kk < sched->getNUsedRes(rj); ++kk) {
                        int k = sched->getUsedRes(rj, kk);
                        for (int j = 0; j < ndims; ++j) {
                            long long delta = sched->getCumulativeReq(rj, kk, j);
                            future_usgs[k * ndims + j] -= delta;						
                        }
                    }	

                    bool ok = fitJob(prb, sched, ranked_res, cur_time, maxPower,
                         future_usgs, changing_running_jobs, i, false);

                    if(ok){
                        max_end_backfilled = running_jobs[rj];
                        break;
                    }

                    // if the first job in queue did not fit we wait the 
                    // end of the following running job
                }

                /* Now we try backfilling jobs: EASY policy says that
                 * a job can run if it fits and its execution won't postpone 
                 * the first job on the waiting queue, i.e. 
                 * its end_time is smaller than max_end_backfilled  */
                for(int bfj : not_considered_jobs){

                    bool can_bf = true;
                    // check if its execution won't postpone the first job in Q
                    long long end_time = cur_time + internal_prb_[bfj].dur;
                    if(end_time >= max_end_backfilled){
                        can_bf = false;
                    }

                    // check the if it fits
                    else{
                        bool ok = fitJob(prb, sched, ranked_res, cur_time, maxPower,
                             usgs, sorted_running_jobs, bfj, true);

                        if(ok){
                            // add the job to running jobs sets
                            auto itrj = running_jobs.find(bfj);
                            if (itrj == running_jobs.end())
                                running_jobs[bfj] = end_time;
                            sorted_running_jobs.push_back(bfj);
                        }
                        else{
                            can_bf = false;
                        }
                    }

                    // if the job was not backfilled, next iteration
                    if(!can_bf)
                        new_ranked_jobs.push_back(bfj);
                }

                // stop scheduling when we found the first job that doesn't fit
                break;
			}
		} // end of the dispatching loop

		// update the list of ranked jobs
		ranked_jobs = new_ranked_jobs;

		prev_time = cur_time;
	} // end of the event loop	

	DBG(DBGOUT << "schedule computation done" << endl;)
}

// Implementation of the schedule function - PBS
void StaticDVFSScheduler::schedule_pbs(DispatchingProblem* prb,
			FixedStartSchedule* sched,
            JobResRanker& jnr,
			ScoreRanker& res_sr,
            ScoreRanker& j_walltTime_ranker,
			long long ref_time,
			long long maxPower,
            int frequency_scaling_mode,
            double perfModel_factor) {
	assert(prb != NULL);
	assert(sched != NULL);
	assert(prb->isProblemStable());
	assert(ref_time >= 0);
	assert(maxPower > 0);

    init_internal_prb(prb);

    frequency_scaling_mode_ = frequency_scaling_mode;
    perfModel_factor_ = perfModel_factor;

	// Cache the number of dimensions
	int ndims = prb->getNDims();

    // list of ranked jobs
    vector<int> ranked_jobs;
    
    // list of ranked resources
    vector<int> ranked_res;

    // list of running jobs and their end time
    map<int, long long> running_jobs;
    vector<int> sorted_running_jobs;

    // list of jobs in queue
    vector<int> waiting_jobs;

	// Compute the size of the resource pool
	int respool_size = prb->getRemovedRes().size() + prb->getStableRes().size();

	// current usage level on each resource, for each dimension
	// NOTE This is a flattened matrix
	// NOTE Initially, no resource is used
	vector<long long> usgs(respool_size * ndims, 0);

	// add an event at the reference time
    events_.clear();
	events_[ref_time] = {};
	
	// fill the list of ranked resources
	// NOTE this is a vector copy
	ranked_res = prb->getStableRes();

	// Fill the list of ranked jobs and add the job that already have a
	// start time to the event list
	for (int i : prb->getStableJobs()) {
		if (prb->hasAssignedStart(i)) {
			DBG(DBGOUT << "job " << i << " has already been scheduled" << endl;)
			long long start_time = prb->getStart(i);
			auto it = events_.find(start_time);
			if (it != events_.end())
				it->second.push_back({i, job_starting});
			else events_[start_time] = {{i, job_starting}};
			// Check if an event for the end time exists. If this is the
			// case, register the jobs as "ending" at the event. Otherwise, 
			// build a new job vector for the event containing only "i".
			long long end_time = start_time + internal_prb_[i].dur;
			it = events_.find(end_time);
			if (it != events_.end())
				it->second.push_back({i, job_ending});
			else events_[end_time] = {{i, job_ending}};

            auto itrj = running_jobs.find(i);
            if (itrj == running_jobs.end())
                running_jobs[i] = end_time;
            sorted_running_jobs.push_back(i);
		}
		else {
			DBG(DBGOUT << "job " << i << " must be scheduled" << endl;)
			// Jobs that do not yet have a start time must be scheduled
			ranked_jobs.push_back(i);
            waiting_jobs.push_back(i);
		}
	}

    // Sort the resources
    sort(ranked_res.begin(), ranked_res.end(), res_sr);

	// if all jobs have already been scheduled (event-loop termination condition)
	// the current schedule can be derived from the previuos schedule (prb)
	if(ranked_jobs.size() == 0){
		for (int i : prb->getStableJobs()) {
			long long start_time = prb->getStart(i);
			assert(prb->isMapped(i));
				for (int u = 0; u < prb->getNUnits(i); ++u) {
					int k = prb->getMapping(i, u);
					sched->mapUnit(i, u, k);
				}
			sched->setStartTime(i, start_time);
		}
	}

	// number of jobs that will arrive in the future (for the oracle algorithm)
	long long ndelayed = 0;

	long long prev_time = 0;

    // loop until:
    // 1) there are time points to process
    // 2) there are jobs to dispatch
    // In practice the loop should always end because of condition 2
    for (auto evt : events_) {
		if (ranked_jobs.size() + ndelayed == 0) {
			DBG(DBGOUT << "no more jobs to schedule" << endl;)
			break; // EARLY EXIT
		}
	
		// access the current time
		long long cur_time = evt.first;
		DBG(DBGOUT << "=== processing time " << cur_time << endl;)
	
		// Update the resource usage to take into account the jobs that
		// end or start at the event time
		for (_job_evt jdesc : evt.second) {
			int job_idx = jdesc.job_idx;
			// If this was a pre-scheduled, starting job, the we need to
			// store the schedulign decision in the output data structure
			if (jdesc.type == job_starting) {
				DBG(DBGOUT << "pre-scheduled job: " << job_idx << endl;)
				// Map the job units
				assert(prb->isMapped(job_idx));
				for (int u = 0; u < prb->getNUnits(job_idx); ++u) {
					int k = prb->getMapping(job_idx, u);
					sched->mapUnit(job_idx, u, k);
				}

				// Store the start time
				assert(prb->getStart(job_idx) == cur_time);
				sched->setStartTime(job_idx, cur_time);

			}
			else if (jdesc.type == job_arriving) {
				DBG(DBGOUT << "arriving job: " << job_idx << endl;)
				// An arriving job is treated as a schedulable job from now on
				ranked_jobs.push_back(job_idx);
				// One less delayed job
				--ndelayed;
			}
			else {
				DBG(DBGOUT << "ending job: " << job_idx << endl;)

                // delete the job from the list of running ones
                running_jobs.erase(job_idx);

                auto it2 = find(sorted_running_jobs.begin(),
                        sorted_running_jobs.end(), job_idx);
                if(it2 != sorted_running_jobs.end())
                    sorted_running_jobs.erase(
                            remove(
                                sorted_running_jobs.begin(), 
                                sorted_running_jobs.end(), job_idx), 
                            sorted_running_jobs.end());
			}
				
			// Update the resource usage
			if (jdesc.type == job_starting || jdesc.type == job_ending) {
				for (int kk = 0; kk < sched->getNUsedRes(job_idx); ++kk) {
					int k = sched->getUsedRes(job_idx, kk);
					for (int j = 0; j < ndims; ++j) {
						long long delta = sched->getCumulativeReq(job_idx, kk, j);
						if (jdesc.type == job_starting)
							usgs[k * ndims + j] += delta;
						else
							usgs[k * ndims + j] -= delta;						
					}
				}	
			}
		}
		
		// Move to the next event if the reference time (i.e. "now") has
		// not yet been reached.
		if (cur_time < ref_time) {
			DBG(DBGOUT << "moving on (this is a past event)" << endl;)
			continue; // EARLY EXIT
		}
		
		if (ranked_jobs.size() == 0) {
			DBG(DBGOUT << "moving on (no waiting jobs, " << 
                    ndelayed << " delayed jobs)" << endl;)
			continue;
		}

		// re-compute the job scores and re-sort the list
		DBG(DBGOUT << "sorting jobs" << endl;)
        switch(sorting_mode_){
            case 0:
                // no sorting
                break;
            case 1:
                // sort by wall time
                sort(ranked_jobs.begin(), 
                        ranked_jobs.end(), j_walltTime_ranker);
                break;
        }

        /* Jobs which need to be scheduled but have not yet
         * been considered in the current scheduling cycle  */
        vector<int> not_considered_jobs;
        not_considered_jobs = ranked_jobs;

		// prepare the new list of ranked jobs
		vector<int> new_ranked_jobs;

		// try to dispatch each job
		for (int i : ranked_jobs) {
			DBG(DBGOUT << "*** processing job " << i << endl;)
				
			// Skip the job if its QET has not yet been reached (this is
			// possible in off-line tests where knowledge of the future is
			// available)
			if (prb->hasAssignedQET(i) && prb->getQET(i) > cur_time) {
				// Add a new event at the arrival time, to be sure that it will
				// be processed
				long long qet = prb->getQET(i);
				auto it = events_.find(qet);
				if (it == events_.end())
					events_[qet] = {{i, job_arriving}};
				else
					it->second.push_back({i, job_arriving});
				// A job has been delayed
				++ndelayed;
				// move on
				DBG(DBGOUT << "moving on (the job arrives at " << prb->getQET(i) << ")" << endl;)

                auto itncj = find(not_considered_jobs.begin(),
                        not_considered_jobs.end(), i);
                if(itncj != not_considered_jobs.end())
                    not_considered_jobs.erase(
                            remove(
                                not_considered_jobs.begin(), 
                                not_considered_jobs.end(), i), 
                            not_considered_jobs.end());
				continue;
			}

            bool ok = fitJob(prb, sched, ranked_res, cur_time, maxPower,
                 usgs, sorted_running_jobs, i, true);

            // the job fits
            if(ok){
                long long end_time = cur_time + internal_prb_[i].dur;
                // add the job to running jobs sets
                auto itrj = running_jobs.find(i);
                if (itrj == running_jobs.end())
                    running_jobs[i] = end_time;
                sorted_running_jobs.push_back(i);

                // delete the job from the list of those to be considered yet
                auto itncj = find(not_considered_jobs.begin(),
                        not_considered_jobs.end(), i);
                if(itncj != not_considered_jobs.end())
                    not_considered_jobs.erase(
                            remove(
                                not_considered_jobs.begin(), 
                                not_considered_jobs.end(), i), 
                            not_considered_jobs.end());
            }
		
            /* PBS-backfilling: try to start jobs until they fit
             * when the first one doesn't fit try to backfill  */
			// if the job could not be dispatched, then it will be
			// considered in future iterations
			else {
				// Store the job in the processing list of the next event
				new_ranked_jobs.push_back(i);

                // delete the job from the list of those to be considered yet
                auto itncj = find(not_considered_jobs.begin(),
                        not_considered_jobs.end(), i);
                if(itncj != not_considered_jobs.end())
                    not_considered_jobs.erase(
                            remove(
                                not_considered_jobs.begin(), 
                                not_considered_jobs.end(), i), 
                            not_considered_jobs.end());

                /* Now we try backfilling jobs: PBS policy says that
                 * a job can run if it fits - only a few jobs are
                 * considered for backfilling (specified by 
                 * backfill_q_length)   */
                int ncons = 0;
                for(int bfj : not_considered_jobs){
                    // consider only the jobs in the backfill q
                    if(ncons < backfill_q_length_){
                        bool can_bf = true;
                        // check if its execution won't postpone the first job in Q
                        long long end_time = cur_time + internal_prb_[bfj].dur;
                        // check the if it fits
                        bool ok = fitJob(prb, sched, ranked_res, cur_time, maxPower,
                             usgs, sorted_running_jobs, bfj, true);

                        if(ok){
                            // add the job to running jobs sets
                            auto itrj = running_jobs.find(bfj);
                            if (itrj == running_jobs.end())
                                running_jobs[bfj] = end_time;
                            sorted_running_jobs.push_back(bfj);
                        }
                        else{
                            can_bf = false;
                        }

                        // if the job was not backfilled, next iteration
                        if(!can_bf)
                            new_ranked_jobs.push_back(bfj);
                    }
                    else
                        new_ranked_jobs.push_back(bfj);

                    ncons++;
                }

                // stop scheduling when we found the first job that doesn't fit
                break;
			}
		} // end of the dispatching loop

		// update the list of ranked jobs
		ranked_jobs = new_ranked_jobs;

		prev_time = cur_time;
	} // end of the event loop	

	DBG(DBGOUT << "schedule computation done" << endl;)
}

// =============================================================================
// = Method implementations (StaticDVFSSchedulingOptimizer)                      =
// =============================================================================

StaticDVFSSchedulingOptimizer::StaticDVFSSchedulingOptimizer(int ndims,
		CompatibilityChecker* cmp_chk,
		IntervalManager* int_man,
		long long maxPower,
        int frequency_scaling_mode,
        double perfModel_factor) : Optimizer(ndims, cmp_chk, int_man) {
	// Build the associated FixedStartSchedule
	_sched = FixedStartSchedule(&_prb);

	// maximal power consumption
	_maxPower = maxPower;

    frequency_scaling_mode_ = frequency_scaling_mode;
    perfModel_factor_ = perfModel_factor;

	// Specify the associated problem version (invalid value)
	_ref_prb_version = -1;
}


long long StaticDVFSSchedulingOptimizer::getSchedStart(long long job_idx) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	return _sched.getStart(it->second);
}

long long StaticDVFSSchedulingOptimizer::getSchedEnd(long long job_idx) {
	assert(_job_map.find(job_idx) != _job_map.end());
	auto it = _job_map.find(job_idx);
	return _sched.getEnd(it->second);
}

void StaticDVFSSchedulingOptimizer::writeSched(ostream& out) {
	_sched.writeSolution(out);
}

long long StaticDVFSSchedulingOptimizer::getProblemVersion() {
	return _prb.getProblemVersion();
}

void StaticDVFSSchedulingOptimizer::setMaxPower(long long maxPower){
    _maxPower = maxPower;
}

bool StaticDVFSSchedulingOptimizer::nextJob(long long ref_time,
						long long& job_idx,
						std::vector<int>& unit_mapping,
                        int& frequency) {
	assert(ref_time >= 0);
	assert(unit_mapping.size() == 0);
	
	// Obtain a schedule if one is not available or if the problem description
	// has changed since the last call to nextJob.
	if (_jobs_by_start.size() == 0 || // no sched available 
		_ref_prb_version != _prb.getProblemVersion()) { // modified problem

		// Commit all jobs
		for (int i : _prb.getAddedJobs()) _prb.commitJob(i);
		for (int i : _prb.getModifiedJobs()) _prb.commitJob(i);	
		for (int i : _prb.getRemovedJobs()) _prb.commitJob(i);
		assert(_prb.getAddedJobs().size() == 0);
		assert(_prb.getModifiedJobs().size() == 0);
		assert(_prb.getRemovedJobs().size() == 0);

		_ref_prb_version = _prb.getProblemVersion();

		// Job and resource ranking component
		JobResRanker jnr(&_prb);
		ScoreRanker res_sr = jnr.getDummyRes_scoreRanker();
        ScoreRanker job_wallTime_ranker = jnr.getJobWallTime_scoreRanker();

		// Obtain a fixed start schedule
		_sched.reset();
        switch(backfill_mode_){
            case 0:
                // using PBS backfilling scheme
                _solver.schedule_pbs(
                        &_prb, &_sched, jnr, res_sr, 
                        job_wallTime_ranker,
                        ref_time, _maxPower, frequency_scaling_mode_, 
                        perfModel_factor_);
                break;
            case 1:
                // using EASY backfilling scheme
                _solver.schedule_easy(
                        &_prb, &_sched, jnr, res_sr, 
                        job_wallTime_ranker,
                        ref_time, _maxPower,
                        frequency_scaling_mode_,
                        perfModel_factor_);
                break;
        }

		assert(_sched.check());

		// Rebuild the job dispatching map
		_jobs_by_start.clear();
		for (int i : _prb.getStableJobs()) {
			// Skip jobs that had been assigned a start time before the call to
			// StaticDVFSScheduler::schedule
			if (_prb.hasAssignedStart(i)) continue;
			// Insert all other jobs in the map
			auto tmp_it = _jobs_by_start.find(_sched.getStart(i));
			if(tmp_it == _jobs_by_start.end())
				_jobs_by_start[_sched.getStart(i)] = {i};
			else 
				tmp_it->second.push_back(i);
			
		}
		
		// Init the internal iterator
		_nexttime_it = _jobs_by_start.begin();
		_nextjob_it = _nexttime_it->second.begin();
	}

	// If there is not ready job before or at ref_time, then return false
	if (_nexttime_it->first > ref_time){  
		return false; // EARLY EXIT
	}

	// If no job is ready for dispatching, return false
	if (_nexttime_it == _jobs_by_start.end()){
		return false; // EARLY EXIT
	}

	// Otherwise, store the (external) index and unit mapping of the job
	int int_idx = *_nextjob_it;
	job_idx = _prb.getJobExtIdx(int_idx);
	for (int u = 0; u < _prb.getNUnits(int_idx); ++u){
		int res_idx = _prb.getResExtIdx(_sched.getUnitMapping(int_idx, u));
		unit_mapping.push_back(res_idx);
	}

    frequency = _sched.getFrequency(int_idx);

	// Advance the iterator
	++_nextjob_it;
	if(_nextjob_it == _nexttime_it->second.end()){
		++_nexttime_it;
	}

	// Everything went ok
	return true;
}

void StaticDVFSSchedulingOptimizer::setJobsPerfModelFactors(
        map<long long, double> jobs_perfModel_factors){
    jobs_perfModel_factors_ = jobs_perfModel_factors;
}
 

} // end of the optimization_core namespace

// =============================================================================
// = Undefine the debug macros                                                 =
// =============================================================================

#undef DBG
#undef DBGOUT

#endif
