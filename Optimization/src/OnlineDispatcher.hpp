#ifndef ONLINEDISPATCHER_HPP_
#define ONLINEDISPATCHER_HPP_
#include <algorithm> 
#include <map>
#include <random>
#include <iostream>
#include <Optimizer.hpp>
#include <FixedStartSchedule.hpp>
#include <Rankers.hpp>

#include <NodeReader.hpp>
#include <JobReader.hpp>
#include <JobQueueReader.hpp>

#include <Util.hpp>

#include "Predictor.hpp"

using namespace std;

namespace optimization_core {

// =============================================================================
// = OnlineDispatcher class                                                       =
// =============================================================================

/* Class to wrap dispatching problems in a online framework.
 * This is a fundamental class to simulate the 'online' behaviour of a real 
 * job dispatcher. OnlineDispatcher has knowledge of all the jobs in the job
 * instance and simulate their arrival following their arrival times.  */
class OnlineDispatcher {

private:	
	// Optmizer used
	Optimizer* _opt;	
    Predictor* _predictor;
	vector<Job>& _jobs;
	vector<JobQueue>& _queues;
	vector<Node>& _nodes;
	int _ndims;

    /* Internal random generator */
    unsigned int seed_;

    /* Assumption: all nodes with higher id are 3.1GHz nodes,
     * lower ids imply 2.1GHz nodes    */
    int _nodeType_threshold;

    /* Threshold used for a performance metric  
     * Minimum duration (s) of jobs considered when computing BSLD.  */
    double _bsld_threshold = 60;
    
    /* Jobs already added to the problem */
    vector<int> _added_jobs;

    /* Dummy jobs added to simulate hot startup */
    vector<long long> _fake_jobs;

    /* Power Cap constraint */
    long long _power_cap;

    /* Varying power cap constraint (read from file)
     * map<event time, power cap value> */
    map<long long, int> _power_caps;

    /* Power cap may be not enforced
     * But we may still want to compare the dispatcher 
     * behaviour w.r.t. a "target" power.
     * Target sys power might be different from the power cap.
     *
     * For example if we use let assume we want to have a EASY-BF
     * scheduler that manages the power trough RAPL-like action.
     * We cannot directly use BackFillingScheduler because it
     * would directly enforce the power cap passed as parameter (thus
     * nullifying the RAPL-action. Instead, we can pass a fake, 
     * larger than desired power cap to BackFillingScheduler and 
     * then use RAPL-like mechanism to respect the target 
     * (desired) power budget    */
    long long _target_sys_power;
    long long _target_node_power;

    /* Parameters for the dynamic RAPL-based power capping */
    int _dynPowerShare_firstMeasure;
    // power allocated to each node
    map<int, long long> _nodes_alloc_power;
    map<int, int> _nodes_reclaim;
    /* Not defined in the paper: parameter to account the 
     * measurement jitter. In this implementation 
     * an arbitrary value is set */
    long long _target_node_wastedPower = 5;
    /* Not explained in the paper */
    double _reclaim_factor = 0.9;
    // min and max allocable power depend on the HW
    long long _node_min_alloc_power = 1;
    long long _node_max_alloc_power = 400;

    /* Power counters */
    /* The real time power consumption should be measured directly
    * Currently, we rely on the power prediction of each
    * running job */
    int _current_power;
    map<long long, double> _historical_powers;

    /* Alternative set of power counters: these
     * should better simulate a real-time measuring 
     * framework  */
    int _system_power;            // should coincide with _current_power
    map<int, int> _nodes_power;   // power consumed in each node
    map<long long, int> _system_power_profile;
    map<int, map<int, int> > _nodes_power_profile;
   
    /* Every _step_ seconds we simulate a power measurements.
     * Power measurements are treated as special events 
     * within the simulator; they don't trigger computation
     * of new schedules but might trigger the RAPL-action (
     * if it is necessary and dynamic power management is
     * selected   */
    long long _power_measure_step_fixed = 5;

    /* For the dynamic frequency scaling based on RAPL we distinguish
     * 3 methods for deciding the impact of power consumption cap
     * on the interested jobs durations.
     * -1) No dynamic RAPL power mechanism
     * 0) No impact: if we bound the power consumption of a node in
     * which a job is running, its duration doesn't change
     * 1) Max impact: if we reduce the power consumption of x%
     * the duration increase will be of x%
     * 3) Intermediate impact: if we reduce the power consumption of x%
     * the duration increase will be of perfModel_factor_ * x%  */
    int _dyn_RAPL_mode;
    /* Input parameter that specifies the performance reduction w.r.t.
     * the power reduction (decided by user) */
    double perfModel_factor_;

    /* We can simulate the dispatcher behaviour with different
     * startup settings, namely we can specify the utilization
     * of the system at start time. The utilization is simulated through
     * fake jobs already in the system. To simplify these jobs only 
     * require CPU resources. Moreover, these jobs power consumption is
     * artificially low (much smaller than the power cap)
     * 0 means a system totally empty.
     * 100 means a system completely full.
     * Intermediate values define intermediate situations  */
    int startup_hotness_;

    /* Input parameter that decides the underlying optimizer
     * used to compute a schedule  */
    int opt_type_;
     
    /* Timers information */
    vector<double> _freq_timers;

    /* Mode to reassign execution frequencies:
     * - 0: greedy algorithm
     * - 1: CP model   */
    int _freqAssign_mode;

    /* Max frequencies allowed - cached for later use.
     * Depend on the real frequencies allowed by Eurora nodes,
     * 2.1 GHz for half of the nodes, 3.4GHz for the remaining ones */
    vector<int> _max_freqs = {2100, 3400};

	// data structure to store information about a job
	typedef struct _job_info {
        long long job_idx;
		long long dur; // Job (real) duration 
		long long original_dur; // Job duration at eqt
		long long ewt; // Expected waiting time
		long long qet;
        double power_pred; // Predicted Consumed Power - whole job
        double power_pred_perNode; // predicted power, per node
        double cur_power;  // Current consumed power - may vary
        double cur_power_perNode;  
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
	} _job_info;

    /* Support function to help sorting job info: job with larger predicted
     * powers are preferred */
    struct job_info_byPower_sorter
    {
        inline bool operator() (const _job_info& ji1, const _job_info& ji2)
        {
            return (ji1.power_pred > ji2.power_pred);
        }
    }; 
	
	/* Map with the info of all jobs in the problem - useful for compute metrics */
	map<long long, _job_info> _jobs_info;

    /* Set of running jobs */
    map<long long, _job_info> _running_jobs;

    /* Set of running jobs grouped by node */
    map<int, vector<long long> > _node_running_jobs;

    /* Set of waiting jobs */
    map<long long, _job_info> _waiting_jobs;

    /* Set of completed jobs */
    map <long long, _job_info> _ended_jobs;

	// custom type to store start/end time and mapping of a job 
	typedef struct _job_schedule {
		long long start;
		long long end;
		vector<int> mapping;
	} _job_schedule;
	
	// map with the final schedule
	map<long long, _job_schedule> _final_schedule;
	
	long long _ref_time;

    long long prev_notMeasure_time_;

    /* In our simulator we can know _a priori_ how many times
     * the power cap will vary: this is a useful information
     * to promptly terminate our dispatcher when all the 
     * meaningful events happened   */
    int _power_cap_variations_left;

    int _power_measurements_left;

	// custom enumeration to divise jobs in arriving/startin/ending
    /* - events are used also to track the varying power cap,
     *   these events have a fake job idx (negative) and are treated like normal
     *   events, i.e. generating a new schedule if needed 
     * - events are used also to simulate a power measuring framework:
     *   at each event the system and node power are measured
     *   (no new scheduling)      */
	typedef enum _job_evt_type {
		job_arriving,
		job_starting,
		job_ending,
        power_cap_variation,
        power_measure
	} _job_evt_type;
	
	// custom type to store the jobs that arrive/start/end at certain event
    /* Events representing power cap variations have a fake
     * job_idx, a negative value (usual idxs must have only positive finite 
     * values); the absolute value of this negative number is the power cap
     * at such event */
	typedef struct _job_evt {
		long long job_ext_idx;
		_job_evt_type type;
	} _job_evt;

	// map with the ending jobs (this is needed to find quickly the next
	// time point to process). Iterators of the map container stay valid
	// even if a new element is inserted 
	map<long long, vector<_job_evt> > _events;

	// cached resource capacities 
	// a vector of size ndims for each resource
	vector< vector<int> > _res_caps;
	// cumulative resource capacities
	vector<int> _cumul_res_caps;
	vector<int> _max_cumul_res_caps;

    /* Values to keep track machine utilization */
    map<long long, vector<double>> _sys_util;
    vector<int> _cumul_res_usgs;
	vector< vector<int> > _res_usgs;

    // cached minumum resource requirements (jobs in q)
    vector<int> _min_reqs;

	// computation time stats for each scheduling event
	vector<int> _nscheduled_jobs;  // number of scheduled jobs
	vector<float> _treq_sched;  // time required by the scheduling

    /* RAPL implementation with the power-duration model factor 
     * dependent on the application type. We compute the perfModel
     * factor for each job the first time the function is called.
     * Such values are stored for future calls (the perfModel
     * factor of a job is constant)   */
    map<long long, double> jobs_perfModel_factors_;
    double compute_job_perfModel_factor(long long job_idx, int application_type){
        auto it = jobs_perfModel_factors_.find(job_idx);
        // the job has already a perfModel factor
        if(it != jobs_perfModel_factors_.end()){
            return it->second;
        }
        // we compute the new perfModel factor
        else{
            /* Assumptions: the factor can range between 0 and 1;
             * the factor is a random value extracted from a normal
             * distribution. Application type has its own distribution 
             * mean (std are the same for all application types)  */
            // create internal random generator
            // seeded with the instance name
            random_device rdev;
            unsigned int seed = seed_ + job_idx;
            seed_seq seeder{seed};
            //mt19937 e2(seeder);
            mt19937 e2;
            // normal distributions for each application type
            normal_distribution<double> dist_CPUbound(0.8, 0.2);
            normal_distribution<double> dist_QE(0.7, 0.1);
            normal_distribution<double> dist_MEMbound(0.3, 0.2);
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

    /* return the job consumed energy 
     * !!! This function has not been properly tested !!! */
    long long get_job_final_energy(int job_idx, long long cur_time){
        long long energy = 0;

        // if the job has not started yet
        if(_jobs_info.find(job_idx) ==_jobs_info.end())
            return 0;

        /* if no frequency re-assignment happened, great! */
        if(_jobs_info[job_idx].exec_hist.size() == 0){
            energy = _jobs_info[job_idx].cur_power *
                _jobs_info[job_idx].dur;
        }
        /* If the job execution saw changes in frequency the
         * computation needs to consider the different phases */
        else{
            /* suppose a job had N phases: the first N-1 are stored 
             * in the execution history list; the N phase is the 
             * current one (no change in freq. since it started  */
            // add the energy contributions of the N-1 phases 
            long long energy_except_last = 0;
            long long dur_except_last = 0;
            for(auto eh : _jobs_info[job_idx].exec_hist){
                dur_except_last += eh.second;
                energy_except_last += eh.first * eh.second;
            }
            energy += energy_except_last;

            // add the contribution of the last phase
            long long last_phase_dur = 
                cur_time - _jobs_info[job_idx].start_time - dur_except_last;
            energy += last_phase_dur * 
                _jobs_info[job_idx].cur_power;
        }

        return energy;
    }

	// add the jobs that need to be scheduled
	void addJob(int job_idx) {			
		// Obtain a reference to the job
		Job& job = _jobs[job_idx];

        // auxiliary structure with job-related info
		_job_info j_info;
        j_info.job_idx = job_idx;
		
		// Obtain the job duration
		// this is the duration used to compute schedules
		long long dur  = job.getEstimatedDuration();

        // Obtain the job predicted power consumption
        // This power is used to respect the power cap
        double power_pred;
        power_pred = ceil(_predictor->get_power_prediction(job));

        /* Some power predictions could be wrong. Extreme small values 
         * (smaller than 1) are extremely suspicious (and can cause later problems):
         * just in case we set them to 1. In this way we are also being robust
         * and conservative by overestimating the power consumptions */
        if(power_pred < 1)
            power_pred = 1;

        double power_pred_perNode = 
            ceil(power_pred / _jobs[job_idx].getNumberOfNodes());

        if(power_pred_perNode <= 1){
            power_pred_perNode = 1;
            power_pred = _jobs[job_idx].getNumberOfNodes();
        }

        /* Later experiments shown that there are inconsistencies
         * (my fault) when dealing with power predictions - sometimes treated as
         * double and sometimes as long. This cause problems, namely 
         * pred_power doesn't correspond to pred_powerPerNode times
         * number of nodes. This is bad for many reasons. So here we forcefully 
         * impose this relationship     */
        long long temp_power_pred = power_pred;
        long long temp_power_pred_perNode = power_pred_perNode;
        if(temp_power_pred_perNode * _jobs[job_idx].getNumberOfNodes() 
                != temp_power_pred)
            power_pred = power_pred_perNode * _jobs[job_idx].getNumberOfNodes();

        /* Some job power predictions are very large values.
         * They are outliers - we know they exist and are described in our paper
         * In order to simplify the implementation here we force
         * a maximum power consumption for each job: if the power
         * prediction is larger we bound the prediction.
         * The maximum power prediction is the
         * maximum power allowed on a node   */
        if(power_pred_perNode > _node_max_alloc_power){
            power_pred_perNode = _node_max_alloc_power;
            power_pred = power_pred_perNode * _jobs[job_idx].getNumberOfNodes();
        }

        j_info.power_pred = power_pred;
        j_info.power_pred_perNode = power_pred_perNode;

        /* We can force our prediction to be treated as perfect - thus 
         * the initial power consumption (cur_power) would be equal to
         * the predicted power. 
         * This is an obviously wrong assumption but however it enables
         * a fair comparison between different policies (and anyway the 
         * power consumption prediction model needs to be changed on 
         * different HPC systems)   */
        // initially, the job power is the predicted one
        double cur_power = power_pred;
        j_info.cur_power = power_pred;
        j_info.cur_power_perNode = power_pred_perNode;

        /* OR we can take into account the possible mis-predictions: the 
         * initial power is therefore different from the predicted one */
        /* In this example we force that 10% of jobs in an instance are 
         * under-estimated by 40%   */
        /* 
        double underestimate_factor = 1.4;
        int understimate_probability = 10;
        bool underprediction;  

        random_device rdev;
        unsigned int seed = seed_ + job_idx;
        seed_seq seeder{seed};
        mt19937 rng(seeder);
        uniform_int_distribution<int> int_distr(1,100);
        int rnd = int_distr(rng);

        rnd <= understimate_probability ? 
            underprediction = true : underprediction = false;

        if(underprediction){
            cur_power *= underestimate_factor;
            j_info.cur_power *= underestimate_factor;
            j_info.cur_power_perNode *= underestimate_factor;
        }
        */
        
        int application_type = _jobs[job_idx].getApplicationType();
        j_info.application_type = application_type;

		// sanitize strange input: if real duration longer than expected (this shouldn't happen
		// but it does), reduce real duration -> simplify testing, we should change 
		// this restriction in the real dispatcher, allow some flexibility
		(job.getEstimatedDuration() < job.getRealDuration() ? 
			 j_info.dur = job.getEstimatedDuration() :
			 j_info.dur = job.getRealDuration());

        j_info.original_dur = j_info.dur;

		// Obtain the job waiting time
		long long ewt = 1800;   // default value for reservations
		for (JobQueue& q : _queues){
			if (q.getId().compare(job.getQueue()) == 0) {
				ewt = q.getMaxMinutesToWait();
				break;
			}
		}
		j_info.ewt = ewt;

        // the job frequency is decided only after mapping
        j_info.frequency = -1;

		// The time already spent in queue
		long long qet = job.getEnterQueueTime();
		j_info.qet = qet;

        j_info.start_time = -1;
        j_info.end_time = -1;
		
		// Add job units one by one
		int nunits = job.getNumberOfNodes();
		vector<vector<long long> > unit_reqs;
		j_info.reqs.resize(nunits);
		j_info.tot_reqs.resize(_ndims,0);

		for (int u = 0; u < nunits; ++u) {
			j_info.reqs[u].resize(_ndims);
			// get the requirement of each job unit
			// (and store them for the internal job_info map)
			long long rcores = job.getNumberOfCores() / nunits;
			j_info.reqs[u][0] = rcores;
			j_info.tot_reqs[0] += rcores;
			long long rgpus = job.getNumberOfGPU() / nunits;
			j_info.reqs[u][1] = rgpus;
			j_info.tot_reqs[1] += rgpus;
			long long rmics = job.getNumberOfMIC() / nunits;
			j_info.reqs[u][2] = rmics;
			j_info.tot_reqs[2] += rmics;
			long long rmem = job.getMemory() / nunits;
			j_info.reqs[u][3] = rmem;
			j_info.tot_reqs[3] += rmem;
			// store the unit requirements
			unit_reqs.push_back({rcores, rgpus, rmics, rmem});
		}

        j_info.energy = -1;

		// Add the job to the optimizer
		_opt->addJob(job_idx, dur, power_pred, cur_power, 
                qet, ewt, application_type, unit_reqs);
        _opt->setFrequency(job_idx,-1);
	
		// store the information about the job in the internal map	
		_jobs_info[job_idx] = {j_info};

        /* Add job to those in the waiting queue */
        auto itr = _waiting_jobs.find(job_idx);
        // only if the job have not been seen before
        if(itr == _waiting_jobs.end())
            _waiting_jobs[job_idx] = _jobs_info[job_idx];

        _added_jobs.push_back(job_idx);

        // compute job powerPerfModel factor (to be used 
        // also by some optimiezers - those using RAPL)
        compute_job_perfModel_factor(job_idx, application_type);
	}

	/* Update the cached resource capacities
	 * 	-1: starting job
	 * 	1:  ending job
	 */
	void update_caps (int job_idx, int job_type, vector<int> mapping){
        int nunits;
        if(job_idx <= _jobs.size())
            nunits = _jobs[job_idx].getNumberOfNodes();
        else
            nunits = _jobs_info[job_idx].node_mapping.size();
        
		for(int u = 0; u < nunits; u++){
			int res_idx = mapping[u];
			for(int d = 0; d < _ndims; d++){
				// update single res capacities
				_res_caps[res_idx][d] = _res_caps[res_idx][d] +
					job_type * _jobs_info[job_idx].reqs[u][d];
				// update cumulative capacities
				_cumul_res_caps[d] = _cumul_res_caps[d] +
					job_type * _jobs_info[job_idx].reqs[u][d];

				// update single res usages
				_res_usgs[res_idx][d] = _res_usgs[res_idx][d] -
					job_type * _jobs_info[job_idx].reqs[u][d];
                // update cumulative usages
                _cumul_res_usgs[d] = _cumul_res_usgs[d] -
					job_type * _jobs_info[job_idx].reqs[u][d];
			}
		}
	}

	/* Update the cached system power
	 * 	1: starting job
	 * 	-1:  ending job
	 */
	void update_power (int job_idx, int job_type){
        _current_power = _current_power + 
            job_type * _jobs_info[job_idx].cur_power;
        if( _current_power < 0)
            cout << "Something wrong... ---> negative power" << endl;
	}

	/* Check if newly arrived job could fit in the system
	 * - given the current cached resource state 
	 *   TODO: very inefficient (check every dim, res, etc..)
	 *   	possible much smarter implementation
	 */
	bool check_res_avail(int job_idx){
		int nunits = _jobs[job_idx].getNumberOfNodes();
		// 1) check cumulative resource capacities
		for(int d = 0; d < _ndims; d++)
			if(_jobs_info[job_idx].tot_reqs[d] > _cumul_res_caps[d])
				return false;
		// 2) check if each job unit can fit
		for(int u = 0; u < nunits; u++){
			// check every dimension
			for(int d = 0; d < _ndims; d++){
				bool fit = false;
				// consider every resource
				for(int r = 0; r < _nodes.size(); r++)
					// if the unit could fit in at least one resource, OK
					if(_jobs_info[job_idx].reqs[u][d] <= 
							_res_caps[r][d]){
						fit = true;
						break;
					}
				if(!fit)
					return false;
			} // dims loop
		} // units loop
		return true;
	}

    /* Change dynamically (at run-time) the frequency 
     * of a running job. The change must affect the job remained
     * duration (in all nodes, the duration is the maximum among all
     * possible ones - for jobs which run on both slowed and not slowed 
     * nodes), its power consumption,  
     * change the ending event in the system.
     * For simplicity we assume that if the job frequency changes, it
     * changes in every node is running (the duration considered
     * would be the longest anyway), therefore it will consume less
     * power also in nodes which were not forced to slow down
     * by RAPL (or other mechanisms).
     * In input is given the power allocated to the job - 
     * computed by other function - and the job frequency is changed 
     * accordingly (it may increase or increase given the current
     * job power consumption and frequency).
     *
     * !!! The implementation of this function is _not_ entirely bug-free
     *     In some (rare) case the resulting duration is wrongly
     *     computed - basically it explodes to meaningless values !!! 
     *
     * This is the method that perform the actual power reduction and 
     * subsequent duration increase. It must be changed to implement 
     * a different power-dilation model. Important thing to remember:
     * after having decided a new power and duration for the job passed
     * as input we have to (if necessary):
     *   1) change the Dispatcher's problem descriptor (_jobs_info)
     *   and auxiliary structure (_running_jobs, _final_schedule)
     *   2) change the event list - removing the old ones and adding 
     *   the new ones
     *   3) notify the optimizer internal problem description (_opt)
     *   about the changes                     */
    bool setJob_FreqPowDur_dynamic(
            int job_idx, int job_allocated_power, long long cur_time){
        long long elapsed_dur = cur_time - _jobs_info[job_idx].start_time;
        long long remained_dur = _jobs_info[job_idx].dur - elapsed_dur;
        double old_power = _jobs_info[job_idx].cur_power;

        long long new_remained_dur;
        double new_power, new_power_perNode;
        long long old_end_time = _jobs_info[job_idx].end_time;

        // the duration increase is proportional to the power variation
        if(_dyn_RAPL_mode == 1){
            /* Reducing the job power */
            if(_jobs_info[job_idx].cur_power_perNode > job_allocated_power){
                double powMul_desired = job_allocated_power /
                    _jobs_info[job_idx].cur_power_perNode;
                double durMul = 1 / powMul_desired;

                /* In this mode, it is possible to indefinitely decrease 
                 * a job power consumption - there's no check on the minimal
                 * power reduction nor on the maximal duration increment 
                 * We force the minimal node allocation power to 1W */
                new_remained_dur = durMul * remained_dur;
                if(new_power < 0)
                    new_power = _node_min_alloc_power;
                new_power = powMul_desired * _jobs_info[job_idx].cur_power;
                if(new_power_perNode < 0)
                    new_power_perNode = _node_min_alloc_power;
                new_power_perNode = 
                    powMul_desired * _jobs_info[job_idx].cur_power_perNode;
            }

            /* Job power may increase  */
            else if(_jobs_info[job_idx].cur_power_perNode <= job_allocated_power){
                // job not at max frequency
                if(_jobs_info[job_idx].original_dur < _jobs_info[job_idx].dur){
                    double powMul_desired = job_allocated_power /
                        _jobs_info[job_idx].cur_power_perNode;
                    double durMul = 1 / powMul_desired;

                    /* when increasing the power of a job, the maximum power 
                     * consumption (specified by it predicted power) must
                     * not be exceeded - also the duration cannot become smaller
                     * than the original one */
                    double np = powMul_desired * _jobs_info[job_idx].cur_power;
                    double npp = 
                        powMul_desired * _jobs_info[job_idx].cur_power_perNode;
                    long long nrd = durMul * remained_dur;

                    (np > _jobs_info[job_idx].power_pred ?
                     new_power = _jobs_info[job_idx].power_pred : new_power = np);
                    (npp > _jobs_info[job_idx].power_pred_perNode ?
                     new_power_perNode = _jobs_info[job_idx].power_pred_perNode :
                     new_power_perNode = npp);

                    (nrd + elapsed_dur < _jobs_info[job_idx].original_dur ?
                     new_remained_dur = _jobs_info[job_idx].original_dur - elapsed_dur : 
                     new_remained_dur = nrd);
                }
                // job already at max frequency
                else{
                    new_power = _jobs_info[job_idx].power_pred;
                    new_power_perNode = _jobs_info[job_idx].power_pred_perNode;
                    new_remained_dur = _jobs_info[job_idx].original_dur - elapsed_dur;
                }

            }
        } // end of _dyn_RAPL_mode == 1

        // the duration increase is proportional to the power variation
        // multiplied by a factor (real value between 0 and 1)
        else if(_dyn_RAPL_mode == 3){
            /* Reducing the job power */
            if(_jobs_info[job_idx].cur_power_perNode > job_allocated_power){
                double powMul_desired = job_allocated_power /
                    _jobs_info[job_idx].cur_power_perNode;
                double durMul = 1 / powMul_desired * perfModel_factor_;
                
                if(powMul_desired == 1)
                    durMul = 1;

                // if the powMul is very close to 1 (curPower_perNode very close to
                // allocated_power), durMul risks to become smaller than 1 (after
                // having been multiplied by the perfModel_factor): this must not
                // happen, reducing the power/freq cannot lead a job to last less
                // time. We fix it imposing a durMult of at least 1
                if(durMul < 1)
                    durMul = 1;

                /* In this mode, it is possible to indefinitely decrease 
                 * a job power consumption - there's no check on the minimal
                 * power reduction nor on the maximal duration increment.
                 * We force the minimal node allocation power to 1W */

                new_remained_dur = durMul * remained_dur;
                if(new_power < 0)
                    new_power = _node_min_alloc_power;
                new_power = powMul_desired * _jobs_info[job_idx].cur_power;
                if(new_power_perNode < 0)
                    new_power_perNode = _node_min_alloc_power;
                new_power_perNode = 
                    powMul_desired * _jobs_info[job_idx].cur_power_perNode;
            }

            /* Job power may increase  */
            else if(_jobs_info[job_idx].cur_power_perNode <= job_allocated_power){
                /* This is not true if the duration increase is not directly
                 * proportional to the power decrease: we may reach situation when
                 * the duration went back to the original one while the power
                 * is still smaller than the predicted one */
                // job not at max frequency
                if(_jobs_info[job_idx].power_pred > _jobs_info[job_idx].cur_power){

                    double powMul_desired = job_allocated_power /
                        _jobs_info[job_idx].cur_power_perNode;
                    double durMul = 1 / powMul_desired;
                    if(powMul_desired == 1)
                        durMul = 1;
                    if(durMul > 1)
                        durMul = 1;

                    /* when increasing the power of a job, the maximum power 
                     * consumption (specified by its predicted power) must
                     * not be exceeded - also the duration cannot become smaller
                     * than the original one */

                    double np = powMul_desired * _jobs_info[job_idx].cur_power;
                    double npp = 
                        powMul_desired * _jobs_info[job_idx].cur_power_perNode;

                    long long nrd = durMul * remained_dur;

                    if(nrd > remained_dur)
                        nrd = remained_dur;

                    long long dur_decrease = remained_dur - nrd;
                    double dur_decr_factorized = dur_decrease * perfModel_factor_;
                    nrd = remained_dur - dur_decr_factorized;

                    assert(powMul_desired >= 1);
                    assert(durMul >= 0);
                    assert(durMul <= 1);
                    assert(nrd <= remained_dur);

                    (np > _jobs_info[job_idx].power_pred ?
                     new_power = _jobs_info[job_idx].power_pred : new_power = np);
                    (npp > _jobs_info[job_idx].power_pred_perNode ?
                     new_power_perNode = _jobs_info[job_idx].power_pred_perNode :
                     new_power_perNode = npp);

                    (nrd + elapsed_dur < _jobs_info[job_idx].original_dur ?
                     new_remained_dur = _jobs_info[job_idx].original_dur - elapsed_dur : 
                     new_remained_dur = nrd);
                }
                // job already at max frequency
                else{
                    new_power = _jobs_info[job_idx].power_pred;
                    new_power_perNode = _jobs_info[job_idx].power_pred_perNode;
                    new_remained_dur = _jobs_info[job_idx].original_dur - elapsed_dur;
                }
            }
        } // end of _dyn_RAPL_mode == 3

        // the duration-power changes differ for each job (mixed approach)
        // each job class has its own performance-model factor
        else if(_dyn_RAPL_mode == 4){
            double jobClass_perfModel_factor = 
                compute_job_perfModel_factor(job_idx,
                        _jobs_info[job_idx].application_type);

            /* Reducing the job power */
            if(_jobs_info[job_idx].cur_power_perNode > job_allocated_power){
                double powMul_desired = job_allocated_power /
                    _jobs_info[job_idx].cur_power_perNode;
                double durMul = 1 / powMul_desired * jobClass_perfModel_factor;
                
                if(powMul_desired == 1)
                    durMul = 1;

                // if the powMul is very close to 1 (curPower_perNode very close to
                // allocated_power), durMul risks to become smaller than 1 (after
                // having been multiplied by the perfModel_factor): this must not
                // happen, reducing the power/freq cannot lead a job to last less
                // time. We fix it imposing a durMult of at least 1
                if(durMul < 1)
                    durMul = 1;

                /* In this mode, it is possible to indefinitely decrease 
                 * a job power consumption - there's no check on the minimal
                 * power reduction nor on the maximal duration increment 
                 * We force the minimal node allocation power to 1W */

                new_remained_dur = durMul * remained_dur;
                if(new_power < 0)
                    new_power = _node_min_alloc_power;
                new_power = powMul_desired * _jobs_info[job_idx].cur_power;
                if(new_power_perNode < 0)
                    new_power_perNode = _node_min_alloc_power;
                new_power_perNode = 
                    powMul_desired * _jobs_info[job_idx].cur_power_perNode;
            }

            /* Job power may increase  */
            else if(_jobs_info[job_idx].cur_power_perNode <= job_allocated_power){

                /* This is not true if the duration increase is not directly
                 * proportional to the power decrease: we may reach situation when
                 * the duration went back to the original one while the power
                 * is still smaller than the predicted one */
                if(_jobs_info[job_idx].power_pred > _jobs_info[job_idx].cur_power){

                    double powMul_desired = job_allocated_power /
                        _jobs_info[job_idx].cur_power_perNode;
                    double durMul = 1 / powMul_desired;
                    if(powMul_desired == 1)
                        durMul = 1;
                    if(durMul > 1)
                        durMul = 1;

                    double np = powMul_desired * _jobs_info[job_idx].cur_power;
                    double npp = 
                        powMul_desired * _jobs_info[job_idx].cur_power_perNode;

                    /* when increasing the power of a job, the maximum power 
                     * consumption (specified by its predicted power) must
                     * not be exceeded - also the duration cannot become smaller
                     * than the original one */
                    long long nrd = durMul * remained_dur;

                    if(nrd > remained_dur)
                        nrd = remained_dur;
                    long long dur_decrease = remained_dur - nrd;
                    double dur_decr_factorized = dur_decrease * jobClass_perfModel_factor;
                    nrd = remained_dur - dur_decr_factorized;

                    (np > _jobs_info[job_idx].power_pred ?
                     new_power = _jobs_info[job_idx].power_pred : new_power = np);
                    (npp > _jobs_info[job_idx].power_pred_perNode ?
                     new_power_perNode = _jobs_info[job_idx].power_pred_perNode :
                     new_power_perNode = npp);
                    (nrd + elapsed_dur < _jobs_info[job_idx].original_dur ?
                     new_remained_dur = _jobs_info[job_idx].original_dur - elapsed_dur : 
                     new_remained_dur = nrd);
                }
                // job already at max frequency
                else{
                    new_power = _jobs_info[job_idx].power_pred;
                    new_power_perNode = _jobs_info[job_idx].power_pred_perNode;
                    new_remained_dur = _jobs_info[job_idx].original_dur - elapsed_dur;
                }
            }
        } // end of _dyn_RAPL_mode == 4

        // the duration doesn't change if we modify the power consumption
        else{
            // wheter the job power increases or decreases there's no 
            // impact on job duration
            /* Reducing the job power */
            if(_jobs_info[job_idx].cur_power_perNode > job_allocated_power){

                double powMul_desired = job_allocated_power /
                    _jobs_info[job_idx].power_pred_perNode;
                double durMul = 1;

                new_remained_dur = durMul * remained_dur;
                new_power = powMul_desired * _jobs_info[job_idx].power_pred;
                new_power_perNode = 
                    powMul_desired * _jobs_info[job_idx].power_pred_perNode;
            }

            /* Job power may increase  */
            else if(_jobs_info[job_idx].cur_power_perNode <= job_allocated_power){

                // job not at max frequency
                if(_jobs_info[job_idx].cur_power_perNode < 
                        _jobs_info[job_idx].power_pred_perNode){
                    double powMul_desired = job_allocated_power /
                        _jobs_info[job_idx].cur_power_perNode;
                    double durMul = 1;

                    double np = powMul_desired * _jobs_info[job_idx].cur_power;
                    double npp = 
                        powMul_desired * _jobs_info[job_idx].cur_power_perNode;

                    (np > _jobs_info[job_idx].power_pred ?
                     new_power = _jobs_info[job_idx].power_pred : new_power = np);
                    (npp > _jobs_info[job_idx].power_pred_perNode ?
                     new_power_perNode = _jobs_info[job_idx].power_pred_perNode :
                     new_power_perNode = npp);

                    new_remained_dur = durMul * remained_dur;
                }
                // job already at max frequency
                else{
                    new_power = _jobs_info[job_idx].power_pred;
                    new_power_perNode = _jobs_info[job_idx].power_pred_perNode;
                    new_remained_dur = _jobs_info[job_idx].dur - elapsed_dur;
                }
            }
        } // end of _dyn_RAPL_mode == 0

        long long new_end_time = cur_time + new_remained_dur;
        if(new_end_time != cur_time){

            // apply changes to online dispatcher
            // remove end events of slowed down jobs and add new end events
            auto it = _events.find(old_end_time); 
            assert(it != _events.end());

            // remove old ending event
            // iterate over all events registered at end time of the slowed job
            for(auto it2 = it->second.begin(); it2 != it->second.end();){
                if(it2->job_ext_idx == job_idx)
                    it2 = it->second.erase(it2);
                else
                    ++it2;
            }
            
            // add new ending event
            auto itne = _events.find(new_end_time);
            if (itne != _events.end())
                itne->second.push_back({job_idx, job_ending});
            else _events[new_end_time] = {{job_idx, job_ending}};

            // update auxiliary structures
            _jobs_info[job_idx].end_time = new_end_time;
            _running_jobs[job_idx].end_time = new_end_time;
            _final_schedule[job_idx].end = new_end_time;

            if(new_power < 1){
                new_power = 1;
            }
            if(new_power_perNode < 1){
                new_power_perNode = 1;
            }

            _jobs_info[job_idx].cur_power = new_power;
            _running_jobs[job_idx].cur_power = new_power;

            _jobs_info[job_idx].cur_power_perNode = new_power_perNode;
            _running_jobs[job_idx].cur_power_perNode = new_power_perNode;

            _jobs_info[job_idx].dur = elapsed_dur + new_remained_dur;
            _running_jobs[job_idx].dur = elapsed_dur + new_remained_dur;

            /* Track power variation in the job execution trace  */
            // if the vector is empty this is the first frequency re-assignment
            if(_jobs_info[job_idx].exec_hist.size() == 0){ 
                /* We must add a pair in the vector: the one related to 
                 * the first part of the execution  */
                _jobs_info[job_idx].exec_hist.push_back(
                        make_pair(old_power, elapsed_dur));
                _running_jobs[job_idx].exec_hist.push_back(
                        make_pair(old_power, elapsed_dur));
            }
            else{
                /* We add a new event-pair: the one related to the last
                 * part of the execution before the new change of freq.  */
                // the power was the one before the change
                /* The duration of the N part is given as: current_time -
                 * job_start_time - sum(duration previous subparts)  */
                long long dur_so_far = 0;
                for(auto eh : _jobs_info[job_idx].exec_hist)
                    dur_so_far += eh.second;
                long long new_part_dur = elapsed_dur - dur_so_far;
                _jobs_info[job_idx].exec_hist.push_back(
                        make_pair(old_power, new_part_dur));
                _running_jobs[job_idx].exec_hist.push_back(
                        make_pair(old_power, new_part_dur));
            }

        }

        // Notify optimizer
        _opt->setPower(job_idx, _jobs_info[job_idx].cur_power);
        _opt->setPowerPred(job_idx, _jobs_info[job_idx].power_pred);
        _opt->setFrequency(job_idx, _jobs_info[job_idx].frequency);
        _opt->setDur(job_idx, _jobs_info[job_idx].dur);

        // measure again the power (to update auxiliary structure)
        measure_power(cur_time);

        return true;
    }

    /* Simulate power measuring framework: here the power
     * measured corresponds to the job predicted power
     * (which could have been previously change by 
     * frequency modifications)         */
    void measure_power(long long time){

        _system_power = 0;
        for(auto np : _nodes_power)
            _nodes_power[np.first] = 0;
        for(auto rj : _running_jobs){
            _system_power += rj.second.cur_power;
            for(int n : rj.second.node_mapping)
                _nodes_power[n] += rj.second.cur_power_perNode;
        }

        _system_power_profile[time] = _system_power;
        _nodes_power_profile[time] = _nodes_power;
    }

    /* Dynamic power capping based on RAPL
     * The RAPL-based power capping is loosely based on
     * Ellsworth et al 2015,
     * "Dynamic Power Sharing for Higher Job Throughput"
     * , in particular the algorithm that decides how the amount
     * of power to allocate to each CPU
     *
     * This is the method deciding the amount of power to be
     * allocated to each node    */
    void enforce_dyn_power_cap(long long cur_time){
        int numdown;
        int temp_pow_alloc;

        // according to paper should be: _power_cap / number of nodes
        // when using only the dynamic control _power_cap has a fake value:
        // use _target_sys_power instead
        // in this implementation the user specified node power 
        // budget can prevail (if node_power_cap > _power_cap / #nodes
        // a violations of the overall cap may happen)

        int node_power_cap;
        node_power_cap = int(round((double) _target_sys_power / _nodes.size()));

        if(_dynPowerShare_firstMeasure){
            // initially, the system available power is allocated uniformly
            // to all nodes
            
            // identify nodes where the cap is currently not respected
            vector<int> nodes_overCap;
            for(auto np : _nodes_power){
                // allocate the node power
                if(node_power_cap < _node_min_alloc_power)
                    _nodes_alloc_power[np.first] = _node_min_alloc_power;
                else
                    _nodes_alloc_power[np.first] = node_power_cap;
            }

            _dynPowerShare_firstMeasure = 0;
        }
        else{
            double used_power = 0;
            for(auto np : _nodes_power)
                used_power += np.second;

            // power-capped scheduler with relaxed constraint or pure RAPL
            // after the first measure used for initialization
            // the power sharing algorithm starts working

            temp_pow_alloc = 0;
            for(auto na : _nodes_alloc_power)
                temp_pow_alloc += na.second;
           
            /* Alloc down */
            numdown = 0;
            long long alloc_sum = 0;
            long long downNodes_alloc_sum = 0;
            for(auto np : _nodes_power){
                if(np.second < 
                        _nodes_alloc_power[np.first] - _target_node_wastedPower){
                    _nodes_alloc_power[np.first] = max(
                            np.second + _target_node_wastedPower,
                            _node_min_alloc_power);
                    _nodes_reclaim[np.first] = 0;
                    ++numdown;
                    downNodes_alloc_sum += _nodes_alloc_power[np.first];
                }
                else{
                    _nodes_reclaim[np.first] = 1;
                }
                alloc_sum += _nodes_alloc_power[np.first];
            }

            if(numdown == 0 &&
                    alloc_sum + _nodes.size() >= _target_sys_power){
                alloc_sum = 0;
                for(auto np : _nodes_power){
                    if(_nodes_alloc_power[np.first] > node_power_cap){
                        _nodes_reclaim[np.first] = 1;
                        _nodes_alloc_power[np.first] = 
                            _nodes_alloc_power[np.first] - 
                            (_nodes_alloc_power[np.first] - node_power_cap) *
                            (1 - _reclaim_factor);
                    }
                    alloc_sum += _nodes_alloc_power[np.first];
                }
            }

            temp_pow_alloc = 0;
            for(auto na : _nodes_alloc_power)
                temp_pow_alloc += na.second;

            /* Alloc up */
            int u = 0;
            /* _power_cap is used by the dispatcher during scheduling 
             * phase: if we want to apply only dynamic power sharing 
             * its value its meaningless --> the power target 
             * should be used    */
            if(_nodes.size() > numdown)
                u = round((_target_sys_power - alloc_sum) / 
                   (double) (_nodes.size() - numdown));

            int spare_power = _target_sys_power - downNodes_alloc_sum;
            for(auto np : _nodes_power){
                if(_nodes_reclaim[np.first]){
                    _nodes_alloc_power[np.first] = min(
                            _nodes_alloc_power[np.first] + u,
                            _node_max_alloc_power);
                    spare_power -= _nodes_alloc_power[np.first];
                }
            }
        }

        temp_pow_alloc = 0;
        for(auto na : _nodes_alloc_power)
            temp_pow_alloc += na.second;
        
        /* After deciding the power allocation, apply RAPL */
        for(auto na : _nodes_alloc_power)
            apply_nodeRAPL(na.first, na.second, cur_time);
    }

    /* Emulate RAPL behaviour: all jobs running on the 
     * input node will be slowed down in order to
     * force the power consumption under the desired value.
     *
     * This is the method that decide how much power must be allocated
     * to each job running on the node passed as parameter  */
    bool apply_nodeRAPL(int node, int node_power_cap, long long cur_time){
        // if the desired reduced power is smaller than the idle 
        // node power then it's impossible to decrease the power
        if(node_power_cap < _node_min_alloc_power && 
                opt_type_ != 0 && opt_type_ != 2){
            return false;
        }

        bool result = true;

        // count the number of jobs not at max frequency
        int jobNotMaxFreq = 0;
        for(long long j : _node_running_jobs[node])
            if(_jobs_info[j].power_pred_perNode >
                    _jobs_info[j].cur_power_perNode)
                ++jobNotMaxFreq;

        if(_nodes_power[node] <= node_power_cap){
            if(jobNotMaxFreq == 0){
                /* if the power consumed by the node is already within the 
                 * budget, and no job on the node was previously slowed, 
                 * RAPL doesn't need to
                 * change frequency/voltage to reduce power  */
            }
            else{
                /* if the power consumed by the node is already within the 
                 * budget but some jobs on the node were previously slowed, 
                 * the frequency/voltage can be increased by RAPL  */
                
                // we try to uniformly reduce the power consumption of all 
                // interested jobs
                map<long long, double> pow_weights;
                for(long long j : _node_running_jobs[node]){
                    double pow_weight = _nodes_power[node] / 
                        (double) _jobs_info[j].cur_power_perNode;
                    pow_weights[j] = pow_weight;
                }
                for(long long j : _node_running_jobs[node]){
                    double pow_weight = _nodes_power[node] / 
                        (double) _jobs_info[j].cur_power_perNode;

                    // power allocation based on current power consumption
                    int new_job_alloc_power = node_power_cap / pow_weights[j];

                    // uniform power allocation to all jobs running on the node
                    if(new_job_alloc_power == 0)
                        new_job_alloc_power = 1;
                    bool setFreq_result = setJob_FreqPowDur_dynamic(
                            j, new_job_alloc_power, cur_time);
                }
            }
        }
        /* if the power exceeds the available budgets, all jobs
         * running on the nodes will be slowed by RAPL  */
        else{
            // we try to uniformly reduce the power consumption of all 
            // interested jobs
            map<long long, double> pow_weights;
            for(long long j : _node_running_jobs[node]){
                double pow_weight = _nodes_power[node] / 
                    (double) _jobs_info[j].cur_power_perNode;
                pow_weights[j] = pow_weight;
            }
            for(long long j : _node_running_jobs[node]){
                // power allocation based on current power consumption
                int new_job_alloc_power = node_power_cap / pow_weights[j];

                // uniform power allocation to all jobs running on the node
                if(new_job_alloc_power == 0)
                    new_job_alloc_power = 1;

                bool setFreq_result = setJob_FreqPowDur_dynamic(
                        j, new_job_alloc_power, cur_time);
                if(!setFreq_result){
                    result = false;
                }
            }
        }
        return result;
    }

    /* Return the maximum system power consumption.
     * Computed using the input scheduler and
     * without power cap  */
    double get_max_power_consumption(
            long long from_time, long long until_time){
        double max_sysPower = 0;

        for(auto spp : _system_power_profile){
            // skip previous event
            if(spp.first < from_time)
                continue;

            // stop when reaches the last event
            if(spp.first > until_time)
                break;

            if(spp.second > max_sysPower)
                max_sysPower = spp.second;
        }
        return max_sysPower;
    }

    /* Function to compute information regarding the
     * system power 
     * - a time window is given as input */
    void analyse_power_usage(
            long long from_time, long long until_time){
        double avg_sysPower = 0;
        map<int, double> avg_nodePow;
        int measures_count = 0;
        long long total_time = 0;
        long long time_overTarget = 0;
        double avg_trgtDistance = 0;
        double avg_dist_overTrgt = 0;
        double avg_dist_underTrgt = 0;
        map<int, long long> time_overTarget_node;
        map<int, double> avg_overTarget_node;
        double mean_time_overTarget_nodes = 0;
        double mean_distance_overTarget_nodes = 0;
        double mean_nodePower = 0;
        double max_sysPower = 0;

        long long longest_timeOverTarget = 0;
        vector<long long> times_overTarget;
        double avg_time_overTarget;

        int energy_available_tot = 0;
        int energy_used_tot = 0;

        for(auto np : _nodes_power){
            avg_nodePow[np.first] = 0;
            time_overTarget_node[np.first] = 0;
            avg_overTarget_node[np.first] = 0;
        }

        long long prev_time = from_time;
        for(auto spp : _system_power_profile){
            // skip previous event
            if(spp.first < from_time)
                continue;

            // stop when reaches the last event
            if(spp.first > until_time)
                break;

            avg_sysPower += spp.second;
            ++measures_count;

            if(spp.second > max_sysPower)
                max_sysPower = spp.second;

            long long time = spp.first;
            long long time_dif = time - prev_time;
            total_time += time_dif;

            energy_available_tot += _target_sys_power * time_dif;
            energy_used_tot += spp.second * time_dif;

            double trgtDistance = _target_sys_power - spp.second;
            trgtDistance /= _target_sys_power;

            avg_trgtDistance += trgtDistance;
            if(spp.second > _target_sys_power){
                time_overTarget += time_dif;
                avg_dist_overTrgt += trgtDistance;
            }
            else
                avg_dist_underTrgt += trgtDistance;

            prev_time = time;
        }
        avg_trgtDistance /= measures_count;
        avg_dist_underTrgt /= measures_count;
        avg_dist_overTrgt /= measures_count;

        prev_time = from_time;
        for(auto npp : _nodes_power_profile){
            // skip previous event
            if(npp.first < from_time)
                continue;

            // stop when reaches the last event
            if(npp.first > until_time)
                break;

            long long time = npp.first;
            long long time_dif = time - prev_time;

            for(auto np : npp.second){
                avg_nodePow[np.first] += np.second;

                double trgtDistance = _target_node_power - np.second;
                trgtDistance /= _target_node_power;

                if(np.second > _target_node_power){
                    time_overTarget_node[np.first] += time_dif;
                    avg_overTarget_node[np.first] += trgtDistance;
                }
            }
            prev_time = time;
        }

        avg_sysPower /= (double) measures_count;
        for(auto np : _nodes_power){
            avg_nodePow[np.first] /= double(measures_count);
            avg_overTarget_node[np.first] /= double(measures_count);
            time_overTarget_node[np.first] /= double(measures_count);

            mean_time_overTarget_nodes += time_overTarget_node[np.first];
            mean_distance_overTarget_nodes += avg_overTarget_node[np.first];
            mean_nodePower += avg_nodePow[np.first];
        }
        mean_time_overTarget_nodes /= _nodes_power.size();
        mean_distance_overTarget_nodes /= _nodes_power.size();
        mean_nodePower /= _nodes_power.size();

        cout << "System average power: " << avg_sysPower << endl;
        cout << "Tot time: " << total_time << endl;
        cout << "Time over target (%) " 
            << (double)time_overTarget / total_time * 100 << endl;
        cout << "Avg distance from target: " << avg_trgtDistance << endl;
        cout << "Avg. over target: " << avg_dist_overTrgt << endl;
        cout << "Avg. under target: " << avg_dist_underTrgt << endl;
        cout << "Node average power: " << mean_nodePower << endl;
        cout << "Time over target (%), per node, avg among nodes: ";
        cout << mean_time_overTarget_nodes;
        cout << endl;
        cout << "Avg. distance from node target (over only), avg among nodes: ";
        cout << mean_distance_overTarget_nodes;
        cout << endl;

        cout << "Total available energy: " << energy_available_tot << endl;
        cout << "Total used energy: " << energy_used_tot << endl;
        cout << "Used/Available %: " << 
            (double) energy_used_tot / energy_available_tot * 100 << endl;

        cout << "System max power: " << max_sysPower << endl;

        cout << "Measures Count: " << measures_count << endl;

        double avg_jobs_perfModel_factors = 0;
        for(auto it : jobs_perfModel_factors_)
            avg_jobs_perfModel_factors += it.second;
        avg_jobs_perfModel_factors /= jobs_perfModel_factors_.size();
        cout << "Avg. jobs_perfModel_factors: " << 
            avg_jobs_perfModel_factors << endl;
    }


    /* Power budget have been exceeded
     * Typically the dispatcher respond to power cap variations by computing
     * a new schedule - if needed jobs in the queue are postponed until 
     * the power budget rises again or some jobs complete. 
     * The "excessive power" situation arises when current system power
     * consumption exceeds the new power constraint:
     * - no new jobs can start
     * - running jobs need to be slowed down  */
    void manage_excessive_power(long long power_slack,
            long long cur_time){
        /* There are no current stable implementation to address this 
         * problem. Still subject of current research. 
         * Relatively simple solution would be to apply DVFS/RAPL 
         * to already running jobs  */
    }

    /* Method to generate different startup situations. 
     * The system may be already partially (even fully) occupied
     * at the beginning of the simulation  */
    void initiate_startup(){
        /* Very simple dummy jobs: each one occupies a single node
         * and requires half/all of the cores in the node (and 1/8 of the
         * total memory). The power consumption is negligible (or not). 
         * As many job as to fill the machine in the 
         * percentage selected as input */

        int power = 5;

        int nfake_jobs = double(_nodes.size()) / 100 * startup_hotness_;
        for(int i = 0; i < nfake_jobs; ++i){

            _job_info j_info;
            j_info.job_idx = _jobs.size() * 2 + i;
            if(i < nfake_jobs / 2)
                j_info.dur = 600;
            else
                j_info.dur = 1200;
            j_info.original_dur = j_info.dur;
            j_info.ewt = 1800;
            j_info.qet = _ref_time;
            j_info.power_pred = power;
            j_info.power_pred_perNode = power;
            j_info.cur_power = power;
            j_info.cur_power_perNode = power;
            j_info.frequency = -1;
            j_info.energy = -1;

            // nodes mapping
            int node;
            j_info.start_time = _ref_time;
            j_info.end_time = _ref_time + j_info.dur;

            // half of the nodes use high nodes, the other half low nodes
            if(i < nfake_jobs / 2){
                j_info.node_type = 0;
                node = i;
            }
            else{
                j_info.node_type = 1;
                node = _nodeType_threshold + i - nfake_jobs / 2;
            }

            j_info.node_mapping.push_back(node);
            j_info.application_type = 0;

            int nunits = 1;
            vector<vector<long long> > unit_reqs;
            j_info.reqs.resize(nunits);
            j_info.tot_reqs.resize(_ndims,0);
            int ncores;
            for (int u = 0; u < nunits; ++u) {
                j_info.reqs[u].resize(_ndims);
                // get the requirement of each job unit
                // (and store them for the internal job_info map)
                long long rcores = _nodes[node].getCoresNumber();
                ncores = rcores;
                j_info.reqs[u][0] = rcores;
                j_info.tot_reqs[0] += rcores;
                long long rgpus = 0;
                j_info.reqs[u][1] = rgpus;
                j_info.tot_reqs[1] += rgpus;
                long long rmics = 0;
                j_info.reqs[u][2] = rmics;
                j_info.tot_reqs[2] += rmics;
                long long rmem = _nodes[node].getTotalMemory() / 8;
                j_info.reqs[u][3] = rmem;
                j_info.tot_reqs[3] += rmem;
                // store the unit requirements
                unit_reqs.push_back({rcores, rgpus, rmics, rmem});
            }

            // add the job to the optimizer
            _opt->addJob(j_info.job_idx, j_info.dur, j_info.power_pred, 
                    j_info.cur_power, j_info.qet, j_info.ewt, 
                    j_info.application_type, unit_reqs);
            _opt->setDur(j_info.job_idx, j_info.dur);
            _opt->setPower(j_info.job_idx, j_info.cur_power);
            _opt->setPowerPred(j_info.job_idx, j_info.power_pred);
            _opt->setFrequency(j_info.job_idx,-1);
 		    _opt->setStartAndMapping(j_info.job_idx, j_info.start_time,
                    j_info.node_mapping);
      
            // store the information about the job in the internal map	
            _jobs_info[j_info.job_idx] = {j_info};
            _running_jobs[j_info.job_idx] = {j_info};

            vector<int> exec_nodes = j_info.node_mapping;
            for(int en : exec_nodes){
                if(find(_node_running_jobs[en].begin(), 
                            _node_running_jobs[en].end(), j_info.job_idx) 
                        == _node_running_jobs[en].end() )
                    _node_running_jobs[en].push_back(j_info.job_idx);
            }

			// update cached resources capacities
			update_caps(j_info.job_idx, -1, j_info.node_mapping);
            // update power
			update_power(j_info.job_idx, 1);

            // add ending event
            long long end_time = j_info.end_time;
			auto it = _events.find(end_time);
			if (it != _events.end())
				it->second.push_back({j_info.job_idx, job_ending});
			else _events[end_time] = {{j_info.job_idx, job_ending}};

            _fake_jobs.push_back(j_info.job_idx);

            // compute job powerPerfModel factor (to be used 
            // also by some optimiezers - those using RAPL)
            compute_job_perfModel_factor(
                    j_info.job_idx, j_info.application_type);
        }

    }

public:
    void setSystemPowerCap(double power_cap){
        assert(power_cap > 0);
        _power_cap = power_cap;
    }

    double getSystemCurrentPower(){
        return _current_power;
    }

    double getSystemPower_atTime(long long time){
        double power;
        auto it = _historical_powers.find(time);
        (it != _historical_powers.end()) ? power = it->second : power = -1;
        return power;
    }

    /* Constructor */
	OnlineDispatcher(Optimizer* opt,
                Predictor* predictor,
				vector<Job>& jobs,
				vector<JobQueue>& qs,
				vector<Node>& nodes,
				long long ref_time,
				int ndims,
                long long power_cap,
                long long target_sys_power,
                long long target_node_power,
                map<long long, int>& power_caps,
                int freqAssign_mode,
                int dynControl_mode,
                double perfModel_factor,
                int startup_hotness,
                int opt_type,
                string job_instance)
		: _opt(opt), _predictor(predictor), _jobs(jobs), 
        _queues(qs), _nodes(nodes), _ref_time(ref_time), 
        _ndims(ndims), _power_cap(power_cap),
        _power_caps(power_caps),
        _dyn_RAPL_mode(dynControl_mode),
        _freqAssign_mode(freqAssign_mode),
        perfModel_factor_(perfModel_factor),
        startup_hotness_(startup_hotness),
        opt_type_(opt_type) {

        unsigned int hashed_job_instance = std::hash<string>{}(job_instance);
        seed_ = hashed_job_instance;

		// add an event at the reference time
		_events[_ref_time] = {};
        prev_notMeasure_time_ = _ref_time;

        (target_sys_power == -1) ? 
            _target_sys_power = power_cap : 
            _target_sys_power = target_sys_power;
        (target_node_power == -1) ?
            _target_node_power = int(round(_target_sys_power / nodes.size())) 
            : _target_node_power = target_node_power;

        // add fake events corresponding to power cap variations
        // assumption: only one power cap for each time
        _power_cap_variations_left = 0;
        for(auto pce : power_caps){
            _power_cap_variations_left += 1;
            long long time = pce.first;
            auto it = _events.find(time);
            if(it == _events.end())
                _events[time] = {{-pce.second, power_cap_variation}};
            else
                it->second.push_back({-pce.second, power_cap_variation});
        }

        // add fake events corresponding to power measurements
        long long max_mks = 0;
        _power_measurements_left = 0;
        for(auto j : jobs)
            max_mks += j.getEstimatedDuration();
        // the power measurement evenents have a fake event id
        // positive number that cannot be confonded with the jobs ids
        long long fake_evnt_id = jobs.size() + 50;
        long long time_counter = 0;
        while(time_counter < max_mks){
            time_counter += _power_measure_step_fixed;
            _power_measurements_left++;
            long long time = _ref_time + time_counter;
            auto it = _events.find(time);
            if(it == _events.end())
                _events[time] = {{fake_evnt_id, power_measure}};
            else
                it->second.push_back({fake_evnt_id, power_measure});
        }

        // set the initial used resources
        _cumul_res_usgs.resize(_ndims, 0);

		_cumul_res_caps.resize(_ndims, 0);
		_max_cumul_res_caps.resize(_ndims, 0);
		// init cached resource capacities
		for(auto node : nodes){
			vector<int> res_cap(ndims);
			res_cap[0] = node.getCoresNumber();
			_cumul_res_caps[0] += node.getCoresNumber();
			_max_cumul_res_caps[0] += node.getCoresNumber();
			res_cap[1] = node.getGPUsNumber();
			_cumul_res_caps[1] += node.getGPUsNumber();
			_max_cumul_res_caps[1] += node.getGPUsNumber();
			res_cap[2] = node.getMICsNumber();
			_cumul_res_caps[2] += node.getMICsNumber();
			_max_cumul_res_caps[2] += node.getMICsNumber();
			res_cap[3] = node.getTotalMemory();
			_cumul_res_caps[3] += node.getTotalMemory();
			_max_cumul_res_caps[3] += node.getTotalMemory();
			_res_caps.push_back(res_cap);

            vector<int> res_usg;
            res_usg.resize(ndims, 0);
            _res_usgs.push_back(res_usg);

            _nodes_power[node.getNodeNumber()] = 0;

            _dynPowerShare_firstMeasure = 1;
		}

        _min_reqs.resize(_ndims, std::numeric_limits<int>::max());

        _system_power = 0;

        /* This depends on the target system characteristics:
         * the nodes with lower ids (first half) are low_freq,
         * nodes ids in the second half are high_freq  */
        _nodeType_threshold = nodes.size() / 2;

        CompatibilityChecker chk; // default checker
        IntervalManager mng; // default manager

        // Add all nodes to the problem
        for (int idx = 0; idx < nodes.size(); ++idx) {
            // Obtain a reference to the node
            Node& node = nodes[idx];
            
            // Obtain the node capacity for all resource types
            long long ccores = node.getCoresNumber();
            long long cgpus = node.getGPUsNumber();
            long long cmics = node.getMICsNumber();
            long long cmem = node.getTotalMemory();
        }

        // startup situation
        initiate_startup();
	}

	void writeSchedule(std::ostream& out){
		_opt->writeSched(out);
	}

    /* This function controls if it would make sense to compute a
     * new schedule or not: it makes sense only if the current 
     * available resources capacity is larger than the min amount
     * of resources asked (by one of the jobs waiting to 
     * be scheduled)  */
    bool is_schedule_needed(){
        // if there is at least one job that can fit with the current capacities
        for(auto job : _waiting_jobs){
            bool fit = true;
            for(int d = 0; d < _ndims; ++d)
                if(job.second.tot_reqs[d] > _cumul_res_caps[d])
                   fit = false;
            if(fit)
                return true;
        }
        return false;
    }

    /* Main dispatching function.
     * After having initialized the internal status, the methods begins
     * processing time events (starting from the reference time (i.e. the 
     * arrival of the first job in the system). 
     * The dispatching continues until there are remaining events (i.e. new
     * job arrivals, job completions, power cap variations, power measurements).
     * At each scheduling event (job arrival/completion, power cap variation)
     * a new schedule is computed (if needed) using the chosen optimizer (i.e.
     * EASY-BF)                 */
	void dispatch(bool dry_run){
		CustomTimer iterTimer;

		// number of schedules computed
		int n_scheds = 0;

        long long last_time = _ref_time;
		vector<long long> jobs_to_schedule; // jobs that need to be scheduled

		for (int i = 0; i < _jobs.size(); ++i) {
			// Jobs that do not yet have a start time must be scheduled
			jobs_to_schedule.push_back(i);
		}

		// loop as long as:
		// 1) there are time points to process
		// 2) there are jobs to dispatch
		// In practice the loop should always end because of condition 2
		for (auto evt : _events) {
			iterTimer.reset();
			iterTimer.start();

			// tells if a new schedule must be computed or not
			bool compute_schedule = false;

			// access the current time
			long long cur_time = evt.first;
            long long time_dif = cur_time - last_time;
            last_time = cur_time;

            /* If we don't need to have power cap variations or 
             * power measuring events (it was useful to test pure scheduling
             * and allocation policies without considering a power budget)
             * the dispatching can stop when all jobs have been scheduled.
             * If this is the case, de-commenting the following lines will 
             * quicken the simulation */
			//if (jobs_to_schedule.size() == 0) {
			//	cout << "no more jobs to schedule" << endl;
			//	break; // EARLY EXIT
			//}
            
            /* Stop processing events when
             * all jobs have been scheduled and we know there won't be more
             * power constraint variations - this is clearly possible
             * only because in our simulator we know in advance the 
             * number of power variations.
             * With this condition the simulation stop when all jobs have 
             * been scheduled and no more power variations are expected:
             * there could be still power measuring events pending but we
             * disregard them     */
			if (jobs_to_schedule.size() == 0 && 
                    _power_cap_variations_left == 0) {
				break; // EARLY EXIT
			}

            /* This last condition is used to process all the events,
             * included the power measurements. The simulation takes
             * longer and some results may change (in RAPL case).  */
			//if (jobs_to_schedule.size() == 0 && 
            //        _power_cap_variations_left == 0 &&
            //        _power_measurements_left == 0) {
            //    break; // EXIT
            //}
            
            // check if the event list contains only measurements events; 
            // in this case we don't print some information
            bool only_power_measure_event = true;
			for(auto je : evt.second)
				if(je.type != power_measure) 
                    only_power_measure_event = false;

            if(!only_power_measure_event){
                //cout << "=== processing time " << cur_time - _ref_time << 
                //    " (" << cur_time << " - " << Util::timeToStr(cur_time) 
                //    << ")" << endl;
                //cout << "Current power cap value: " << _power_cap << 
                //    " - system power consumption: " << getSystemCurrentPower() 
                //        << endl;
            }

            // add resources usage ratio at event 
            vector<double> usg_ratio(_ndims);
            for(int i = 0; i < _ndims; i++){
                double ratio = (double) _cumul_res_usgs[i] /
                        (double) _max_cumul_res_caps[i];
                usg_ratio[i] = ratio;
            }
            _sys_util[cur_time] = usg_ratio;

			if(evt.second.size() == 0)
				compute_schedule = true;

            vector<long long> just_arrived;

			for (_job_evt jdesc : evt.second) {

				long long job_idx = jdesc.job_ext_idx;
				if (jdesc.type == job_starting) {
				}
				else if (jdesc.type == job_arriving) {
					// An arriving job is treated as a schedulable job from now on
					// In the text-based framework, arriving jobs might be in the 
					// jobs_to_schedule list already --> in this case we should not
					// add them again
					if(std::find(jobs_to_schedule.begin(), jobs_to_schedule.end(), job_idx) 
							== jobs_to_schedule.end())
						jobs_to_schedule.push_back(job_idx);

					if(std::find(just_arrived.begin(), just_arrived.end(), job_idx) 
							== just_arrived.end())
						just_arrived.push_back(job_idx);
				}
				else if (jdesc.type == job_ending){
					// remove the job from the optimizer
					_opt->remJob(job_idx);

                    if(job_idx <= _jobs.size()){
                        // update cached resources capacities
                        update_caps(job_idx, 1, _final_schedule[job_idx].mapping);

                        // update power
                        update_power(job_idx, -1);

                        // compute energy consumed by job in its life
                        _jobs_info[job_idx].energy = 
                            get_job_final_energy(job_idx, cur_time);

                    }
                    else{
                        // update cached resources capacities
                        update_caps(job_idx, 1, _jobs_info[job_idx].node_mapping);
                        // update power
                        update_power(job_idx, -1);

                        // remove the job from the fake-jobs list
                        if(find(_fake_jobs.begin(), 
                                    _fake_jobs.end(), job_idx) 
                                != _fake_jobs.end() )
                            _fake_jobs.erase(
                                    remove(_fake_jobs.begin(), 
                                        _fake_jobs.end(), job_idx), 
                                    _fake_jobs.end());
                    }

                    auto itr = _running_jobs.find(job_idx);
                    assert(itr != _running_jobs.end());

                    // add ending job to completed set
                    _ended_jobs[job_idx] = itr->second;

                    // delete ending job from running set
                    _running_jobs.erase(itr);

                    // delete ending job from node running set
                    vector<int> exec_nodes = _jobs_info[job_idx].node_mapping;
                    for(int en : exec_nodes){
                        if(find(_node_running_jobs[en].begin(), 
                                    _node_running_jobs[en].end(), job_idx) 
                                != _node_running_jobs[en].end() )
                            _node_running_jobs[en].erase(
                                    remove(_node_running_jobs[en].begin(), 
                                        _node_running_jobs[en].end(), job_idx), 
                                    _node_running_jobs[en].end());
                    }

				}
                // power cap variation
                else if(jdesc.type == power_cap_variation){
                    int power_cap = abs(job_idx);
                    // set the new power budget
                    assert(power_cap > 0);
                    setSystemPowerCap(power_cap);
                    _power_cap_variations_left--;
                }
                // power measurement
                else if(jdesc.type == power_measure){
                    // measure current system and node power
                    _power_measurements_left--;
                    measure_power(cur_time);

                    /* If we are employing dynamic power capping methods,
                     * a power measurements event triggers the mechanisms  */
                    if(_dyn_RAPL_mode != -1){
                        /* Enforce the power cap: problem in this implementation
                         * the power cap is enforced only AFTER scheduling -
                         * the power consumption could already exceed the limit.
                         * It's important that this periodical check and enforcing
                         * is performed very frequently   */
                         enforce_dyn_power_cap(cur_time);
                    }
                }
			}
			
			// Move to the next event if the reference time (i.e. "now") has
			// not yet been reached.
			if (cur_time < _ref_time) {
				continue; // EARLY EXIT
			}

			if (jobs_to_schedule.size() == 0) {
				continue;
			}

            /* Dispatcher-level power control: before trying to compute
             * a new schedule and start new jobs the dispatcher compares
             * the current system power with the power cap to check if 
             * the constraint is still respected --> useful only if the 
             * power cap is not fixed (TODO: add this feature)
             * - we check the estimated power consumption of each job to
             *   be scheduled; if none can fit within the power budget
             *   no new job is started
             * - if the power constraint is already broken we do not 
             *   start new jobs and we try to slow down (decrease frequency
             *   and power, increase duration) those already running */
            int nfittin_jobs = 1;

            // no job consumes a (predicted) power small enough 
            if(nfittin_jobs == 0){
                // the power budget is exceeded by the currently running jobs
                /* The system power can be larger than the power cap if we
                 * employ RAPL-like mechanism: the power consumption of the 
                 * scheduled jobs will be reduced by RAPL   */
                if(int(getSystemCurrentPower()) > _power_cap &&
                        _dyn_RAPL_mode == -1){
                    long long power_slack = getSystemCurrentPower() - _power_cap;

                    /* if we reach this point to ensure not to exceed the power 
                     * budget we must reduce the power consumption of the 
                     * currently running jobs (the power must decrease of 
                     * power slack    */
                    manage_excessive_power(power_slack, cur_time);
                }
                continue;
            }

            /* The power budget may change at run-time: we must let the
             * optimizer know */
            _opt->setMaxPower(_power_cap);

			// prepare the new list of ranked jobs
			vector<long long> new_jobs_to_schedule;
			map<long long, long long> scheduled_jobs;
			int sched_job_counter = 0;

			// try to dispatch each job
			for (auto i : jobs_to_schedule) {
				// Skip the job if its QET has not yet been reached 
				if (_jobs[i].getEnterQueueTime() > cur_time) {
					// Add a new event at the arrival time, to be sure that it will
					// be processed
					long long qet = _jobs[i].getEnterQueueTime();
					auto it = _events.find(qet); 
					if (it == _events.end())
						_events[qet] = {{i, job_arriving}};
					else{
						// check that the arriving event has not been added yet
						bool already_in = false;
						for(auto job_evts : it->second)
							if(i == job_evts.job_ext_idx)
								already_in = true;
						if(!already_in)
							it->second.push_back({i, job_arriving});
					}
					continue;
				}

				// add to the optimizer those jobs which arrived in this time event 
                if(find(_added_jobs.begin(), _added_jobs.end(), i) 
                        == _added_jobs.end())
                    addJob(i);
				sched_job_counter++;
			}

			// schedule the jobs using the chosen optimizer
			long long tmp_job_idx;
			vector<int> tmp_unit_mapping;
            int tmp_frequency;

			typedef struct tmp_job_info{
				long long tmp_job_idx;
				vector<int> tmp_unit_mapping;
                int tmp_frequency;
			} tmp_job_info;	

			vector<tmp_job_info> started_jobs;
			started_jobs.clear();

			/* Compute a new schedule when:
			 * 1) a job arrives and there are enough available resources
			 * 2) a job terminates (freeing busy resources)
			 */
            if(!only_power_measure_event){
                for(auto je : evt.second){
                    //cout << "(" << je.job_ext_idx << "," << je.type << ") ";
                    if((je.type == job_arriving && check_res_avail(je.job_ext_idx))
                                || je.type == job_ending)
                        compute_schedule = true;
                }
            }
            if(!compute_schedule){
                continue;
            }
            if(!is_schedule_needed()){
                continue;
            }

			n_scheds++;

            /* This is the call to the dispatching function to the chosen 
             * optimizer: _opt->nextJob().
             * Inside nextJob() a start time and resource allocation is 
             * assigned to each job to be scheduled; some jobs will have a start 
             * time equal to cur_time, other will be scheduled for future events.
             *
             * nextJob() will send as output only those jobs with a start time
             * equal to the current time (thus this value is passed as 
             * parameter). Consequently, the online dispatcher only starts the 
             * jobs with that start time - the remaining ones
             * will be considered at future events (and possibly their start
             * times could change)      */
			while(_opt->nextJob(
                        cur_time, tmp_job_idx, 
                        tmp_unit_mapping, 
                        tmp_frequency)){

				scheduled_jobs[tmp_job_idx] = {cur_time};

				// Check if an event for the end time exists. If this is the
				// case, register the jobs as "ending" at the event. Otherwise,
				// build a new job vector for the event containing only "i".
                
                /* Important: the optimizer takes its decision based on the 
                 * "estimated" duration (the values indicated by users at 
                 * submission time - the durations registered in _opt).
                 * In reality, jobs can finish before the estimated duration
                 * and the online dispatcher must set the end event corresponding
                 * to a job just started according to the real duration.
                 * In our case we have the real jobs' durations thanks to the
                 * historical Eurora's traces (even the synthetic benchmarks
                 * have both an estimated and a real duration)    */
                
				/* The job duration is the Real one - the estimated one
				 * is still used when computing a schedule */
				int realDur;
                /* Due to some data collection problems, we can have in certain 
                 * situation real duration greater than the expected one.
                 * This is a bug due to the information gathering system and here
                 * we force the real duration to smaller than the estimated one */
				(_jobs[tmp_job_idx].getEstimatedDuration() < _jobs[tmp_job_idx].getRealDuration() ? 
					 realDur = _jobs[tmp_job_idx].getEstimatedDuration() :
					 realDur = _jobs[tmp_job_idx].getRealDuration());

				long long end_time = cur_time + realDur;
				auto it = _events.find(end_time);
				if (it != _events.end())
					it->second.push_back({tmp_job_idx, job_ending});
				else _events[end_time] = {{tmp_job_idx, job_ending}};

                // we keep track of the started jobs
				started_jobs.push_back({tmp_job_idx,tmp_unit_mapping,tmp_frequency});
				
				// Check if a starting event for the start time exists. If this is the
				// case, register the jobs as "starting" at the event. Otherwise,
				// build a new job vector for the event containing only "i".
				long long start_time = cur_time;
				it = _events.find(start_time);
				if (it != _events.end())
					it->second.push_back({tmp_job_idx, job_starting});
				else _events[start_time] = {{tmp_job_idx, job_starting}};

                // update the map with the info for each job
                _jobs_info[tmp_job_idx].start_time = start_time;
                _jobs_info[tmp_job_idx].end_time = end_time;
                _jobs_info[tmp_job_idx].node_mapping = tmp_unit_mapping;

				// update cached resources capacities
				update_caps(tmp_job_idx, -1, tmp_unit_mapping);
                // update power
				update_power(tmp_job_idx, 1);

                // update waiting jobs set
                auto w_itr = _waiting_jobs.find(tmp_job_idx);
                if(w_itr != _waiting_jobs.end())
                    // delete ending job from waiting set
                    _waiting_jobs.erase(w_itr);

                auto itr = _running_jobs.find(tmp_job_idx);
                // only if the job was not seen before
                if(itr == _running_jobs.end())
                    _running_jobs[tmp_job_idx] = _jobs_info[tmp_job_idx];
                
                vector<int> exec_nodes = tmp_unit_mapping;
                for(int en : exec_nodes){
                    if(find(_node_running_jobs[en].begin(), 
                                _node_running_jobs[en].end(), tmp_job_idx) 
                            == _node_running_jobs[en].end() )
                        _node_running_jobs[en].push_back(tmp_job_idx);
                }
 
				// update final schedule
				// if the job is not the final schedule yet, add it
				auto sched_it = _final_schedule.find(tmp_job_idx);
				_job_schedule tmp_jsched;
				tmp_jsched.start = start_time;
				tmp_jsched.end = end_time;
				tmp_jsched.mapping = tmp_unit_mapping;

				if (sched_it == _final_schedule.end())
					_final_schedule[tmp_job_idx] = {tmp_jsched};

				tmp_unit_mapping.clear();
			}

			// notify optimizer of the newly dispatched jobs
			for(auto job : started_jobs){
				_opt->setStartAndMapping(job.tmp_job_idx, cur_time, 
                        job.tmp_unit_mapping);
				_opt->setDur(job.tmp_job_idx, 
                        _jobs[job.tmp_job_idx].getEstimatedDuration());
				_opt->setDur(job.tmp_job_idx, 
                        _jobs_info[job.tmp_job_idx].dur);
                _opt->setPower(job.tmp_job_idx, 
                        _jobs_info[job.tmp_job_idx].cur_power);
                _opt->setPowerPred(job.tmp_job_idx, 
                        _jobs_info[job.tmp_job_idx].power_pred);
                _opt->setFrequency(job.tmp_job_idx, 
                        _jobs_info[job.tmp_job_idx].frequency);
			}
           
			// put the remaining jobs (those not scheduled yet) in the new_ranked_jobs
			for(auto job_idx : jobs_to_schedule){
				auto it = scheduled_jobs.find(job_idx);
				if(it == scheduled_jobs.end())
					new_jobs_to_schedule.push_back(job_idx);
			}

            auto it = _historical_powers.find(cur_time);
            if(it == _historical_powers.end())
                _historical_powers[cur_time] = _current_power;

			// update the list of ranked jobs
			jobs_to_schedule = new_jobs_to_schedule;

			iterTimer.stop();

            // update job info with node mappings
            for(auto job_sched : _final_schedule){
                int job_idx = job_sched.first;

                /* The 2 following assumptions strictly depend on the target
                 * system (Eurora). They could be dropped for a more 
                 * general simulator */

                // assumption 1): a job with a job unit on a 3.1GHz node is considered
                // a 3.1 node type even if all remaining job units executed
                // on 2.1GHz nodes
                int node_type = 0;
                for(int u = 0; u < _jobs[job_idx].getNumberOfNodes(); ++u)
                    if(job_sched.second.mapping[u] >= _nodeType_threshold)
                        node_type = 1;

                /* assumption 2): all job units must run on nodes of 
                 * the same type 
                 * - when a job is mapped on a node for the first time its frequency 
                 *   is set to the max  
                 * - the frequency must be set only when the job is started: 
                 *   if the frequency is already assigned we must not change it
                 *   (we could interfere with previous slow-downs) */
                if(_jobs_info[job_idx].frequency == -1){
                    (node_type == 0) ? _jobs_info[job_idx].frequency = 2100 :
                        _jobs_info[job_idx].frequency = 3400;
                    (node_type == 0) ? _opt->setFrequency(job_idx,2100) :
                        _opt->setFrequency(job_idx,3400);
                }

                _jobs_info[job_idx].node_type = node_type;
                _jobs_info[job_idx].node_mapping = job_sched.second.mapping;


                /* In _final_schedule there are all jobs scheduled (present and past)
                 * Here we must control that the job whose node_type we're setting
                 * has not already finished (if we don't check the setting operation
                 * will automatically create a corresponding item in the 
                 * _running_jobs map)  */
                auto itr = _ended_jobs.find(job_idx);
                if(itr == _ended_jobs.end()){ // the job has NOT ended
                    _running_jobs[job_idx].node_type = node_type;
                    _running_jobs[job_idx].node_mapping = job_sched.second.mapping;
                    if(_running_jobs[job_idx].frequency == -1)
                        (node_type == 0) ? _running_jobs[job_idx].frequency = 2100 :
                            _running_jobs[job_idx].frequency = 3400;
                }

			}

            /* Quite verbose print of all internal events */
			//cout << "(OnlineDispatcher) Events status recap --------------" << endl;
			//for(auto e : _events){
			//	vector<long long> added_jobs;
			//	
			//	cout << (e.first == cur_time ? "--->" : "" ) << " Time " << e.first << 
            //        " - " << ": " ;
			//	for(auto te: e.second){
			//		if(te.type != job_arriving){
			//			bool toMark = false;
			//			if(e.first == cur_time)
			//				added_jobs.push_back(te.job_ext_idx);
			//			toMark = find(added_jobs.begin(), 
            //                    added_jobs.end(), 
            //                    te.job_ext_idx) != added_jobs.end();
			//			cout << (toMark ? "**" : "" ) << "(" << 
            //                te.job_ext_idx << "; " ;
			//			cout << (te.type == job_starting ? "S" : "E" ) << 
            //                te.type << "), ";
			//		}
			//		else
			//			cout << "(" << te.job_ext_idx << "; A" << te.type << "), ";
			//	}
            //    cout << endl;
			//}
	
			_nscheduled_jobs.push_back(sched_job_counter);
			_treq_sched.push_back(iterTimer.getVal() / 1000.0);

		} // end of the event loop	

        /* Now we have final recap prints.
         * Dry-run: execution without power cap used to estimate
         * the maximum power consumption of a job instance (computed
         * using List Scheduler as optimizer).
         * The different output with or without dry-run is related to 
         * my experimental setting   */

        /* The alternative BSLD is computed excluding outliers */
        int alternative_bsld_threshold = 1000;
        if(!dry_run){
            cout << "****" << endl;	
            cout << "FINAL SCHEDULE " << endl;
            double avg_bsld_alt = 0;
            double avg_dur_increase = 0;
            for(auto job_sched : _final_schedule){
                cout << job_sched.first << "\tST: " << job_sched.second.start;
                cout << "\tFreq: " << _jobs_info[job_sched.first].frequency;
                cout << "\tFinal Duration: " << _jobs_info[job_sched.first].dur;
                cout << "\tOriginal Duration: " << 
                    _jobs_info[job_sched.first].original_dur;
                avg_dur_increase += _jobs_info[job_sched.first].dur /
                    _jobs_info[job_sched.first].original_dur;
                double wait_time = 
                    job_sched.second.start - _jobs_info[job_sched.first].qet;
                double run_time = 
                    _jobs_info[job_sched.first].end_time - 
                    _jobs_info[job_sched.first].start_time;
                cout << "\tWait Time: " << wait_time;
                double bsld = max((double)1,
                        (wait_time + run_time) / (double)
                        max(_bsld_threshold, 
                            (double)_jobs_info[job_sched.first].original_dur));
                if(bsld < alternative_bsld_threshold)
                    avg_bsld_alt += bsld;
                cout << "\tBSLD: " << bsld;
                cout << "\tUsed resources: ";
                for(int r : job_sched.second.mapping)
                    cout << r << ";";
                cout << endl;
            }
            avg_dur_increase /= _final_schedule.size();
            avg_bsld_alt = avg_bsld_alt / _jobs.size();
            cout << "Alternative BSLD: " << avg_bsld_alt << endl;
            //cout << "Avg. duration increase: " << avg_dur_increase << endl;
            cout << "&&" << endl;	
            printMetrics(cout);
            //cout << "&&" << endl;	
            //writeSolution_stdFormat(cout);
            //cout << "&&" << endl;	
            cout << "- # Computed Schedules: " << n_scheds << endl;
            double avg_nsched = 0;
            for(int ns : _nscheduled_jobs)
                avg_nsched += ns;
            avg_nsched /= double(_nscheduled_jobs.size());
            cout << "- Avg. # Jobs per schedule: " << avg_nsched << endl;
            double avg_treq = 0;
            for(double ts : _treq_sched)
                avg_treq += ts;
            avg_treq /= _treq_sched.size();
            cout << "- Avg. Schedule computation time: " << avg_treq << endl;
            analyse_power_usage(_ref_time, last_time);
        }
        else{
            double max_sysPower = get_max_power_consumption(_ref_time, last_time);
            cout << "Maximum power consumption: " << max_sysPower << endl;
        }
	}


    /* The makespan is a quite useless metric in online settings */
	double getMKS(){
		// Find the maximum EWT and the makespan
		double max_ewt = numeric_limits<double>::min();
		double mks = 0;

		for(auto job_sched : _final_schedule){
			int idx = job_sched.first;
			max_ewt = (_ref_time > _jobs_info[idx].ewt ? _ref_time :
                    _jobs_info[idx].ewt);
			mks = (mks > job_sched.second.end ? mks : job_sched.second.end);
		}
		mks -= _ref_time;
		return mks;
	}

	void printMetrics(ostream& out){
		// Compute some metrics
		double nqt = 0; // weighted queue time
		double tqt = 0; // total queue time
		vector<double> utl(_ndims, 0); // utilization
        double tbsld;  // ratio between time in the system and runtime
        long long total_energy = 0;  // the system consumption
        int njobs_atMaxFreq = 0;  // number of jobs runnign at max freq
        int nslowed_jobs = 0;  // number of slowed down jobs
        double perc_slowedJobs = 0;
        double avg_start_time = 0;
		for(auto job_sched : _final_schedule){
			int idx = job_sched.first;
            if(idx > _jobs.size())
                continue;
            avg_start_time += job_sched.second.start - _ref_time;
			// Waiting time for this job
			double wait_time = job_sched.second.start - _jobs_info[idx].qet;
			// Update total queue time
			tqt += wait_time;
			// Weight for this this job
			double wgt = 1.0 / _jobs_info[idx].ewt;
			// Update weighted queue time
			nqt += wgt * wait_time;
			// Compute the total utilization of each resource
			for (int d = 0; d < _ndims; ++d)
				utl[d] += _jobs_info[idx].tot_reqs[d] * _jobs_info[idx].dur;

            double run_time = _jobs_info[idx].end_time - _jobs_info[idx].start_time;

            if(_jobs_info[idx].dur == _jobs_info[idx].original_dur)
                njobs_atMaxFreq++;
            else{
                nslowed_jobs++;
            }

            /* bsld_threshold is used to exclude too short jobs from the computation */
            tbsld += max((double)1,
                    (wait_time + run_time) / (double)
                    max(_bsld_threshold, (double)_jobs_info[idx].original_dur));

            if(_jobs_info[idx].energy != -1)
                total_energy += _jobs_info[idx].energy;
		}
		// Average normalized queue time
		double avg_nqt = nqt / _jobs.size();
		avg_start_time = avg_start_time / _jobs.size();
			
		// One more metric (utilization ratio)
		vector<double> utl_ratio(_ndims, 0);
		for (int d = 0; d < _ndims; ++d)
			utl_ratio[d] = utl[d] / (_opt->getCap(d) * getMKS()) ;

        double avg_bsld = tbsld / _jobs.size();
			
		// Instance descriptor (utilization in an ideal schedule)
		vector<double> uis(_ndims, 0);
		for (auto job_sched : _final_schedule) {
			int idx = job_sched.first;
			for (int d = 0; d < _ndims; ++d) 
				uis[d] += _jobs_info[idx].tot_reqs[d] * _jobs_info[idx].dur;
		}
		double max_uis = 0;
		for (int d = 0; d < _ndims; ++d){
			uis[d] /= (double) _opt->getCap(d);
			max_uis = (max_uis > uis[d] ? max_uis : uis[d]);
		}
		for (int d = 0; d < _ndims; ++d)
			uis[d] /= max_uis;

        vector<double> avg_util(_ndims);
        vector<double> tot_util(_ndims);
        int n_event;
        for(auto event_util : _sys_util){
            vector<double> util = event_util.second;
            for(int d = 0; d < _ndims; ++d)
                tot_util[d] += util[d];
            n_event++;
        }
        for(int d = 0; d < _ndims; ++d)
            avg_util[d] = tot_util[d] / n_event;

        perc_slowedJobs = (double) nslowed_jobs / _jobs.size() * 100;
		out << "STATS:" << endl;
		out << "- makespan (sec): " << getMKS() << endl;
		out << "- total queue time (sec): " << tqt << endl;
		out << "- avg. normalized queue time (#EWTs): " << avg_nqt << endl;
		out << "- avg. start time (s): " << avg_start_time << endl;
		out << "- BSLD: " << avg_bsld << endl;
        out << "- Slowed jobs %: " << perc_slowedJobs << endl;
		out << "- Total Energy: " << total_energy / 1000 << endl;
		out << "- utilization ratio: " << utl_ratio[0];
		for (int d = 1; d < _ndims; ++d) out << " / " << utl_ratio[d];
		out << endl;
		out << "- utilization in an ideal schedule: " << uis[0];
		for (int d = 1; d < _ndims; ++d)
			cout << " / " << uis[d];
		out << endl;
        out << "- avg. system utilization: " << avg_util[0];
		for (int d = 1; d < _ndims; ++d)
			cout << " / " << avg_util[d];
		out << endl;
	}

	/* Write a solution in the standard format
	 * - print all the jobs started so far (even the completed ones)
	 */
	void writeSolution_stdFormat(ostream& out){
		for(auto job_sched : _final_schedule){
			int job_idx = job_sched.first;
			out << _jobs[job_idx].getJobId() << ";" <<  
                _jobs[job_idx].getJobName() << ";";
			out << _jobs[job_idx].getUserName() << ";" << 
                _jobs[job_idx].getQueue() << ";";
			out << Util::timeToStr(_jobs[job_idx].getEnterQueueTime()) << "__";
			for(int u = 0; u < _jobs[job_idx].getNumberOfNodes(); ++u){
				out << job_sched.second.mapping[u] << ";";
				out << _jobs_info[job_idx].reqs[u][0] << ";" << 
                    _jobs_info[job_idx].reqs[u][1] << ";";
				out << _jobs_info[job_idx].reqs[u][2] << ";" << 
                    _jobs_info[job_idx].reqs[u][3]*1024;
				out << "#";
			}
			out << "__" << Util::timeToStr(job_sched.second.start) << ";";
			out << Util::timeToStr(job_sched.second.end) << ";";
			out << _jobs[job_idx].getNumberOfNodes() << ";" << 
                _jobs[job_idx].getNumberOfCores() << ";";
			out << _jobs[job_idx].getMemory()/1024 << ";" << 
                _jobs_info[job_idx].dur << endl;
		}
	}

	/* Write the dispatcher state in standard format
	 *   - do not consider finished job (at the reference time)
	 *   - consider the running jobs with the solver computed start times
	 *   - use future knowledge (text-based only) for jobs not seen yet by dispatcher
	 */
	void writeDispState_stdFormat(ostream& out, long long ref_time){
		// started jobs
		for(auto job_sched : _final_schedule){
			int job_idx = job_sched.first;
			// consider only running jobs
			if(job_sched.second.end >= ref_time){
				out << _jobs[job_idx].getJobId() << ";" <<  
                    _jobs[job_idx].getJobName() << ";";
				out << _jobs[job_idx].getUserName() << ";" << 
                    _jobs[job_idx].getQueue() << ";";
				out << Util::timeToStr(_jobs[job_idx].getEnterQueueTime()) << "__";
				for(int u = 0; u < _jobs[job_idx].getNumberOfNodes(); ++u){
					out << job_sched.second.mapping[u] << ";";
					out << _jobs_info[job_idx].reqs[u][0] << ";" << 
                        _jobs_info[job_idx].reqs[u][1] << ";";
					out << _jobs_info[job_idx].reqs[u][2] << ";" << 
                        _jobs_info[job_idx].reqs[u][3]*1024;
					out << "#";
				}
				out << "__" << Util::timeToStr(job_sched.second.start) << ";";
				out << Util::timeToStr(job_sched.second.end) << ";";
				out << _jobs[job_idx].getNumberOfNodes() << ";" << 
                    _jobs[job_idx].getNumberOfCores() << ";";
				out << _jobs[job_idx].getMemory()/1024 << 
					";" << Util::timeHHMMToStr(_jobs_info[job_idx].dur);
				out << ";NA;NA;COMPLETED" << endl;
			}
		}
		// future jobs
		// TODO: very inefficient (but actually only debug purpose)
		// run through all job
		for(int job_idx = 0; job_idx < _jobs.size(); job_idx++){
			// if a job is not in the final schedule, then it has not 
			// been scheduled yet
			if(_final_schedule.find(job_idx) ==
					_final_schedule.end()){
				out << _jobs[job_idx].getJobId() << ";" <<  
                    _jobs[job_idx].getJobName() << ";";
				out << _jobs[job_idx].getUserName() << ";" << 
                    _jobs[job_idx].getQueue() << ";";
				out << Util::timeToStr(_jobs[job_idx].getEnterQueueTime()) << "__";
				int nunits = _jobs[job_idx].getNumberOfNodes();
				for(int u = 0; u < nunits; ++u){
					// -1 for the not assigned nodes 
					out << "-1;";
					out << _jobs[job_idx].getNumberOfCores()/nunits << ";" 
						<< _jobs[job_idx].getNumberOfGPU()/nunits << ";";
					out << _jobs[job_idx].getNumberOfMIC()/nunits
					       	<< ";" << _jobs[job_idx].getMemory()/nunits*1024;
					out << "#";
				}
				// fixed impossible value for not assingned end/start times
				out << "__2007-01-15 00:00:00;2007-01-15 00:00:00;";
				out << _jobs[job_idx].getNumberOfNodes() << ";" << 
                    _jobs[job_idx].getNumberOfCores() << ";";
				out << _jobs[job_idx].getMemory()/1024 << 
					";" << Util::timeHHMMToStr(_jobs_info[job_idx].dur);
				out << ";NA;NA;COMPLETED" << endl;
			}
		}
	}

}; // end of OnlineDispatcher

}  // end of optimization_core namespace

#endif
