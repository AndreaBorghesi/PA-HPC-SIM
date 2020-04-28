#ifndef INSTANCEANALYSER_HPP_
#define INSTANCEANALYSER_HPP_
#include <algorithm> 
#include <map>
#include <iomanip>
#include <limits>
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
// = Instance Analyser class                                                   =
// =============================================================================

/* This class is used to analyse an instance: compute some interesting
 * metrics regarding the composition (in terms of job requests) of the instance */

/* Class to analyze the dispatching problem  */
class InstanceAnalyser {

private:	
    Predictor* predictor_;
	vector<Job>& jobs_vector_;
	vector<JobQueue>& queues_;
	vector<Node>& nodes_;
	int ndims_;

    long long ref_time_;

    /* Minimum duration (s) of jobs considered when computing BSLD.  */
    double bsld_threshold_ = 60;

	// data structure to store information about a job
	typedef struct _job_info {
        long long job_idx;
		long long dur; // Job (real) duration 
		long long ewt; // Expected waiting time
		long long qet;
        double power_pred;  // Predicted Consumed Power
        double cur_power;  // Current consumed power - may vary
		// requirements for each job unit
        long long st;
        long long et;
		vector< vector<long long > > reqs;
		// sum of requirements for each dimensions
		vector<long long> tot_reqs; 
        // nodes mapping
        vector<int> node_mapping;
        long long node_type;   // type of used nodes (we assume no mixed nodes)
        int application_type; // 0: average, 1: cpu-intensive, 2: mem-intensive
        int frequency;
        long long energy;  
	} job_info_;

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
	map<long long, job_info_> jobs_;


public:
    void addJob(int job_idx);

    void analyse();

    InstanceAnalyser(vector<Job>& jobs_vector,
				vector<JobQueue>& qs,
				vector<Node>& nodes,
                Predictor* predictor,
                long long ref_time,
                int ndims);

    /* Simple destructor */
    virtual ~InstanceAnalyser() {}

}; 

// =============================================================================
// = Method Implementations (InstanceAnalyser)                                 =
// =============================================================================

void InstanceAnalyser::addJob(int job_idx){
	// Obtain a reference to the job
	Job& job = jobs_vector_[job_idx];

	// structure used only to later compute metrics 
    // - no impact on scheduling decisions
	_job_info j_info;
    j_info.job_idx = job_idx;
	
	// Obtain the job duration
	// this is the duration used to compute schedules
	long long dur  = job.getEstimatedDuration();

    // Obtain the job predicted power consumption
    // This power is used to respect the power cap
    double power_pred;
    power_pred = predictor_->get_power_prediction(job);

    /* Some power predictions could be wrong. Extreme small values 
     * (smaller than 1) are extremely suspicious (and can cause later problems):
     * just in case we set them to 1. In this way we are also being robust
     * and conservative by overestimating the power consumptions */
    if(power_pred < 1)
        power_pred = 1;

    j_info.power_pred = power_pred;

    // initially, the job power is the predicted one
    double cur_power = power_pred;
    j_info.cur_power = power_pred;

    int application_type = jobs_vector_[job_idx].getApplicationType();
    j_info.application_type = application_type;

	/* sanitize strange input: if real duration longer than expected 
     * (this shouldn't happen but it does - probably due to bugs in the 
     * historical trace collecting mechanism), reduce real duration 
     * -> simplify testing      */
	(job.getEstimatedDuration() < job.getRealDuration() ? 
		 j_info.dur = job.getEstimatedDuration() :
		 j_info.dur = job.getRealDuration());

	// Obtain the job waiting time
	long long ewt = 1800;   // default value for reservations
	for (JobQueue& q : queues_){
		if (q.getId().compare(job.getQueue()) == 0) {
			ewt = q.getMaxMinutesToWait();
			break;
		}
	}
	j_info.ewt = ewt;

    // the job frequency is decided only after mapping
    j_info.frequency = -1;

	// The time already spent in queue
	long long qet = job.getEnterQueueTime() - ref_time_;
	j_info.qet = qet;

    long long st = job.getStartTime() - ref_time_;
    long long et = st + dur;

    j_info.st = st;
    j_info.et = et;
	
	// Add job units one by one
	int nunits = job.getNumberOfNodes();
	vector<vector<long long> > unit_reqs;
	j_info.reqs.resize(nunits);
	j_info.tot_reqs.resize(ndims_,0);

	for (int u = 0; u < nunits; ++u) {
		j_info.reqs[u].resize(ndims_);
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

	// store the information about the job in the internal map	
	jobs_[job_idx] = {j_info};
}

void InstanceAnalyser::analyse(){
    cout << "[InstanceAnalyser] analysis begin" << endl;

    double totDur = 0;
    int short_jobs_count = 0;
    vector<int> totReqs;
    totReqs.resize(ndims_, 0);
    double totPower = 0;
    double avgDur, avgPower;
    double maxDur = 0;
    double minDur = std::numeric_limits<double>::max();
    vector<double> avgReqs;
    avgReqs.resize(ndims_, 0);

    int res_count = 0;
    int longpar_count = 0;
    int debug_count = 0;
    int par_count = 0;

    int plain_appType_count = 0;
    int mem_appType_count = 0;
    int cpu_appType_count = 0;

    int tot_unitsPerJob = 0;
    double avg_unitsPerJob = 0;

    long long max_end_time = 0;
    long long max_qet = 0;

    int max_jobs_same_eqt_count = 0;

    for(auto ji : jobs_){
        int jobs_same_eqt_count = 0;
        
        totDur += ji.second.dur;
        if(ji.second.dur < 60)
            ++short_jobs_count;

        if(ji.second.dur > maxDur)
            maxDur = ji.second.dur;
        if(ji.second.dur < minDur)
            minDur = ji.second.dur;

        totPower += ji.second.power_pred;

        for(int k = 0; k < ndims_; ++k)
            totReqs[k] += ji.second.tot_reqs[k];

        if(ji.second.et > max_end_time)
            max_end_time = ji.second.et;
        if(ji.second.qet > max_qet)
            max_qet = ji.second.qet;

        tot_unitsPerJob += ji.second.reqs.size();

        switch(ji.second.ewt){
            case 1800:
                ++res_count;
                break;
            case 3600:
                ++debug_count;
                break;
            case 21600:
                ++par_count;
                break;
            case 86400:
                ++longpar_count;
                break;
        }

        switch(ji.second.application_type){
            case 0:
                ++plain_appType_count;
                break;
            case 1:
                ++cpu_appType_count;
                break;
            case 2:
                ++mem_appType_count;
                break;
        }

        for(auto oj : jobs_)
            if(oj.first != ji.first)
                if(oj.second.qet == ji.second.qet)
                    ++jobs_same_eqt_count;

        if(max_jobs_same_eqt_count < jobs_same_eqt_count)
            max_jobs_same_eqt_count = jobs_same_eqt_count;
    }

    avgDur = totDur / (double) jobs_.size();
    avgPower = totPower / (double) jobs_.size();
    for(int k = 0; k < ndims_; ++k)
        avgReqs[k] = totReqs[k] / (double) jobs_.size();
    avg_unitsPerJob = tot_unitsPerJob / (double) jobs_.size();

    double stdDur = 0;
    double stdPower = 0;
    double stdUnitsPerJob = 0;
    vector<double> stdReqs;
    stdReqs.resize(ndims_, 0);

    for(auto ji : jobs_){
        stdDur += (ji.second.dur - avgDur)*(ji.second.dur - avgDur);
        stdPower += (ji.second.power_pred - avgPower)
            *(ji.second.power_pred - avgPower);

        for(int k = 0; k < ndims_; ++k)
            stdReqs[k] += (ji.second.tot_reqs[k] - avgReqs[k])*
                (ji.second.tot_reqs[k] - avgReqs[k]);

        stdUnitsPerJob += (ji.second.reqs.size() - avg_unitsPerJob)
            * (ji.second.reqs.size() - avg_unitsPerJob);
    }
    stdDur /= (double) jobs_.size();
    stdPower /= (double) jobs_.size();
    stdUnitsPerJob /= (double) jobs_.size();
    for(int k = 0; k < ndims_; ++k)
        stdReqs[k] /= (double) jobs_.size();
    stdDur = sqrt(stdDur);
    stdPower = sqrt(stdPower);
    stdUnitsPerJob = sqrt(stdUnitsPerJob);
    for(int k = 0; k < ndims_; ++k)
        stdReqs[k] = sqrt(stdReqs[k]);

    //long long time_window = max_qet - ref_time_;
    long long time_window = max_qet;
    double job_arrival_freq = time_window / (double) jobs_.size();

    cout << std::fixed;
    cout << std::setprecision(2);

    cout << "[InstanceAnalyser] # Jobs: " << jobs_.size() << 
        "; time window: " << time_window << "; job arrival frequency: " 
        << job_arrival_freq << "; max. # of jobs with same QET (" 
        << "Queue Enter Time, i.e. arrival time): " <<
        max_jobs_same_eqt_count << endl;

    cout << "[InstanceAnalyser] Avg. Duration: " << avgDur << 
        "; # short jobs: " << short_jobs_count << "; short jobs %: " 
        << short_jobs_count / (double) jobs_.size() * 100 <<
        "; avg. power: " << avgPower << endl;
    cout << "[InstanceAnalyser] Avg. Reqs: ";
    for(int k = 0; k < ndims_; ++k)
        cout << avgReqs[k] << ", ";
    cout << "; avg. units per job: " << avg_unitsPerJob << endl;

    cout << "[InstanceAnalyser] Standard Deviation of Durations: " << stdDur << 
        "; std. power: " << stdPower << endl;
    cout << "[InstanceAnalyser] Standard Deviation of Reqs: ";
    for(int k = 0; k < ndims_; ++k)
        cout << stdReqs[k] << ", ";
    cout << "; std. units per job: " << stdUnitsPerJob << endl;

    cout << "[InstanceAnalyser] Avg. app. type  %: " << 
        plain_appType_count / (double) jobs_.size() * 100;
    cout << "; cpu app. type  %: " << 
        cpu_appType_count / (double) jobs_.size() * 100;
    cout << "; mem app. type  %: " << 
        mem_appType_count / (double) jobs_.size() * 100;
    cout << endl;

    cout << "[InstanceAnalyser] Reservation %: " << 
        res_count / (double) jobs_.size() * 100;
    cout << "; debug  %: " << 
        debug_count / (double) jobs_.size() * 100;
    cout << "; par  %: " << 
        par_count / (double) jobs_.size() * 100;
    cout << "; longpar  %: " << 
        longpar_count / (double) jobs_.size() * 100;
    cout << endl;
}

InstanceAnalyser::InstanceAnalyser(vector<Job>& jobs_vector,
				vector<JobQueue>& qs,
				vector<Node>& nodes,
                Predictor* predictor,
                long long ref_time,
                int ndims) : 
    jobs_vector_(jobs_vector), queues_(qs), nodes_(nodes), 
    predictor_(predictor), ref_time_(ref_time), ndims_(ndims){
        for(int i = 0; i < jobs_vector.size(); ++i)
            addJob(i);

}


}  // end of optimization_core namespace

#endif
