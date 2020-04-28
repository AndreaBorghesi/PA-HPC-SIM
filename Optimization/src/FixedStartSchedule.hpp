// Author: Andrea Borghesi
// University of Bologna
// andrea.borghesi3@unibo.it
//
// Class representing the scheduling sub-problem results
//

#ifndef FIXEDSTARTSCHEDULE_HPP_
#define FIXEDSTARTSCHEDULE_HPP_

#include <algorithm>
#include <stdio.h>
#include <map>
#include <iostream>
#include <fstream>
#include <OptimizerImpl.hpp>

using namespace std;

// =============================================================================
// = Define the debug macros                                                   =
// =============================================================================

// decomment/comment this enable/disable debug output for this file
#define DEBUG_FIXEDSTARTSCHEDULE_HPP_

// Define the debug macros
#ifndef NDEBUG
	#ifdef DEBUG_FIXEDSTARTSCHEDULE_HPP_
		#define DBG(X) X
		#define DBGOUT (cerr << "[fst-sched] ")
	#else
		#define DBG(X)
	#endif
#else
	#define DBG(X)
#endif

namespace optimization_core {
	
	
// =============================================================================
// = Classess                                                                  =
// =============================================================================

/* Struct to represent the statistics/metrics of a 
 * Fixed Start schedule
 * - useful to assess the quality of a schedule */
typedef struct schedule_stats {
	long long makespan;
	double tqt;
	double avg_nqt;
	vector<double> utl_ratio;
} schedule_stats;

/* Class to represent a Fixed Start Schedule.

   Some notes:
   - A schedule can be built only for a stable problem 
   - If the associated problem is modified, then schedule becomes invalid
   - An invalid schedule cannot be changed
   - Basic information from an invalid schedule can still be accessed (e.g.
	 start times, unit mapping...)
   - Metrics can be computed only on a valid schedule */
class FixedStartSchedule {
	private:
		// References to problem data
		DispatchingProblem* _prb;
		
		// Cached number of jobs, resources and dimensions
		int _ndims;
		int _pool_size;
		
		/* Reference problem version */
		long long _ref_prb_version;

		/* Start times of all jobs */
		vector<long long> _starts;
		
		/* End times of all jobs */
		vector<long long> _ends;

        /* Frequencies of all jobs */
        vector<int> _frequencies;
						
		// list of the indices of the nodes where each job unit is mapped
		vector< vector<int> > _unit_mapping;
		// list of the indices of the used nodes
		vector< vector<int> > _used_res;
		// requirement on each node, for each dimension
		vector<vector<vector<long long> > > _reqs_by_res;
		
	public:
		// =====================================================================
		// = Constructor                                                       =
		// =====================================================================
		
		/* Default constructor. The created instance is in an invalid state */
		FixedStartSchedule() : _prb(NULL) {}
		
		/* Main constructor. NOTE: the problem must contain only stable jobs
		   and resources for this method to be called */
		FixedStartSchedule(DispatchingProblem* prb) : _prb(prb) {
			reset();
		}
		
		/* Reset the schedule to its initial state. This method can be called
		   only when the associated problem is stable  */
		void reset() {
			assert(_prb->isProblemStable());
			
			// Store the current problem version
			_ref_prb_version = _prb->getProblemVersion();
			
			// Cache the number of dimensions
			_ndims = _prb->getNDims();
						
			// Cache the size of the job pool (this includes invalid jobs)
			_pool_size = _prb->getStableJobs().size() + _prb->getNInvalidJobs();
						
			// Resize the data structures to hold all information
			// NOTE: the initial values are all invalid
			// NOTE: most data structure will contain cells for both valid and
			// invalid jobs. All accessor method contain "assert" statements to
			// ensure that the job being accessed is valid.
			_starts.clear();
			_starts.resize(_pool_size, -1);
			
			_ends.clear();
			_ends.resize(_pool_size, -1);

			_frequencies.clear();
			_frequencies.resize(_pool_size, -1);
			
			_used_res.clear();
			_used_res.resize(_pool_size);
			
			_reqs_by_res.clear();
			_reqs_by_res.resize(_pool_size);
			
			_unit_mapping.clear();
			_unit_mapping.resize(_pool_size);
			for (int i : _prb->getStableJobs()) {
				_unit_mapping[i] = vector<int>(_prb->getNUnits(i), -1);
			}
		}
		
		/* check if the current schedule is valid */
		bool isValid() {
			return _ref_prb_version == _prb->getProblemVersion();
		}
		
		// set the start time of a job
		void setStartTime(int job_idx, long long start_time) {
			assert(isValid());
			assert(_prb->isJobValid(job_idx));
			// store the start time
			_starts[job_idx] = start_time;
			// store the end time
			_ends[job_idx] = start_time + _prb->getDur(job_idx);
		}

		// set the frequency of a job
		void setFrequency(int job_idx, int frequency) {
			assert(isValid());
			assert(_prb->isJobValid(job_idx));
			// store the frequency
			_frequencies[job_idx] = frequency;
		}

		
		// Assign a job unit to a resource
		void mapUnit(int job_idx, int unit_idx, int res_idx) {
			assert(isValid());
			assert(_prb->isJobValid(job_idx));
			assert(unit_idx >= 0 && unit_idx < _unit_mapping[job_idx].size());
			assert(!_prb->isResRemoved(res_idx));
			assert(_unit_mapping[job_idx][unit_idx] < 0);
			
			// store the unit mapping
			_unit_mapping[job_idx][unit_idx] = res_idx;
			
			// Check if the specified resource has already been used
			int kk = 0;
			for (; kk < _used_res[job_idx].size()
					  	 && _used_res[job_idx][kk] != res_idx; ++kk);
			
			// If the resource is being used for the first time by this node,
			// add the index and a new requirement vector
			if (kk == _used_res[job_idx].size()) {
				// Store the index of the new used resource
				_used_res[job_idx].push_back(res_idx);
				// Prepare a new, all zero, vector for the cumulative reqs.
				_reqs_by_res[job_idx].push_back(vector<long long>(_ndims, 0));
			}
			
			// Update the requirements on the node
			// NOTE: at this point, kk refers to the correct vector cell
			for (int j = 0; j < _ndims; ++j) {
				_reqs_by_res[job_idx][kk][j] += _prb->getReq(job_idx, unit_idx, j);
			}
		}
		
		// Reset all unit mappings for a job
		void resetMapping(int job_idx) {
			assert(isValid());
			assert(_prb->isJobValid(job_idx));
			
			// Reset all cumulative requirements
			_reqs_by_res[job_idx].clear();
			
			// Reset the list of used resources
			_used_res[job_idx].clear();
			
			// Reset the unit mapping
			for (int u = 0; u < _prb->getNUnits(job_idx); ++u)
				_unit_mapping[job_idx][u] = -1;
		}
		
		// =====================================================================
		// = Methods to access information                                     =
		// =====================================================================
				
		/* Access the associated DispatchingProblem */
		DispatchingProblem* getProblem() { return _prb; }
						
		// Access the number of resource dimensions (cached)
		int getNDims() { return _ndims; }
		
		// Access the start time
		long long getStart(int job_idx) {
			assert(job_idx >= 0 && job_idx < _starts.size());
			return _starts[job_idx];
		}
		
		// Access the end time
		long long getEnd(int job_idx) {
			assert(job_idx >= 0 && job_idx < _ends.size());
			return _ends[job_idx];
		}

		// Access the frequency
		long long getFrequency(int job_idx) {
			assert(job_idx >= 0 && job_idx < _frequencies.size());
			return _frequencies[job_idx];
		}
		
		// obtain the index of the node where a specific unit has been mapped
		long long getUnitMapping(int job_idx, int unit_idx) {
			assert(job_idx >= 0 && job_idx < _unit_mapping.size());
			assert(unit_idx >= 0 && unit_idx < _unit_mapping[job_idx].size());
			return _unit_mapping[job_idx][unit_idx];
		}
		
		// Obtain the number of used nodes
		// NOTE: method for handling the cumulative resource requirements
		long long getNUsedRes(int job_idx) {
			assert(job_idx >= 0 && job_idx < _used_res.size());
			return _used_res[job_idx].size();
		}
		
		// Obtain the index of the used nodes
		// NOTE: method for handling the cumulative resource requirements
		long long getUsedRes(int job_idx, int res_ordinal) {
			assert(job_idx >= 0 && job_idx < _used_res.size());
			assert(res_ordinal >= 0 && res_ordinal < _used_res[job_idx].size());
			return _used_res[job_idx][res_ordinal];
		}
		
		// Obtain the cumulative requirement of all units mapped to a certain
		// resource, for a specific dimension
		// NOTE: method for handling the cumulative resource requirements
		long long getCumulativeReq(int job_idx, int res_ordinal, int dim_idx) {
			assert(job_idx >= 0 && job_idx < _reqs_by_res.size());
			assert(res_ordinal >= 0 && res_ordinal < _reqs_by_res[job_idx].size());
			assert(dim_idx >= 0 && dim_idx < _reqs_by_res[job_idx][res_ordinal].size());
			return _reqs_by_res[job_idx][res_ordinal][dim_idx];
		}
		
		// =====================================================================
		// = Functions to compute the main evaluation metrics                  =
		// =====================================================================
		
		// Compute the weighted queue time
		double getMetricWQT(){
			assert(isValid());
			
			// Cache the max Expected Waiting Time
			double max_ewt = _prb->getMaxEWT();

			// prepare the result
			double wqt = 0;
			// Loop over all jobs
			for(int i : _prb->getStableJobs()){
				long long start = _starts[i]; // start time
				long long eqt = _prb->getQET(i); // Queue Entering Time
				double wgt = max_ewt / _prb->getEWT(i);
				// update the metric
				wqt += (start - eqt) * wgt;
			}
			return wqt;
		}

		// return the makespan (based on the estimated job duration)
		long long getMetricMK(){
			assert(isValid());
			
			long long mks = * std::max_element(_ends.begin(),_ends.end());
			return mks;
		}

		// =====================================================================
		// = Method to perform a feasibility check                             =
		// =====================================================================
		
		// check the validity of the solution: returns 1 if the solution is correct, 0 otherwise
		bool check(){
			assert(isValid());
			
			// Check if a correct initialization was done
			if (_prb == NULL) {
				DBG(DBGOUT << "incorrect initialization" << endl;)
				return false;
			}
			
			// solution found check
			for(int i : _prb->getStableJobs()){
				if(_starts[i] < 0) {
					return false;
				}
				for (int u = 0; u < _prb->getNUnits(i); ++u)
					if(_unit_mapping[i][u] < 0){
						return false;
					}
			}
						
			// Check the resource feasibility at each start time
			// TODO this check is inefficient. A better solution would require
			// to construct resource profiles
			int nres = _prb->getStableRes().size();
			for (int i : _prb->getStableJobs()) {
				// Time point being checked
				long long tcheck = _starts[i];
				
				// The initial usage for each resource and dimension is 0
				vector<long long> usg(nres * _ndims, 0);
				
				// Update the resource usage
				for (int k : _prb->getStableJobs()) {
					// Skip the job if it is not running
					if (_ends[k] <= tcheck || _starts[k] > tcheck) continue;
					
					// Otherwise, loop over all used nodes
					for (int jj = 0; jj < _used_res[k].size(); ++jj) {
						// jj is an ordinal. Obtain the corresponding node index
						int j = _used_res[k][jj];
						// update the resource usage
						for (int u = 0; u < _ndims; ++u) 
							usg[j * _ndims + u] += _reqs_by_res[k][jj][u];
					}

				}
				
				// Check the resource usage
				for (int j : _prb->getStableRes())
					for (int u = 0; u < _ndims; ++u)
						if (usg[j * _ndims + u] > _prb->getCap(j, u)){
							return false;
						}
			}
			
			// If all checks were ok, then return true
			return true;
		}


		// Method to write the results on a text file
		void writeSolution(ostream& out){
			assert(isValid());
			
			for(int i = 0; i < _starts.size(); ++i){
				// Skip jobs with negative start times
				if (_starts[i] < 0) continue;
				
				out << i << "\t" << _starts[i]  << "\t";
				//cout << " Job " << i << " NUnits: " << _jobs->getNUnits(i) << endl;
				for(int ii : _unit_mapping[i]){
					out << ii << ";" ;
				}
				out << endl;
			}
		}

		// Method to print a solution
		void printSolution(){
			for(int i = 0; i < _starts.size(); i++){
				// Skip jobs with a negative start time
				if (_starts[i] < 0) continue;
				
				cout << i << "\t" << _starts[i]  << "\t";
				for(int ii : _unit_mapping[i]){
					cout << ii << ";" ;
				}
				cout << endl;
			}
		}

		// Method to print a solution ST,ET relative to ref_time
		void printSolution_relativeTimes(long long ref_time){
			for(int i = 0; i < _starts.size(); i++){
				// Skip jobs with a negative start time
				if (_starts[i] < 0) continue;
				cout << i << "\t" << _starts[i] - ref_time << "; " 
                    << _ends[i] - ref_time  << "\t";
				for(int ii : _unit_mapping[i]){
					cout << ii << ";" ;
				}
				cout << endl;
			}
		}

		void printDetailedSolution(){
			for (int i = 0; i < _starts.size(); ++i){
				// Skip jobs with a negative start
				if (_starts[i] < 0) continue;
				
				cout << i << "\t" << _starts[i] << "-" << _ends[i] << "\t";
				for(int ii = 0; ii < _used_res[i].size(); ++ii){
					cout << _used_res[i][ii] << " (";
					for(int k = 0; k < _ndims; ++k)
						cout << " " << _reqs_by_res[i][ii][k];
					cout << ");";
				}
				cout << endl;
			}
		}

		// evaluate the current solution values
		schedule_stats evalSchedule(long long ref_time){
			assert(isValid());

			// struct to store the stats
			schedule_stats result;

			// Compute some metrics
			double nqt = 0; // weighted queue time
			double tqt = 0; // total queue time
			vector<double> utl(_ndims, 0); // utilization
			for(int job_sched : _prb->getStableJobs()){
				// Waiting time for this job
				double wait_time = getStart(job_sched) - _prb->getQET(job_sched);
				// Update total queue time
				tqt += wait_time;
				// Weight for this this job
				double wgt = 1.0 / _prb->getEWT(job_sched);
				// Update weighted queue time
				nqt += wgt * wait_time;
				// Compute the total utilization of each resource
				for (int d = 0; d < _ndims; ++d){
					for(int u = 0; u < _prb->getNUnits(job_sched); u++){
						for(int r = 0; r < getNUsedRes(job_sched); r++){
							utl[d] +=  getCumulativeReq(job_sched, r, d) * _prb->getDur(job_sched);
						}
					}
				}
			}
			// Average normalized queue time
			double avg_nqt = nqt / _prb->getStableJobs().size();
				
			// One more metric (utilization ratio)
			vector<double> utl_ratio(_ndims, 0);
			for (int d = 0; d < _ndims; ++d){
				long long tot_cap = 0;
				for(int r : _prb->getStableRes()){
					tot_cap += _prb->getCap(r,d);
				}
				utl_ratio[d] = utl[d] / (tot_cap * (getMetricMK()-ref_time));
			}
				
			result.makespan = getMetricMK() - ref_time;
			 
			result.tqt = tqt;
			result.avg_nqt = avg_nqt;
			for (int d = 0; d < _ndims; ++d) 
				result.utl_ratio.push_back(utl_ratio[d]);

			return result;
	}

};

} // end of namespace

// =============================================================================
// = Undefine the debug macros                                                 =
// =============================================================================

#undef DBG
#undef DBGOUT

#endif
