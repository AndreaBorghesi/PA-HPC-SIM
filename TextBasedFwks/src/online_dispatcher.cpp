// Author: Andrea Borghesi
// University of Bologna
// andrea.borghesi3@unibo.it
//
// Class devoted to read from jobs info from text files and create the online scheduler
//

// Header files to read and manage the input files
#include <NodeReader.hpp>
#include <JobReader.hpp>
#include <JobQueueReader.hpp>

#include <ListScheduler.hpp>
#include <BackFillingScheduler.hpp>
#include <StaticDVFSScheduler.hpp>

// Online Dispatcher
#include <OnlineDispatcher.hpp>
#include "Predictor.hpp"

#include <InstanceAnalyser.hpp>

// Timers
#include <Util.hpp>

#include <limits>

// Namespaces
using namespace optimization_core;
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

// =============================================================================
// = Function to build the problem definition data structures                  =
// =============================================================================

// RESOURCE DIMENSTIONS
typedef enum {
	d_core = 0,
	d_gpu,
	d_mic,
	d_mem
} res_dims;

void initOptimizer(Optimizer* opt,
					vector<Node>& nodes,
					vector<Job>& jobs,
					vector<JobQueue>& queues) {			
    // each job will be added by the scheduler at the time of its arrival

	// Add all nodes to the problem
	for (int idx = 0; idx < nodes.size(); ++idx) {
		// Obtain a reference to the node
		Node& node = nodes[idx];
		
		// Obtain the node capacity for all resource types
		long long ccores = node.getCoresNumber();
		long long cgpus = node.getGPUsNumber();
		long long cmics = node.getMICsNumber();
		long long cmem = node.getTotalMemory();
		
		// Add a new resource in the optimizer 
		opt->addRes(idx, {ccores, cgpus, cmics, cmem});
	}
}

/* This function read the power caps file and create the 
 * power cap variation events  */
map<long long, int >read_power_caps(string pcaps_file){
    map<long long, int> power_variations;

	ifstream input(pcaps_file.c_str());
	if (!input) 
	{
		cout << "ERROR: Could not open file \"" << pcaps_file << "\"" << endl;
		exit(1);
	}
	string line;
	int nread = 0;
	while(getline(input,line))
	{
        vector<string> info = Util::split(line,";");
        int time_delta = atoi(info[0].c_str());
        int power_cap = atoi(info[1].c_str());
        power_variations[time_delta] = power_cap;
    }
    return power_variations;
}


// =============================================================================
// = MAIN                                                                      =
// =============================================================================

int main(int argc, char* argv[]){
	// Parse the command line
	if (argc < 5) {
		cout << "USAGE: " << argv[0] << 
			" <node file> <queue file> <job file> <prediction file> " <<
            "<opt_type> <power_caps_file> <freqAssign_mode> <dynamic_control>" <<
            " <power_cap> (<sys_power_target>) (<node_power_target>) "
            << "(dynControl_perfModel_factor) (startup) (<out_fname>)" << endl;
		exit(1);
	}
	
	cout << "================================================================" << endl;
	cout << "= Online Dispatcher                                            =" << endl;
	cout << "================================================================" << endl;
	cout << endl;
	
	// Input files
	string fNode = argv[1];
	string fQ = argv[2];
	string fJob = argv[3];
	string fPred = argv[4];
	int opt_type = atoi(argv[5]);
	string fPcaps = argv[6];
    int freqAssign_mode = atoi(argv[7]);
    int dynControl_mode = atoi(argv[8]);
	int power_cap = 5000000;
    int sys_pow_target = -1;
    int node_pow_target = -1;
    double perfModel_factor = 0.7;
    int startup = 0;
	if(argc == 10)
		power_cap = atoi(argv[9]);
	
	char* out_fname = NULL;
	if (argc == 11){
		power_cap = atoi(argv[9]);
		out_fname = argv[10];
    }

	if (argc == 13){
		power_cap = atoi(argv[9]);
		sys_pow_target = atoi(argv[10]);
		node_pow_target = atoi(argv[11]);
		perfModel_factor = atof(argv[12]);
    }

	if (argc == 14){
		power_cap = atoi(argv[9]);
		sys_pow_target = atoi(argv[10]);
		node_pow_target = atoi(argv[11]);
		perfModel_factor = atof(argv[12]);
		startup = atoi(argv[13]);
    }

	if (argc == 15){
		power_cap = atoi(argv[9]);
		sys_pow_target = atoi(argv[10]);
		node_pow_target = atoi(argv[11]);
		perfModel_factor = atof(argv[12]);
		startup = atoi(argv[13]);
		out_fname = argv[14];
    }

	// Number of dimensions (fixed)
    /* The dimension are the types of resources present in the system. 
     * In the case of Eurora we have 4 kind of resources: cores, GPUs,
     * MICs and memory      */
	const int ndims = 4;
	
	// Timers
	CustomTimer read_timer;
	CustomTimer init_timer;
	CustomTimer schd_timer;
	
	cout << "Reading the input files" << endl;
	read_timer.start();
	
	// read node data
	NodeReader nreader(fNode);
	vector<Node> node_data = nreader.read();
	
	// Read job data from the input file
	JobReader jreader(fJob);
	vector<Job> job_data = jreader.read();
		
	// read queue data
	JobQueueReader qreader(fQ);
	vector<JobQueue> queue_data = qreader.read();

    // read power cap information
    map<long long, int> relative_time_pCaps = read_power_caps(fPcaps);
		
	read_timer.stop();
	cout << "Input files read" << endl;
	
	cout << "Initializing the optimizer" << endl;
	init_timer.start();
	
	// Build a compatibility checker
	CompatibilityChecker chk; // default checker
	
	// Build an interval manager
	IntervalManager mng; // default manager

    // Build a power predictor
    Predictor* predictor = new Predictor(fPred);

	// Compute a "fake" reference time (i.e. the minimum QET among all jobs)
	long long ref_time = numeric_limits<long long>::max();
	for (Job job : job_data) 
		ref_time = (ref_time < job.getEnterQueueTime() 
                ? ref_time : job.getEnterQueueTime());

    /* If we want to simply analyze an instance, without dispatching   */
	InstanceAnalyser* analyser = new InstanceAnalyser(
            job_data, queue_data, node_data, predictor, ref_time, ndims);
    if(opt_type == 50){
        analyser->analyse();
        exit(0);
    }

    /* If opt_type == 100 we perform a 'dry run': we launch the dispatcher
     * with no power cap. It can be useful to compute the maximum power 
     * consumption generated by a job instance if no limits are set.  */
    bool dry_run = false;
    if(opt_type == 100){
        // dry-run to discover maximum power consumption --> no power cap
        power_cap = numeric_limits<int>::max() - 2;
        dry_run = true;
    }

    /* The StaticDVFSSchedulingOptimizer employs DVFS; we experimented with 
     * different implementations of DVFS mechanisms (especially the relation
     * between power reduction and duration increase. The different models
     * could be selected by setting the dvfs_frequency_scalingMode variable.
     * This version of the dispatcher allows _only_ one RAPL-like 
     * implementation (see StaticDVFSScheduler.hpp for details), thus
     * the variable loses its meaning (still kept for legacy purposes)  */
    int dvfs_frequency_scalingMode = 0;
        
	// Build the optimizer
	// NOTE: I am using a pointer to take advantage of polymorphism
	Optimizer* opt;
	switch(opt_type){
		case 0:
			// Build the list scheduler optimizer
			opt = new ListSchedulingOptimizer_powerCaps(ndims, &chk, &mng, power_cap);
			break;
		case 1:
			// Build the backfilling scheduler optimizer
			opt = new BackFillSchedulingOptimizer(ndims, &chk, &mng, power_cap);
			break;
		case 2:
			// Build the static DVFS scheduler optimizer
			opt = new StaticDVFSSchedulingOptimizer(
                    ndims, &chk, &mng, power_cap, 
                    dvfs_frequency_scalingMode, perfModel_factor);
			break;
		case 100:
			// Build the list scheduler optimizer (dry run)
			opt = new ListSchedulingOptimizer_powerCaps(ndims, &chk, &mng, power_cap);
			break;

		default:
			opt = new ListSchedulingOptimizer_powerCaps(ndims, &chk, &mng, power_cap);
	}

	initOptimizer(opt, node_data, job_data, queue_data);

    map<long long, int > power_variations;
    for(auto it = relative_time_pCaps.begin(); it != relative_time_pCaps.end();
            it++){
        long long abs_time = ref_time + it->first;
        power_variations[abs_time] = it->second;
    }

	cout << "Creating new OnlineDispatcher instance.." << endl;
    OnlineDispatcher* online_dispatcher = new OnlineDispatcher(opt, predictor, 
            job_data, queue_data, node_data, ref_time, ndims, power_cap,
            sys_pow_target, node_pow_target, power_variations, 
            freqAssign_mode, dynControl_mode, perfModel_factor, startup, 
            opt_type, fJob);

	init_timer.stop();

	cout << "Begin scheduling.." << endl;

	schd_timer.start();

	online_dispatcher->dispatch(dry_run);

	cout << "End scheduling " << endl;

	schd_timer.stop();

    cout << "Timers:" << endl;	
	cout << "- time for reading the input files (sec): " << read_timer.getVal() / 1000.0 << endl;
	cout << "- time for initializing the optimizer (sec): " << init_timer.getVal() / 1000.0 << endl;
	cout << "- time for computing a schedule (sec): " << schd_timer.getVal() / 1000.0 << endl;

	/* Deallocate the optimizer */
	delete opt;
	/* Deallocate the dispatcher */
	delete online_dispatcher;
}


// =============================================================================
// = Undefine the debug macros                                                 =
// =============================================================================

#undef DBG
#undef DBGOUT
