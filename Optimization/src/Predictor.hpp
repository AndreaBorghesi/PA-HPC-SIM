#ifndef PREDICTOR_HPP_
#define PREDICTOR_HPP_

#include <algorithm>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include "Util.hpp"
#include "Job.hpp"

using namespace std;

// =============================================================================
// = Define the debug macros                                                   =
// =============================================================================

// comment/decomment this to enable debug output for this file
#define DEBUG_PREDICTOR_HPP

#ifndef NDEBUG
	#ifdef DEBUG_PREDICTOR_HPP
		#define DBG(X) X
		#define DBGOUT (cerr << "[predictor] ")
	#else
		#define DBG(X)
	#endif
#else
	#define DBG(X)
#endif

// =============================================================================
// = Predictor                                                                 =
// =============================================================================

class Predictor {
private:
    /* fixed power consumption of the resources, modeled on Eurora supercomputer */
    double CPU_IDLE = 20.0;
    double GPU_IDLE = 12.0;
    double CPU_21 = 20.0;
    double COEFF_ANG = 6.25;
    double GPU_ACTIVE = 50.0;
    double MIC = 115.0;
    double CPU_31 = 20.0;
    double CPU_31_S = 60.0;
    double COEFF_ANG_31 = 10.6;

    string _preditions_file;
    map<string, double> _jobs_predicted_powers;

    /* Predict power for new jobs */
    double predict_power(Job job);

public:
    /* Constructor */
    Predictor(string predictions_file);

    /* Simple Destructor */
    ~Predictor();

    /* Read all the predictions already (offline) made */
    void read_predictions();

    /* Method to retrieve the predicted power consumption of a job) */
    double get_power_prediction(Job job);
};


// =============================================================================
// = Method implementations (Predictor)                                         =
// =============================================================================

// =============================================================================
// = Method implementations (CPSolver relaxed resources)                       =
// =============================================================================

Predictor::Predictor(string predictions_file){
    _preditions_file = predictions_file;
    read_predictions();
}

Predictor::~Predictor(){
}

void Predictor::read_predictions(){
    ifstream input(_preditions_file.c_str());
    if (!input){
        cout << "ERROR: Could not open file \"" << _preditions_file << "\"" << endl;
        exit(1);
    }
	string line;
	while(getline(input,line))
	{
		vector<string> info = Util::split(line,";");
        string job_id = info[0];
        double power = atof(info[1].c_str());
        _jobs_predicted_powers[job_id] = power;
    }
}

/* The power prediction can be obtained from the list of 
 * already predicted jobs or it can computed online.
 * The prediction model depend on the target supercomputer */
double Predictor::get_power_prediction(Job job){
	double power = 0;
    auto it = _jobs_predicted_powers.find(job.getJobId());
    if (it != _jobs_predicted_powers.end())   // if the job power was already predicted
        return it->second;
    else   // new jobs - prediction not available
        power = predict_power(job);
	return power;
}

double Predictor::predict_power(Job job){
    double prediction = 0;
    /* This is an example of (dummy) power prediction: the predicted
     * power consumption is equal to the sum of maximum consumptions
     * of a job requested CPUs, MICs and GPUs     */
    prediction += CPU_IDLE * job.getNumberOfNodes();
    prediction += GPU_ACTIVE * job.getNumberOfGPU();
    prediction += MIC * job.getNumberOfMIC();

    prediction += CPU_IDLE * job.getNumberOfNodes();
    prediction += GPU_IDLE * job.getNumberOfGPU();

    return prediction;
}

// =============================================================================
// = Undefine the debug macros                                                 =
// =============================================================================

#undef DBG
#undef DBGOUT

#endif
