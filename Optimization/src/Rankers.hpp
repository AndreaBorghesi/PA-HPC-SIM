#ifndef RANKERS_HPP_
#define RANKERS_HPP_

#include <OptimizerImpl.hpp>
#include <string>
#include <vector>
#include <cfloat>
#include <map>

using namespace std;

// =============================================================================
// = Define the debug macros                                                   =
// =============================================================================

// enable debug output for this file
#define DEBUG_RANKERS_HPP_

// Define the debug macros
#ifndef NDEBUG
	#ifdef DEBUG_FIXEDSTARTSCHEDULE_HPP_
		#define DBG(X) X
		#define DBGOUT (cerr << "[ranker] ")
	#else
		#define DBG(X)
	#endif
#else
	#define DBG(X)
#endif

namespace optimization_core {

// =============================================================================
// = General ranker super-class                                                =
// =============================================================================

class Ranker{
	public:
		virtual bool operator()(int i, int j){
			return 0;
		}

		virtual int getSize(){
			return 0;
		}

		virtual vector<double> getScore_ofElement(int i){
			vector<double> result;
			return result;
		}

		// Randomly swap n scores couples
		virtual void swapRndmScores(int n){
		}
};


// =============================================================================
// = Single-score comparison class                                             =
// =============================================================================

class ScoreRanker : public Ranker{
	private:
		map<int,double> _scores;
	public:
		ScoreRanker(int size) {
		}
		
		void addScore(int i, double score) {
			_scores[i] = score;
		}
		
		bool operator()(int i, int j){
			return _scores[i] < _scores[j];
		}

		int getSize(){
			return _scores.size();
		}

		vector<double> getScore_ofElement(int i){
			assert(i >= 0 && i < _scores.size());
			vector<double> result;
			result.push_back(_scores[i]);
			return result;
		}

		// Randomly swap n scores couples
		void swapRndmScores(int n){
			assert(n>0);
			
			for(int i = 0; i < n; i++){
				// pick the random first score
				auto it = _scores.begin();
				advance(it, rand() % _scores.size());
				int first_rndmScore_key = it->first;
				double first_rndmScore_val = it->second;

				// pick the random second score
				it = _scores.begin();
				advance(it, rand() % _scores.size());
				int second_rndmScore_key = it->first;
				double second_rndmScore_val = it->second;

				// swap score values 
				_scores[first_rndmScore_key] = second_rndmScore_val;
				_scores[second_rndmScore_key] = first_rndmScore_val;
			}
		}
};

// =============================================================================
// = Factory class to build rankers                                            =
// =============================================================================

class JobResRanker {
private:
	
	/* Associated DispatchingProblem instance */
	DispatchingProblem* _prb;
	
	/* Reference version */
	long long _ref_prb_version;

private:
	
	// =========================================================================
	// = Score computation functions                                            =
	// =========================================================================
	
	
public:
	// =========================================================================
	// = Functions to build rankers                                            =
	// =========================================================================

	/* Build a ranker for resources */
	ScoreRanker getDummyRes_scoreRanker(){
		assert(_ref_prb_version == _prb->getProblemVersion());
	
        /* This is dummy implementation: all resource have the same score */    
		ScoreRanker sr(_prb->getStableRes().size());
		for(int j : _prb->getStableRes())
			sr.addScore(j, 1);
		return sr;
	}
		
	/* Build a ranker that sorts jobs by increasing wall time  */
	ScoreRanker getJobWallTime_scoreRanker(){
		assert(_ref_prb_version == _prb->getProblemVersion());
		
		ScoreRanker sr(_prb->getStableJobs().size());
		for(int i : _prb->getStableJobs())
			sr.addScore(i, _prb->getDur(i));
		return sr;
	}

	/* Build a ranker that sorts jobs by increasing ending time */
	ScoreRanker getJobEndTime_scoreRanker(
            map<int, long long> running_jobs){
		assert(_ref_prb_version == _prb->getProblemVersion());

		ScoreRanker sr(running_jobs.size());
        for(auto rj : running_jobs)
            sr.addScore(rj.first, rj.second);
		return sr;
	}
	
	// =========================================================================
	// = Constructors                                                          =
	// =========================================================================

	JobResRanker(DispatchingProblem* prb) : _prb(prb) {
		assert(_prb->isProblemStable());
		
		// Store the reference problem version
		_ref_prb_version = _prb->getProblemVersion();
	}
	
};


} // end of optimization_core namespace

// =============================================================================
// = Undefine the debug macros                                                 =
// =============================================================================

#undef DBG
#undef DBGOUT

#endif
