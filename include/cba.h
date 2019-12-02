#pragma once
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <string>
#include <ceres/ceres.h>
#include <perspective_projection.h>

#ifdef _WIN64
	#ifdef CMPBA_DLLIMPORT
	#define DLLAPI __declspec(dllimport)
	#else
	#define DLLAPI __declspec(dllexport)
	#endif
#elif _WIN32
   	#ifdef CMPBA_DLLIMPORT
	#define DLLAPI __declspec(dllimport)
	#else
	#define DLLAPI __declspec(dllexport)
	#endif
#elif __APPLE__
	#define DLLAPI
    	#include "TargetConditionals.h"
    	#if TARGET_OS_IPHONE && TARGET_IPHONE_SIMULATOR
        	// define something for simulator   
    	#elif TARGET_OS_IPHONE
        	// define something for iphone  
    	#else
        #define TARGET_OS_OSX 1
        // define something for OSX
    	#endif
#elif __linux
    	// linux
	#define DLLAPI
#elif __unix // all unices not caught above
    	// Unix
	#define DLLAPI
#elif __posix
    	// POSIX
	#define DLLAPI
#endif



namespace cba {

	class DLLAPI CBAInterface {
	public:
		CBAInterface( double * parameters, int * parameterBlockPointers, double * observations, int nObs, std::vector<std::string> projFuncTypes);


		// Getters
		double * getParameters()  const{ return parameters_; };
		int getNumObs() const { return nObs_; };
		int * getBlockPointers() const { return blockPointers_; };
		std::string getProjFuncType(int i) const { return projFuncTypes_[i]; };
		double * getObservations() const {return observations_; };
		double getWeight(int i) const {return weights_[i];};
		double * getAuxiliaryData(int i) const {return auxdata[i];};
		bool getRobustify() const { return robustify_; };



	private:

		bool useConstraints_;
		int nObs_;
		std::vector<std::string> projFuncTypes_;


		double * parameters_;
		double * constraints_;
		double * constraintWeights_;
		double * errorWeights_;
		double * observations_;
		double * weights_;
		int* blockPointers_;
		int* parmask_;
		std::vector<double *> auxdata;

		// options
		int num_iterations_;
		int num_threads_;
		double funcTolerance_;
		double gradTolerance_;
		double parTolerance_;
		bool robustify_;
		bool verbose_;

	};

	void Solve(CBAInterface interface);

}
