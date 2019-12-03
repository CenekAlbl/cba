#include <ceres/ceres.h>
#include <projection.h>
#include <perspective_projection.h>
#include <cba.h>


namespace cba {

	void setCeresSolverOptions(ceres::Solver::Options* options) {


	}

	void buildProblem(const CBAInterface & cbaInterface, ceres::Problem * problem) {
		int blockId = 0;
		int observationIdx = 0;
		for (int i = 0; i < cbaInterface.getNumObs(); i++)
		{
			std::cout << "Filling up observation: " << i << "\n";
			cba::Projection * projection = cba::ProjectionFactory::createProjectionInstance(cbaInterface.getProjFuncType(i));
			std::cout << "Create cost function: " << i << "\n";
			ceres::CostFunction * costFunction = projection->getCostFunction(cbaInterface.getObservations() + observationIdx, cbaInterface.getWeight(i),cbaInterface.getAuxiliaryData(i));
			std::cout << "Lossfunction: " << i << "\n";	
			ceres::LossFunction *lossFunction = cbaInterface.getRobustify() ? new ceres::HuberLoss(1.0) : NULL;

			// build residual blocks
			std::vector<double *> blocks;
			for (int j = 0; j < projection->getNumBlocks(); j++)
			{
				std::cout << "Filling up block: " << blockId;
				std::cout << " with block pointer " << cbaInterface.getBlockPointers()[blockId];
				blocks.push_back(cbaInterface.getParameters()+cbaInterface.getBlockPointers()[blockId]);
				std::cout << " with ";
				for (int k = 0; k < projection->getBlockSizes()[j]; k++)
				{
					std::cout << blocks.back()[k] << " ";
				}
				std::cout << "\n";
				
				blockId++;
			}

			problem->AddResidualBlock(costFunction, lossFunction, blocks);
			observationIdx +=projection->getResidualSize();
		}

		// //take care of constant parameters
		// for (int i = 0; i < cbaInterface.getNumParBlocks(); i++)
		// {
		// 	ceres::SubsetParameterization *constant_parameterization = NULL;
		// 	double * block = cbaInterface.getParameters()+cbaInterface.getBlockPointers()[i];
		// 	std::vector<int> constantParsWithinBlock;
		// 	for (int i = 0; i < 9; i++)
		// 	{
		// 		if (sba_problem->fixed_parmask[j * sba_problem->num_camera_parameters_ + i]) {
		// 			constant_pars.push_back(i);
		// 		}
		// 	}
		// 	constant_parameterization = new ceres::SubsetParameterization(9, constant_pars);
		// 	problem->SetParameterization(camera, constant_parameterization);
		// }

	
	}

	void Solve(CBAInterface interface) {
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_SCHUR;
		options.minimizer_progress_to_stdout = true;
		ceres::Solver::Summary summary;
		ceres::Problem problem;
		buildProblem(interface,&problem);
		ceres::Solve(options, &problem, &summary);
		std::cout << summary.FullReport() << "\n";
	}


	CBAInterface::CBAInterface( double * parameters, int * parameterBlockPointers, double * observations, int nObs, std::vector<std::string> projFuncTypes)
	{
		useConstraints_ = false;
		nObs_ = nObs;
		projFuncTypes_ = projFuncTypes;


		parameters_ = parameters;
		observations_ = observations;
		constraints_ = NULL;
		constraintWeights_ = NULL;
		errorWeights_ = NULL;
		blockPointers_ = parameterBlockPointers;
		parmask_ = NULL;

		// options
		num_iterations_ = 5;
		num_threads_ = 1;
		funcTolerance_ = 1e-3;
		gradTolerance_ = 1e-3;
		parTolerance_ = 1e-3;
		robustify_ = false;
		verbose_  = false;

	}
	
}









