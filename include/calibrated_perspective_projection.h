#pragma once
#include <projection.h>
#include <ceres/rotation.h>

namespace cba {

	class CalibratedPerspectiveProjection : public Projection
	{
	public:
		class ProjFunc : public ProjFuncInterface {
		public:
			//inheriting the base class constructor
			using ProjFuncInterface::ProjFuncInterface;
			//overriding the projection function in operator() for ceres
			template <typename T>
			bool operator()(T const * eax, T const * C, T const * X, T * resiudals) const;

			virtual std::string identify() { return "It's a Calibrated Perspective Projection"; }
		
		};

		virtual ceres::CostFunction* getCostFunction(double const * const obsv, double weight, double const * const data = NULL) const;

		virtual int getNumBlocks() const { return numBlocks_; };
		virtual int getResidualSize() const { return residualSize_; };
		virtual const int * getBlockSizes() const { return blockSizes_; };
		
		~CalibratedPerspectiveProjection();
	
		static const int numBlocks_ = 3;
		static const int residualSize_ = 2;
		static constexpr int blockSizes_[] = {3,3,3}; 

	private:
		static ProjectionRegister<CalibratedPerspectiveProjection> projectionRegister_;


		

	};

	
	

}

