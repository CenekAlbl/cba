#pragma once
#include <projection.h>
#include <ceres/rotation.h>

namespace cba {

	class PerspectiveProjection : public Projection
	{
	public:
		class ProjFunc : public ProjFuncInterface {
		public:
			//inheriting the base class constructor
			using ProjFuncInterface::ProjFuncInterface;
			//overriding the projection function in operator() for ceres
			template <typename T>
			bool operator()(T const * eax, T const * C, T const * K, T const * X, T * resiudals) const;

			virtual std::string identify() { return "It's Perspective Projection"; }
		
		};

		virtual ceres::CostFunction* getCostFunction(double const * const obsv, double weight, double const * const data = NULL) const;

		virtual int getNumBlocks() const { return 4; };
		virtual int getResidualSize() const { return 2; };
		virtual const int * getBlockSizes() const { return blockSizes_; };
		
		~PerspectiveProjection();
	
		static const int numBlocks_ = 3;
		static const int residualSize_ = 2;
		static constexpr int blockSizes_[] = {3,3,5,3}; 

	private:
		static ProjectionRegister<PerspectiveProjection> projectionRegister_;


		

	};

	
	

}

