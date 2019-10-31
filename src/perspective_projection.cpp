#include <cba.h>
#include <perspective_projection.h>

namespace cba {

	constexpr int PerspectiveProjection::blockSizes_[];

	ceres::CostFunction * PerspectiveProjection::getCostFunction(double const * const obsv, double weight, double const * const data) const
	{
		return new ceres::AutoDiffCostFunction<PerspectiveProjection::ProjFunc, 2, PerspectiveProjection::blockSizes_[0],
							 PerspectiveProjection::blockSizes_[1], PerspectiveProjection::blockSizes_[2], PerspectiveProjection::blockSizes_[3]>
			(new PerspectiveProjection::ProjFunc(obsv, weight, data));
	}

	PerspectiveProjection::~PerspectiveProjection()
	{
	}

	template<typename T>
	bool PerspectiveProjection::ProjFunc::operator()(T const * eax, T const * C, T const * k, T const * X, T * residuals) const{
		T translatedX[3];
		translatedX[0] = X[0] - C[0];
		translatedX[1] = X[1] - C[1];
		translatedX[2] = X[2] - C[2];
		T rotatedX[3];
		ceres::AngleAxisRotatePoint(eax, translatedX, rotatedX);
		T x, y;
		x = rotatedX[0] / rotatedX[2];
		y = rotatedX[1] / rotatedX[2];
		x = k[0] * x + k[4] * y + k[2];
		y = k[1] * y + k[3];
		residuals[0] = T(obsv_[0]) - x;
		residuals[1] = T(obsv_[1]) - y;
		return true;
	}

	

	ProjectionRegister<PerspectiveProjection> PerspectiveProjection::projectionRegister_("PerspectiveProjection");

}

