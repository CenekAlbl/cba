#include <cba.h>
#include <calibrated_perspective_projection.h>

namespace cba {

	constexpr int CalibratedPerspectiveProjection::blockSizes_[];

	ceres::CostFunction * CalibratedPerspectiveProjection::getCostFunction(double const * const obsv, double weight, double const * const data) const
	{
		return new ceres::AutoDiffCostFunction<CalibratedPerspectiveProjection::ProjFunc, 2, CalibratedPerspectiveProjection::blockSizes_[0],
							 CalibratedPerspectiveProjection::blockSizes_[1], CalibratedPerspectiveProjection::blockSizes_[2]>
			(new CalibratedPerspectiveProjection::ProjFunc(obsv, weight, data));
	}

	CalibratedPerspectiveProjection::~CalibratedPerspectiveProjection()
	{
	}

	template<typename T>
	bool CalibratedPerspectiveProjection::ProjFunc::operator()(T const * eax, T const * t, T const * X, T * residuals) const{
        T rotatedX[3];
		ceres::AngleAxisRotatePoint(eax, X, rotatedX);
		T translatedX[3];
		translatedX[0] = rotatedX[0] + t[0];
		translatedX[1] = rotatedX[1] + t[1];
		translatedX[2] = rotatedX[2] + t[2];
		T x, y;
		x = translatedX[0] / translatedX[2];
		y = translatedX[1] / translatedX[2];
		residuals[0] = T(obsv_[0]) - x;
		residuals[1] = T(obsv_[1]) - y;
		return true;
	}

	

	ProjectionRegister<CalibratedPerspectiveProjection> CalibratedPerspectiveProjection::projectionRegister_("CalibratedPerspectiveProjection");

}

