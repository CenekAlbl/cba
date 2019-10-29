#include "../include/projection.h"

namespace cba {
	ProjectionFactory::MapType * ProjectionFactory::map_;


	Projection * ProjectionFactory::createProjectionInstance(std::string const projectionType)
	{

		return getMap()[projectionType]();

	}



	ProjFuncInterface::ProjFuncInterface(double const * const obsv, double const weight, double const * const data)
		:obsv_(obsv),weight_(weight),data_(data)
	{
	}

}
