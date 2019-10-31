#include <cba.h>
#include <projection.h>
#include <perspective_projection.h>


int main(int argc, char ** argv) {
	cba::CBAInterface &cbaInterface();

	//cmpba::PerspectiveProjection * dummy = new cmpba::PerspectiveProjection();
	std::string projType("PerspectiveProjection");
	/*cmpba::Projection * proj = new cmpba::PerspectiveProjection();
	ceres::CostFunction * cost_func = proj->getCostFunction(NULL, NULL, NULL);*/
	cba::Projection * proj =  cba::ProjectionFactory::createProjectionInstance(projType);
	ceres::CostFunction * cost_func = proj->getCostFunction(NULL, NULL, NULL);
	std::cout << cost_func->num_residuals() << std::endl;
	return 0;
}