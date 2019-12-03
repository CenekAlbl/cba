#include <iostream>
#include "generate_scene.h"
#include "cba.h"



int main(int argc, char ** argv){
    int ncams = 2;
    int npts = 20;
    int nparams = ncams*6 + npts*3;
    int nobs = ncams*npts;
    Eigen::MatrixXd X = cba::generateRandomScene(10,20);
    Eigen::MatrixXd P1 = cba::generateRandomCamera(X,1000,30);
    Eigen::MatrixXd P2 = cba::generateRandomCamera(X,1000,30);
    Eigen::MatrixXd u1 = cba::projectPointsInCamera(P1,X);
    Eigen::MatrixXd u2 = cba::projectPointsInCamera(P2,X);
    std::cout << "Points: " << X << "\n";
    std::cout << "Cameras: " << P1 << "\n" << P2 << "\n";
    std::cout << "Projections: " << u1 << "\n";

    
    double * parameters = new double[nparams];
    ceres::RotationMatrixToAngleAxis(P1.leftCols(3).data(),parameters);
    std::memcpy(parameters+3,P1.rightCols(1).data(),3*sizeof(double));
    ceres::RotationMatrixToAngleAxis(P2.leftCols(3).data(),parameters+6);
    std::memcpy(parameters+9,P2.rightCols(1).data(),3*sizeof(double));
    std::memcpy(parameters + 12,X.data(),npts*3*sizeof(double));
    Eigen::MatrixXd observations(2,nobs);
    observations << u1, u2;
    
    std::vector<std::string> projFuncs;
    std::vector<int> parameterBlockPointers; 
    for (int i = 0; i < int(nobs/2); i++)
    {
        projFuncs.push_back(("CalibratedPerspectiveProjection"));
        parameterBlockPointers.push_back(0); // camera 1 rotation
        parameterBlockPointers.push_back(3); // camera 1 translation
        parameterBlockPointers.push_back(12+i*3); // 3D point
    }

    for (int i = 0; i < int(nobs/2); i++)
    {
        projFuncs.push_back(("CalibratedPerspectiveProjection"));
        parameterBlockPointers.push_back(6); // camera 2 rotation
        parameterBlockPointers.push_back(9); // camera 2 translation
        parameterBlockPointers.push_back(12+i*3); // 3D point
    }
    
    cba::CBAInterface  interface(parameters,  parameterBlockPointers.data(),  (double *)observations.data(), nobs, projFuncs);

    cba::Solve(interface);



}