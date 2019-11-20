#include <iostream>
#include "generate_scene.h"
#include "cba.h"



int main(int argc, char ** argv){
    int ncams = 2;
    int npts = 20;
    int nparams = ncams*6 + npts*3;
    Eigen::MatrixXd X = cba::generateRandomScene(10,20);
    Eigen::MatrixXd P1 = cba::generateRandomCamera(X,1000,30);
    Eigen::MatrixXd P2 = cba::generateRandomCamera(X,1000,30);
    Eigen::MatrixXd u1 = cba::projectPointsInCamera(P1,X);
    Eigen::MatrixXd u2 = cba::projectPointsInCamera(P2,X);
    std::cout << "Points: " << X << "\n";
    std::cout << "Cameras: " << P1 << "\n" << P2 << "\n";
    std::cout << "Projections: " << u1 << "\n";

    cba::CBAInterface interface;
    double * parameters = new double[nparams];
    ceres::RotationMatrixToAngleAxis(P1.leftCols(3).data(),parameters);
    std::memcpy(P1.rightCols(1).data(),parameters+3,3);
    ceres::RotationMatrixToAngleAxis(P1.leftCols(3).data(),parameters+6);
    std::memcpy(P2.rightCols(1).data(),parameters+9,3);
    Eigen::MatrixXd observations(2,2*npts);
    observations << u1, u2;
    
    interface.setParameters(parameters);
    interface.setObservations(observations.data());
    cba::Solve(interface);



}