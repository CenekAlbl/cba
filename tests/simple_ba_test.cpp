#include <iostream>
#include "generate_scene.h"
#include "cba.h"



int main(int argc, char ** argv){
    Eigen::MatrixXd X = cba::generateRandomScene(10,20);
    Eigen::MatrixXd P = cba::generateRandomCamera(X,1000,30);
    Eigen::MatrixXd u = cba::projectPointsInCamera(P,X);
    std::cout << "Points: " << X << "\n";
    std::cout << "Cameras: " << P << "\n";
    std::cout << "Projections: " << u << "\n";



}