#include "generate_scene.h"
#include <iostream>



int main(int argc, char ** argv){
    Eigen::MatrixXd X = cba::generateRandomScene(10,20);
    std::cout << X;

}