#include <Eigen/Eigen>
#include <ceres/rotation.h>
#include <ceres/ceres.h>

namespace cba {

    Eigen::MatrixXd generateRandomScene(double size, int nPoints){    
        Eigen::MatrixXd X = Eigen::MatrixXd::Random(3,nPoints)*size;
        return X;
    }

    int generateRandomCamera(Eigen::MatrixXd X, double distanceFromScene, double focalLength){
        // center of the point cloud
        Eigen::Vector3d center = X.rowwise().mean();
        //Random camera orientation
        Eigen::Vector3d eax = Eigen::Vector3d::Random();
        Eigen::Matrix3d R;
        ceres::AngleAxisToRotationMatrix(eax.data(),R.data());
        // Set camera center such that the camera is looking at the scene
        Eigen::Vector3d t =  center + Eigen::Vector3d(0,0,distanceFromScene);
        Eigen::Vector3d randCamLoc = R*t;

    }

}


