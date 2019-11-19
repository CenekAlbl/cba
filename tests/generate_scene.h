#include <Eigen/Eigen>
#include <ceres/rotation.h>
#include <ceres/ceres.h>

namespace cba {

    Eigen::MatrixXd generateRandomScene(double size, int nPoints){    
        Eigen::MatrixXd X = Eigen::MatrixXd::Random(3,nPoints)*size;
        return X;
    }

    Eigen::MatrixXd generateRandomCamera(Eigen::MatrixXd X, double distanceFromScene, double focalLength){
        // center of the point cloud
        Eigen::Vector3d center = X.rowwise().mean();
        //Random camera orientation
        Eigen::Vector3d eax = Eigen::Vector3d::Random();
        Eigen::Matrix3d R;
        ceres::AngleAxisToRotationMatrix(eax.data(),R.data());
        // Set camera center such that the camera is looking at the scene
        Eigen::Vector3d t =  Eigen::Vector3d(0,0,distanceFromScene);
         Eigen::MatrixXd P(3,4);
         P << R,t;
        return P;
    }

    Eigen::MatrixXd projectPointsInCamera(Eigen::MatrixXd P, Eigen::MatrixXd X){
        Eigen::MatrixXd Xh(4,X.cols());
        Xh.topRows(3) << X;
        Xh.bottomRows(1) = Eigen::MatrixXd::Ones(1,X.cols());
        Eigen::MatrixXd uh = P*Xh;
        Eigen::MatrixXd u = uh.topRows(2);
        u.array() /= uh.bottomRows(1).replicate(2,1).array();
        return u;
    }

}


