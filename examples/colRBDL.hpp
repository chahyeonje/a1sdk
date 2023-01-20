//
// Created by a1 on 23. 1. 14.
//

#ifndef UNITREE_LEGGED_SDK_COLRBDL_HPP
#define UNITREE_LEGGED_SDK_COLRBDL_HPP

#include <eigen3/Eigen/Eigen>
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "sharedMemory.h"

extern pSHM smem;

class colRBDL
{
public:
    colRBDL(RigidBodyDynamics::Model* model);
    void GetResidul(Eigen::VectorXd& residual);
    void UpdateState(const Eigen::VectorXd& generalizedPosition,
                     const Eigen::VectorXd& generalizedVelocity,
                     const Eigen::VectorXd& torque);


private:
    void calcResidual();
private:
    Eigen::VectorXd mBeta;
    double mDt;
    Eigen::VectorXd mCalcTorque;
    Eigen::Matrix<double, 18, 18> mGain;
    RigidBodyDynamics::Math::MatrixNd mH;
    RigidBodyDynamics::Math::MatrixNd mHprev;
    RigidBodyDynamics::Model* mModel;
    Eigen::VectorXd mMomentum;
    Eigen::VectorXd mQ;
    Eigen::VectorXd mQd;
    Eigen::VectorXd mQdd;
    Eigen::VectorXd mQdPrev;
    Eigen::VectorXd mQPrev;
    Eigen::VectorXd mResidual;
    double mResFL;
    double mResFR;
    double mResRL;
    double mResRR;

    bool mHysFL;
    bool mHysFR;
    bool mHysRL;
    bool mHysRR;

    bool mConFL;
    bool mConFR;
    bool mConRL;
    bool mConRR;

    Eigen::VectorXd mTau;

};


#endif //UNITREE_LEGGED_SDK_COLRBDL_HPP
