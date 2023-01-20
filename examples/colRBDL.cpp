//
// Created by a1 on 23. 1. 14.
//

#include "colRBDL.hpp"

colRBDL::colRBDL(RigidBodyDynamics::Model *model)
        :mBeta(Eigen::VectorXd::Zero(18))
        ,mCalcTorque(Eigen::VectorXd::Zero(18))
        ,mConFL(1)
        ,mConFR(1)
        ,mConRL(1)
        ,mConRR(1)
        ,mDt(0.005)
        ,mGain(Eigen::Matrix<double,18,18>::Identity() * 50)
        ,mH(Eigen::MatrixXd::Zero(18,18))
        ,mHprev(Eigen::MatrixXd::Zero(18,18))
        ,mHysFL(1)
        ,mHysFR(1)
        ,mHysRL(1)
        ,mHysRR(1)
        ,mModel(model)
        ,mMomentum(Eigen::VectorXd::Zero(18))
        ,mQ(Eigen::VectorXd::Zero(19))
        ,mQd(Eigen::VectorXd::Zero(18))
        ,mQdd(Eigen::VectorXd::Zero(18))
        ,mQdPrev(Eigen::VectorXd::Zero(18))
        ,mQPrev(Eigen::VectorXd::Zero(19))
        ,mResidual(Eigen::VectorXd::Zero(18))
        ,mTau(Eigen::VectorXd::Zero(18))
{
    mGain(12,12) = 50;
    mGain(13,13) = 50;
    mGain(14,14) = 50;
    mGain(15,15) = 50;
    mGain(16,16) = 50;
    mGain(17,17) = 50;

    std::cout << "Degree of freedom overview : " << std::endl;
    std::cout << RigidBodyDynamics::Utils::GetModelDOFOverview(*mModel);

    std::cout << "Model Hierarchy:" << std::endl;
    std::cout << RigidBodyDynamics::Utils::GetModelHierarchy(*mModel);

    std::cout << "q : " << mModel->q_size << ", qdot : " << mModel->qdot_size << std::endl;

    std::cout << mGain << std::endl;
}

void colRBDL::calcResidual()
{
    mBeta = mTau - (mH - mHprev) / mDt * mQd;
    mMomentum = mMomentum + mCalcTorque * mDt - mBeta * mDt + mResidual * mDt;
    mResidual = mGain * (-mMomentum + mH * mQd);

    mResFL = sqrt(pow(mResidual[6],2) + pow(mResidual[7],2) + pow(mResidual[8],2));
    mResFR = sqrt(pow(mResidual[9],2) + pow(mResidual[10],2) + pow(mResidual[11],2));
    mResRL = sqrt(pow(mResidual[12],2) + pow(mResidual[13],2) + pow(mResidual[14],2));
    mResRR = sqrt(pow(mResidual[15],2) + pow(mResidual[16],2) + pow(mResidual[17],2));

    resRR1 = mResidual[15];
    resRR2 = mResidual[16];
    resRR3 = mResidual[17];

    mHprev = mH;
    mQPrev = mQ;
    mQdPrev = mQd;

    if(mConFL == 1 && mResFL < 7)
    {
        mConFL = 0;
    }
    else if(mConFL == 0 && mResFL > 8)
    {
        mConFL = 1;
    }

    if(mConFR == 1 && mResFR < 7)
    {
        mConFR = 0;
    }
    else if(mConFR == 0 && mResFR > 8)
    {
        mConFR = 1;
    }

    if(mConRL == 1 && mResRL < 3)
    {
        mConRL = 0;
    }
    else if(mConRL == 0 && mResRL >4)
    {
        mConRL = 1;
    }

    if(mConRR == 1 && mResRR < 3)
    {
        mConRR = 0;
    }
    else if(mConRR == 0 && mResRR >4)
    {
        mConRR = 1;
    }
}

void colRBDL::GetResidul(Eigen::VectorXd &residual)
{
//    for(int i=0; i<18; i++)
//    {
//        residual[i] = mResidual[i];
//    }
    residual[0] = mResFR;
    residual[1] = mResFL;
    residual[2] = mResRR;
    residual[3] = mResRL;
}

void colRBDL::UpdateState(const Eigen::VectorXd& generalizedPosition,
                          const Eigen::VectorXd& generalizedVelocity,
                          const Eigen::VectorXd& torque)
{
    mQ[0] = generalizedPosition[0];
    mQ[1] = generalizedPosition[1];
    mQ[2] = generalizedPosition[2];

    mQ[3] = generalizedPosition[4]; /// q_x
    mQ[4] = generalizedPosition[5]; /// q_y
    mQ[5] = generalizedPosition[6]; /// q_z
    mQ[18] = generalizedPosition[3]; /// q_w

    mQ[6] = generalizedPosition[10];
    mQ[7] = generalizedPosition[11];
    mQ[8] = generalizedPosition[12];

    mQ[9] = generalizedPosition[7];
    mQ[10] = generalizedPosition[8];
    mQ[11] = generalizedPosition[9];

    mQ[12] = generalizedPosition[16];
    mQ[13] = generalizedPosition[17];
    mQ[14] = generalizedPosition[18];

    mQ[15] = generalizedPosition[13];
    mQ[16] = generalizedPosition[14];
    mQ[17] = generalizedPosition[15];

    mQd[0] = generalizedVelocity[0]; /// base_TX
    mQd[1] = generalizedVelocity[1]; /// base_TY
    mQd[2] = generalizedVelocity[2]; /// base_TZ

    mQd[3] = generalizedVelocity[5]; /// base_RZ
    mQd[4] = generalizedVelocity[4]; /// base_RY
    mQd[5] = generalizedVelocity[3]; /// base_RX

    mQd[6] = generalizedVelocity[9]; /// FL_hip_RX
    mQd[7] = generalizedVelocity[10];/// FL_thigh_RY
    mQd[8] = generalizedVelocity[11];/// FL_calf_RY

    mQd[9] = generalizedVelocity[6]; /// FR_hip_RX
    mQd[10] = generalizedVelocity[7];/// FR_thigh_RY
    mQd[11] = generalizedVelocity[8];/// FR_calf_RY

    mQd[12] = generalizedVelocity[15];/// RL_hip_RX
    mQd[13] = generalizedVelocity[16];/// RL_thigh_RX
    mQd[14] = generalizedVelocity[17];/// RL_calf_RX

    mQd[15] = generalizedVelocity[12];/// RR_hip_RX
    mQd[16] = generalizedVelocity[13];/// RR_thigh_RX
    mQd[17] = generalizedVelocity[14];/// RR_calf_RX

    mCalcTorque[0] = torque[0];
    mCalcTorque[1] = torque[1];
    mCalcTorque[2] = torque[2];

    mCalcTorque[3] = torque[5];
    mCalcTorque[4] = torque[4];
    mCalcTorque[5] = torque[3];

    mCalcTorque[6] = torque[9];
    mCalcTorque[7] = torque[10];
    mCalcTorque[8] = torque[11];

    mCalcTorque[9] = torque[6];
    mCalcTorque[10] = torque[7];
    mCalcTorque[11] = torque[8];

    mCalcTorque[12] = torque[15];
    mCalcTorque[13] = torque[16];
    mCalcTorque[14] = torque[17];

    mCalcTorque[15] = torque[12];
    mCalcTorque[16] = torque[13];
    mCalcTorque[17] = torque[14];

    RigidBodyDynamics::CompositeRigidBodyAlgorithm(* mModel, mQ,mH, true);
    RigidBodyDynamics::NonlinearEffects(* mModel, mQPrev, mQdPrev, mTau);
    this->calcResidual();
}