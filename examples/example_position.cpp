/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <iomanip>
#include <rbdl/rbdl.h>
#include <QApplication>
#include "colRBDL.hpp"
#include "qt/mainwindow.h"
#include "sharedMemory.h"

double resFL;
double resFR;
double resRL;
double resRR;
double simTime;
double resRR1;
double resRR2;
double resRR3;

using namespace std;
using namespace UNITREE_LEGGED_SDK;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
std::string modelFile = std::string(URDF_RSC_DIR)+ "/a1.urdf";
bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(modelFile.c_str(), model, true);
colRBDL coldTest(model);

Eigen::VectorXd generalizedPos;
Eigen::VectorXd generalizedVel;
Eigen::VectorXd torque;
Eigen::VectorXd residual;

class Custom
{
public:
    Custom(uint8_t level)
    : safe(LeggedType::A1)
    , udp(level)
    {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void StateUpdate();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    float qInit[3]={0};
    float qDes[3]={0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};  
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};

void Custom::UDPRecv()
{  
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

void Custom::RobotControl() 
{
    motiontime++;
    simTime = double(motiontime) * 0.002 ;
    udp.GetRecv(state);

    this->StateUpdate();
    coldTest.GetResidul(residual);

    for(int i=0; i<4; i++){
//        std::cout << std::setw(16) << std::right << state.motorState[i].q ;
        std::cout << std::setw(16) << std::right << residual[i];
    }
    std::cout << std::endl;

    resFR = residual[0];
    resFL = residual[1];
    resRR = residual[2];
    resRL = residual[3];

    // gravity compensation
    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

    // if( motiontime >= 100){
    if( motiontime >= 0){
        // first, get record initial position
        // if( motiontime >= 100 && motiontime < 500){
        if( motiontime >= 0 && motiontime < 10){
            qInit[0] = state.motorState[FR_0].q;
            qInit[1] = state.motorState[FR_1].q;
            qInit[2] = state.motorState[FR_2].q;
        }
        // second, move to the origin point of a sine movement with Kp Kd
        // if( motiontime >= 500 && motiontime < 1500){
        if( motiontime >= 10 && motiontime < 400){
            rate_count++;
            double rate = rate_count/200.0;                       // needs count to 200
            Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0; 
            Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
            
            qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
            qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
            qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
        }
        double sin_joint1, sin_joint2;
        // last, do sine wave
        if( motiontime >= 400){
            sin_count++;
            sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
            sin_joint2 = -0.6 * sin(1.8*M_PI*sin_count/1000.0);
            qDes[0] = sin_mid_q[0];
            qDes[1] = sin_mid_q[1] + sin_joint1;
            qDes[2] = sin_mid_q[2] + sin_joint2;
        }

        for(int i=0; i<4; i++)
        {
            switch(i)
            {
                case FR_:
                    cmd.motorCmd[FR_0].q = qDes[0];
                    cmd.motorCmd[FR_0].dq = 0;
                    cmd.motorCmd[FR_0].Kp = Kp[0];
                    cmd.motorCmd[FR_0].Kd = Kd[1];
                    cmd.motorCmd[FR_0].tau = -0.65f;

                    cmd.motorCmd[FR_1].q = qDes[1];
                    cmd.motorCmd[FR_1].dq = 0;
                    cmd.motorCmd[FR_1].Kp = Kp[0];
                    cmd.motorCmd[FR_1].Kd = Kd[1];
                    cmd.motorCmd[FR_1].tau = 0.0f;

                    cmd.motorCmd[FR_2].q = qDes[2];
                    cmd.motorCmd[FR_2].dq = 0;
                    cmd.motorCmd[FR_2].Kp = Kp[0];
                    cmd.motorCmd[FR_2].Kd = Kd[1];
                    cmd.motorCmd[FR_2].tau = 0.0f;
                    break;
                case FL_:
                    cmd.motorCmd[FL_0].q = qDes[0];
                    cmd.motorCmd[FL_0].dq = 0;
                    cmd.motorCmd[FL_0].Kp = Kp[0];
                    cmd.motorCmd[FL_0].Kd = Kd[1];
                    cmd.motorCmd[FL_0].tau = 0.65f;

                    cmd.motorCmd[FL_1].q = qDes[1];
                    cmd.motorCmd[FL_1].dq = 0;
                    cmd.motorCmd[FL_1].Kp = Kp[0];
                    cmd.motorCmd[FL_1].Kd = Kd[1];
                    cmd.motorCmd[FL_1].tau = 0.0f;

                    cmd.motorCmd[FL_2].q = qDes[2];
                    cmd.motorCmd[FL_2].dq = 0;
                    cmd.motorCmd[FL_2].Kp = Kp[0];
                    cmd.motorCmd[FL_2].Kd = Kd[1];
                    cmd.motorCmd[FL_2].tau = 0.0f;

                    break;
                case RR_:
                    cmd.motorCmd[RR_0].q = qDes[0];
                    cmd.motorCmd[RR_0].dq = 0;
                    cmd.motorCmd[RR_0].Kp = Kp[0];
                    cmd.motorCmd[RR_0].Kd = Kd[1];
                    cmd.motorCmd[RR_0].tau = -0.65f;

                    cmd.motorCmd[RR_1].q = qDes[1];
                    cmd.motorCmd[RR_1].dq = 0;
                    cmd.motorCmd[RR_1].Kp = Kp[0];
                    cmd.motorCmd[RR_1].Kd = Kd[1];
                    cmd.motorCmd[RR_1].tau = 0.0f;

                    cmd.motorCmd[RR_2].q = qDes[2];
                    cmd.motorCmd[RR_2].dq = 0;
                    cmd.motorCmd[RR_2].Kp = Kp[0];
                    cmd.motorCmd[RR_2].Kd = Kd[1];
                    cmd.motorCmd[RR_2].tau = 0.0f;

                    break;
                case RL_:
                    cmd.motorCmd[RL_0].q = qDes[0];
                    cmd.motorCmd[RL_0].dq = 0;
                    cmd.motorCmd[RL_0].Kp = Kp[0];
                    cmd.motorCmd[RL_0].Kd = Kd[1];
                    cmd.motorCmd[RL_0].tau = 0.65f;

                    cmd.motorCmd[RL_1].q = qDes[1];
                    cmd.motorCmd[RL_1].dq = 0;
                    cmd.motorCmd[RL_1].Kp = Kp[0];
                    cmd.motorCmd[RL_1].Kd = Kd[1];
                    cmd.motorCmd[RL_1].tau = 0.0f;

                    cmd.motorCmd[RL_2].q = qDes[2];
                    cmd.motorCmd[RL_2].dq = 0;
                    cmd.motorCmd[RL_2].Kp = Kp[0];
                    cmd.motorCmd[RL_2].Kd = Kd[1];
                    cmd.motorCmd[RL_2].tau = 0.0f;

                    break;
            }
        }   /// generate trajectory of legs

    }

    if(motiontime > 10){
        safe.PositionLimit(cmd);
        safe.PowerProtect(cmd, state, 1);
        // You can uncomment it for position protection
        // safe.PositionProtect(cmd, state, 0.087);
    }

    udp.SetSend(cmd);

}

void Custom::StateUpdate()
{
    generalizedPos[0] = 0;
    generalizedPos[1] = 0;
    generalizedPos[2] = 0.5; /// about 0.5m
    generalizedPos[3] = state.imu.quaternion[0];
    generalizedPos[4] = state.imu.quaternion[1];
    generalizedPos[5] = state.imu.quaternion[2];
    generalizedPos[6] = state.imu.quaternion[3];

    generalizedVel[0] = 0;
    generalizedVel[1] = 0;
    generalizedVel[2] = 0;
    generalizedVel[3] = state.imu.gyroscope[0];
    generalizedVel[4] = state.imu.gyroscope[1];
    generalizedVel[5] = state.imu.gyroscope[2];

    torque[0] = 0;
    torque[1] = 0;
    torque[2] = 0;
    torque[3] = 0;
    torque[4] = 0;
    torque[5] = 0;

    for(int i=0; i<12; i++)
    {
        generalizedPos[i+7] = state.motorState[i].q;
        generalizedVel[i+6] = state.motorState[i].dq;
        torque[i+6] = state.motorState[i].tauEst;
    }
    coldTest.UpdateState(generalizedPos, generalizedVel,torque);
}


int main(int argc, char* argv[])
{
//    smem->start = 0;
    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    residual = Eigen::VectorXd::Zero(4);
    generalizedPos = Eigen::VectorXd::Zero(19);
    generalizedVel = Eigen::VectorXd::Zero(18);
    torque = Eigen::VectorXd::Zero(18);
    
    Custom custom(LOWLEVEL);
    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    a.exec();

    while(1){
        sleep(10);
    };

    return 0; 
}
