#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "colRBDL.hpp"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <rbdl/rbdl.h>
#include <iomanip>

using namespace UNITREE_LEGGED_SDK;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

RigidBodyDynamics::Model* model = new RigidBodyDynamics::Model();
std::string modelFile = std::string(URDF_RSC_DIR)+ "/a1.urdf";
bool modelLoaded = RigidBodyDynamics::Addons::URDFReadFromFile(modelFile.c_str(), model, true);
colRBDL coldTest(model);

class Custom
{
public:
    Custom(uint8_t level = 0x00)
    :safe(LeggedType::A1)
    ,udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        udp.InitCmdData(cmd);
    }

    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};

    int motiontime = 0;
    float dt = 0.005;     // 0.001~0.01
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime += 2;
    udp.GetRecv(state);

//    printf("%f %f %f %f %f\n", state.imu.rpy[1], state.imu.rpy[2], state.position[0], state.position[1], state.velocity[0]);
//    coldTest.UpdateState(robot.getQ().e(), robot.getQD().e(), MPCcontroller.GetTorque().e());
//    orientation.quaternion; // w,x,y,z

//    for (int i = 0; i < 20; i++) {
//        std::cout << std::setw(16) << std::right << lstate.motorState[i].q;
//    }
//    std::cout << std::endl;


    cmd.mode = 0;
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0]  = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;


    if(motiontime > 0 && motiontime < 2000){
        cmd.mode = 1;
        cmd.bodyHeight = -0.2;
        printf("cmd.bodyHeight : %f \n", cmd.bodyHeight);

    }
    if(motiontime > 2000 && motiontime < 4000){
        cmd.mode = 1;
        cmd.bodyHeight = 0.1;
        printf("cmd.bodyHeight : %f \n", cmd.bodyHeight);
    }
    if(motiontime > 4000 && motiontime < 6000){
        cmd.mode = 1;
        cmd.bodyHeight = 0.0;
        printf("cmd.bodyHeight : %f \n", cmd.bodyHeight);
    }
    if(motiontime > 6000){
        cmd.mode = 0;
        cmd.velocity[0] = 0;
        printf("idle \n");
    }
    udp.SetSend(cmd);
}

int main(void) 
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
//    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();


    while(1){
        sleep(10);
    };

    return 0; 
}
