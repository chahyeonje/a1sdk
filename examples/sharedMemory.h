//
// Created by a1 on 23. 1. 18.
//

#ifndef UNITREE_LEGGED_SDK_SHAREDMEMORY_H
#define UNITREE_LEGGED_SDK_SHAREDMEMORY_H

extern double resFL;
extern double resFR;
extern double resRL;
extern double resRR;
extern double simTime;
extern double resRR1;
extern double resRR2;
extern double resRR3;

typedef struct _SHM_
{
    bool start ;

}SHM, *pSHM;
#endif //UNITREE_LEGGED_SDK_SHAREDMEMORY_H
