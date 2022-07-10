#include "Api_CentringCable.hpp"

#include "centringCable.h"
APICentringCable::APICentringCable()
{
    if(!gProcess)   gProcess = new CentringCable();
}

APICentringCable::~APICentringCable()
{
}

int APICentringCable::api_init()
{
    if (!gProcess)   gProcess = new CentringCable();
    
    CentringCable *process = (CentringCable*)gProcess;
    process->init();
	process->initNoCableStatus();
    return 0;
}

int APICentringCable::api_destory()
{
    if (gProcess)   delete gProcess;
    return 0;
}

int APICentringCable::api_loadMoveData(string filePath, MoveStatus & moveStatus)
{
    int err = 0;
    if (!gProcess)  return -1;

    CentringCable *process = (CentringCable*)gProcess;
    err = process->loadMoveData(filePath, moveStatus);
    
    return err;
}

//	//结果位置输出
int APICentringCable::api_cableFinalPose(Mat *img, int nCamera, int nFrame, MoveInfo &outMovePose)
{
	int err = 0;
	if (!gProcess)  return -1;
	
	CentringCable *process = (CentringCable*)gProcess;
	err = process->cableFinalPose(img, nCamera, nFrame, outMovePose);

	return err;
}

int APICentringCable::api_cableStatus(Mat * img, int nCamera, int nFrame, MoveInfo & outMoveInfo, MoveStatus & outMoveStatus)
{
    int err = 0;
    if (!gProcess)  return -1;

    CentringCable *process = (CentringCable*)gProcess;
    err = process->cableStatus(img, nCamera, nFrame, outMoveInfo, outMoveStatus);

    return err;
}

int APICentringCable::api_optimalMove(Mat * img, MoveInfo & moveInfo, double & moveValue, bool isMoving)
{
    int err = 0;
    if (!gProcess)  return -1;

    CentringCable *process = (CentringCable*)gProcess;
    err = process->optimalMove(img, moveInfo, moveValue, isMoving);
    return err;
}

int APICentringCable::api_estimateMove(Mat *img, int nFrame, int nCamera, int ms, int &moveTime, bool &isOk)
{
    int err = 0;
    if (!gProcess)  return -1;

    CentringCable *process = (CentringCable*)gProcess;
    
    err = process->estimateMove(img, nFrame, nCamera, ms, moveTime, isOk);
    return err;
}

int APICentringCable::api_realTime(Mat *img, int nFrame, int nCamera, int condDist, int finePlcTime_1, int finePlcTime_2, int &plcTime, int &direct, bool &isOK)
{
    int err = 0;
    if (!gProcess)  return -1;

    CentringCable *process = (CentringCable*)gProcess;

    err = process->realTime(img, nFrame, nCamera, condDist, finePlcTime_1, finePlcTime_2, plcTime,direct, isOK);
    return err;
    return 0;
}
