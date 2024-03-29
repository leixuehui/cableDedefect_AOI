#pragma once
#include <opencv2/opencv.hpp>
#include "centringCableParam.h"

using namespace std;
using namespace cv;

#define CENTRINGCABLE_EXPORTS

#ifdef CENTRINGCABLE_EXPORTS
#define CENTRINGCABLEDLL_API __declspec(dllexport)
#else
#define CENTRINGCABLEDLL_API __declspec(dllimport)
#endif // CENTRINGCABLE_EXPORTS

class CENTRINGCABLEDLL_API APICentringCable
{
public:
    APICentringCable();
    ~APICentringCable();
    
    int api_init();
    /**********************************************/
    // loadMoveData
    // 载入当前运动数据
    // Input:
    //      filePath    //运动参数文档
    // Output:
    //      moveStatus  //运动数据
    /**********************************************/
    int api_loadMoveData(string filePath, MoveStatus &moveStatus);

    /**********************************************/
    // cableStatus
    // 判断当前线缆位置与运动方向
    // Input:
    //      img             排列后的图像组(leftUp,leftDown,rightUp; leftUp, leftDown, rightUp, rightDown)
    //		nCamera         相机个数
    //      nFrame          采集帧数(通常大于1，防止单一位置出现跳变值）
    // Output:
    //      outMoveInfo     运动位置信息
    // 		outMoveStatus   运动状态
    /**********************************************/
    int api_cableStatus(Mat *img, int nCamera, int nFrame, MoveInfo &outMoveInfo, MoveStatus &outMoveStatus);

    /**********************************************/
    // optimalMove
    // 获取最优运动数据
    // Input:
    //      img             //线缆图像组
    //      moveInfo        //运动数据(内部数据优化）
    //      isMoving        //是否正在运动（true:处于采集状态，false:处于预测状态）
    // Output:
    //      moveValue       //提供电机运动数据
    /**********************************************/
    int api_optimalMove(Mat *img, MoveInfo &moveInfo, double &moveValue, bool isMoving = true);
private:
    void *gProcess = nullptr;
};