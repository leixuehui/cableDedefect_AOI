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

    int api_destory();

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


	//结果位置输出
	int api_cableFinalPose(Mat *img, int nCamera, int nFrame, MoveInfo &outMovePose);

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
    int api_estimateMove(Mat *img, int nFrame, int nCamera, int ms, int &moveTime, bool &isOk);

    /**********************************************/
    // api_realTime
    // 获取最优运动数据
    // Input:
    //      img                 线缆图像组
    //      nFrame              每个相机传入几帧
	//      nCamera             几个相机工作
	//      correctDist         平均偏移量的阈值
	//      finePlcTime_1       进入换向范围的plc细调步长
	//      finePlcTime_2       进入20%范围的plc细调步长
    // Output:
    //      plcTime             返回plc运动时间
	//      direct              返回plc运动方向
	//      isOK                返回函数结果
    /**********************************************/
    int api_realTime(Mat *img, int nFrame, int nCamera, int condDist, int finePlcTime_1, int finePlcTime_2, 
		              int &plcTime, int &direct, bool &isOK);

private:
    void *gProcess = nullptr;
};