#include <opencv2/opencv.hpp>
#include <vector>
#include "centringCableParam.h"

using namespace std;
using namespace cv;


//*************************************extend***************************************
//1、当前状态检测与测试定速移动一段距离（当存在线缆时，固定一个方向移动；当不存在线
//缆时，朝一个方向移动至限位，若中间存在线缆则转为第一种情况，否则到达限位后朝另一个
//方向再次移动）
//
//2、记录线缆中间位置数据起始记录与结束记录
//
//3、假设图像线缆运动方程为线性方程(相机的畸变会导致非线性，实验发现非线性比较轻微)
//建立线性方程距离中心位置差的最优解
//**********************************************************************************

struct FittingCoeff
{
    double a[maxCameraNum];
    double c[maxCameraNum];
    int nCamera;
};

struct PosStatus
{
    float lastPos[maxCameraNum];
    float loss;
    float diffLoss;

	float nNoCable[noCableLimit];    //判断视野内连续无目标（黑图） leon
	int   nCurBlack;
};

struct DirectArray
{
    int direct[nDirect];
    int nCur;

	int nChangeDir;
	int nPlcFine;
};

class CentringCable
{
public:
    CentringCable();
    ~CentringCable();

    int init();
	int initNoCableStatus();
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
    int optimalMove(Mat *img, MoveInfo &moveInfo, double &moveValue, bool isMoving = true);

    /**********************************************/
    // estimateMove
    // 获取运动方向与跨度
    // Input:
    //      img             //线缆图像组
    //      nFrame          //采集帧数
    //      nCamera         //相机个数
    //      ms              //采图基准时间
    // Output:
    //      moveTime       //提供电机运动时间
    /**********************************************/
    int estimateMove(Mat *img, int nFrame, int nCamera, int ms, int &moveTime, bool &isOk);
    
    /**********************************************/
    // realTime
    // 实时判断
    // Input:
    //      img             //线缆图像组
    //      nCamera         //相机个数
    //      condDist        //平均中心偏差阈值
    // Output:
    //      isOK            //返回当前帧是否满足对中条件
	/*      plcTime           返回plc运动时间
	        direct            plc的运动方向
	*/
    /**********************************************/
    int realTime(Mat *img, int nFrame, int nCamera, int condDist, int finePlcTime_1, int finePlcTime_2, int &plcTime, int &direct, bool &isOK);


    /**********************************************/
    // loadMoveData
    // 载入当前运动数据
    // Input:
    //      filePath    //运动参数文档
    // Output:
    //      moveStatus  //运动数据
    /**********************************************/
    int loadMoveData(string filePath, MoveStatus &moveStatus);

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
    int cableStatus(Mat *img, int nCamera, int nFrame, MoveInfo &outMoveInfo, MoveStatus &outMoveStatus);

	//结果位置输出
	int cableFinalPose(Mat *img, int nCamera, int nFrame, MoveInfo &outMovePose);

private:
    /**********************************************/
    // getLinePos
    // 提取线缆中心位置
    // Input:
    //      img         //载入图片组
    //      nCamera     //相机个数
    // Output:
    //      xMean       //线缆图像中心
    /**********************************************/
    int getLinePos(Mat *img, int *xMean, int nCamera);

    /**********************************************/
    // ransac
    // ransac直线拟合
    // Input:
    //      points      //点集
    // Output:
    //      coeffA      //时间系数(x系数）
    //      coeffC      //常量系数
    /**********************************************/
    void ransac(const vector<cv::Point> &points, double &coeffA, double &coeffC);

    /**********************************************/
    // medianFilter
    // 中值滤波
    // Input:
    //      xMean       //线缆中心位置数据组
    //      len         //组长
    //      kSize       //核宽
    // Output:
    //      NULL
    /**********************************************/
    void medianFilter(int *xMean, int len, int kSize);

    /**********************************************/
    // fittingMove
    // 运动拟合
    // Input:
    //      moveInfo        //线缆运动数据
    // Output:
    //      coeff           //运动拟合系数
    /**********************************************/
    int fittingMove(MoveInfo &moveInfo, FittingCoeff &coeff);

private:
    PosStatus posStatus;
    float pos[256];
    DirectArray directs;
    int nCurFrame;
};