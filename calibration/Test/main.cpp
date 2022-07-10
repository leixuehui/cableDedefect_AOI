#include <opencv2/opencv.hpp>
#include <iostream>

#include "../../src/Api_CentringCable.hpp"
#include "../../src/centringCableParam.h"

using namespace std;
using namespace cv;

#define  picture_new 1
#define nCamera 4

//test
int main()
{
    APICentringCable api;
    api.api_init();
    
    Mat imgSrc[nCamera];
	int i = 10;

	string leftUp, leftDown, rightUp, rightDown;
#if picture_new
	if (nCamera == 3)
	{
		leftUp   = "D:/Image/test/centreCable/8月/camera3_1/left_up/1 (";
		leftDown = "D:/Image/test/centreCable/8月/camera3_1/left_down/1 (";
		rightUp  = "D:/Image/test/centreCable/8月/camera3_1/right_up/1 (";
	}
	else if (nCamera == 4)
	{
		leftUp = "./0906/left_up/1 (";
		leftDown  = "./0906/left_down/1 (";
		rightDown = "./0906/right_down/1 (";
		rightUp   = "./0906/right_up/1 (";
	}

#endif

    //ofstream out("line.txt");
    MoveInfo moveInfo;
    MoveStatus moveStatus;

    api.api_loadMoveData("moveData.txt", moveStatus);
    double moveValue;
    
    int n = 0;
    int step = 0;

    Mat calibImg[9];

#if 0    初始版本
    for (int n = i; n < i + 3; n++)
    {
        calibImg[(n - i) * 3] = imread(leftUp + to_string(n) + ").jpg", 0);
        calibImg[(n - i) * 3 + 1] = imread(leftDown + to_string(n) + ").jpg", 0);
        calibImg[(n - i) * 3 + 2] = imread(rightUp + to_string(n) + ").jpg", 0);
    }

    api.api_cableStatus(calibImg, 3, 3, moveInfo, moveStatus);

    if (moveStatus.direct == CABLE_DIRECT_STATUS_UP)         step = -1;
    else if (moveStatus.direct == CABLE_DIRECT_STATUS_DOWN)  step = 1;
    else if (moveStatus.direct == CABLE_DIRECT_STATUS_OK)
    {
        cout << "optimal pos:" << i << endl;
    }

    while (1 & moveStatus.direct != CABLE_DIRECT_STATUS_OK)
    {
        imgSrc[0] = imread(leftUp + to_string(i) + ").jpg", 0);
        imgSrc[1] = imread(leftDown + to_string(i) + ").jpg", 0);
        imgSrc[2] = imread(rightUp + to_string(i) + ").jpg", 0);

        if (n < 30)
        {
            api.api_optimalMove(imgSrc, moveInfo, moveValue);
        }
        else
        {
            n = 0;
            api.api_optimalMove(imgSrc, moveInfo, moveValue, false);

            if (moveValue < 0)
            {
                cout << "optimal pos:" << i + moveValue * step << endl;
                break;
            }
        }
        n++;
        i += step;
        //out<<xMean[0]<<","<<xMean[1]<<","<<xMean[2]<<std::endl;
    }
    //out.close();

#else   实时判断版本
    bool isOK = false;
	int nFrame = 3;
    int direct = 1;
	int plcTime = 0;

    while (1)
    {
#if picture_new
			imgSrc[0] = imread(leftUp + to_string(i) + ").jpg", 0);
			imgSrc[1] = imread(leftDown + to_string(i) + ").jpg", 0);
			imgSrc[2] = imread(rightUp + to_string(i) + ").jpg", 0);
		if (nCamera == 4)
		{
			imgSrc[3] = imread(rightDown + to_string(i) + ").jpg", 0);
		}
#endif

        cout<<"pos:"<<i<<endl;

		int correctDist = 30;         //平均偏移量满足条件的阈值
		int finePlcTime_1 = 240;      //进入换向范围的plc细调步长
		int finePlcTime_2 = 150;      //进入20%范围的plc精细调步长
        api.api_realTime(imgSrc, nFrame, nCamera, correctDist, finePlcTime_1, finePlcTime_2, plcTime, step, isOK);
        if (isOK)    break;
        if (step < 0)   direct = -direct;

        //int moveTime;
        //api.api_estimateMove(calibImg, 3, 3, 5, moveTime, isOK);

        i = i + direct;
        
        if (isOK)
        {
			printf("\n OK \n");
			break;
        }

    }

    cout << "optimal pos:" << i << endl;
#endif
    
	//api.api_destory();
    return 0;
}
