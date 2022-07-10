#include <opencv2/opencv.hpp>
#include <iostream>

#include "../../src/Api_CentringCable.hpp"
#include "../../src/centringCableParam.h"

#define picture_old 0
#define  picture_new 1
#define nCamera 3


using namespace std;
using namespace cv;
//test
int main()
{
    APICentringCable api;
    
	Mat calibImg[9];       //4相机为12  3相机为9
    Mat imgSrc[3];         //4相机为4   3相机为3

	//int i = 485;  //1000行数
	int i = 1;    //取第几张图做起始

#if picture_old
    string leftDown = "D:/Image/test/centreCable/camera3_1/left_down/1 (";
    string leftUp = "D:/Image/test/centreCable/camera3_1/left_up/1 (";
    string rightUp = "D:/Image/test/centreCable/camera3_1/right_up/1 (";
	int i = 1068;
#endif

	string leftUp, leftDown, rightUp,rightDown;
#if picture_new
	if (nCamera == 3)
	{
		leftUp = "D:/Image/test/centreCable/Data(0824)/left_up/1_";
		leftDown = "D:/Image/test/centreCable/Data(0824)/left_down/1_";
		rightUp = "D:/Image/test/centreCable/Data(0824)/right_up/1_";
	}
	else if (nCamera == 4)
	{
		leftUp = "D:/Image/test/centreCable/org_image-1000行数/left_up/1_";
		leftDown = "D:/Image/test/centreCable/org_image-1000行数/left_down/1_";
		rightUp = "D:/Image/test/centreCable/org_image-1000行数/right_up/1_";
		rightDown = "D:/Image/test/centreCable/org_image-1000行数/right_down/1_";
	}


#endif

    //ofstream out("line.txt");
    MoveInfo moveInfo;
    MoveStatus moveStatus;

    api.api_loadMoveData("moveData.txt", moveStatus);
    int direct;
    double moveValue;
    

	//int i = 1;
    int n = 0;
    int step = 0;
	int res = 0;

    for (int n = i; n < i + 3; n++)
    {
#if picture_old
		calibImg[(n - i) * 3] = imread(leftUp + to_string(n) + ").jpg", 0);
        calibImg[(n - i) * 3 + 1] = imread(leftDown + to_string(n) + ").jpg", 0);
        calibImg[(n - i) * 3 + 2] = imread(rightUp + to_string(n) + ").jpg", 0);
#endif

#if picture_new
		int j = i;
		if (nCamera == 3)
		{
			calibImg[(n - i) * 3] = imread(leftUp + to_string(j) + ".jpg", 0);
			calibImg[(n - i) * 3 + 1] = imread(leftDown + to_string(j) + ".jpg", 0);
			calibImg[(n - i) * 3 + 2] = imread(rightUp + to_string(j) + ".jpg", 0);
		}
		else if (nCamera == 4)
		{
			calibImg[(n - i) * 4] = imread(leftUp + to_string(j) + ".jpg", 0);
			calibImg[(n - i) * 4 + 1] = imread(leftDown + to_string(j) + ".jpg", 0);
			calibImg[(n - i) * 4 + 2] = imread(rightUp + to_string(j) + ".jpg", 0);
			calibImg[(n - i) * 4 + 3] = imread(rightDown + to_string(j) + ".jpg", 0);
		}

#endif
    }

	double calcuTime = cv::getTickCount();
	//三帧 or 四帧为相同高度的一组图

#if 0 输出实时位置
	res = api.api_cableFinalPose(calibImg, nCamera, 1, moveInfo);
	for (int k=0;k<nCamera;k++)
	{
		printf("camera[%d]-pose:%d \n", k, moveInfo.xMean[k][0]);
	}
#endif

	//************
	res = api.api_cableStatus(calibImg, nCamera, 3, moveInfo, moveStatus);


	if (res != 0)
	{
		printf("some img is null");
		return res;
	}

	calcuTime = ((double)getTickCount() - calcuTime) / getTickFrequency()*1000;
	printf("api_cableStatus tickTime=%0.2f ms \n", calcuTime);

	//TODO  软件直接接受控制plc向上向下
    if (moveStatus.direct == CABLE_DIRECT_STATUS_UP)         step = -1;
    else if (moveStatus.direct == CABLE_DIRECT_STATUS_DOWN)  step = 1;
    else if (moveStatus.direct == CABLE_DIRECT_STATUS_OK)
    {
        cout << "【original】optimal pos:" << i << endl;
    }

	//if (step == 0)  
		step = 1;

    while (1 & moveStatus.direct != CABLE_DIRECT_STATUS_OK)          //  !=  结果为0 和 1；再与1做位运算，这样和与运算结果一致
    {

#if picture_old
		imgSrc[0] = imread(leftUp + to_string(i) + ").jpg", 0);
        imgSrc[1] = imread(leftDown + to_string(i) + ").jpg", 0);
        imgSrc[2] = imread(rightUp + to_string(i) + ").jpg", 0);
#endif

#if picture_new
		if (nCamera == 3)
		{
			imgSrc[0] = imread(leftUp + to_string(i) + ".jpg", 0);
			imgSrc[1] = imread(leftDown + to_string(i) + ".jpg", 0);
			imgSrc[2] = imread(rightUp + to_string(i) + ".jpg", 0);
		}
		else if ( nCamera == 4)
		{
			imgSrc[0] = imread(leftUp + to_string(i) + ".jpg", 0);
			imgSrc[1] = imread(leftDown + to_string(i) + ".jpg", 0);
			imgSrc[2] = imread(rightUp + to_string(i) + ".jpg", 0);
			imgSrc[3] = imread(rightDown + to_string(i) + ".jpg", 0);
		}
#endif

        if (n < 25)     //30帧为一个周期
        {
			double calcuTime_optimal= cv::getTickCount();
            res = api.api_optimalMove(imgSrc, moveInfo, moveValue);

			calcuTime_optimal = ((double)getTickCount() - calcuTime_optimal) / getTickFrequency() * 1000;   //ms级
			printf("【%d】optimalMove tickTime=%0.2f ms \n",n+1, calcuTime_optimal);

			if (res != 0)
			{
				printf("api_optimalMove is error");
				return res;
			}

			if (moveValue == -100)
			{
				cout << endl << "【-100】optimal pos:" << i  << endl;
				break;
			}
			
        }
        else
        {
            n = 0;
            api.api_optimalMove(imgSrc, moveInfo, moveValue, false);

            if (moveValue < 0)                                                //注意点
            {
				cout << endl <<"optimal pos:" << i + moveValue * step << endl;
				break;
            }
        }
        n++;
        i += step;
        //out<<xMean[0]<<","<<xMean[1]<<","<<xMean[2]<<std::endl;
    }
    //out.close();

    //用时域来做
    return 0;
}
