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
    // ���뵱ǰ�˶�����
    // Input:
    //      filePath    //�˶������ĵ�
    // Output:
    //      moveStatus  //�˶�����
    /**********************************************/
    int api_loadMoveData(string filePath, MoveStatus &moveStatus);

    /**********************************************/
    // cableStatus
    // �жϵ�ǰ����λ�����˶�����
    // Input:
    //      img             ���к��ͼ����(leftUp,leftDown,rightUp; leftUp, leftDown, rightUp, rightDown)
    //		nCamera         �������
    //      nFrame          �ɼ�֡��(ͨ������1����ֹ��һλ�ó�������ֵ��
    // Output:
    //      outMoveInfo     �˶�λ����Ϣ
    // 		outMoveStatus   �˶�״̬
    /**********************************************/
    int api_cableStatus(Mat *img, int nCamera, int nFrame, MoveInfo &outMoveInfo, MoveStatus &outMoveStatus);


	//���λ�����
	int api_cableFinalPose(Mat *img, int nCamera, int nFrame, MoveInfo &outMovePose);

    /**********************************************/
    // optimalMove
    // ��ȡ�����˶�����
    // Input:
    //      img             //����ͼ����
    //      moveInfo        //�˶�����(�ڲ������Ż���
    //      isMoving        //�Ƿ������˶���true:���ڲɼ�״̬��false:����Ԥ��״̬��
    // Output:
    //      moveValue       //�ṩ����˶�����
    /**********************************************/
    int api_optimalMove(Mat *img, MoveInfo &moveInfo, double &moveValue, bool isMoving = true);

    /**********************************************/
    // optimalMove
    // ��ȡ�����˶�����
    // Input:
    //      img             //����ͼ����
    //      moveInfo        //�˶�����(�ڲ������Ż���
    //      isMoving        //�Ƿ������˶���true:���ڲɼ�״̬��false:����Ԥ��״̬��
    // Output:
    //      moveValue       //�ṩ����˶�����
    /**********************************************/
    int api_estimateMove(Mat *img, int nFrame, int nCamera, int ms, int &moveTime, bool &isOk);

    /**********************************************
    // api_realTime
    // ��ȡ�����˶�����
    // Input:
    //      img                 ����ͼ����
    //      nFrame              ÿ��������뼸֡
	//      nCamera             �����������
	//      correctDist         ƽ��ƫ��������ֵ
	//      finePlcTime_1       ���뻻��Χ��plcϸ������
	//      finePlcTime_2       ����20%��Χ��plc��ϸ������
    // Output:
    //      plcTime             ����plc�˶�ʱ��
	//      direct              ����plc�˶�����
	//      isOK                ���غ������
    /**********************************************/
    int api_realTime(Mat *img, int nFrame, int nCamera, int condDist, int finePlcTime_1, int finePlcTime_2, 
		              int &plcTime, int &direct, bool &isOK);

private:
    void *gProcess = nullptr;
};