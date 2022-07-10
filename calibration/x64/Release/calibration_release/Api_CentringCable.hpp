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
private:
    void *gProcess = nullptr;
};