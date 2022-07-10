#include <opencv2/opencv.hpp>
#include <vector>
#include "centringCableParam.h"

using namespace std;
using namespace cv;


//*************************************extend***************************************
//1����ǰ״̬�������Զ����ƶ�һ�ξ��루����������ʱ���̶�һ�������ƶ�������������
//��ʱ����һ�������ƶ�����λ�����м����������תΪ��һ����������򵽴���λ����һ��
//�����ٴ��ƶ���
//
//2����¼�����м�λ��������ʼ��¼�������¼
//
//3������ͼ�������˶�����Ϊ���Է���(����Ļ���ᵼ�·����ԣ�ʵ�鷢�ַ����ԱȽ���΢)
//�������Է��̾�������λ�ò�����Ž�
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

	float nNoCable[noCableLimit];    //�ж���Ұ��������Ŀ�꣨��ͼ�� leon
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
    // ��ȡ�����˶�����
    // Input:
    //      img             //����ͼ����
    //      moveInfo        //�˶�����(�ڲ������Ż���
    //      isMoving        //�Ƿ������˶���true:���ڲɼ�״̬��false:����Ԥ��״̬��
    // Output:
    //      moveValue       //�ṩ����˶�����
    /**********************************************/
    int optimalMove(Mat *img, MoveInfo &moveInfo, double &moveValue, bool isMoving = true);

    /**********************************************/
    // estimateMove
    // ��ȡ�˶���������
    // Input:
    //      img             //����ͼ����
    //      nFrame          //�ɼ�֡��
    //      nCamera         //�������
    //      ms              //��ͼ��׼ʱ��
    // Output:
    //      moveTime       //�ṩ����˶�ʱ��
    /**********************************************/
    int estimateMove(Mat *img, int nFrame, int nCamera, int ms, int &moveTime, bool &isOk);
    
    /**********************************************/
    // realTime
    // ʵʱ�ж�
    // Input:
    //      img             //����ͼ����
    //      nCamera         //�������
    //      condDist        //ƽ������ƫ����ֵ
    // Output:
    //      isOK            //���ص�ǰ֡�Ƿ������������
	/*      plcTime           ����plc�˶�ʱ��
	        direct            plc���˶�����
	*/
    /**********************************************/
    int realTime(Mat *img, int nFrame, int nCamera, int condDist, int finePlcTime_1, int finePlcTime_2, int &plcTime, int &direct, bool &isOK);


    /**********************************************/
    // loadMoveData
    // ���뵱ǰ�˶�����
    // Input:
    //      filePath    //�˶������ĵ�
    // Output:
    //      moveStatus  //�˶�����
    /**********************************************/
    int loadMoveData(string filePath, MoveStatus &moveStatus);

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
    int cableStatus(Mat *img, int nCamera, int nFrame, MoveInfo &outMoveInfo, MoveStatus &outMoveStatus);

	//���λ�����
	int cableFinalPose(Mat *img, int nCamera, int nFrame, MoveInfo &outMovePose);

private:
    /**********************************************/
    // getLinePos
    // ��ȡ��������λ��
    // Input:
    //      img         //����ͼƬ��
    //      nCamera     //�������
    // Output:
    //      xMean       //����ͼ������
    /**********************************************/
    int getLinePos(Mat *img, int *xMean, int nCamera);

    /**********************************************/
    // ransac
    // ransacֱ�����
    // Input:
    //      points      //�㼯
    // Output:
    //      coeffA      //ʱ��ϵ��(xϵ����
    //      coeffC      //����ϵ��
    /**********************************************/
    void ransac(const vector<cv::Point> &points, double &coeffA, double &coeffC);

    /**********************************************/
    // medianFilter
    // ��ֵ�˲�
    // Input:
    //      xMean       //��������λ��������
    //      len         //�鳤
    //      kSize       //�˿�
    // Output:
    //      NULL
    /**********************************************/
    void medianFilter(int *xMean, int len, int kSize);

    /**********************************************/
    // fittingMove
    // �˶����
    // Input:
    //      moveInfo        //�����˶�����
    // Output:
    //      coeff           //�˶����ϵ��
    /**********************************************/
    int fittingMove(MoveInfo &moveInfo, FittingCoeff &coeff);

private:
    PosStatus posStatus;
    float pos[256];
    DirectArray directs;
    int nCurFrame;
};