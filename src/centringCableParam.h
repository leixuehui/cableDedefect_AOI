#pragma once
enum
{
    CABLE_MOVE_ERROR_NONE = 5000,   //���¶��д������
    CABLE_MOVE_ERROR_IMAGE_EMPTY,   //����ͼ��Ϊ��
    CABLE_MOVE_ERROR_DATA_LOAD,     //���ص���˶���������
};

enum DirStatus
{
    CABLE_DIRECT_STATUS_NONE = 0,   //����޷��ĵ�ͼ��
    CABLE_DIRECT_STATUS_UP,         //�ж������ƶ�
    CABLE_DIRECT_STATUS_DOWN,       //�ж������ƶ�
    CABLE_DIRECT_STATUS_OK          //�ж�λ���ʺ�
};

#define maxCameraNum 8      //����������
#define maxFrameNum 100     //������֡��
#define rowFrame 1000       //һ��ͼ����Ƶ

struct MoveInfo                             //һ��ʱ���ڵ������˶����(����ɼ�֡���ӳ٣�
{
    int xMean[maxCameraNum][maxFrameNum];   //�˶�ʱ��
    int nCamera;                            //�������
    int nFrame = 0;                         //��ǰ�ɼ�����
};

struct MoveStatus
{
    int direct;     //�˶�����(0:�޷���1:���ϣ�2:����, 3:λ�þ���)
    bool getLimit;  //�Ƿ��ȡ��λ�˶�
    double time;    //�˶�ʱ��
    double shift;   //�˶�λ��
    double velocity;//�˶��ٶ�
};

const int nDirect = 5;
const int noCableLimit = 6;
//const int finePlcTime_1 = 240;
//const int finePlcTime_2 = 150;