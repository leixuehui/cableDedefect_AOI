#pragma once
enum
{
    CABLE_MOVE_ERROR_NONE = 5000,   //线缆对中错误码段
    CABLE_MOVE_ERROR_IMAGE_EMPTY,   //传入图像为空
    CABLE_MOVE_ERROR_DATA_LOAD,     //加载电机运动数据有误
};

enum DirStatus
{
    CABLE_DIRECT_STATUS_NONE = 0,   //相机无法拍到图像
    CABLE_DIRECT_STATUS_UP,         //判断向上移动
    CABLE_DIRECT_STATUS_DOWN,       //判断向下移动
    CABLE_DIRECT_STATUS_OK          //判断位置适合
};

#define maxCameraNum 8      //最大相机个数
#define maxFrameNum 100     //最大接受帧数
#define rowFrame 2000       //一幅图像行频

struct MoveInfo                             //一段时间内的线缆运动情况(假设采集帧无延迟）
{
    int xMean[maxCameraNum][maxFrameNum];   //运动时域
    int nCamera;                            //相机个数
    int nFrame = 0;                         //当前采集个数
};

struct MoveStatus
{
    int direct;     //运动方向(0:无方向，1:向上，2:向下, 3:位置居中)
    bool getLimit;  //是否采取限位运动
    double time;    //运动时间
    double shift;   //运动位移
    double velocity;//运动速度
};