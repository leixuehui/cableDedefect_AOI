#include "centringCable.h"

float percentB = 0.9;
int currentDist = 30;

static int sortIntList(
	const void *d1,
	const void *d2)
{
	int t1 = *((int*)d1);
	int t2 = *((int*)d2);

	if (t1 > t2)      return 1;
	else if (t1 < t2) return -1;
	else             return 0;
}

int otsuThreshold(
	const unsigned char *image,
	const int rows,
	const int cols,
	const int x0,
	const int y0,
	const int dx,
	const int dy,
	const int vvv,
	const int minThr = 0,
	const int maxThr = 60)
{
	const unsigned char *np;
	int thresholdValue = 1;
	int ihist[256];
	int i, j, k; // various counters
	int n, n1, n2, gmin, gmax;
	double m1, m2, sum, csum, fmax, sb;

	memset(ihist, 0, sizeof(ihist));
	gmin = 255; gmax = 0;

	for (i = y0 + 1; i < y0 + dy - 1; i++)
	{
		np = &image[i*cols + x0 + 1];
		for (j = x0 + 1; j < x0 + dx - 1; j++)
		{
			ihist[*np]++;
			if (*np > gmax) gmax = *np;
			if (*np < gmin) gmin = *np;
			np++; /* next pixel */
		}
	}

#if 1 // truncate
	for (int v = 0; v < minThr; v++)
	{
		ihist[v] = 0;
	}

	for (int v = maxThr + 1; v < 256; v++)
	{
		ihist[v] = 0;
	}
#endif

	// set up everything
	sum = csum = 0.0;
	n = 0;

	for (k = 0; k <= 255; k++)
	{
		sum += (double)k * (double)ihist[k]; /* x*f(x)*/
		n += ihist[k]; /* f(x) */
	}

	if (!n)
	{
		// if n has no value, there is problems...
		// fprintf(stderr, "NOT NORMAL thresholdValue = 160\n");
		return (0);
	}

	// do the otsu global thresholding method
	fmax = -1.0;
	n1 = 0;

	for (k = 0; k < 255; k++)
	{
		n1 += ihist[k];
		if (!n1) { continue; }
		n2 = n - n1;
		if (n2 == 0) { break; }
		csum += (double)k *ihist[k];
		m1 = csum / n1;
		m2 = (sum - csum) / n2;
		sb = (double)n1 *(double)n2 *(m1 - m2) * (m1 - m2);
		/* bbg: note: can be optimized. */

		if (sb >= fmax)
		{
			fmax = sb;
			thresholdValue = k;
		}
	}

#if 1
	if (vvv & 1)
	{
		fprintf(stderr,
			"# OTSU: thresholdValue = %d gmin=%d gmax=%d\n",
			thresholdValue, gmin, gmax);

		return(thresholdValue);
	}
#endif

	return thresholdValue;
}

static int otsuAdaptThreshold(
	const unsigned char *image,
	const int rows,
	const int cols,
	const int thrSegNumIn)
{
	int thrSegNum = thrSegNumIn;
	const int maxSegNum = 1024;
	int thr[maxSegNum];

	if (thrSegNum > maxSegNum)
		thrSegNum = maxSegNum;

	if (thrSegNum > rows)
		thrSegNum = rows;

	if (thrSegNum < 1)
		thrSegNum = 1;

	//	printf("thr: ");
	for (int i = 0; i < thrSegNum; i++)
	{
		int x0 = 0;
		int y0 = i*(rows / thrSegNum);;
		int w = cols;
		int h = rows / thrSegNum;
		thr[i] = otsuThreshold(image, rows, cols, x0, y0, w, h, 0);
		//		printf("%d  ", thr[i]);
	}
	//	printf("\n");

	qsort(thr, thrSegNum, sizeof(int), sortIntList);

	return thr[thrSegNum / 2];
}

static void calcHistSingleChannel(
	const unsigned char *pImg,
	const int width,
	const int height,
	float * pHist,
	const int binNum)
{
	memset(pHist, 0, sizeof(float)*binNum);

	const unsigned char *pData = pImg;
	int pixelNum = width * height;

	for (int i = 0; i < pixelNum; i++)
	{
		pHist[*(pData++)]++;
	}
}


static cv::Rect getContourRect(
	const vector<Point> &pts)
{
	Rect rst = Rect(0, 0, 1, 1);

	if (pts.size() <= 2)
		return rst;

	int minx = 1000000, maxx = 0, miny = 1000000, maxy = 0;

	for (size_t i = 0; i < pts.size(); i++)
	{
		if (pts[i].x < minx)  minx = pts[i].x;
		if (pts[i].x > maxx)  maxx = pts[i].x;
		if (pts[i].y < miny)  miny = pts[i].y;
		if (pts[i].y > maxy)  maxy = pts[i].y;
	}

	rst.x = minx;
	rst.y = miny;
	rst.width = maxx - minx + 1;
	rst.height = maxy - miny + 1;

	return rst;
}

static int findCableBoudary2(
	const Mat &imgB,
	int *xLeftSeed,
	int *xRightSeed,
	int *flags)
{
	vector<vector<Point> > contours;
	vector<vector<Point> >::iterator  iter;

	findContours(imgB, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	cv::Mat imgB2 = Mat::zeros(imgB.rows, imgB.cols, CV_8UC1);

	for (iter = contours.begin(); iter != contours.end(); )
	{
		Rect tempRect = getContourRect(*iter);

		if ((int)iter->size() < (imgB.rows / 4) ||
			tempRect.height < (imgB.rows / 4))
		{
			iter = contours.erase(iter);
			continue;
		}
		else
		{
			for (size_t i = 0; i < iter->size(); i++)
			{
				imgB2.at<uchar>((*iter)[i].y, (*iter)[i].x) = 255;
			}

			iter++;
		}
	}
	unsigned char* pImgB2 = (unsigned char*)imgB2.data + imgB2.step1();
	for (int i = 1; i < imgB2.rows - 1; i++)
	{
		int findLeftBoundary = 0;
		int findRightBoundary = 0;

		for (int j = 0; j < imgB2.cols; j++)
		{
			if (findLeftBoundary == 0 &&
				pImgB2[j] == 255)
			{
				findLeftBoundary = 1;
				xLeftSeed[i] = j;
			}

			if (findRightBoundary == 0 &&
				pImgB2[imgB2.cols - 1 - j] == 255)
			{
				findRightBoundary = 1;
				xRightSeed[i] = imgB2.cols - 1 - j;
			}

			if (findLeftBoundary == 1 &&
				findRightBoundary == 1)
			{
				break;
			}
		}

		if (findLeftBoundary != 1)
		{
			xLeftSeed[i] = 0;
		}

		if (findRightBoundary != 1)
		{
			xRightSeed[i] = imgB2.cols - 1;
		}

		if (xLeftSeed[i] > xRightSeed[i])
		{
			xLeftSeed[i] = 0;
			xRightSeed[i] = imgB2.cols - 1;
		}
		assert(xRightSeed[i] < imgB2.cols && xLeftSeed[i] >= 0);
		pImgB2 += imgB2.step1();

	}

	xLeftSeed[0] = xLeftSeed[1];
	xRightSeed[0] = xRightSeed[1];
	xLeftSeed[imgB2.rows - 1] = xLeftSeed[imgB2.rows - 2];
	xRightSeed[imgB2.rows - 1] = xRightSeed[imgB2.rows - 2];

	return 0;
}

static int scaleBoudary(
	const int *xLeftSeed,
	const int *xRightSeed,
	const int length,
	const int scale, // scale should be int
	int *xLeft,
	int *xRight)
{
	for (int i = 0; i < length - 1; i++)
	{
		for (int n = 0; n < scale; n++)
		{
			xLeft[i*scale + n] = xLeftSeed[i] * (scale - n) + n*xLeftSeed[i + 1];
			xRight[i*scale + n] = xRightSeed[i] * (scale - n) + n*xRightSeed[i + 1];
		}
	}

	for (int n = 0; n < scale; n++)
	{
		xLeft[(length - 1)*scale + n] = xLeftSeed[length - 1] * scale;
		xRight[(length - 1)*scale + n] = xRightSeed[length - 1] * scale;
	}

	return 0;
}

static void refineCableBoudary2(
	const Mat &imgB,
	int *xLeftSeed,
	int *xRightSeed)
{
	const int bWidth = 16;
	const int bLength = 2 * bWidth;

	unsigned char *pImg = (unsigned char*)imgB.data;
	int imgW = imgB.cols;
	int imgH = imgB.rows;
	int imgStep = imgB.step1();
	Mat imgL = cv::Mat(imgH, bLength, CV_8UC1);
	Mat imgR = cv::Mat(imgH, bLength, CV_8UC1);
	int imgLStep = imgL.step1();
	int imgRStep = imgR.step1();
	unsigned char *pImgL = (unsigned char *)imgL.data;
	unsigned char *pImgR = (unsigned char *)imgR.data;

	for (int i = 0; i < imgH; i++)
	{
		if (xLeftSeed[i] - bWidth < 0)
		{
			xLeftSeed[i] = bWidth;
		}
		if (xRightSeed[i] + bWidth >= imgW)
		{
			xRightSeed[i] = imgW - 1 - bWidth;
		}
		memcpy(pImgL, &pImg[xLeftSeed[i] - bWidth], bLength * sizeof(unsigned char));
		memcpy(pImgR, &pImg[xRightSeed[i] - bWidth], bLength * sizeof(unsigned char));
		pImg += imgStep;
		pImgL += imgLStep;
		pImgR += imgRStep;
	}

	int thrL = otsuAdaptThreshold((unsigned char*)imgL.data, imgL.rows, imgL.cols, 15);
	int thrR = otsuAdaptThreshold((unsigned char*)imgR.data, imgR.rows, imgR.cols, 15);

	Mat imgBinL;
	Mat imgBinR;

	//	imshow("imgL", imgL);
	//	imshow("imgR", imgR);

	Mat imgTmpL;
	Mat imgTmpR;

	cv::threshold(imgL, imgBinL, thrL, 255, THRESH_BINARY);
	cv::threshold(imgR, imgBinR, thrR, 255, THRESH_BINARY);

#if 0
	Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	morphologyEx(imgTmpL, imgBinL, MORPH_OPEN, element1);

	Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	morphologyEx(imgTmpR, imgBinR, MORPH_CLOSE, element2);
#endif
	//	imshow("imgBinL", imgBinL);
	//	imshow("imgBinR", imgBinR);

	int imgWB = imgL.cols;
	int imgHB = imgL.rows;

	unsigned char* pImgLB = (unsigned char*)imgBinL.data;
	unsigned char* pImgRB = (unsigned char*)imgBinR.data;

	for (int i = 0; i < imgHB; i++)
	{
		int findLeftBoundary = 0;
		int findRightBoundary = 0;

		// left boundary
		for (int j = 0; j < imgWB; j++)
		{
			if (findLeftBoundary == 1)
				break;

			if (pImgLB[j] == 255)
			{
				findLeftBoundary = 1;
				xLeftSeed[i] += j - bWidth;
				xLeftSeed[i] = max(0, min(imgW - 1, xLeftSeed[i]));
			}
		}

		// right boundary
		for (int j = imgWB - 1; j >= 0; j--)
		{
			if (findRightBoundary == 1)
				break;

			if (pImgRB[j] == 255)
			{
				findRightBoundary = 1;
				xRightSeed[i] += j - bWidth;
				xRightSeed[i] = max(0, min(imgW - 1, xRightSeed[i]));
				//assert(xRightSeed[i] < 800);
			}
		}

		pImgLB += imgBinL.step1();
		pImgRB += imgBinR.step1();
	}

}

static int segmentCable(
	const Mat &imgIn,
	const Mat &imgIn2,
	int* xLeft,
	int* xRight)
{
	Mat imgB;
	int darkThr = 40;
	float percent = 0.1;

#if 0
	//memset(hist, 0.0f, 256 * sizeof(float));
	float hist[256] = { 0.0f };
	calcHistSingleChannel(imgIn.data, imgIn.step1(), imgIn.rows, hist, 256);

	int maxGray = 0;
	int maxCount = 0;
	maxGray = max_element(hist, hist + 30) - hist;
	maxCount = hist[maxGray];

	int maxGray1 = 0, maxGray2 = 0;
	float countRange = percent*maxCount;
	for (int i = maxGray; i < 100; i++)
	{
		if (maxGray2&&maxGray1)
		{
			break;
		}
		if (!maxGray1&&hist[i] < countRange&&hist[i + 1] < countRange)
		{
			maxGray1 = i - 1;
		}
		if (!maxGray2&&hist[i] > countRange&&maxGray1&&hist[i + 1] > countRange)
		{
			maxGray2 = i;
		}
	}
	if (maxGray2 == 0 && hist[darkThr] != 0)
	{
		maxGray2 = darkThr;
	}

	int minCount = maxCount;
	int thr_used = 0;
	for (int i = maxGray1; i < maxGray2; i++)
	{
		if (minCount > hist[i])
		{
			minCount = hist[i];
			thr_used = i;
		}
	}

	float score = 0.0;

	for (int i = 0; i < thr_used; i++)
	{
		score += hist[i] * 1.0;
	}
	score /= imgIn.rows*imgIn.cols*1.0;

	if (thr_used < 5 || score > percentB)
	{
		printf("黑图-计算出的灰度值：%d, 最大数值灰度值：%d. \n", thr_used, maxGray);
		return -1;
	}
	//printf("灰度正常！\n");
#endif


#if 1   新替换的阈值分割
	const int thrSegNum = 1;
	int thr[thrSegNum];
	int thrMax[thrSegNum];

	for (int i = 0; i < thrSegNum; i++)
	{
		int x0 = i*(imgIn.cols / thrSegNum);    //竖条分割
		int y0 = 0;
		int w = imgIn.cols / thrSegNum;
		int h = imgIn.rows;

		thr[i] = otsuThreshold(imgIn.data, imgIn.rows, imgIn.cols, x0, y0, w, h, 0, 3, 255);
	}

	qsort(thr, thrSegNum, sizeof(int), sortIntList);     //升序
	int thr_used = thr[0];
#endif	
	cv::threshold(imgIn, imgB, thr_used, 255, THRESH_BINARY);

	int foreGray = 0, foreCount = 0;
	for (int a=0; a<imgB.rows; a++)
	{
		const uchar *imgInGray = imgIn.ptr<uchar>(a);
		const uchar *imgBinGray = imgB.ptr<uchar>(a);

		for (int b=0; b<imgB.cols; b++)
		{
			if (imgBinGray[b] == 255)
			{
				foreGray += imgInGray[b];
				foreCount++;
			}
		}
	}
	foreGray /= max(1,foreCount);
	if (foreGray < 10)
	{		
		printf("黑图-分割阈值：%d,平均灰度：%d \n", thr_used, foreGray);
		return -1;
	}

	int xLeftSeed[4096];
	int xRightSeed[4096];
	int flags[4096];
	int imgH = imgB.rows;
	int imgW = imgB.cols;

	int temp_rst = findCableBoudary2(imgB, xLeftSeed, xRightSeed, flags);

	int scale = imgIn2.cols / imgIn.cols;

	scaleBoudary(xLeftSeed, xRightSeed, imgH, (imgIn2.rows / imgIn.rows), xLeft, xRight);

	refineCableBoudary2(imgIn2, xLeft, xRight);

	return 0;
}

static int straightenCable(
	const Mat &frameIn,
	int *xLeftList,
	int *xRightList,
	Mat &frameOut)
{
	unsigned char *pImgIn = NULL;
	unsigned char *pImgOut = NULL;

	int imgW = frameIn.cols;
	int imgH = frameIn.rows;

	int meanCenter = imgW / 2;

	frameOut = Mat::zeros(imgH, imgW, CV_8UC1);

	pImgIn = (unsigned char *)frameIn.data;
	pImgOut = (unsigned char *)frameOut.data;

	for (int i = 0; i < imgH; i++)
	{
		int length = xRightList[i] - xLeftList[i];

		if (length <= 0 || length > imgW)   //如果不考虑则 Mat release() 会产生BUG
		{
			pImgIn += imgW;
			pImgOut += imgW;
			continue;
		}

		int center = (xRightList[i] + xLeftList[i]) / 2;
		int dmotion = meanCenter - center;
		int xLeftNew = dmotion + xLeftList[i];
		int xRightNew = dmotion + xRightList[i];

		memcpy(&pImgOut[xLeftNew],
			&pImgIn[xLeftList[i]],
			sizeof(unsigned char) * length);

		xLeftList[i] = xLeftNew;
		xRightList[i] = xRightNew;

		pImgIn += imgW;
		pImgOut += imgW;
	}

	return 0;
}

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

CentringCable::CentringCable()
{
}

CentringCable::~CentringCable()
{
}



int CentringCable::init()
{
    //爬坡初始化
    for (int i = 0; i < maxCameraNum; i++)
    {
        posStatus.lastPos[i] = -1;
    }
    posStatus.loss = INT_MAX;
    posStatus.diffLoss = INT_MAX;

    for(int i = 0; i < nDirect; i++)
    {
        directs.direct[i] = 1024;
        directs.nCur = 0;                //统计换向次数有关
    }

    nCurFrame = 0;                       //统计每次的线缆平均偏差有关
    return 0;
}

int CentringCable::initNoCableStatus()
{
	for (int i = 0; i < noCableLimit; i++)
	{
		posStatus.nNoCable[i] = 100;
	}
	posStatus.nCurBlack = 0;           //统计视野内连续无目标（黑图）

	return 0;
}

/**********************************************/
// getLinePos
// 提取线缆中心位置
// Input:
//      img         //载入图片组
//      nCamera     //相机个数
// Output:
//      xMean       //线缆图像中心
/**********************************************/
int CentringCable::getLinePos(Mat *img, int *xMean, int nCamera)
{
    vector<Mat> imgS(nCamera);
    int downScale = 4;
    int lineWidth;

    int xLeft[maxCameraNum][rowFrame] = { 0 }, xRight[maxCameraNum][rowFrame] = { 0 };

    //提取线路边缘
    for (int i = 0; i < nCamera; i++)
    {
        if (img[i].empty()) return CABLE_MOVE_ERROR_IMAGE_EMPTY;
        Size smallSize = Size(img[i].cols / downScale, img[i].rows / downScale);
        cv::resize(img[i], imgS[i], smallSize);
		segmentCable(imgS[i], img[i], xLeft[i], xRight[i]);

    }

    for (int i = 0; i < nCamera; i++)
    {
        xMean[i] = 0;
        lineWidth = 0;
        for (int j = 0; j < rowFrame; j++)
        {
            xMean[i] += (xLeft[i][j] + xRight[i][j]) / 2;
            lineWidth += xRight[i][j] - xLeft[i][j];
        }
        lineWidth /= rowFrame;
        xMean[i] /= rowFrame;

        // TODO
        //如果定义的图像宽度大于分割时预定的大小则无效，对应segmentCable函数percentB变量
        //if (lineWidth >(1 - percentB * 0.8) * img->cols)
        //    xMean[i] = 0;
    }
    
    return 0;
}

/**********************************************/
// ransac
// ransac直线拟合
// Input:
//      points      //点集
// Output:
//      coeffA      //时间系数(x系数）
//      coeffC      //常量系数
/**********************************************/
void CentringCable::ransac(const vector<cv::Point> &points, double &coeffA, double &coeffC)
{
    double sampleRate = 0.5;    //采样率
    double dist = 5;            //接受距离
    double condition = 0.8;     //结束条件最小包含比例
    int nRep = 10;              //循环周期

    int nMaxGet = 0;            //最大获取点数
    int nGet = 0;               //当前获取点数
    vector<cv::Point> _points, _sample;
    Vec4f param;
    int nSample = points.size() * sampleRate;
    double a = 0,c = 0;
    double cos = 1;
    for (int i = 0; i < nRep; i++)
    {
        _points = points;
        random_shuffle(_points.begin(), _points.end());
        _sample.assign(_points.begin(), _points.begin() + nSample);    //分配  类似.cpy
        fitLine(_sample, param, DIST_L2, 0, 1e-2, 1e-2);
        a = param[1] / param[0];
        c = param[3] - a * param[2];
        cos = 1/sqrt(a * a + 1);                                        //cos ~tan =a =k之间的关系

        nGet = 0;
        for (int i = 0; i < points.size(); i++)
        {
            if (cos * abs(a * points[i].x + c - points[i].y) < dist)
            {
                nGet++;
            }
        }
        
        if (nGet > nMaxGet)
        {
            nMaxGet = nGet;
            coeffA = a;
            coeffC = c;
        }
        
        if (nGet > condition * points.size())   break;
    }
}

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
void CentringCable::medianFilter(int *xMean, int len, int kSize)
{
    int halfSize = kSize / 2;
    int *_xMean = new int[kSize];
    int *xMeanClone = new int[len];

    int pos = 0;
    int _subSize = 0;
    memcpy(xMeanClone, xMean, sizeof(int) * len);
    for (int i = 0; i < halfSize; i++)
    {
        pos = 0;
        _subSize = __min(i + halfSize, len - 1) - pos + 1;
        memcpy(_xMean, xMeanClone + pos, _subSize * sizeof(int));
        std::sort(_xMean, _xMean + _subSize);
        xMean[i] = _xMean[_subSize / 2];
    }
    for (int i = halfSize; i < len - halfSize; i++)
    {
        memcpy(_xMean, xMeanClone + i, kSize * sizeof(int));
        std::sort(_xMean, _xMean + kSize);
        xMean[i] = _xMean[halfSize];
    }
    for (int i = len - halfSize; i < len; i++)
    {
        pos = __max(0, i - halfSize);
        _subSize = len - 1 - pos + 1;
        memcpy(_xMean, xMeanClone + pos, _subSize * sizeof(int));
        xMean[i] = _xMean[_subSize / 2];
    }
    delete[] _xMean;
    delete[] xMeanClone;
}

/**********************************************/
// fittingMove
// 运动拟合
// Input:
//      moveInfo        //线缆运动数据
// Output:
//      coeff           //运动拟合系数
/**********************************************/
int CentringCable::fittingMove(MoveInfo &moveInfo, FittingCoeff &coeff)
{
    int nCamera = moveInfo.nCamera;
    int nFrame = moveInfo.nFrame;
    coeff.nCamera = nCamera;
    vector<cv::Point> mMean;
    Vec4f param;
    
    for (int i = 0; i < nCamera; i++)
    {
        medianFilter(moveInfo.xMean[i], moveInfo.nFrame, 5);
        for (int j = 0; j < nFrame; j++)
        {
            //if (moveInfo.xMean[i][j] != 0)
            //{
            mMean.push_back({ j, moveInfo.xMean[i][j] });
            //}
        }
        ransac(mMean, coeff.a[i], coeff.c[i]);               //ransacFitline函数
        mMean.clear();
    }
    
    return 0;
}

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
int CentringCable::optimalMove(Mat *img, MoveInfo &moveInfo, double &moveValue, bool isMoving)
{
    int nCamera = moveInfo.nCamera;
    int imgCenter = img->cols / 2;
    double numerator = 0;
    double denominator = 0;
    moveValue = 0;
    double zerosRate = 0.3;                 //允许最大零值个数
	int nZeros = 0, finalCount = 0;
    if (isMoving)
    {
        int *_xMean = new int[nCamera];
        int res = getLinePos(img, _xMean, nCamera);
        if (res != 0)                               //存在空指针图
			return res;

		//TODO   leon   在运动过程中有最优值可以添加判断做停止
		//int meanLinePos = 0;
		for (int i=0; i< nCamera; i++)
		{
			//meanLinePos += _xMean[i];
			printf("_xMean[%d]:%d  ", i,_xMean[i]);

			if (_xMean[i] >= imgCenter - currentDist && _xMean[i] <= imgCenter + currentDist)
				finalCount++;
		}

		printf("\n[finalCount=%d]\n\n", finalCount);

		if (finalCount == nCamera)
		{
			moveValue = -100;
			printf("  【END-finalCount=%d】\n", finalCount);

			for (int i=0; i<nCamera;i++)
			{
				if (i < nCamera-1)
				{
					printf("   camera[%d] diff=%d  ", i, abs(imgCenter - _xMean[i]));
				}
				else
					printf("   camera[%d] diff=%d  \n\n", i, abs(imgCenter - _xMean[i]));
			}
		}

        for (int i = 0; i < nCamera; i++)
        {
            moveInfo.xMean[i][moveInfo.nFrame] = _xMean[i];
        }
        moveInfo.nFrame++;
        delete[] _xMean;
    }
    else            //30帧一周期做一个判断
    {
        //迭代结束条件（直接固定时间跑）
        //if (moveInfo.nFrame < 10)   return 0;
        //计算0值个数
        nZeros = 0;
        for (int i = 0; i < moveInfo.nCamera; i++)
        {
            for (int j = 0; j < moveInfo.nFrame; j++)
            {
                if(moveInfo.xMean[i][j] == 0)   nZeros++;
            }
        }
        //黑图不予计算(假设从中间到边缘的移动时间少于规定运动时间）
		float blackRate = 1.0f * nZeros / (nCamera * moveInfo.nFrame);
		printf("【黑图个数：%d，比率：%0.2f】 \n", nZeros, blackRate);

        if(blackRate > zerosRate)
        {
            moveInfo.nFrame = 0;
            return 0;
        }
        FittingCoeff coeff;
        fittingMove(moveInfo, coeff);
        
        for (int i = 0; i < coeff.nCamera; i++)
        {
            numerator += imgCenter * coeff.a[i] - coeff.a[i] * coeff.c[i];     // y=(a1x+c1-400)^2 +(a2x+c2-400)^2 +..  for循环去单个，进行求导   [400是800宽的一般
            denominator += coeff.a[i] * coeff.a[i];
        }
        //获取迭代位移
        moveValue = numerator / denominator - moveInfo.nFrame;                 // y=(a1x+c1-400)^2 +(a2x+c2-400)^2 +..  进行求导 取最优解
		if (moveValue <= -100 && moveValue > -101)
			moveValue = -101.2;

		printf("moveValue=%0.2f \n", moveValue);        //获取最优解，重新初始化运动采样
        moveInfo.nFrame = 0;
    }
    
    return 0;
}

/**********************************************/
// estimateMove
// 实时判断是否为对中区域
// Input:
//      img             //线缆图像组
//      moveInfo        //运动数据(内部数据优化）
//      isMoving        //是否正在运动（true:处于采集状态，false:处于预测状态）
// Output:
//      moveValue       //提供电机运动数据
/**********************************************/
int CentringCable::estimateMove(Mat *img, int nFrame, int nCamera, int ms, int &moveTime, bool &isOk)
{
    //int nCamera = moveInfo.nCamera;
    int imgCenter = img->cols / 2;
    isOk = false;

    int *_xMean = new int[nCamera];
    float *_xMeanf = new float[nCamera];
#if 0
#else
    for (int i = 0; i < nCamera; i++)
    {
        getLinePos(img, _xMean, nCamera);
    }
    
    
#endif
    if (posStatus.lastPos[0] == -1) //初始帧
    {
        posStatus.loss = 0;
        for (int i = 0; i < nCamera; i++)
        {
            posStatus.lastPos[i] = _xMeanf[i];
            posStatus.loss += (_xMeanf[i] - imgCenter) * (_xMeanf[i] - imgCenter);
        }
        posStatus.loss = sqrt(posStatus.loss);
        moveTime = ms;
        return 0;
    }
    else
    {
        float loss = 0;
        for (int i = 0; i < nCamera; i++)
        {
            loss += (_xMeanf[i] - imgCenter) * (_xMeanf[i] - imgCenter);
        }
        loss = sqrt(loss);
        
        //移动位移
        float *moveDist = new float[nCamera];
        //移动速率
        float *moveVelo = new float[nCamera];
        //距离中心偏移量
        float *moveOffset = new float[nCamera];
        memset(moveOffset, 0, sizeof(float) * nCamera);

        if (loss <= posStatus.loss) //方向正确(计算当前方向速度)
        {
            float veloShift = 0.f, velo = 0.f;
            //求解当前方向最优移动时间
            //argmin(ms){∑(moveOffset-moveVelo * ms)^2}
            //=> ms = 2∑moveVelo * moveOffset / ∑ moveVelo
            for (int i = 0; i < nCamera; i++)
            {
                moveOffset[i] += abs(_xMeanf[i] - imgCenter);
                moveDist[i] = abs(_xMeanf[i] - posStatus.lastPos[i]);
                moveVelo[i] = moveDist[i] / ms;
                veloShift += moveVelo[i] * moveOffset[i] * 2;
                velo += moveVelo[i];
            }
            //至少一倍时间至多两倍时间
            moveTime = __min(__max(ms, veloShift / velo), 2 * ms);
        }
        else                        //方向相反(速度不一样，不做计算)
        {
            moveTime = -ms;
        }
        
        //1、时间变多回去；2、时间达到一定小值回去
        //损失更新
        posStatus.loss = loss;
        //位置更新
        for (int i = 0; i < nCamera; i++)
        {
            posStatus.lastPos[i] = _xMeanf[i];
        }

        delete[] moveDist;
        delete[] moveOffset;
        delete[] moveVelo;
    }

    delete[] _xMean;
    delete[] _xMeanf;
    return 0;
}

/**********************************************/
// realTime
// 实时判断
// Input:
//      img             //线缆图像组
//      nCamera         //相机个数
//      condDist        //平均中心偏差阈值
// Output:
//      plcTime           返回plc运动时间
//      direct            plc的运动方向*
//      isOK              返回当前帧是否满足对中条件
/**********************************************/
int CentringCable::realTime(Mat *img, int nFrame, int nCamera, int condDist, int finePlcTime_1, int finePlcTime_2, int &plcTime, int &direct, bool &isOK)
{
    int *_xMean = new int[nCamera];
    int imgCenter = img->cols / 2;
	int medianCount = 0;
	direct = 1;
	plcTime = 10000;        //10s

    getLinePos(img, _xMean, nCamera);          //函数里 for循环计算每相机图的位置

	if (posStatus.lastPos[0] == -1)      //init()中 初始化时都为-1
	{
		directs.nChangeDir = 0;          //累计的换向次数；当有换向了后做判断无目标图才有意义
		directs.nPlcFine = 0;            //精细调整步长的计数，不为0后，即用小步长去调整
	}
	//------------------------------------------------------------------------
	// 累计下无目标时候的个数，做是否换向的判断   -leon
	int noCableNum = 0;
	for (int n = 0; n < nCamera; n++)
	{
		if (_xMean[n] <= 0)
			noCableNum++;
	}
	if (noCableNum == nCamera)
	{
		posStatus.nNoCable[posStatus.nCurBlack] = 0;
	}

	int _noCableSum = 0;
	for (int i =0 ; i <noCableLimit; i++)       //默认6
	{
		_noCableSum += posStatus.nNoCable[i];
	}
	if (_noCableSum == 0 && directs.nChangeDir != 0)
	{
		direct = -1;                            //连续noCableLimit =6 次各相机视野都为无目标，即换方向
		initNoCableStatus();
	}
	posStatus.nCurBlack = (posStatus.nCurBlack + 1) % noCableLimit;

	//----------------------------------------------------------------- end

    for (int n = 0; n < nCamera; n++)
    {
        //找不到线缆则为无效帧组
        if(_xMean[n] <= 0)
        {
            nCurFrame = 0;
			cout << endl << "no cable" << endl;
            init();

            goto EXIT;
        }
        pos[nFrame * n + nCurFrame] = _xMean[n];                    //nCurFrame每次会取 与nFrame的余数

		if (_xMean[n] >= imgCenter*0.7 &&_xMean[n] <= imgCenter*1.3)  
			medianCount++;
    }

//	int * a = &nFrame;
    if (nCurFrame == nFrame - 1)     //当处理完输入帧数的周期时
    {
        float _pos[maxCameraNum];
        float _loss = 0;
        for (int i = 0; i < nCamera; i++)
        {
            //_pos[i] = 0;
            for (int j = 0; j < nFrame; j++)
            {
                //_pos[i] += pos[i * nFrame + j];
                sort(&pos[i * nFrame], &pos[(i + 1) * nFrame]);
                _pos[i] = pos[i * nFrame + nFrame / 2];            //取中值
            }
            //_pos[i] /= nFrame;
            _loss += abs(_pos[i] - imgCenter);
        }
        _loss /= nCamera;                                          //平均偏移量
        
        if (_loss <= condDist)
        {
            isOK = true;
			//cout << "condDist_loss is OK" << endl;               //可直接返回否
			printf("\n【condDist_loss #%.2f < %d is OK】 \n", _loss, condDist);
        }
        else
        {
            isOK = false;
        }

        if (posStatus.lastPos[0] == -1)      //初始化时都置为-1
        {
            for (int n = 0; n < nCamera; n++)
            {
                posStatus.lastPos[n] = _pos[n];
            }
        }
        else
        {
#if 0
			posStatus.diffLoss = _loss - posStatus.loss;        // 原来- 只需要在居中位置换向 
            if(posStatus.diffLoss > 0)  direct = -1;
            else                        direct = 1;
#endif

			posStatus.diffLoss = _loss - posStatus.loss;        //  只需要在居中范围换向 
			printf("    _loss :%0.2f \n", _loss);

			if (medianCount >= nCamera -2 && _loss < imgCenter *0.3)
			{
				if (posStatus.diffLoss > 0)  
				{
					direct = -1;       //当前次的平均偏差比上次的大就换向
					directs.nChangeDir++;

					if (_loss < imgCenter *0.2 || directs.nPlcFine >= 1)
					{
						directs.nPlcFine++;
						plcTime = finePlcTime_2;
					}
					else
						plcTime = finePlcTime_1;

					printf("   【变向】diffLoss :%0.2f  _loss :%0.2f ,plcTime=%d \n\n", posStatus.diffLoss, _loss, plcTime);
				}
				else
				{
					direct = 1;
					if (_loss < imgCenter *0.2)
						plcTime = finePlcTime_2;
					else
						plcTime = finePlcTime_1;

					printf("   【不变】diffLoss :%0.2f  _loss :%0.2f ,plcTime=%d \n", posStatus.diffLoss, _loss, plcTime);
				}
			}

			/*************************************/

            for (int n = 0; n < nCamera; n++)
            {
                posStatus.lastPos[n] = _pos[n];
            }
			
        }
        //结束条件1: 误差差值变小，当前处于相对误差小值
        //if (abs(posStatus.diffLoss) < condDist * 0.3 && direct > 0)
        //{
        //    isOK = true;
        //}

        //结束条件2: 区域范围重复往返，当前处于相对误差小值
		//连续5个次数内，累加和小于等于-1  说明至少换向三次 2-3=-1 
        directs.direct[directs.nCur] = direct;

        int _directSum = 0;
        for(int i = 0; i < nDirect; i++)       
        {
            _directSum += directs.direct[i];
        }


        //if (abs(_directSum) <= 2 && direct > 0)      /*连续五次为一个周期 3-2=1   有两次换向，且最后一次为正向*/
		if ( _directSum <= 1 && direct > 0)            /*连续五次为一个周期 2-3=-1  有三次换向，且最后一次为正向*/
        {
            isOK = true;
			printf("\n【_directSum <=1,有两次换向 --OK】\n");
        }
        directs.nCur = (directs.nCur + 1) % nDirect;

        posStatus.loss = _loss;
    }

    #if 0
    //取中值
    if (nCurFrame == nFrame - 1)
    {
        if (posStatus.lastPos[0] == -1)
        {
            posStatus.loss = 0;
            for (int i = 0; i < nCamera; i++)
            {
                sort(pos + i * nFrame, pos + (i + 1) * nFrame);
                posStatus.lastPos[i] = pos[i * nFrame + nhalfFrame];
                posStatus.loss += abs(posStatus.loss - imgCenter);  //L1
            }
            posStatus.loss /= nCamera;
        }
        else
        {
            float loss = 0;
            for (int i = 0; i < nCamera; i++)
            {
                sort(pos + i * nFrame, pos + (i + 1) * nFrame);
                posStatus.lastPos[i] = pos[i * nFrame + nhalfFrame];
                loss += abs(loss - imgCenter);  //L1
            }
            loss /= nCamera;
            
            
            (posStatus.loss - loss) / abs(posStatus.loss - loss);
        }
        
    }
    #endif
    nCurFrame = (nCurFrame + 1) % nFrame;

EXIT:
    delete[] _xMean;
    return 0;
}

/**********************************************/
// loadMoveData
// 载入当前运动数据
// Input:
//      filePath    //运动参数文档
// Output:
//      moveStatus  //运动数据
/**********************************************/
int CentringCable::loadMoveData(string filePath, MoveStatus &moveStatus)
{
    ifstream in;
    in.open(filePath, ios::in);
    if(!in.is_open())   return CABLE_MOVE_ERROR_DATA_LOAD;

    string lineStr;
    string keyStr;
    string valueStr;
    int eqlPos = 0;
    int spcPos = 0;
    while (!in.eof())
    {
        std::getline(in, lineStr);
        //剔除空格
        spcPos = lineStr.find(' ');

        while (spcPos != -1)
        {
            lineStr.replace(spcPos, 1, "");
            spcPos = lineStr.find(' ');
        }

        eqlPos = lineStr.find('=');
        keyStr = lineStr.substr(0,eqlPos);
        valueStr = lineStr.substr(eqlPos + 1);
        
        if          (keyStr == "time")      moveStatus.time = atof(valueStr.c_str());
        else if     (keyStr == "shift")     moveStatus.shift = atof(valueStr.c_str());
        else if     (keyStr == "velocity")  moveStatus.velocity = atof(valueStr.c_str());
    }
    in.close();
    return 0;
}

//输出结果的实时位置
int CentringCable::cableFinalPose(Mat *img, int nCamera, int nFrame, MoveInfo &outMovePose)
{
	if (!img)    return CABLE_MOVE_ERROR_IMAGE_EMPTY;

	int imgMiddle= img->cols/2;

	outMovePose.nCamera = nCamera;
	outMovePose.nFrame = nFrame;
	int *_xMean = new int[nCamera];

	for (int i = 0; i < nCamera; i++)
	{
		outMovePose.xMean[i][0] = 0;
	}

	for (int n = 0; n < nFrame; n++)
	{
		int res = getLinePos((img + n * nCamera), _xMean, nCamera);
		if (res != 0)
			return res;
	}

	for (int k=0;k<nCamera;k++)
	{
		_xMean[k] -= imgMiddle;
		outMovePose.xMean[k][0] = _xMean[k];      //各个相机的对应帧的位置
	}
	
	return 0;
}

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
int CentringCable::cableStatus(Mat *img, int nCamera, int nFrame, MoveInfo &outMoveInfo, MoveStatus &outMoveStatus)
{
    if(!img)    return CABLE_MOVE_ERROR_IMAGE_EMPTY;
    
    int imgMiddle;

    outMoveInfo.nCamera = nCamera;
    outMoveInfo.nFrame = 0;
    int *_xMean = new int[nCamera];
    
    for (int i = 0; i < nCamera; i++)
    {
        outMoveInfo.xMean[i][0] = 0;
    }
    for(int n = 0; n < nFrame; n++)
    {
        int res = getLinePos((img + n * nCamera), _xMean, nCamera);
		if (res != 0)  
			return res;
        for (int i = 0; i < nCamera; i++)
        {
            outMoveInfo.xMean[i][0] += _xMean[i];       //将各个相机的对应帧的位置累加到一起
        }
    }
    for (int i = 0; i < nCamera; i++)
    {
        outMoveInfo.xMean[i][0] /= nCamera;
    }
    delete[] _xMean;

    //判断多相机是否都为 无目标
    bool noCable = true;
    for (int i = 0; i < nCamera; i++)
    {
        if (outMoveInfo.xMean[i][0] != 0)
        {
            noCable = false;
            break;
        }
    }

    //无线缆存在时取任意方向移至限位
    if (noCable)
    {
        outMoveStatus.direct = CABLE_DIRECT_STATUS_NONE;
        outMoveStatus.getLimit = true;
        return 0;
    }

    imgMiddle = img->cols / 2;
    double leftMean, rightMean;

	int finalCount = 0;
	for (int i=0; i<nCamera; i++)
	{
		if (outMoveInfo.xMean[i][0] >= imgMiddle - currentDist && outMoveInfo.xMean[i][0] <= imgMiddle + currentDist)
			finalCount++;
	}
    switch (nCamera)
    {
    case 3:
        leftMean = __max(outMoveInfo.xMean[0][0] , outMoveInfo.xMean[1][0]);
        //if ((abs(outMoveInfo.xMean[0][0] - imgMiddle) + abs(outMoveInfo.xMean[1][0] - imgMiddle) +
        //   abs(outMoveInfo.xMean[2][0] - imgMiddle)) / 3 <= 20)
        if (finalCount == nCamera)        {
            outMoveStatus.direct = CABLE_DIRECT_STATUS_OK;
            outMoveStatus.getLimit = false;
        }
        else if (leftMean > outMoveInfo.xMean[2][0])                       // 左>右
        {
            if (outMoveInfo.xMean[2][0] == 0 && leftMean < imgMiddle)
            {
                //outMoveStatus.direct = CABLE_DIRECT_STATUS_UP; 
				outMoveStatus.direct = CABLE_DIRECT_STATUS_DOWN;
            }
            else    
            {
                //outMoveStatus.direct = CABLE_DIRECT_STATUS_DOWN; 
				outMoveStatus.direct = CABLE_DIRECT_STATUS_UP;
            }
            outMoveStatus.getLimit = false;
        }
        else                                                           // 左<右
        {
            if (leftMean == 0 && outMoveInfo.xMean[2][0] < imgMiddle)      
            {
                //outMoveStatus.direct = CABLE_DIRECT_STATUS_DOWN;     
				outMoveStatus.direct = CABLE_DIRECT_STATUS_UP;
            }
            else
            {
                //outMoveStatus.direct = CABLE_DIRECT_STATUS_UP;
				outMoveStatus.direct = CABLE_DIRECT_STATUS_DOWN;
            }
            outMoveStatus.getLimit = false;
        }

        break;
    case 4:
        leftMean = __max(outMoveInfo.xMean[0][0] , outMoveInfo.xMean[1][0]);
        rightMean = __max(outMoveInfo.xMean[2][0] , outMoveInfo.xMean[3][0]);

        //if ((abs(outMoveInfo.xMean[0][0] - imgMiddle) + abs(outMoveInfo.xMean[1][0] - imgMiddle) +
        //    abs(outMoveInfo.xMean[2][0] - imgMiddle) + abs(outMoveInfo.xMean[3][0] - imgMiddle)) / 4 <= 20)
		if (finalCount == nCamera)
        {
            outMoveStatus.direct = CABLE_DIRECT_STATUS_OK;
            outMoveStatus.getLimit = false;
        }
        else if (leftMean > rightMean)                                  //左>右
        {
            if (rightMean == 0 && leftMean < imgMiddle)
            {
                //outMoveStatus.direct = CABLE_DIRECT_STATUS_UP;
				outMoveStatus.direct = CABLE_DIRECT_STATUS_DOWN;
            }
            else
            {
                //outMoveStatus.direct = CABLE_DIRECT_STATUS_DOWN;
				outMoveStatus.direct = CABLE_DIRECT_STATUS_UP;
            }
            outMoveStatus.getLimit = false;
        }
        else                                                           //左<右
        {
            if (leftMean == 0 && rightMean < imgMiddle)
            {
                outMoveStatus.direct = CABLE_DIRECT_STATUS_DOWN;
            }
            else
            {
                outMoveStatus.direct = CABLE_DIRECT_STATUS_UP;
            }
            outMoveStatus.getLimit = false;
        }

        break;
    default:
        break;
    }
    
    return 0;
}

