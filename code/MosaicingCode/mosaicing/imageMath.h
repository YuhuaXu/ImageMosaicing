
#pragma once

//#include "ati.h"
#include "Point.h"
#include "Bitmap.h"
#include "usingCV24.h"

struct Sample
{
	int subSample[10];
};

// 线性变换 
bool LinearTransform(const pool::BitmapImage *pSrc, pool::BitmapImage *pDst, 
					 const float fK, const float fB);

void LineOf2Points1(float &a,float &b,float &c,
				   float x1,float y1,float x2,float y2);

void GammaCorrection(IplImage* src, IplImage* dst, float gamma);

//求一个点在四象限中的角度（弧度），0 ~ 2*pi
int AngleofPoint360(float x, float y, float &angleArc);

template <class T>
T AngleofPoint(T x,T y,T &angle)
{
	if(x>=0)// 1，4象限
	{
		if(y>=0) // 1  x>0 y>0
		{
			if(x!=0)
			{
				angle=(float)atan(y/x);
			}
			else // x = 0
			{
				if(y!=0)
				{
					angle = pi/2;
				}
				else // y = 0
				{
					angle = 0;
				}
			}

		}
		else // 4  x>0  y<0
		{
			if(x!=0)
			{
				angle=(float)atan(y/x);
			}
			else
			{
				angle=-pi/2;
			}
		}
	}
	else// 2，3象限
	{
		if(y>=0) // 2
		{
			if(x!=0)  // x<0 y>0
			{
				angle=pi + (float)atan(y/x);
			}
			else
			{
				angle=pi/2;
			}
		}
		else //3  x<0  y<0
		{
			if(x!=0)
			{
				angle=(float)atan(y/x) - pi; 
			}
			else
			{
				angle=-pi/2;
			}
		}
	}
	return angle;
}

// result = pSrc1[0]*pSrc2[0] + pSrc1[1]*pSrc2[1] + pSrc1[2]*pSrc2[2] + pSrc1[3]*pSrc2[3] + ......
template<class T1, class T2, class T3>
bool Product(T1 *pSrc1, T2 *pSrc2, int nSize,
			 T3 &result)
{
	if( (pSrc1==NULL)||(pSrc2==NULL) )
		return false;

	result = 0;

	for(int i=0; i<nSize; i++)
	{
		result += pSrc1[i] * pSrc2[i];
	}

	return true;
}

//计算 1/sqrt(x)  1/sqrt(x)----开根号的倒数快速求解方法
 float InvSqrt (float x);



// Point1 = R*Point0 + T;
// Point0 = R^-1*(Point1-T)
 void Point0ToPoint1(float x0,float y0,
						   float &x1,float &y1,
						   float sina, float cosa, float dx, float dy);

// Point1 = R*Point0 + T;
// Point0 = R^-1*(Point1-T)
 void Point1ToPoint0(float x1,float y1,
						   float &x0,float &y0,
						   float sina, float cosa, float dx, float dy);

//// 2的n次方
//unsigned long Pow2(unsigned long n);

//bool Sub2Image24(IplImage* pSrc1, IplImage* pSrc2, IplImage* pDst);

// 直线表示 ax+by+c=0  转到  rho = x*cos(theta)+y*sin(theta)
void ABCToPolar(float a,float b,float c,
				float &rho,float &theta);

pool::fPoint TimeRotationMatrix2D(pool::fPoint src, float cosa, float sina);

pool::SfPoint TimeRotationMatrix2D(pool::SfPoint src, float cosa, float sina);

// 操作“与”
bool And2(unsigned char* pSrc1, int w1, int h1, int ws1, 
		 unsigned char* pSrc2,
		 unsigned char* pDst);

//计算重心
int CalCenterOfGravity( pool::Point *pointSet, int setSize, float &centerX, float &centerY);

//计算重心
int CalCenterOfGravity(const std::vector<pool::sfPoint> &pointSet, float &centerX, float &centerY);

//计算重心
int CalCenterOfGravity( pool::Point *pointSet, int setSize, float &centerX, float &centerY);

//计算重心
int CalCenterOfGravity(const vector<pool::Point> &pointSet, float &centerX,float &centerY);

// 函数功能：计算一张二值图的非零点的个数
int CalValidPixel(pool::BitmapImage *src,int &counter);

// 求一堆点里面的4个端点的坐标  
void Find4EndPoints(pool::Point *pdata,int npt, pool::sfPoint endpoint[4]);

// 求两个圆的交点
bool IntersectionPointsOfTwoCircles(float x1, float y1, float r1,
									float x2, float y2, float r2,
									float &intersectX1, float &intersectY1,
									float &intersectX2, float &intersectY2 );

// 遍历采样
bool Sampling(vector<Sample> &samplingResult, int sampleSize, int subSampleSize);

//****************************************************************************************************
// 获得特征向量
//****************************************************************************************************
// 特征向量存入fM(降序排列), 按行存储
// 特征值存入fV
// CvMat* M = cvCreateMat(N, N, CV_64FC1);
// CvMat* E = cvCreateMat(N, N, CV_64FC1);
// CvMat* I = cvCreateMat(N, 1, CV_64FC1);
int GetEigenMat(double* k, int N, 
				double *fM, // 特征向量
				double *fV, // 特征值
				CvMat* M, 
				CvMat* E,
				CvMat* I
				);

template<class T1, class T2>
void Norm2(T1 x, T1 y, T2 &res)
{
	res = sqrt(x*x+y*y);
}

template<class T1, class T2>
void Norm3(T1 x, T1 y, T1 z, T2 &res)
{
	res = sqrt((T2)x*x + y*y + z*z);
}