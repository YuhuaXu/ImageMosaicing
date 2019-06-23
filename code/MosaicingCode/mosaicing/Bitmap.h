
#pragma once
//#include <afx.h>
//#include "stdafx.h"
#include "math.h"
#include <vector>
#include <algorithm>
#include <iostream>
#include <time.h>
#include "MemoryPool.h"
#include "Point.h"
#include "Basic.h"

using namespace std;

#define _OUT 
#define _IN

// 极坐标表示直线
struct PolarLine
{
	float p;
	float th;

	PolarLine()
	{
		p=0;
		th=0;
	}
};

struct LineABC
{
	float a, b, c;

	LineABC()
	{
		a = b = c;
	}
};

struct ProjectMat
{
	float m[9];
};

struct ProjectMat64F
{
	double m[9];
};

namespace pool
{
	const float pi = 3.1415926f;
	const float PI_32F = 3.14159265358979f;
	const double PI_64F = 3.14159265358979;
	const float PI = 3.1415926f;
	const float PI_HALF = 3.1415926f/2;
	const float fRadToDegree = 180/pi;
	//typedef unsigned char UCHAR;

	struct PixelRGB
	{
		int r,g,b;
	};

	struct AngleOfPoint
	{
		float angle;
		int index;
		int fValid;

		bool   operator <  (const AngleOfPoint& rhs)  const   //升序排序时必须写的函数
		{   
			return angle<rhs.angle; 
		}
	};

	// 梯度
	struct Gradient32F
	{
		float gx;
		float gy;
		float w; // 权重
		int seq; 
	};

	struct Color
	{
		int b, g, r, a;

		Color(int b, int g, int r)
		{	
			this->b = b;
			this->g = g;
			this->r = r;
		}

		Color()
		{

		}
	};

	struct BitmapImage
	{
		unsigned char* imageData; 
		int width; // 像素宽度
		int height;// 像素高度
		int widthStep; // 物理宽度 4的倍数
		int nChannels; // 通道数 灰度图为1, 彩色图为3
		BitmapImage()
		{
			imageData = NULL;
			width=0;
			height=0;
			widthStep=0;
			nChannels=0;
		}
		BitmapImage(unsigned char* pImgData, int width, int height, int widthStep, int nChannels)
		{
			this->imageData = pImgData;
			this->width = width;
			this->height = height;
			this->widthStep = widthStep;
			this->nChannels = nChannels;
		}
	};

	// "块"矩形结构体
	struct BlockInf {
		int xbeg; // x起点坐标（两个对角点）
		int xend; // x终点坐标
		int ybeg; // y起点坐标
		int yend; // y终点坐标
		BlockInf()
		{
			xbeg=0;
			xend=0;
			ybeg=0;
			yend=0;
		}
	};

	struct InterestRegion32F
	{
		float xbeg; // 起点x
		float xend; // 终点x
		float ybeg; // 起点y
		float yend; // 终点y
	};

	struct MatchPoint
	{
		float x;
		float y;
		float fMatchScore;
	};

	//struct AngleOfPoint
	//{
	//	float angle;
	//	int index;
	//	int fValid;

	//	bool   operator <  (const AngleOfPoint& rhs)  const   //升序排序时必须写的函数
	//	{   
	//		return angle<rhs.angle; 
	//	}
	//};
}

// 获取图像的高、宽
int GetImageSize(const pool::BitmapImage *imSrc,int &w,int &h,int &ws);

// 新分配了内存
pool::BitmapImage* CreateBitmap8U( int width, int height, int nChannels);

// 新分配了内存
extern "C" __declspec(dllexport)
void ReleaseBitmap8U(pool::BitmapImage* &pDstImage);

void SaveBitmap(char* path, pool::BitmapImage* &pDstImage);

// 新分配了内存的拷贝
pool::BitmapImage* CloneBitmap8U(const pool::BitmapImage* pSrcImage );

// 函数功能：在图片中截取图片，给定两个 对角点
// 新分配了内存
pool::BitmapImage* CutPatchImage(const pool::BitmapImage *src,
								pool::IntPoint startp, pool::IntPoint endp);

pool::BitmapImage* CutPatchImage(const pool::BitmapImage *src,
								 const pool::InterestRegion2 &roi);

// 函数功能：写数据到图像中的某个矩形区域   
// 对角点：startp   endp
int WriteBackToImageRegion(pool::BitmapImage *src, pool::BitmapImage *seg, 
						   pool::IntPoint startp, pool::IntPoint endp);

// 函数功能：写数据到图像中的某个矩形区域   
// 对角点：startp   endp
int WriteBackToImageRegion(pool::BitmapImage *src, pool::BitmapImage *seg, const pool::InterestRegion2 &roi);

bool IsBitmapValid(const pool::BitmapImage *srcImage);

// 把整幅图像置零
bool ZeroImage(pool::BitmapImage *pSrcImage);

void DestroyMemoryPool();

// 获取CPU个数
int GetCpuNum();

// 验证是否在有效期内
int InValidDate(int deadLine_year, int deadLine_month);

// 访问16位图像的元素
inline ushort GetImgVal16(char* imageData, int x, int y, int ws)
{
	return *(ushort*)(imageData + y*ws + x*2);
}

// 清空vector内存
template < class T >
void ClearVectorMemory(std::vector< T >& vt)
{
	std::vector< T > vtTemp;
	veTemp.swap(vt);
}