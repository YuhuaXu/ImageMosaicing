/*

*/
#pragma once

#include "Bitmap.h"
//#include "ati.h"


//// 均值滤波
//int MeanSmooth( IplImage* pSrc, IplImage* pDst,  int radius);

// 高斯滤波
//bool GaussFilter(pool::BitmapImage *pSrc, pool::BitmapImage *pDst, const float fSigma = 0.8f, int winSize = 5);

// *********************************************************************************
// 函数功能：高斯滤波
// winSize，高斯窗口的宽度
// *********************************************************************************
bool GaussFilter_Int(pool::BitmapImage *pSrc, pool::BitmapImage *pDst, float fSigma = 0.8f, int winSize = 5);

// *********************************************************************************
// 函数功能：高斯滤波
// winSize，高斯窗口的宽度
// *********************************************************************************
bool GaussFilter2( const unsigned char *pSrcData, const int wSrc, const int hSrc, const int wsSrc, 
				 unsigned char *pDstData,
				 const float fSigma = 0.8f, const int winSize = 5);

// *********************************************************************************
// 函数功能：高斯滤波 带Mask图像
// winSize，高斯窗口的宽度
// 允许pSrc==pDst
// *********************************************************************************
bool GaussFilterMask(const pool::BitmapImage *pSrc, 
					 pool::BitmapImage *pMaskImage,
					 pool::BitmapImage *pDst, const float fSigma = 0.8f, int winSize = 5);

// 均值滤波
bool MeanFilter(pool::BitmapImage *pSrc, pool::BitmapImage *pDst, int winSize);

// *********************************************************************************
// 均值滤波
// *********************************************************************************
bool MeanFilterSSE(const pool::BitmapImage *pSrc, pool::BitmapImage *pDst, int winSize);

// 1D高斯平滑
int GaussSmoothData1D(float* srcData, float* dstData, int N, float sigma, int winR);