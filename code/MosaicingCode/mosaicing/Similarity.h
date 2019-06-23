#pragma once

#include "Bitmap.h"
#include <string>
#include "usingCV24.h"

// model的均值和标准差已知
bool NccMatch2D(pool::BitmapImage *left, pool::BitmapImage *right,
				float meanM, float stdM, 
				int validWidth,
				int du, int dv,
				float &fScore);

bool CalNCCScore(pool::BitmapImage *left, pool::BitmapImage *right,
				 int dx, int dy,
				 float &fScore,
				 int minOverlapPointsNum);

// model的均值和方差已知
bool CalNCCScore(pool::BitmapImage *model, pool::BitmapImage *target,
				 float meanM, float stdM,
				 int dx, int dy,
				 float &fScore);

// 计算匹配度
// strMethod = "SAD" 或 "NCC"
bool CalMatchScore(pool::BitmapImage *left, pool::BitmapImage *right,
				   int du, int dv,
				   float &fScore,
				   int minOverlapPointsNum,
				   string strMethod);

// strMethod = "SAD" 或 "NCC"
// right-templateImg
bool CalMatchScore2(pool::BitmapImage *left, pool::BitmapImage *right,
					int dx, int dy,
					float &fScore,
					string strMethod);

// 计算匹配度
bool CalMatchScore_Circular(pool::BitmapImage *left, pool::BitmapImage *right,
							float &fScore );

// 功能：加权2D-NCC
// 每一列的权重相同
bool NCC_2D_Weight(pool::BitmapImage *left, pool::BitmapImage *right,
				   int du, int dv,
				   float &fScore,
				   float* pWeightColLeft = NULL, float *pWeightColRight = NULL,
				   pool::BitmapImage* pSignT = NULL, pool::BitmapImage* pSignMWiden = NULL) ;

// 功能：加权2D-NCC-For Polar
// 每一列的权重相同
bool NCC_2D_Weight_For_Polar(pool::BitmapImage *left, pool::BitmapImage *right,
							 int du, int dv,
							 int validWidth,
							 float &fScore,
							 const float* pWeightCol = NULL);

// 计算匹配度
// strMethod = "SAD" 或 "NCC"
bool CalMatchScore_BPC(const pool::BitmapImage *left, const pool::BitmapImage *right,
					   int *pSumTable2WidenM, int* pSubTable2T,
					   const int du, const int dv,
					   float &fScore,
					   const float meanLeft, const float stdLeft, 
					   const float meanRight, const float stdRight );

// 计算匹配度
// strMethod = "NCC"
bool CalMatchScore(const pool::BitmapImage *left, const pool::BitmapImage *right,
				   const int du, const int dv,
				   float &fScore,
				   const float meanLeft, const float stdLeft, 
				   const float meanRight, const float stdRight );

// 计算匹配度
// "NCC"
// nDataLeft * 2 = nDataRight
//bool CalMatchScore1D(int* left, int *right, int nDataLeft, int nDataRight, 
//					 const int du,
//					 float &fScore,
//					 const float meanLeft, const float stdLeft, 
//					 const float meanRight, const float stdRight );

// 计算匹配度
// "NCC"
// nDataLeft * 2 = nDataRight
//bool CalMatchScore1D(float* left, float *right, int nDataLeft, int nDataRight, 
//					 const int du,
//					 float &fScore,
//					 const float meanLeft, const float stdLeft, 
//					 const float meanRight, const float stdRight,
//					 float* pWeight = NULL);


// 计算匹配度
// 利用steger的方法
// left-----
// const Gradient32F* pGad1, 归一化的梯度
// const Gradient32F* pGad2, 归一化的梯度
bool CalStegerMatchScore(int wT, int hT, int wsT,
						 int wM, int hM, int wsM, 
						 const pool::Gradient32F* pGad1, const pool::Gradient32F* pGad2, 
						 const int du, const int dv,
						 float &fScore,
						 const int minOverlapPointsNum );


bool Projection1D( pool::BitmapImage* pSrc, int *pProjection, string axis = "X", pool::InterestRegion2 *pROI = NULL );

bool Projection1D( pool::BitmapImage* pSrc, float *pProjection, string axis = "X", pool::InterestRegion2 *pROI = NULL );

// 一维投影
// pWeightCol----每一列的权重
bool Projection1D_Weighted( pool::BitmapImage* pSrc, int *pProjection, string axis /*= "X" */, 
						   float* pWeightCol, pool::InterestRegion2 *pROI = NULL );

// 一维投影
// pWeightCol----每一列的权重
bool Projection1D_Weighted( pool::BitmapImage* pSrc, float *pProjection, string axis /*= "X" */, 
						   float* pWeightCol, pool::InterestRegion2 *pROI = NULL, int shift = 0 );

void CalOverlapRegion(int widthLeft, int heightLeft, 
					  int widthR, int heightR,
					  int dy, int dx,
					  int &xLeftBeg, int &yLeftBeg, int &xLeftEnd, int &yLeftEnd);


bool CalSumTable2( pool::BitmapImage* pSrc, int* pSubTable2 );

bool CalSumTable( pool::BitmapImage* pSrc, int* pSubTable );

bool CalSumTable2(IplImage* pSrc, int* pSubTable2 );

bool CalSumTable(IplImage* pSrc, int* pSubTable );

// 用于space time stereo
bool CalSumTable(IplImage** pSrc, int* pSubTable, int nImages );

// 用于space time stereo
bool CalSumTable2(IplImage** pSrc, int* pSubTable2, int nImages );

bool GetSum( const int* pSumTable, int w, int h, int ws, 
			pool::InterestRegion2 &roi, int &sum );

// 计算图像的增量符号
bool CalIncrementSign(const pool::BitmapImage* pSrc, pool::BitmapImage* pSign);

// 计算图像的增量符号
template<class T1>
bool CalIncrementSign1D(const T1 *pData, int nData, unsigned char* pSign)
{
	if( (pData==NULL) || (pSign==NULL) )
		return false;

	memset(pSign, 0, nData);

	T1 lastVal = pData[0];
	// 先处理第1行

	for(int x=1; x<nData; x++)
	{
		if(pData[x]>=lastVal)
		{
			pSign[x] = 255;
		}
		lastVal = pData[x];
	}
	return true;
}

// 根据增量符号图像，计算屏蔽模板
bool CalBasicMask(const pool::BitmapImage* pSign1, const pool::BitmapImage* pSign2, 
				  pool::BitmapImage* pMask, float &validRatio);

// 根据增量符号图像，计算屏蔽模板
// pSign2的高度是pSign1的高度的2倍
bool CalBasicMask(const pool::BitmapImage* pSign1, const pool::BitmapImage* pSign2, 
				  int dy,
				  pool::BitmapImage* pMask, float &validRatio);

// 根据增量符号图像，计算屏蔽模板
bool CalBasicMask2(const pool::BitmapImage* pSign1, const pool::BitmapImage* pSign2, 
				  pool::BitmapImage* pMask, float &validRatio);