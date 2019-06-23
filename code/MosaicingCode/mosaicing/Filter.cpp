//#include "stdafx.h"


#include "ImageFilter.h"
#include "ImageIO.h"
#include "mvMath.h"
#include "MemoryPool.h"

#include "xmmintrin.h"
#include "emmintrin.h"



//int MeanSmooth( IplImage* pSrc, IplImage* pDst,  int radius)
//{
//	if( ( pSrc==NULL ) || (pDst==NULL) )
//		return 0;
//
//	cvSmooth(pSrc, pDst, CV_BLUR, radius, radius);
//	return 1;
//}

// *********************************************************************************
// 函数功能：高斯滤波 带Mask图像
// winSize，高斯窗口的宽度
// 允许pSrc==pDst
// *********************************************************************************
bool GaussFilterMask(const pool::BitmapImage *pSrc, 
					 pool::BitmapImage *pMaskImage,
					 pool::BitmapImage *pDst, const float fSigma, int winSize)
{
	if(pSrc==NULL)
		return false;
	if(pDst==NULL)
		return false;
	if(pMaskImage==NULL)
		return false;

	CMemoryPool memPool;

	const float MIN_SIGMA  = 0.1f; // 
	if(fSigma<MIN_SIGMA)
		return false;

	int w, h, ws;
	GetImageSize( pSrc, w,h, ws );

	// 计算模板尺寸
	const int MAX_WIN_SIZE = 201;
	if(winSize>=MAX_WIN_SIZE)
		winSize = MAX_WIN_SIZE;
	int winR = winSize/2;

	float fInvSigma2 = 1/(fSigma*fSigma);

	float aGray[MAX_WIN_SIZE] = {0}; // 存放

	// 计算模板系数
	float aTemplate[MAX_WIN_SIZE] = {0};
	float fAcc = 0;

	for(int i=-winR; i<=winR; i++)
	{
		for(int j=-winR; j<=winR; j++)
		{
			//aTemplate[i + winR] = exp( -i*i/(2*fSigma*fSigma) ) / fSigma;
			float currentElem = exp( -(i*i+j*j) * 0.5f * fInvSigma2 ) * fInvSigma2;
			aTemplate[(i+winR)*winSize + (j+winR)] = currentElem;

			fAcc += currentElem;
		}
	}
	// 归一化
	float fInvAcc = 1/fAcc;
	for(int i=-winR; i<=winR; i++)
	{
		for(int j=-winR; j<=winR; j++)
		{
			aTemplate[(i+winR)*winSize + (j+winR)] *= fInvAcc;
		}
	}

	int w_1 = w - 1;
	int h_1 = h - 1;
	int w2 = w * 2;
	int h2 = h * 2;

	for(int row = 0; row<h; row++)
	{
		unsigned char* pSrcRow = pSrc->imageData + row*ws;
		unsigned char* pDstRow = pDst->imageData + row*ws;
		unsigned char* pMaskRow = pMaskImage->imageData + row*ws;
		for(int col = 0; col<w; col++)
		{
			if(pMaskRow[col]<=0)
				continue;
			if((col==555)&&(row==733))
				col = col;
			
			float res = 0;
			for(int i=-winR; i<=winR; i++)
			{
				int y = i + row;

				if(y<0)
					y = -y;
				else if(y>h_1)
					y = h2 - y - 2;

				unsigned char *ptr = pSrc->imageData + y*ws; // 指向图像
				for(int j=-winR; j<=winR; j++)
				{
					int x = j+col;
					if(x<0)
						x = -x;
					else if(x>w_1)
						x = w2 - x -2;
					res  += (unsigned char)ptr[x] * (*(aTemplate + (i+winR)*winSize + j+winR));
				}
			}
			pDstRow[col] = (unsigned char)res;
		}
	}
	
	return true;
}

// 模板卷积运算, 任意尺寸 R>=3 
float TemplateConvolveAny( pool::BitmapImage *src, 
						  float *pTemplate, 
						  const int r, 
						  const pool::Point &pt )
{
	if( src==NULL )
	{
		return 0;
	}

	int w = 0, h = 0, ws = 0;
	w = src->width;
	h = src->height;
	ws = src->widthStep;

	int h_1 = h - 1;
	int w_1 = w - 1;

	int R = 2*r + 1;

	int w2 = w * 2;
	int h2 = h * 2;

	float res = 0.f;

	for(int i=-r; i<=r; i++)
	{
		int y = i+pt.y;

		if(y<0)
			y = -y;
		else if(y>h_1)
			y = h2 - y - 2;
	
		unsigned char *ptr=src->imageData + (i+pt.y)*ws; // 指向图像
		for(int j=-r; j<=r; j++)
		{
			int x = j+pt.x;
			if(x<0)
				x = -x;
			else if(x>w_1)
				x = w2 - x -2;
			res  += (unsigned char)ptr[j+pt.x] * (*(pTemplate + (i+r)*R + j+r));
		}
	}

	return res;
}


// *********************************************************************************
// 函数功能：高斯滤波
// winSize，高斯窗口的宽度
// *********************************************************************************
bool GaussFilter2( const unsigned char *pSrcData, const int wSrc, const int hSrc, const int wsSrc, 
					    unsigned char *pDstData,
						const float fSigma, const int winSize)
{
	if(pSrcData==NULL)
		return false;
	if(pDstData==NULL)
		return false;

	pool::BitmapImage srcImage;
	srcImage.imageData = (unsigned char*)pSrcData;
	srcImage.width = wSrc;
	srcImage.height = hSrc;
	srcImage.widthStep = wsSrc;
	srcImage.nChannels = 1;

	pool::BitmapImage dstImage;
	dstImage.imageData = pDstData;
	dstImage.width = wSrc;
	dstImage.height = hSrc;
	dstImage.widthStep = wsSrc;
	dstImage.nChannels = 1;

	return GaussFilter_Int( &srcImage, &dstImage, fSigma, winSize);
	//return GaussFilter( &srcImage, &dstImage, fSigma, winSize);
}

int GaussSmoothData1D(float* srcData, float* dstData, int N, float sigma, int winR)
{
	if( (srcData==NULL) || (dstData==NULL) )
	{
		return -1;
	}

	int winSize = winR*2 + 1;

	//CMemoryPool memPool;

	const float MIN_SIGMA  = 0.001f; // 
	if(sigma<MIN_SIGMA)
		sigma = MIN_SIGMA;

	int w = N, h = 1, ws = N;

	// 计算模板尺寸
	const int MAX_WIN_SIZE = 201;
	if(winSize>=MAX_WIN_SIZE)
		winSize = MAX_WIN_SIZE;
	//winR = winSize/2;

	float fInvSigma = 1/sigma;

	float aGray[MAX_WIN_SIZE] = {0}; // 存放

	// 计算模板系数
	float aTemplate[MAX_WIN_SIZE] = {0};
	float fAcc = 0;
	for(int i=-winR; i<=winR; i++)
	{
		aTemplate[i + winR] = exp( -i*i*0.5f*fInvSigma*fInvSigma ) * fInvSigma;
		fAcc += aTemplate[i + winR];
	}

	// 归一化
	float fInvAcc = 1/fAcc;
	for(int i=0; i<winSize; i++)
	{
		//aTemplate[i] /= fAcc;
		aTemplate[i] *= fInvAcc;
	}

	int w_1 = w - 1;
	int h_1 = h - 1;
	int w2 = w * 2;
	int h2 = h * 2;

	memset( dstData, 0, w*h*sizeof(float));

	int w_winR = w - winR;
	int h_winR = h - winR;

	for(int j=0; j<w; j++)
	{
		float fAccGray = 0;
		for( int m=-winR; m<=winR; m++ )
		{
			int x = j + m;
			if(x<0)
				x = 0;
			else if(x>w_1)
				x = w_1;

			fAccGray += aTemplate[m+winR] * srcData[x];
		}

		dstData[j] = fAccGray;
	}

	return 0;
}


// *********************************************************************************
// 函数功能：高斯滤波
// winSize，高斯窗口的宽度
// 允许pSrc==pDst
// *********************************************************************************
bool GaussFilter(pool::BitmapImage *pSrc, pool::BitmapImage *pDst, const float fSigma, int winSize)
{
	if(pSrc==NULL)
		return false;
	if(pDst==NULL)
		return false;

	CMemoryPool memPool;

	const float MIN_SIGMA  = 0.1f; // 
	if(fSigma<MIN_SIGMA)
		return false;

	int w, h, ws;
	GetImageSize( pSrc, w,h, ws );

	// 计算模板尺寸
	const int MAX_WIN_SIZE = 201;
	if(winSize>=MAX_WIN_SIZE)
		winSize = MAX_WIN_SIZE;
	int winR = winSize/2;

	float fInvSigma = 1/fSigma;

	float aGray[MAX_WIN_SIZE] = {0}; // 存放

	// 计算模板系数
	float aTemplate[MAX_WIN_SIZE] = {0};
	float fAcc = 0;
	for(int i=-winR; i<=winR; i++)
	{
		//aTemplate[i + winR] = exp( -i*i/(2*fSigma*fSigma) ) / fSigma;
		aTemplate[i + winR] = exp( -i*i*0.5f*fInvSigma*fInvSigma ) * fInvSigma;
		fAcc += aTemplate[i + winR];
	}
	// 归一化
	float fInvAcc = 1/fAcc;
	for(int i=0; i<winSize; i++)
	{
		//aTemplate[i] /= fAcc;
		aTemplate[i] *= fInvAcc;
	}
	
	int w_1 = w - 1;
	int h_1 = h - 1;
	int w2 = w * 2;
	int h2 = h * 2;

	unsigned char * pDstImageData = pDst->imageData;
	
	// 横向滤波
	//float* pFilterData = new. float[ws*h];
	float* pFilterData = (float*)memPool.Operate( NULL, ws*h*sizeof(float) );

	memset( pFilterData, 0, w*h );
	int w_winR = w - winR;
	int h_winR = h - winR;
	
	for(int i=0; i<h; i++)
	{
		unsigned char* pRow = pSrc->imageData + i*ws;
		float* pRowFiltered = pFilterData + i * ws;
		for(int j=winR; j<w_winR; j++)
		{
			float fAccGray = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int x = j + m;
				if(x<0)
					x = -x;
				else if(x>w_1)
					x = w2 - x -2;

				fAccGray += aTemplate[m+winR] * pRow[x];
			}
			pRowFiltered[j] = fAccGray;
		}
	}
	
	// 纵向滤波
	for(int j=0; j<w; j++)
	{
		for(int i=0; i<h; i++)
		{
			float fAccGray = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int y = i + m;
				if(y<0)
					y = -y;
				else if(y>h_1)
					y = h2 - y - 2;

				fAccGray += aTemplate[m+winR] * *(pFilterData + y*ws + j);
			}
			//int grayResult = 0;
			//RangeCut( fAccGray+0.5f, 0, 255, grayResult );
			
			*(pDstImageData + i*ws + j) = (unsigned char)(fAccGray + 0.5f);
			//*(pDstImageData + i*ws + j) = (unsigned char)(fAccGray);
			//*(pDstImageData + i*ws + j) = fAccGray;
			//*(pDstImageData + i*ws + j) = 255;
		}
	}
	
	// 释放内存
	//delete.[] pFilterData; pFilterData = NULL;
	memPool.Operate( pFilterData ); pFilterData = NULL;

	return true;
}

// *********************************************************************************
// 均值滤波
// *********************************************************************************
bool MeanFilterSSE(const pool::BitmapImage *pSrc, pool::BitmapImage *pDst, int winSize)
{
	const int MAX_WIN_SIZE = 200;
	if(pSrc==NULL)
		return false;
	if(pDst==NULL)
		return false;
	if(winSize%2==0) // 必须为奇数
		return false;
	if(winSize>MAX_WIN_SIZE)
		return false;

	int w = 0, h = 0, ws = 0;
	GetImageSize( pSrc, w, h, ws );

	unsigned char* pSrcData8U = pSrc->imageData;
	unsigned char* pDstData8U = pDst->imageData;

	int ws8 = pSrc->widthStep;
	int ws16 = ws8 * 2;
	ushort ws128 = (ws16+15)/16*16; // byte
	ushort w16 = ws128>>1;
	ushort w128 = ws128>>4; 

	CMemoryPool memPool;
	
	//__m128i *pBufferSrc128 = new. __m128i[w128*h];
	__m128i *pBufferSrc1280 = (__m128i*)memPool.Operate( NULL, w128*h*sizeof(__m128i) + 16 ) ;
	//__m128i *pBufferDst128 = new. __m128i[w128*h];
	__m128i *pBufferDst1280 = (__m128i*)memPool.Operate( NULL, w128*h*sizeof(__m128i) + 16 ) ;

	__m128i *pBufferSrc128 = (__m128i *)( ((int)pBufferSrc1280 + 15)/16*16 );
	__m128i *pBufferDst128 = (__m128i *)( ((int)pBufferDst1280 + 15)/16*16 );

	memset(pBufferDst128, 0, ws128*h );

	ushort* pSrc16 = (ushort*)pBufferSrc128;
	ushort* pDst16 = (ushort*)pBufferDst128;

	// 把8位的数据转成32位的数据
	for(int y=0; y<h; y++)
	{
		unsigned char* pRow8 = pSrcData8U + y*ws8;
		ushort* pRow16 = pSrc16 + y*w16;
		for(int x=0; x<w; x++)
		{
			pRow16[x] = (ushort)pRow8[x];
		}
	}
	float fCoefficient = 1.0f/(winSize*winSize);

	int winR = winSize/2;
	// 
	int w_winR = w - winR;
	int h_winR = h - winR;

	//__m128i* pBufferRow128 = new. __m128i[w128];

	// 纵向滤波
	// 初始化
	for(int y= winR; y<h_winR; y++)
	{
		for(int x=0; x<w128; x++)
		{
			__m128i m128 = _mm_setzero_si128(); // 置0
			__m128i* pCenterDst = pBufferDst128 + y*w128;

			for(int m=-winR; m<=winR; m++)
			{
				__m128i* pCurrentSrc = pBufferSrc128 + (y+m)*w128 + x;
				m128 = _mm_add_epi32(m128, *pCurrentSrc);
			}
			*(pCenterDst + x) = m128;
		}
	}
	
	// 横向滤波
	for(int i=winR; i<h_winR; i++)
	{
		unsigned char* pDstRow = pDstData8U + i * ws8;
		ushort* pRow16 = pDst16 + i * w16;

		// 初始化
		ushort accGray16U = 0;
		int j = winR;
		for( int m=-winR; m<=winR; m++ )
		{
			int x = j + m;
			accGray16U += pRow16[x];
		}
		ushort previousFirst = pRow16[0];
		ushort currentFirst = 0;
		ushort currentTail = 0;

		pDstRow[j] = (unsigned char)(accGray16U*fCoefficient);

		// loop 
		for(int j=winR+1; j<w_winR; j++)
		{
			currentTail = pRow16[j+winR];
			accGray16U = accGray16U - previousFirst + currentTail;

			pDstRow[j] = (unsigned char)(accGray16U * fCoefficient);

			previousFirst = pRow16[j-winR];
		}
	}
	
	// 先纵向滤波
	//delete[] pBufferSrc128;
	//delete[] pBufferDst128;
	memPool.Operate(pBufferSrc1280);
	memPool.Operate(pBufferDst1280);
	//delete[] pBufferRow128;
	
	return true;
}

// *********************************************************************************
// 均值滤波
// *********************************************************************************
bool MeanFilter(pool::BitmapImage *pSrc, pool::BitmapImage *pDst, int winSize)
{
	if(pSrc==NULL)
		return false;
	if(pDst==NULL)
		return false;
	if(winSize%2==0) // 必须为奇数
		return false;

	int winR = winSize/2;

	float fCoefficient = 1.0f/(winSize*winSize);

	int w = 0, h = 0, ws = 0;
	GetImageSize( pSrc, w, h, ws );

	int w_1 = w - 1;
	int h_1 = h - 1;
	int w2 = w * 2;
	int h2 = h * 2;

	unsigned char * pDstImageData = pDst->imageData;

	// 一、横向滤波
	CMemoryPool memPool;
	unsigned long* pFilterData = (unsigned long*)memPool.Operate( NULL, ws*h*sizeof(unsigned long) );

	memset( pFilterData, 0, sizeof(unsigned long)*ws*h );
	int w_winR = w - winR;
	int h_winR = h - winR;
	
	for(int i=winR; i<h_winR; i++)
	{
		unsigned char* pRow = pSrc->imageData + i * ws;
		unsigned long* pRowFiltered = pFilterData + i * ws;
		// 初始化
		unsigned long accGray32U = 0;
		int j = winR;
		for( int m=-winR; m<=winR; m++ )
		{
			int x = j + m;
			accGray32U += pRow[x];
		}
		ulong previousFirst = pRow[0];
		ulong currentFirst = 0;
		ulong currentTail = 0;
		pRowFiltered[j] = accGray32U;
		// loop
		for(int j=winR+1; j<w_winR; j++)
		{
			currentTail = pRow[j+winR];
			accGray32U = accGray32U - previousFirst + currentTail;
			pRowFiltered[j] = accGray32U;

			previousFirst = pRow[j-winR];
		}
	}
	// 上边
	for(int i=0; i<winR; i++)
	{
		unsigned char* pRow = pSrc->imageData + i*ws;
		unsigned long* pRowFiltered = pFilterData + i * ws;
		for(int j=0; j<w; j++)
		{
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int x = j + m;
				// 边界判断
				if(x<0)
					x = -x;
				else if(x>w_1)
					x = w2 - x -2;
				accGray32U += pRow[x];
			}
			pRowFiltered[j] = accGray32U;
		}
	}
	// 下边
	for(int i=h_winR; i<h; i++)
	{
		unsigned char* pRow = pSrc->imageData + i*ws;
		unsigned long* pRowFiltered = pFilterData + i * ws;
		for(int j=0; j<w; j++)
		{
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int x = j + m;
				// 边界判断
				if(x<0)
					x = -x;
				else if(x>w_1)
					x = w2 - x -2;
				accGray32U += pRow[x];
			}
			pRowFiltered[j] = accGray32U;
		}
	}
	// 左边
	for(int i=0; i<h; i++)
	{
		unsigned char* pRow = pSrc->imageData + i*ws;
		unsigned long* pRowFiltered = pFilterData + i * ws;
		for(int j=0; j<winR; j++)
		{
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int x = j + m;
				// 边界判断
				if(x<0)
					x = -x;
				else if(x>w_1)
					x = w2 - x -2;
				accGray32U += pRow[x];
			}
			pRowFiltered[j] = accGray32U;
		}
	}
	// 右边
	for(int i=0; i<h; i++)
	{
		unsigned char* pRow = pSrc->imageData + i*ws;
		unsigned long* pRowFiltered = pFilterData + i * ws;
		for(int j=w_winR; j<w; j++)
		{
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int x = j + m;
				// 边界判断
				if(x<0)
					x = -x;
				else if(x>w_1)
					x = w2 - x -2;
				accGray32U += pRow[x];
			}
			pRowFiltered[j] = accGray32U;
		}
	}

	//unsigned long * pBuffer = new. unsigned long[w*h];
	//unsigned long * pBuffer2 = pBuffer;
	
	// 
	// 二、纵向滤波
	for(int i=winR; i<h_winR; i++)
	{
		//int i_by_ws = i*ws;
		unsigned char* pRow = pDstImageData + i*ws;
		for(int j=winR; j<w_winR; j++)
		{
			//unsigned char* pCol = pDstImageData + j;
			unsigned long* pFilterDataTemp = pFilterData+ j;
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int y = i + m;
				accGray32U += *( pFilterDataTemp + y*ws );
			}
			unsigned char dstGray = (unsigned char)(accGray32U*fCoefficient);
			pRow[j] = dstGray;
		}
	}
	
	// 第2次处理边界
	// 上边
	for(int i=0; i<winR; i++)
	{
		for(int j=0; j<w; j++)
		{
			unsigned char* pCol = pDstImageData + j;
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int y = i + m;
				// 边界判断
				if(y<0)
					y = -y;
				else if(y>h_1)
					y = h2 - y - 2;
				accGray32U += *(pFilterData + y*ws + j);
			}
			unsigned char dstGray = (unsigned char)(accGray32U*fCoefficient);
			*(pCol + i*ws) = dstGray;
		}
	}
	// 下边
	for(int i=h_winR; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			unsigned char* pCol = pDstImageData + j;
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int y = i + m;
				// 边界判断
				if(y<0)
					y = -y;
				else if(y>h_1)
					y = h2 - y - 2;
				accGray32U += *(pFilterData + y*ws + j);
			}
			unsigned char dstGray = (unsigned char)(accGray32U*fCoefficient);
			*(pCol + i*ws) = dstGray;
		}
	}
	// 左边
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<winR; j++)
		{
			unsigned char* pCol = pDstImageData + j;
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int y = i + m;
				// 边界判断
				if(y<0)
					y = -y;
				else if(y>h_1)
					y = h2 - y - 2;
				accGray32U += *(pFilterData + y*ws + j);
			}
			unsigned char dstGray = (unsigned char)(accGray32U*fCoefficient);
			*(pCol + i*ws) = dstGray;
		}
	}
	// 右边
	for(int i=0; i<h; i++)
	{
		for(int j=w_winR; j<w; j++)
		{
			unsigned char* pCol = pDstImageData + j;
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int y = i + m;
				// 边界判断
				if(y<0)
					y = -y;
				else if(y>h_1)
					y = h2 - y - 2;
				accGray32U += *(pFilterData + y*ws + j);
			}
			unsigned char dstGray = (unsigned char)(accGray32U*fCoefficient);
			*(pCol + i*ws) = dstGray;
		}
	}

	// 释放内存
	//delete.[] pFilterData; pFilterData = NULL;
	memPool.Operate(pFilterData); pFilterData = NULL;
	
	return true;
}

// *********************************************************************************
// 函数功能：高斯滤波
// winSize，高斯窗口的宽度
// *********************************************************************************
bool GaussFilter_Int(pool::BitmapImage *pSrc, pool::BitmapImage *pDst, float fSigma, int winSize)
{
	if(pSrc==NULL)
		return false;
	if(pDst==NULL)
		return false;

	const float MIN_SIGMA  = 0.01f; // 
	if(fSigma<MIN_SIGMA)
		return true;

	int w, h, ws;
	GetImageSize( pSrc, w,h, ws );

	// 计算模板尺寸
	const int MAX_WIN_SIZE = 401;
	if(winSize>=MAX_WIN_SIZE)
		winSize = MAX_WIN_SIZE;
	int winR = winSize/2;

	float fInvSigma = 1/fSigma;

	float aGray[MAX_WIN_SIZE] = {0}; // 存放

	// 计算模板系数
	float aTemplate[MAX_WIN_SIZE] = {0};
	unsigned long aTemplate32U[MAX_WIN_SIZE] = {0};
	float fAcc = 0;
	for(int i=-winR; i<=winR; i++)
	{
		//aTemplate[i + winR] = exp( -i*i/(2*fSigma*fSigma) ) / fSigma;
		aTemplate[i + winR] = exp( -i*i*0.5f*fInvSigma*fInvSigma ) * fInvSigma;
		fAcc += aTemplate[i + winR];
	}
	// 归一化
	float fInvAcc = 1/fAcc;
	for(int i=0; i<winSize; i++)
	{
		//aTemplate[i] /= fAcc;
		aTemplate[i] *= fInvAcc;
	}

	// 把系数放大，变成整数
	unsigned long shiftStep = 12;
	unsigned long shiftStep2 = shiftStep * 2;

	unsigned long one10 = 1<<shiftStep; // 左移
	for(int i=0; i<winSize; i++)
	{
		aTemplate32U[i] = (unsigned long)(aTemplate[i] * one10 + 0.5f);
	}

	int w_1 = w - 1;
	int h_1 = h - 1;
	int w2 = w * 2;
	int h2 = h * 2;

	unsigned char * pDstImageData = pDst->imageData;

	// 一、横向滤波
	CMemoryPool memPool;
	unsigned long* pFilterData = (unsigned long*)memPool.Operate( NULL, ws*h*sizeof(unsigned long) );

	memset( pFilterData, 0, sizeof(unsigned long)*ws*h );
	int w_winR = w - winR;
	int h_winR = h - winR;

	for(int i=winR; i<h_winR; i++)
	{
		unsigned char* pRow = pSrc->imageData + i * ws;
		unsigned long* pRowFiltered = pFilterData + i * ws;
		for(int j=winR; j<w_winR; j++)
		{
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int x = j + m;
				accGray32U += aTemplate32U[m+winR] * pRow[x];
			}
			pRowFiltered[j] = accGray32U;
		}
	}

	// 第1次处理边界
	// 上边
	for(int i=0; i<winR; i++)
	{
		unsigned char* pRow = pSrc->imageData + i*ws;
		unsigned long* pRowFiltered = pFilterData + i * ws;
		for(int j=0; j<w; j++)
		{
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int x = j + m;
				// 边界判断
				if(x<0)
					x = -x;
				else if(x>w_1)
					x = w2 - x -2;
				accGray32U += aTemplate32U[m+winR] * pRow[x];
			}
			pRowFiltered[j] = accGray32U;
		}
	}
	// 下边
	for(int i=h_winR; i<h; i++)
	{
		unsigned char* pRow = pSrc->imageData + i*ws;
		unsigned long* pRowFiltered = pFilterData + i * ws;
		for(int j=0; j<w; j++)
		{
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int x = j + m;
				// 边界判断
				if(x<0)
					x = -x;
				else if(x>w_1)
					x = w2 - x -2;
				accGray32U += aTemplate32U[m+winR] * pRow[x];
			}
			pRowFiltered[j] = accGray32U;
		}
	}
	// 左边
	for(int i=0; i<h; i++)
	{
		unsigned char* pRow = pSrc->imageData + i*ws;
		unsigned long* pRowFiltered = pFilterData + i * ws;
		for(int j=0; j<winR; j++)
		{
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int x = j + m;
				// 边界判断
				if(x<0)
					x = -x;
				else if(x>w_1)
					x = w2 - x -2;
				accGray32U += aTemplate32U[m+winR] * pRow[x];
			}
			pRowFiltered[j] = accGray32U;
		}
	}
	// 右边
	for(int i=0; i<h; i++)
	{
		unsigned char* pRow = pSrc->imageData + i*ws;
		unsigned long* pRowFiltered = pFilterData + i * ws;
		for(int j=w_winR; j<w; j++)
		{
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int x = j + m;
				// 边界判断
				if(x<0)
					x = -x;
				else if(x>w_1)
					x = w2 - x -2;
				accGray32U += aTemplate32U[m+winR] * pRow[x];
			}
			pRowFiltered[j] = accGray32U;
		}
	}

	// 二、纵向滤波
	for(int i=winR; i<h_winR; i++)
	{
		int i_by_ws = i*ws;
		for(int j=winR; j<w_winR; j++)
		{
			unsigned char* pCol = pDstImageData + j;
			unsigned long* pFilterDataTemp = pFilterData+ j;
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int y = i + m;
				accGray32U += aTemplate32U[m+winR] * *( pFilterDataTemp + y*ws );
			}
			unsigned char dstGray = (unsigned char)(accGray32U>>shiftStep2);
			*(pCol + i_by_ws) = dstGray;
		}
	}
	// 第2次处理边界
	// 上边
	for(int i=0; i<winR; i++)
	{
		for(int j=0; j<w; j++)
		{
			unsigned char* pCol = pDstImageData + j;
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int y = i + m;
				// 边界判断
				if(y<0)
					y = -y;
				else if(y>h_1)
					y = h2 - y - 2;
				accGray32U += aTemplate32U[m+winR] * *(pFilterData + y*ws + j);
			}
			unsigned char dstGray = (unsigned char)(accGray32U>>shiftStep2);
			*(pCol + i*ws) = dstGray;
		}
	}
	// 下边
	for(int i=h_winR; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			unsigned char* pCol = pDstImageData + j;
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int y = i + m;
				// 边界判断
				if(y<0)
					y = -y;
				else if(y>h_1)
					y = h2 - y - 2;
				accGray32U += aTemplate32U[m+winR] * *(pFilterData + y*ws + j);
			}
			unsigned char dstGray = (unsigned char)(accGray32U>>shiftStep2);
			*(pCol + i*ws) = dstGray;
		}
	}
	// 左边
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<winR; j++)
		{
			unsigned char* pCol = pDstImageData + j;
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int y = i + m;
				// 边界判断
				if(y<0)
					y = -y;
				else if(y>h_1)
					y = h2 - y - 2;
				accGray32U += aTemplate32U[m+winR] * *(pFilterData + y*ws + j);
			}
			unsigned char dstGray = (unsigned char)(accGray32U>>shiftStep2);
			*(pCol + i*ws) = dstGray;
		}
	}
	// 右边
	for(int i=0; i<h; i++)
	{
		for(int j=w_winR; j<w; j++)
		{
			unsigned char* pCol = pDstImageData + j;
			unsigned long accGray32U = 0;
			for( int m=-winR; m<=winR; m++ )
			{
				int y = i + m;
				// 边界判断
				if(y<0)
					y = -y;
				else if(y>h_1)
					y = h2 - y - 2;
				accGray32U += aTemplate32U[m+winR] * *(pFilterData + y*ws + j);
			}
			unsigned char dstGray = (unsigned char)(accGray32U>>shiftStep2);
			*(pCol + i*ws) = dstGray;
		}
	}

	// 释放内存
	//delete.[] pFilterData; pFilterData = NULL;
	memPool.Operate(pFilterData); pFilterData = NULL;

	return true;
}

//
//// 均值滤波
//bool MeanFilter(pool::BitmapImage *pSrc, pool::BitmapImage *pDst, const int siFilterR)
//{
//	if(pSrc==NULL)
//		return false;
//	if(pDst==NULL)
//		return false;
//
//	
//
//	return true;
//}


// 中值滤波
bool MedianFilter(pool::BitmapImage *pSrc, pool::BitmapImage *pDst, const int siFilterR)
{
	if(pSrc==NULL)
		return false;
	if(pDst==NULL)
		return false;

	

	return true;
}