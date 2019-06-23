
//#include "stdafx.h"

#include "Bitmap.h"
#include "memoryPool.h"
#include "usingCV24.h"



bool IsBitmapValid(const pool::BitmapImage *pSrcImage)
{
	if(pSrcImage==NULL)
		return false;
	else if(pSrcImage->imageData==NULL)
		return false;
	else
		return true;
}

bool ZeroImage(pool::BitmapImage *pSrcImage)
{
	if(pSrcImage==NULL)
		return false;

	memset( pSrcImage->imageData, 0, pSrcImage->widthStep*pSrcImage->height );

	return true;
}

// 销毁内存池
void DestroyMemoryPool()
{
	CMemoryPool::Destroy();
}

// 获取CPU个数
int GetCpuNum()
{
	SYSTEM_INFO si;  
	GetSystemInfo(&si);  
	int count = si.dwNumberOfProcessors;  
	return count;
}

// 设置使用期限
int InValidDate(int deadLine_year, int deadLine_month)
{
	SYSTEMTIME sys;    

	GetLocalTime( &sys ); 

	if( sys.wYear<deadLine_year ) 
	{
		return 1;
	}

	if( ( sys.wYear==deadLine_year ) && ( sys.wMonth<=deadLine_month ) )
	{
		return 1; // valid
	}
	else
	{
		return 0; // invalid
	}

	return 1;
}

void SaveBitmap(char* path, pool::BitmapImage* &pDstImage)
{
	IplImage* img = cvCreateImage(cvSize(pDstImage->width, pDstImage->height), 8, pDstImage->nChannels);
	memcpy(img->imageData, pDstImage->imageData, pDstImage->widthStep*pDstImage->height);
	
	cvSaveImage(path, img);

	cvReleaseImage(&img);
}
