#include "stdafx.h"
//#include "afxwin.h"
#include "Mosaic1By1.h"
#include "MosaicImage.h"
#include <vector>
#include <list>
//#include "harriscorner.h"
#include "Matrix.h"
//#include "usingAtlImage.h"
#include "Lock.h"
#include "MosaicWithoutPos.h"
#include "opencv2\opencv.hpp"

//#include "stitcher.hpp"
//#include <opencv2/ts/ts.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/stitching/stitcher.hpp>

//#include <afxstat_.h>

//struct MosaicImageInfo
//{
//	IplImage* pSrc;
//	ProjectMat T;
//	int success;
//	int emergency;
//	MosaicImageInfo()
//	{
//		pSrc = NULL;
//		success = 0;
//		emergency = 0;
//	}
//};

static BitmapImage* pResultImage = NULL;

static SfPoint* gpInnerPoints1 = NULL;
static SfPoint* gpInnerPoints2 = NULL;
static int nInnerPoints = 0;

static list<int> listEmergency;

static CornerDescriptor* gpCornerDescriptor1 = NULL;
static CornerDescriptor* gpCornerDescriptor2 = NULL;

// 当前送进来的图像数
static int gCurrentImagesNum = 0;
static unsigned int gTotalImagesNum = 0; // 已经送进来的图像的总数
static list<MosaicImageInfo> listMosaicInfo;

static list<MosaicImageInfo> listImagesPool;

static bool isFirstRun = true;

// RD=0时为读, RD=1时为写
// 读图像从listImagesPool的前面读
// 加入新图像从listImagesPool的后面添加
IplImage* ManageImagesPool(const IplImage* pSrc, int &emergency, int RW)
{
	// 加锁
	CMutexLock lock;
	lock.Lock();

	if(0==RW) // 读
	{
		if(!listImagesPool.empty())
		{
			IplImage* dst = NULL;
			list<MosaicImageInfo>::iterator iter = listImagesPool.begin();
			dst = (*iter).pSrc;
			emergency = (*iter).emergency;
			listImagesPool.pop_front(); // 弹出
			return dst;
		}
	}
	else if(1==RW) // 向listImagesPool加入新图像
	{
		if(pSrc)
		{
			MosaicImageInfo temp;
			temp.pSrc = cvCloneImage(pSrc);
			temp.emergency = emergency;
			temp.success = 0;
			listImagesPool.push_back(temp); // 加入
		}
	}

	// 解锁
	lock.Unlock();

	return NULL;
}

void CvImage2BitmapImage(IplImage* pSrc, BitmapImage* pDst)
{
	if((pSrc==NULL) || (pDst==NULL))
		return;
	pDst->imageData = (unsigned char*)pSrc->imageData;
	pDst->width = pSrc->width;
	pDst->height = pSrc->height;
	pDst->widthStep = pSrc->widthStep;
	pDst->nChannels = pSrc->nChannels;
}

int PushImage(MosaicImageInfo info, int imagesNum)
{
	if(gTotalImagesNum<imagesNum)
	{
		listMosaicInfo.push_back(info);
	}
	else
	{
		list<MosaicImageInfo>::iterator iter;
		iter = listMosaicInfo.begin();
		cvReleaseImage( &(*iter).pSrc ); // 先释放内存
		
		listMosaicInfo.pop_front();
		listMosaicInfo.push_back(info);
	}
	
	return 0;
}

// 图像拼接线程函数
DWORD WINAPI MosaicImagesThread(LPVOID pThreadParam)
{
	if(NULL==pThreadParam)
		return -1;

	CMosaic* pMosaic = (CMosaic*)pThreadParam;

	while(true)
	{
		IplImage* pSrc = NULL;
		IplImage* pMosaicResult = NULL;
		int emergency = 0;
		int RW = 0; // 读图片
		pSrc = ManageImagesPool(NULL, emergency, RW);

		//pMosaic->Mosaic(pSrc, pMosaicResult, bEergency);
		pMosaic->MosaicDynamic(pSrc);
		
		cvReleaseImage(&pSrc);

		//cout<<pMosaic->EndDynamicMosaic()<<endl;
		if(pMosaic->EndDynamicMosaic()==true)
		{
			//cout<<"\a";
			//break;
			//::AfxEndThread(0);
			return 0;
		}
	}

	return 0;
}

// 异步方式的图像拼接
int CMosaic::MosaicAsynchronous(const IplImage* pSrc)
{
	int RW = 1; // 添加新图片

	int emergency = 0;
	ManageImagesPool(pSrc, emergency, RW);
	
	if(isFirstRun)
	{
		isFirstRun = false;

		// 开图像拼接线程
		//CWinThread *pPrsThread = ::AfxBeginThread(MosaicImagesThread, this);
		CreateThread(NULL, 0, MosaicImagesThread, NULL, 0, NULL);
	}

	return 0;
}

void CMosaic::ClearPool()
{
	UpdateResult(false);
	cvZero(m_pMosaicResult);
	m_bEndDynamicMosaic = false;
	gTotalImagesNum = 0;
	gCurrentImagesNum = 0;
	isFirstRun = true;
	list<MosaicImageInfo>::iterator iter;
	iter = listImagesPool.begin();
	for(int i=0; i<(int)listImagesPool.size(); i++)
	{
		cvReleaseImage( &(*iter).pSrc );
		iter++;
	}
	listImagesPool.clear();
	listMosaicInfo.clear();
}

// 拼接, 动画效果
int CMosaic::MosaicDynamic(const IplImage* pSrc0)
{

	return 0;
}

// 功能：生成动态拼接图
int CMosaic::GenerateDynamicPana(const IplImage* pImages[], int imagesNum, ProjectMat* pImgT)
{
	if(pImages==NULL)
		return -1;
	if(m_pMosaicResult==NULL)
		return -1;
	if(imagesNum<=0)
		return -1;

	IntPoint leftUpPtNow;

	cvZero(m_pMosaicResult);

	int wRes, hRes, wsRes;
	wRes = m_pMosaicResult->width; hRes = m_pMosaicResult->height; wsRes = m_pMosaicResult->widthStep;
	int wSrc = pImages[0]->width;
	int hSrc = pImages[0]->height;
	//IntPoint leftUpPt; // class member

	// 确定新图像的大小
	float maxX = 0, maxY = 0, minX = 0, minY = 0;		
	SfPoint* pBegBox = new SfPoint[imagesNum+4], *pEndBox = new SfPoint[imagesNum+4];
	SfPoint* pQuadrangleCorners = new SfPoint[imagesNum*4]; // 4边形的顶点
	SfPoint corner[4];
	for(int n=0; n<imagesNum; n++)
	{
		// 对图像的各个顶点进行变换
		corner[0].x = 0;
		corner[0].y = 0;
		corner[1].x = float(pImages[n]->width-1);
		corner[1].y = 0;
		corner[2].x = float(pImages[n]->width-1);
		corner[2].y = float(pImages[n]->height-1);
		corner[3].x = 0; 
		corner[3].y = float(pImages[n]->height-1);
		float boxMaxX = -1<<29, boxMaxY = -1<<29, boxMinX = 1<<29, boxMinY = 1<<29;
		for(int i=0; i<4; i++)
		{
			float xSrc = corner[i].x, ySrc = corner[i].y;
			float xDst, yDst;
			float* pM = pImgT[n].m;
			xDst = (xSrc*pM[0] + ySrc*pM[1] + pM[2]) / (xSrc*pM[6] + ySrc*pM[7] + pM[8]);
			yDst = (xSrc*pM[3] + ySrc*pM[4] + pM[5]) / (xSrc*pM[6] + ySrc*pM[7] + pM[8]);
			pQuadrangleCorners[n*4+i].x = xDst;
			pQuadrangleCorners[n*4+i].y = yDst;

			if(xDst>maxX)
				maxX = xDst;
			if(xDst<minX)
				minX = xDst;
			if(yDst>maxY)
				maxY = yDst;
			if(yDst<minY)
				minY = yDst;

			if(xDst>boxMaxX)
				boxMaxX = xDst;
			if(xDst<boxMinX)
				boxMinX = xDst;
			if(yDst>boxMaxY)
				boxMaxY = yDst;
			if(yDst<boxMinY)
				boxMinY = yDst;
		}
		pBegBox[n].x = boxMinX; pBegBox[n].y = boxMinY;
		pEndBox[n].x = boxMaxX; pEndBox[n].y = boxMaxY;
	}

	if(imagesNum==1)
	{
		// 如果只有一张图像, 则放中间
		leftUpPtNow.x = wRes/2 - wSrc/2;
		leftUpPtNow.y = hRes/2 - hSrc/2;
	}
	else
	{
		float dXNow_Last = pBegBox[imagesNum-1].x - pBegBox[imagesNum-2].x;
		float dYNow_Last = pBegBox[imagesNum-1].y - pBegBox[imagesNum-2].y;
		

		leftUpPtNow.x = Max(leftUpPt.x + dXNow_Last, 0);
		leftUpPtNow.x = Min(leftUpPtNow.x, wRes-wSrc);

		leftUpPtNow.y = Max(leftUpPt.y + dYNow_Last, 0);
		leftUpPtNow.y = Min(leftUpPtNow.y, hRes-hSrc);
	}
	float dX = leftUpPtNow.x - pBegBox[imagesNum-1].x;
	float dY = leftUpPtNow.y - pBegBox[imagesNum-1].y;

	for(int n=0; n<imagesNum; n++)
	{
		// 判断变换后的图像是否与m_pDynamicPana有交集
		if( ( ((pBegBox[n].x+dX)>wRes-1) && ((pEndBox[n].x+dX)<0) ) || 
			( ((pBegBox[n].y+dY)>hRes-1) && ((pEndBox[n].y+dY)<0) ) )
		{
			// 没有交集
			continue;
		}
		else
		{
			// 注册图像
			for(int n=0; n<imagesNum; n++)
			{
				int wSrc, hSrc, wsSrc;
				wSrc = pImages[n]->width; hSrc = pImages[n]->height; wsSrc = pImages[n]->widthStep;

				// 双线性插值版本dst -> src
				int wSrc_1 = pImages[n]->width-1;
				int hSrc_1 = pImages[n]->height-1;
				float invH[9];
				InverseMatrix(pImgT[n].m, 3, invH, 1e-12f);
				SfPoint corner[4], cornerBar[4];
				corner[0].x = 0;									corner[0].y = 0;
				corner[1].x = (float)(pImages[n]->width-1);			corner[1].y = 0;
				corner[2].x = (float)(pImages[n]->width-1);			corner[2].y = (float)(pImages[n]->height-1);
				corner[3].x = 0;									corner[3].y = (float)(pImages[n]->height-1);
				float minX = 1<<29, minY = 1<<29, maxX = -1<<29, maxY = -1<<29;
				for(int i=0; i<4; i++)
				{
					// 单应变换
					ApplyProject9(pImgT[n].m, corner[i].x, corner[i].y, cornerBar[i].x, cornerBar[i].y);
					cornerBar[i].x += (0 + dX);
					cornerBar[i].y += (0 + dY);
					if(cornerBar[i].x<minX)
					{
						minX = cornerBar[i].x;
					}
					if(cornerBar[i].x>maxX)
					{
						maxX = cornerBar[i].x;
					}
					if(cornerBar[i].y<minY)
					{
						minY = cornerBar[i].y;
					}
					if(cornerBar[i].y>maxY)
					{
						maxY = cornerBar[i].y;
					}
				}
				int begY = int(minY-0.5f), endY = int(maxY+0.5f);
				int begX = int(minX-0.5f), endX = int(maxX+0.5f);
				begX = Max(begX, 0);
				endX = Min(endX, wRes-1);

				begY = Max(begY, 0);
				endY = Min(endY, hRes-1);
				//cout<<begX<<" "<<endX<<" "<<begY<<" "<<endY<<endl;
				for(int yDst=begY; yDst<=endY; yDst++)
				{
					unsigned char* pRowDst = (unsigned char*)m_pMosaicResult->imageData + yDst*wsRes;

					for(int xDst=begX; xDst<=endX; xDst++)
					{
						float xMid = xDst - 0 - dX;
						float yMid = yDst - 0 - dY;
						float xSrc32F, ySrc32F;
						ApplyProject9(invH, xMid, yMid, xSrc32F, ySrc32F);
						// 双线性插值
						int xSrc = int(xSrc32F);
						int ySrc = int(ySrc32F);

						float p = ySrc32F - ySrc;
						float q = xSrc32F - xSrc;

						if( (ySrc32F<0)||(ySrc32F>=hSrc_1) )
							continue;
						if( (xSrc32F<0)||(xSrc32F>=wSrc_1) )
							continue;

						unsigned char* pSrcTemp = (unsigned char*)pImages[n]->imageData + ySrc*wsSrc + 3*xSrc;

						// 双线性插值
						if( (*(pRowDst + 3*xDst)==0) && 
							(*(pRowDst + 3*xDst)==0) && 
							(*(pRowDst + 3*xDst)==0) )
						{
							*(pRowDst + 3*xDst) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
								*(pSrcTemp + 3)*(1-p)*q + 
								*(pSrcTemp + wsSrc)*p*(1-q) + 
								*(pSrcTemp + wsSrc + 3)*p*q );

							pSrcTemp = (unsigned char*)pImages[n]->imageData + ySrc*wsSrc + 3*xSrc+1;
							*(pRowDst + 3*xDst+1) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
								*(pSrcTemp + 3)*(1-p)*q + 
								*(pSrcTemp + wsSrc)*p*(1-q) + 
								*(pSrcTemp + wsSrc + 3)*p*q );

							pSrcTemp = (unsigned char*)pImages[n]->imageData + ySrc*wsSrc + 3*xSrc+2;
							*(pRowDst + 3*xDst+2) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
								*(pSrcTemp + 3)*(1-p)*q + 
								*(pSrcTemp + wsSrc)*p*(1-q) + 
								*(pSrcTemp + wsSrc + 3)*p*q );
						}
					}
				}
			}
		}
	}

	leftUpPt = leftUpPtNow;

	return 0;
}

int CMosaic::Mosaic(const IplImage* pSrc0, IplImage* &pMosaicResult, const bool bEmergency)
{
	return 0;
}

CMosaic::CMosaic()
{
	m_pMosaicResult = NULL;
	gpCornerDescriptor1 = NULL;
	gpCornerDescriptor2 = NULL;
	m_nCorner1 = m_nCorner2 = 0;
	m_pGray1 = m_pGray2 = NULL;
	m_bEndDynamicMosaic = false;
}

CMosaic::~CMosaic()
{
	Destroy();

	cvReleaseImage(&m_pMosaicResult);
}

void CMosaic::Destroy()
{
	cvReleaseImage(&m_pMosaicResult);

	delete[] gpCornerDescriptor1; gpCornerDescriptor1 = NULL;
	delete[] gpCornerDescriptor2; gpCornerDescriptor2 = NULL;
	gCurrentImagesNum = 0;
	gTotalImagesNum = 0;

	m_nCorner1 = m_nCorner2 = 0;

	cvReleaseImage(&m_pGray1);
	cvReleaseImage(&m_pGray2);

	list<MosaicImageInfo>::iterator iter;
	iter = listMosaicInfo.begin();
	for(int i=0; i<(int)listMosaicInfo.size(); i++)
	{
		cvReleaseImage( &(*iter).pSrc );
		iter++;
	}
	listMosaicInfo.clear();
	listImagesPool.clear();

	delete[] gpInnerPoints1; gpInnerPoints1 = NULL;
	delete[] gpInnerPoints2; gpInnerPoints2 = NULL;
}

void CMosaic::SetParams( MosaicParams param )
{
	this->m_params = param;
}

IplImage* CMosaic::GetMosaicResult(bool bExternal)
{
	// 加锁
	CMutexLock lock;
	lock.Lock();
	if(false==bExternal)
	{
		cvReleaseImage(&m_pMosaicResult);
		if(NULL==pResultImage)
			return NULL;
		m_pMosaicResult = cvCreateImage(cvSize(pResultImage->width,pResultImage->height), 8, pResultImage->nChannels);
		memcpy(m_pMosaicResult->imageData, pResultImage->imageData, pResultImage->widthStep*pResultImage->height);

		ReleaseBitmap8U(pResultImage);

		UpdateResult(true);
		return m_pMosaicResult;
	}
	else
	{
		UpdateResult(false);
		return m_pMosaicResult;
	}
	// 解锁
	lock.Unlock();
}

IplImage* CMosaic::GetDynamicMosaicResult(bool bExternal)
{
	// 加锁
	CMutexLock lock;
	lock.Lock();
	if(false==bExternal)
	{
		UpdateResult(true);
		return m_pMosaicResult;
	}
	else
	{
		UpdateResult(false);
		return m_pMosaicResult;
	}
	// 解锁
	lock.Unlock();
}

int GetInnerPoints(SfPoint* &pInner1, SfPoint* &pInner2, int &nInner)
{
	pInner1 = gpInnerPoints1;
	pInner2 = gpInnerPoints2;
	nInner = nInnerPoints;

	return 0;
}

IplImage* MosaicMultiImages2( IplImage* pImagesCV[], int imagesNum, 
								  int &errorCode, int maxFeatruesNum, 
								  int winR, float minScore, float ransacDist/**/, 
								  int searchR /**/ )
{
	//Stitcher sticth;
	BitmapImage** pImages = new BitmapImage*[imagesNum];

	for(int i=0; i<imagesNum; i++)
	{
		pImages[i] = CreateBitmap8U(pImagesCV[i]->width, pImagesCV[i]->height, pImagesCV[i]->nChannels);
		memcpy(pImages[i]->imageData, pImagesCV[i]->imageData, pImagesCV[i]->widthStep * pImagesCV[i]->height );
	}

	BitmapImage* pResult = NULL;
	pResult = MosaicMultiImages(pImages, imagesNum, errorCode, maxFeatruesNum, winR, minScore, ransacDist, searchR);

	IplImage* pResultCV = NULL;
	if(pResult)
	{
		pResultCV = cvCreateImage( cvSize(pResult->width, pResult->height), 8, pResult->nChannels );
		memcpy(pResultCV->imageData, pResult->imageData, pResult->widthStep*pResult->height);
	}
	ReleaseBitmap8U(pResult);

	for(int i=0; i<imagesNum; i++)
	{
		ReleaseBitmap8U(pImages[i]);
	}
	delete[] pImages; pImages = NULL;

	return pResultCV;
}