#include "usingCV24.h"
#include "opencv2/stitching/stitcher.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include <algorithm>
#include "MosaicImage.h"
#include "ImageIO.h"
//#include "BasicMethod.h"
#include "Matrix.h"
#include "mvMath.h"
//#include "HarrisCorner.h"
//#include "DrawFigure.h"
#include "usingAtlImage.h"
//#include "SobelEdge.h"
#include "imageMath.h"
#include "Statistic.h"
#include "ImageGeometry.h"
#include "mvMath.h"
//#include "FitSurface.h"
#include "MosaicWithoutPos.h"
#include "usingAtlImage.h"
#include "opencv2\opencv.hpp"

using namespace cv;

CMosaicHarris::CMosaicHarris()
{
}

CMosaicHarris::~CMosaicHarris()
{
	delete[] m_aCorners1;
	delete[] m_aCorners2;
	delete[] m_aCorners1_32f;
	delete[] m_aCorners2_32f;

	//delete[] m_pEllipseParam1; m_pEllipseParam1 = NULL;
	//delete[] m_pEllipseParam2; m_pEllipseParam2 = NULL;
}


//// 函数功能：提取harris角点特征
//// max_corners_num
//bool CMosaicHarris::HarrisFeature( IplImage *pSrcImage, 
//				   CvPoint2D32f* pCorners, int &cornerNum, 
//				   const int max_corners_num )
//{
//	if(pSrcImage==NULL)
//		return false;
//	if(pCorners==NULL)
//		return false;
//
//	// 角点的最大个数
//	//const int max_corners = 1000;
//
//	IplImage *pGrayImage = NULL;
//	IplImage *eig_image = NULL;
//	IplImage *temp_image = NULL;
//
//	int w = 0, h = 0, ws = 0;
//	GetImageSize( pSrcImage, w, h, ws );
//
//	pGrayImage = cvCloneImage( pSrcImage );
//	eig_image = cvCreateImage( cvSize(w, h), IPL_DEPTH_32F, 1 );
//	temp_image = cvCreateImage( cvSize(w, h), IPL_DEPTH_32F, 1 );
//
//	cornerNum = max_corners_num;
//	double quality_level = 0.01;
//	double min_distance = 15;
//	int block_size = 5;
//	int use_harris = 1;
//	double k = 0.04;
//	cvGoodFeaturesToTrack(pGrayImage, eig_image, temp_image,
//		pCorners, &cornerNum,
//		quality_level, min_distance,
//		NULL, 
//		block_size,
//		use_harris, 
//		k);
//
//#ifdef _DEBUG
//		printf("角点数: %d，", cornerNum);
//#endif
//
//	//精确角点位置 
//	int max_iter = 20; // 最大迭代次数
//	cvFindCornerSubPix( pGrayImage, 
//		pCorners, // 输入角点的初始坐标, 也存储精确坐标
//		cornerNum, 
//		cvSize(10,10), 
//		cvSize(-1,-1), 
//		cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, max_iter, 0.05 ) 
//		);
//
//	ReleaseImage(pGrayImage);
//	ReleaseImage(eig_image);
//	ReleaseImage(temp_image);
//
//	return true;
//} 


//函数功能：计算相关度
float CMosaicHarris::CalCorelaton2(BitmapImage *im1, BitmapImage *im2,
						  int x1, int y1, int x2, int y2,
						  int winRadius)
{

	unsigned char* pointer1 = NULL;
	unsigned char* pointer2 = NULL;
	pointer1 = im1->imageData;
	pointer2 = im2->imageData;

	int w1, h1, ws1;
	GetImageSize( im1, w1, h1, ws1 );

	int w2, h2, ws2;
	GetImageSize( im2, w2, h2, ws2 );
	
	int startx1, startx2, endx1, endx2;
	int starty1, starty2, endy1, endy2;
	startx1 = x1- winRadius;
	starty1 = y1 - winRadius;
	endx1 = x1+ winRadius;
	endy1 = y1 + winRadius;

	startx2 = x2 - winRadius;
	starty2 = y2 - winRadius;
	endx2 = x2 + winRadius;
	endy2 = y2 + winRadius;

	float ave1 = 0,ave2 = 0;
	float acc1=0,acc2=0,acc3=0;

	for(int i=-winRadius; i<=winRadius; i++)
	{
		for(int j=-winRadius; j<=winRadius; j++)
		{
			int temp1 = 0, temp2 = 0;
			temp1 = (unsigned char)*( pointer1 + ws1*(y1+i) +(x1+j) );
			temp2 = (unsigned char)*( pointer2 + ws2*(y2+i) +(x2+j) );

			ave1 += temp1;
			ave2 += temp2;

			acc1 += temp1*temp1; // 化简算法
			acc2 += temp2*temp2; // 化简算法
			acc3 += temp1*temp2; // 化简算法

			//acc1+=(temp1-ave1) * (temp2-ave2); // 原
			//acc2+=((temp1-ave1)*(temp1-ave1)); // 原
			//acc3+=((temp2-ave2)*(temp2-ave2)); // 原
		}
	}

	int winWidth = winRadius*2 + 1;

	ave1 = ave1/(winWidth*winWidth);
	ave2 = ave2/(winWidth*winWidth);
	//return acc1/(sqrt(acc2)*sqrt(acc3));//原
	return fabs(acc3-winWidth*winWidth*ave1*ave2)/
		sqrt((acc1-winWidth*winWidth*ave1*ave1)*(acc2-winWidth*winWidth*ave2*ave2));//化简算法
}



//bool CMosaicHarris::ShowFeaturePoints(IplImage* pGrayImage, const std::vector<CvPoint2D32f> &vecPoints, 
//					   char* pName, char* pSavePath )
//{
//	int w, h, ws;
//	GetImageSize( pGrayImage, w, h, ws );
//
//	IplImage* pShow1 = cvCreateImage( cvSize(w,h), 8, 3 );
//	cvCvtColor( pGrayImage, pShow1, CV_GRAY2BGR );
//
//	int color[2] = {255, 128};
//	int iColor = 0;
//	for(int i=0; i<vecPoints.size(); i++)
//	{
//		if(iColor<1)
//		{
//			iColor++;
//		}
//		else
//		{
//			iColor = 0;
//		}
//
//		DrawCrossMark( pShow1, vecPoints[i].x, vecPoints[i].y, 3, color[0] );
//
//		// 写上文字信息
//		CvFont font;
//		float hscale = 0.4;
//		float vscale = 0.4;
//		int linewidth = 1;
//		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX | CV_FONT_HERSHEY_SIMPLEX,hscale,vscale,0,linewidth);
//		char text1[20];
//		sprintf_s(text1, "%d", i);
//		cvPutText(pShow1, text1, cvPoint(vecPoints[i].x+2, vecPoints[i].y), &font, cvScalar(0,255,0));
//	}
//	ShowImage( pShow1, pName );
//
//	if(pSavePath)
//	{
//		SaveImage( pShow1, pSavePath );
//		//WaitKey(1);
//	}
//
//	cvReleaseImage(&pShow1);
//
//	return true;
//}

bool CMosaicHarris::ShowFeaturePoints(BitmapImage* pGrayImage, SfPoint* pPoints, int ptNum, 
					   TCHAR* pName)
{

	return true;
}

bool CMosaicHarris::ShowFeaturePoints(BitmapImage* pGrayImage, CornerDescriptor* pPoints, int ptNum, 
									  TCHAR* pName)
{

	return true;
}


//bool CMosaicHarris::ShowFeaturePoints(IplImage* pGrayImage, SfPoint* pPoints, int ptNum, 
//									  char* pName)
//{
//	int w, h, ws;
//	GetImageSize( pGrayImage, w, h, ws );
//
//	IplImage* pShow1 = cvCreateImage( cvSize(w,h), 8, 3 );
//	cvCvtColor( pGrayImage, pShow1, CV_GRAY2BGR );
//
//	for(int i=0; i<ptNum; i++)
//	{
//		DrawCrossMark( pShow1, pPoints[i].x, pPoints[i].y, 3, 255 );
//	}
//	ShowImage( pShow1, pName );
//
//	cvReleaseImage(&pShow1);
//
//	return true;
//}




// **********************************************************************
// 函数功能：剔除误匹配 + 投影矩阵计算
// **********************************************************************
bool Ransac(const std::vector<SfPoint> &vecMatchedPoints1, 
			const std::vector<SfPoint> &vecMatchedPoints2,
			float aProjectMat[9],
			float fRansacDist = 1, int sampleTimes = 2000)
{
	bool isSuccess = true;

	const int MIN_SAMPLING_NUM = 4;

	float fRansacDistSquare = fRansacDist*fRansacDist;

	int ptNum = (int)vecMatchedPoints1.size();
	if(ptNum<MIN_SAMPLING_NUM)
		return false;

	float fInvPtNum = 1.0f/ptNum;

	const int maxTimes = 5000; // 最大采样次数
	if( sampleTimes>maxTimes )
	{
		sampleTimes = maxTimes;
	}

	ProjectMat *pProjectMat =  new ProjectMat[sampleTimes];

	std::vector<SampleIndexs> vecHaveSampled; // 已经采样过的点
	vecHaveSampled.reserve(sampleTimes);

	//随机采样
	srand((unsigned)time(0));

	int maxSupport = 0;
	int maxSupportIndex = 0;
	int realSamTimes = 0;
	for ( int t=0; t<sampleTimes; )
	{
		realSamTimes++;

		if(realSamTimes>=maxTimes)
		{
			break;
		}

		int aSampleIndex[MIN_SAMPLING_NUM];

		SfPoint samPoint1[MIN_SAMPLING_NUM];
		SfPoint samPoint2[MIN_SAMPLING_NUM];

		do
		{
			aSampleIndex[0] = rand()%ptNum;
			aSampleIndex[1] = rand()%ptNum;
			aSampleIndex[2] = rand()%ptNum;
			aSampleIndex[3] = rand()%ptNum;
		}
		while( ( aSampleIndex[0]==aSampleIndex[1] ) || 
			( aSampleIndex[0]==aSampleIndex[2] ) ||
			( aSampleIndex[0]==aSampleIndex[3] ) ||
			( aSampleIndex[1]==aSampleIndex[2] ) ||
			( aSampleIndex[1]==aSampleIndex[3] ) ||
			( aSampleIndex[2]==aSampleIndex[3] ) ); // 要求采样4个不同的点

		SampleIndexs currentSamIndexs;
		currentSamIndexs.elem.push_back( aSampleIndex[0] );
		currentSamIndexs.elem.push_back( aSampleIndex[1] );
		currentSamIndexs.elem.push_back( aSampleIndex[2] );
		currentSamIndexs.elem.push_back( aSampleIndex[3] );
		sort(currentSamIndexs.elem.begin(), currentSamIndexs.elem.end()); // 排序

		bool isCurrentSamIndexUsed = false;
		for(int i=0; i<(int)vecHaveSampled.size(); i++)
		{
			if( ( vecHaveSampled[i].elem[0]==currentSamIndexs.elem[0] ) &&
				( vecHaveSampled[i].elem[1]==currentSamIndexs.elem[1] ) &&
				( vecHaveSampled[i].elem[2]==currentSamIndexs.elem[2] ) &&
				( vecHaveSampled[i].elem[3]==currentSamIndexs.elem[3] ) )
			{
				isCurrentSamIndexUsed = true;
			}
		}
		if(isCurrentSamIndexUsed)
			continue;
		
		vecHaveSampled.push_back( currentSamIndexs );

		// 采样
		samPoint1[0] = vecMatchedPoints1[aSampleIndex[0]];
		samPoint1[1] = vecMatchedPoints1[aSampleIndex[1]];
		samPoint1[2] = vecMatchedPoints1[aSampleIndex[2]];
		samPoint1[3] = vecMatchedPoints1[aSampleIndex[3]];

		samPoint2[0] = vecMatchedPoints2[aSampleIndex[0]];
		samPoint2[1] = vecMatchedPoints2[aSampleIndex[1]];
		samPoint2[2] = vecMatchedPoints2[aSampleIndex[2]];
		samPoint2[3] = vecMatchedPoints2[aSampleIndex[3]];

		float aProjectMat2[9] = {0};
		SolveProjectMatrix3( samPoint1, samPoint2, MIN_SAMPLING_NUM, aProjectMat2 );

		//SolveProjectMatrix( vecMatchedPoints2[56].x, samPoint2, MIN_SAMPLING_NUM, aProjectMat2 );

		for(int i=0; i<9; i++)
		{
			pProjectMat[t].m[i] = aProjectMat2[i];
		}

		// 计算这个假设的支持度
		int currentSupport = 0;
		for(int i=0; i<ptNum; i++)
		{
			SfPoint point1Bar;
			ApplyProjectMat3( vecMatchedPoints2[i], point1Bar, aProjectMat2 );
			
			float fDistSquare = 0;
			DistanceSquareOfTwoPoints( point1Bar.x, point1Bar.y, 
									   vecMatchedPoints1[i].x, vecMatchedPoints1[i].y,
									   fDistSquare);
			if(fDistSquare<fRansacDistSquare)
			{
				currentSupport++;
			}
		}
		if(currentSupport>maxSupport)
		{
			maxSupport = currentSupport;
			maxSupportIndex = t;
			if(maxSupport*fInvPtNum>0.99f)
			{
				break;
			}
		}

		t++; // 采样数
	}

	// 分成内点和外点
	std::vector<SfPoint> vecInnerPoints1, vecInnerPoints2;
	int currentSupport = 0;
	for(int i=0; i<ptNum; i++)
	{
		SfPoint point1Bar;
		ApplyProjectMat3( vecMatchedPoints2[i], point1Bar, pProjectMat[maxSupportIndex].m );

		float fDistSquare = 0;
		DistanceSquareOfTwoPoints( point1Bar.x, point1Bar.y, 
			vecMatchedPoints1[i].x, vecMatchedPoints1[i].y,
			fDistSquare);
		if(fDistSquare<fRansacDistSquare)
		{
			vecInnerPoints1.push_back( vecMatchedPoints1[i] );
			vecInnerPoints2.push_back( vecMatchedPoints2[i] );
		}
	}

#ifdef _DEBUG
	
#endif

	//if(!vecInnerPoints1.empty())
	//	cout<<"InnerPointsNum "<<vecInnerPoints1.size()<<endl;
	
	// 用内点重新计算投影矩阵
	if( !SolveProjectMatrix3( &vecInnerPoints1[0], &vecInnerPoints2[0], (int)vecInnerPoints1.size(), aProjectMat) )
	{
		isSuccess = false;
	}

#ifdef _DEBUG
	int resultSupport = 0;
	for(int i=0; i<ptNum; i++)
	{
		SfPoint point1Bar;
		ApplyProjectMat3( vecMatchedPoints2[i], point1Bar, aProjectMat );

		float fDistSquare = 0;
		DistanceSquareOfTwoPoints( point1Bar.x, point1Bar.y, 
			vecMatchedPoints1[i].x, vecMatchedPoints1[i].y,
			fDistSquare);
		if(fDistSquare<fRansacDistSquare)
		{
			resultSupport++;
		}
	}
	cout<<"resultSupport  "<<resultSupport<<endl;
#endif

	delete[] pProjectMat; pProjectMat = NULL;
	
	return true;
}


// **********************************************************************
// 函数功能：剔除误匹配 + 投影矩阵计算
// **********************************************************************


// 填充Image2
// strMethod, "bilinear"
bool CMosaicHarris::FillImage( BitmapImage* pDstImage, BitmapImage* pSrcImage, 
			   InterestRegion2 region,
			   int dx, int dy,
			   float aProjectMat[9], string strMethod )
{
	if(pDstImage==NULL)
		return false;
	if(pSrcImage==NULL)
		return false;

	std::vector<pool::SfPoint> vecRevisePoints1(m_vecInnerPoints1.size()), vecRevisePoints2(m_vecInnerPoints1.size());
	for(int i=0; i<(int)m_vecInnerPoints1.size(); i++)
	{
		int dx2 = 0, dy2 = 0;
		dx2 = dx;
		dy2 = dy;

		vecRevisePoints1[i].x = m_vecInnerPoints1[i].x + dx2;
		vecRevisePoints1[i].y = m_vecInnerPoints1[i].y + dy2;

		float fXDst, fYDst;
		ApplyProjectMat2( (float)m_vecInnerPoints2[i].x, (float)m_vecInnerPoints2[i].y, 
			fXDst, fYDst, aProjectMat );

		fXDst += dx2;
		fYDst += dy2;

		vecRevisePoints2[i].x = fXDst;
		vecRevisePoints2[i].y = fYDst;
		
		float fXSrc, fYSrc;
		ApplyProjectMat2Inv( fXDst-dx2, fYDst-dy2, fXSrc, fYSrc, aProjectMat );
	}
	
	int srcW, srcH, srcWs;
	GetImageSize( pSrcImage, srcW, srcH,srcWs );
	int dstW, dstH, dstWs;
	GetImageSize( pDstImage, dstW, dstH, dstWs );
	
	int srcW_1 = srcW - 1;
	int srcH_1 = srcH - 1;
	int wsSrc = pSrcImage->widthStep;

	int xBeg = region.xbeg;
	int yBeg = region.ybeg;
	int xEnd = region.xend;
	int yEnd = region.yend;

	int nChannels = pDstImage->nChannels;

	for(int yDst = yBeg; yDst<yEnd; yDst++)
	{
		unsigned char* pRowDst = pDstImage->imageData + yDst*dstWs;
		for(int xDst=xBeg; xDst<xEnd; xDst++)
		{
			float fXSrc, fYSrc;
			//ApplyProjectMat2( xDst, yDst, 
			//	fXSrc, fYSrc, aProjectMat );

			//xDst -= dx;
			//yDst -= dy;
			ApplyProjectMat2Inv( xDst-dx, yDst-dy, fXSrc, fYSrc, aProjectMat );

			if( (fXSrc>=0) && (fXSrc<srcW_1) && (fYSrc>=0) && (fYSrc<srcH_1) )
			{
				//双线性插值
				int ySrc = int(fYSrc);
				int xSrc = int(fXSrc);

				float p = fYSrc - ySrc;
				float q = fXSrc - xSrc;

				if(nChannels==1)
				{
					unsigned char* pSrcTemp = pSrcImage->imageData + ySrc*srcWs + xSrc;

					*(pRowDst + xDst) = unsigned char((unsigned char)*(pSrcTemp)*(1-p)*(1-q) + 
						(unsigned char)*(pSrcTemp + 1)*(1-p)*q + 
						(unsigned char)*(pSrcTemp + srcWs)*p*(1-q) + 
						(unsigned char)*(pSrcTemp + srcWs + 1)*p*q );
				}
				else if(3==nChannels)
				{
					int i = ySrc;
					int j = xSrc;
					{
						unsigned char* pTempB = pSrcImage->imageData + i*wsSrc+ 3*j;
						unsigned char grayB1 = (unsigned char)*(pTempB);
						unsigned char grayB2 = (unsigned char)*(pTempB + 3);
						unsigned char grayB3 = (unsigned char)*(pTempB + wsSrc);
						unsigned char grayB4 = (unsigned char)*(pTempB + wsSrc + 3);
						pRowDst[3*xDst] = unsigned char( grayB1*(1-p)*(1-q) + 
							grayB2*(1-p)*q + 
							grayB3*p*(1-q) + 
							grayB4*p*q );
					}

					{
						unsigned char* pTempG = pSrcImage->imageData + i*wsSrc+ 3*j + 1;
						unsigned char grayG1 = (unsigned char)*(pTempG);
						unsigned char grayG2 = (unsigned char)*(pTempG + 3);
						unsigned char grayG3 = (unsigned char)*(pTempG + wsSrc);
						unsigned char grayG4 = (unsigned char)*(pTempG + wsSrc + 3);
						pRowDst[3*xDst + 1] = unsigned char( grayG1*(1-p)*(1-q) + 
							grayG2*(1-p)*q + 
							grayG3*p*(1-q) + 
							grayG4*p*q );
					}

					{					
						unsigned char* pTempR = pSrcImage->imageData + i*wsSrc+ 3*j + 2;
						unsigned char grayR1 = (unsigned char)*(pTempR);
						unsigned char grayR2 = (unsigned char)*(pTempR + 3);
						unsigned char grayR3 = (unsigned char)*(pTempR + wsSrc);
						unsigned char grayR4 = (unsigned char)*(pTempR + wsSrc + 3);
						pRowDst[3*xDst + 2] = unsigned char( grayR1*(1-p)*(1-q) + 
							grayR2*(1-p)*q + 
							grayR3*p*(1-q) + 
							grayR4*p*q );
					}
				}
			}
		}
	}
	
#ifdef _DEBUG
	//IplImage* pShowMatch = cvCreateImage( cvSize(pDstImage->width, pDstImage->height), 8, 3 );
	//cvCvtColor( pDstImage, pShowMatch, CV_GRAY2BGR );
	//for(int i=0; i<vecRevisePoints1.size(); i++)
	//{
	//	DrawCrossMark( pShowMatch, vecRevisePoints1[i].x, vecRevisePoints1[i].y, 3, 0, 0, 255 );
	//	DrawCrossMark( pShowMatch, vecRevisePoints2[i].x, vecRevisePoints2[i].y, 2, 255, 0, 0 );
	//}
	//ShowImage( pShowMatch, "ShowMatchRevise" );
	//cvReleaseImage(&pShowMatch);
#endif
	
	return true;
}



// 函数功能：图像融合
bool CMosaicHarris::Merge( BitmapImage* pSrc1, BitmapImage* pSrc2, 
		   BitmapImage* &pResultImage, 
		   float aProjectMat[9])
{
	if(pSrc1==NULL)
		return false;
	if(pSrc2==NULL)
		return false;
	if(pSrc1->nChannels!=pSrc2->nChannels)
	{
		return false;
	}
	
	int w1 = 0, h1 = 0, ws1 = 0;
	GetImageSize( pSrc1, w1, h1, ws1 );

	int w2 = 0, h2 = 0, ws2 = 0;
	GetImageSize( pSrc2, w2, h2, ws2 );

	SfPoint aImageCorner1[4];
	aImageCorner1[0].x = 0;
	aImageCorner1[0].y = 0;
	aImageCorner1[1].x = (float)(w1 - 1);
	aImageCorner1[1].y = 0;
	aImageCorner1[2].x = (float)(w1 - 1);
	aImageCorner1[2].y = (float)(h1 - 1);
	aImageCorner1[3].x = 0;
	aImageCorner1[3].y = (float)(h1 - 1);

	SfPoint aImageCorner2[4];
	aImageCorner2[0].x = 0;
	aImageCorner2[0].y = 0;
	aImageCorner2[1].x = (float)(w2 - 1);
	aImageCorner2[1].y = 0;
	aImageCorner2[2].x = (float)(w2 - 1);
	aImageCorner2[2].y = (float)(h2 - 1);
	aImageCorner2[3].x = 0;
	aImageCorner2[3].y = (float)(h2 - 1);

	SfPoint aImageCorner2Bar[4];
	SfPoint aImageCorner[8];
	for(int i=0; i<4; i++)
	{
		ApplyProjectMat3( aImageCorner2[i], aImageCorner2Bar[i], aProjectMat ); // 先转4个顶点
		
		aImageCorner[i] = aImageCorner1[i];
		aImageCorner[i+4] = aImageCorner2Bar[i];
	}

	float fMinX, fMinY, fMaxX, fMaxY;
	FindBorder( aImageCorner, 8, fMinX, fMinY, fMaxX, fMaxY );

	int resultW = (int)(fMaxX - fMinX + 0.5f) + 1; // 结果宽度
	int resultH = (int)(fMaxY - fMinY + 0.5f) + 1; // 结果高度

	// 使得Image1不需要做插值
	int dx1 = int(0 - fMinX + 0.5f); // Image1的在新图像中的平移量
	int dy1 = int(0 - fMinY + 0.5f);

	pResultImage = CreateBitmap8U( resultW, resultH, pSrc1->nChannels );
	memset( pResultImage->imageData, 255, pResultImage->widthStep * pResultImage->height );
	int nChannels = pSrc1->nChannels;

	int resultWs = pResultImage->widthStep;
	// 先填充Image1
	for( int y=0; y<h1; y++ )
	{
		if(1==nChannels)
		{
			memcpy( pResultImage->imageData + (y+dy1)*resultWs + dx1, pSrc1->imageData + y*ws1, ws1 );
		}
		else if(3==nChannels)
		{
			memcpy( pResultImage->imageData + (y+dy1)*resultWs + 3*dx1, pSrc1->imageData + y*ws1, ws1 );
		}
	}

	float fMinXBar, fMinYBar, fMaxXBar, fMaxYBar;
	FindBorder( aImageCorner2Bar, 4, fMinXBar, fMinYBar, fMaxXBar, fMaxYBar );

	InterestRegion2 region;
	region.xbeg = int(fMinXBar + dx1);
	region.ybeg = int(fMinYBar + dy1);
	region.xend = int(fMaxXBar + dx1);
	region.yend = int(fMaxYBar + dy1);

	// 先填充Image2
	FillImage( pResultImage, pSrc2,
		region,
		dx1, dy1,
		aProjectMat, "bilinear" );

#ifdef _DEBUG
	//SaveImageCv( pResultImage, "c:\\harrisJointResult.bmp" );
#endif
	
	return true;
}


// 灰度图转彩色图
bool CMosaicHarris::GrayToColor2(BitmapImage* pSrcGray, BitmapImage* pDstColor)
{
	if( (pSrcGray==NULL) || (pDstColor==NULL) )
		return false;

	int w, h, ws8, ws24;
	GetImageSize(pSrcGray, w, h, ws8);
	ws24 = pDstColor->widthStep;

	for(int y=0; y<h; y++)
	{
		unsigned char *pRow8 = pSrcGray->imageData + y * ws8;
		unsigned char *pRow24 = pDstColor->imageData + y * ws24;
		for(int x=0; x<w; x++)
		{
			pRow24[3*x + 0] = pRow8[x];
			pRow24[3*x + 1] = pRow8[x];
			pRow24[3*x + 2] = pRow8[x];
		}
	}

	return true;
}

// 彩色图转灰度图
// Gray = R*0.299 + G*0.587  + B*0.114;
bool ColorToGray1(BitmapImage *pSrcColor, BitmapImage *pDstGray)
{
	//SaveImageATL(pSrcColor, "c:\\color.bmp");

	if( (pSrcColor==NULL) || (pDstGray==NULL) )
		return false;

	int w = 0, h = 0;
	w = pSrcColor->width;
	h = pSrcColor->height;


	int ws8 = pDstGray->widthStep;

	int ws24 = pSrcColor->widthStep;
	for(int y=0; y<h; y++)
	{
		unsigned char* pRow8 = pDstGray->imageData + y*ws8;
		unsigned char* pRow24 = pSrcColor->imageData + y*ws24;
		for(int x=0; x<w; x++)
		{
			int B = pRow24[x*3];
			int G = pRow24[x*3 + 1];
			int R = pRow24[x*3 + 2];

			pRow8[x] = (unsigned char)(R*0.299f + G*0.587f  + B*0.114f);
		}
	}

	return true;
}

// Harris拼接
bool CMosaicHarris::MosaicHarris(BitmapImage* pSrc1, BitmapImage* pSrc2, float aProjectMat[9],
								 SfPoint* pCorner1 /*= NULL*/, SfPoint *pCorner2 /*= NULL*/,
								 int nCorner1, int nCorner2)
{   

	return 0;
}

// Harris拼接
bool CMosaicHarris::MosaicHarrisDescriptor(BitmapImage* pSrc1, BitmapImage* pSrc2, float aProjectMat[9],
								 //SfPoint* pCorner1, SfPoint* pCorner2, 
								 CornerDescriptor* pDescriptor1 /*= NULL*/, CornerDescriptor *pDescriptor2 /*= NULL*/,
								 int nCorner1, int nCorner2)
{   
	if(pSrc1==NULL)
		return false;
	if(pSrc2==NULL)
		return false;
	if( (pSrc1->nChannels!=1) || (pSrc2->nChannels!=1) )
		return false;

	// 提取harris角点
	int MAX_CORNERS_NUM = m_maxFeatureNum;
	MAX_CORNERS_NUM = max( nCorner2, max(nCorner1, MAX_CORNERS_NUM) );
	//m_aCorners1 = new SfPoint[MAX_CORNERS_NUM];
	//m_aCorners2 = new SfPoint[MAX_CORNERS_NUM];

	int cornersNum1 = 0;
	int cornersNum2 = 0;

	//cout<<"using cv"<<endl;
	//memcpy(m_aCorners1, pCorner1, nCorner1 * sizeof(SfPoint) );
	//memcpy(m_aCorners2, pCorner2, nCorner2 * sizeof(SfPoint) );
	cornersNum1 = nCorner1;
	cornersNum2 = nCorner2;
#ifdef _DEBUG
	std::cout<<"cornersNum1 "<<cornersNum1<<std::endl;
	std::cout<<"cornersNum2 "<<cornersNum2<<std::endl;	
	//COutData::OutAllPoints2Txt(m_aCorners2, cornersNum2, ("c:\\cornerXu.txt") );
#endif

	//m_aCorners1_32f = new SfPoint[MAX_CORNERS_NUM];
	//m_aCorners2_32f = new SfPoint[MAX_CORNERS_NUM];
	//memcpy( m_aCorners1_32f, m_aCorners1, sizeof(SfPoint)*MAX_CORNERS_NUM );
	//memcpy( m_aCorners2_32f, m_aCorners2, sizeof(SfPoint)*MAX_CORNERS_NUM );

	// 角点匹配
	std::vector<CornerDescriptor> vecMatchedPoints1;
	std::vector<CornerDescriptor> vecMatchedPoints2;
	int matchWinRadius = m_winRadius;
	float fSimilarityT = m_minScore;
	int matchMethod = m_matchMethod;

	// NCC匹配特征
	MatchHarrisFeature( pSrc1, pSrc2, 
			pDescriptor1, cornersNum1, 
			pDescriptor2, cornersNum2, 
			vecMatchedPoints1, vecMatchedPoints2, 
			matchWinRadius, fSimilarityT );

	bool success = false;
	if(!vecMatchedPoints1.empty())
	{
		//WaitKey(1);
#ifdef _DEBUG	
		cout<<"matched num "<<(int)vecMatchedPoints1.size()<<endl;
		//ShowMatch( pSrc1, pSrc2, &vecMatchedPoints1[0], &vecMatchedPoints2[0], (int)vecMatchedPoints1.size() );
		cout<<"开始Ransac"<<endl;
#endif
		// 剔除误匹配 + 投影矩阵计算
		float ransacDist = m_ransacDist;
		if( !Ransac( pSrc1, pSrc2, vecMatchedPoints1, vecMatchedPoints2, aProjectMat, ransacDist ) )
			return false;
#ifdef _DEBUG	
		cout<<"Ransac完成"<<endl;
#endif
		int innerPairsT = 50;

		int nInnerOld = (int)m_vecInnerPoints1.size(), nInnerNew = 0;
		//if( (int)m_vecInnerPoints1.size()>4 )//
		if(  ((int)m_vecInnerPoints1.size()<innerPairsT) && ((int)m_vecInnerPoints1.size()>6) )//
		{
			//do 
			{
				nInnerOld = (int)m_vecInnerPoints1.size();
				// 再匹配
				int localSearchR = 15;
				vecMatchedPoints1.clear();
				vecMatchedPoints2.clear();

				//MatchFeaturePointsLogPolarProjectionSumTable(pSrc1, pSrc2, 
				//	m_aCorners1_32f, cornersNum1, 
				//	m_aCorners2_32f, cornersNum2, 
				//	vecMatchedPoints1, vecMatchedPoints2, 
				//	matchWinRadius, fSimilarityT, 
				//	aProjectMat, localSearchR );
				// NCC匹配特征
				MatchHarrisFeature( pSrc1, pSrc2, 
					pDescriptor1, cornersNum1, 
					pDescriptor2, cornersNum2, 
					vecMatchedPoints1, vecMatchedPoints2, 
					matchWinRadius, fSimilarityT, localSearchR, aProjectMat );

				nInnerNew = (int)m_vecInnerPoints1.size();
#ifdef _DEBUG
				//ShowMatch( pSrc1, pSrc2, &vecMatchedPoints1[0], &vecMatchedPoints2[0], (int)vecMatchedPoints1.size(), 
				//	_T("c:\\match-second-no-ransac.bmp") );
#endif

				if( !Ransac( pSrc1, pSrc2, vecMatchedPoints1, vecMatchedPoints2, aProjectMat, ransacDist ) )
					return false;
				// 再匹配结束		

				success = true;
			} 
			//while (nInnerOld<nInnerNew);
		}
		if( (int)m_vecInnerPoints1.size()>=innerPairsT )
		{
			success = true;
		}
		if((int)m_vecInnerPoints1.size()<10)
		{
			success = false;
		}
		else
		{
			success = true;
		}

		//float projectMatBetter[9];
		//int winR = 10;
		//int samplingTimes = 30;
		//FindBetterParameter( pSrc1, pSrc2, 
		//	m_aCorners1_32f, cornersNum1, m_aCorners2_32f, cornersNum2, 
		//	aProjectMat, projectMatBetter, winR, samplingTimes );
		//memcpy(aProjectMat, projectMatBetter, sizeof(float)*9 );

#ifdef _DEBUG
		cout<<"last max-dist-error "<<aProjectMat[8]<<endl;
#endif
	}

	return success;
}

// 选择最好的匹配对
// From: David J. Holtkamp and A. Ardeshir Goshtasby. "Precision Registration and Mosaicking of Multi-camera Images". 2009
bool CMosaicHarris::SelectBestMatchPairs_ProjectiveInvariant( SfPoint* pMatchPoints1, SfPoint* pMatchPoints2, int pairsNum, 
															 SfPoint best5Pairs1[5], SfPoint best5Pairs2[5] )
{
	if( (pMatchPoints1==NULL) || (pMatchPoints2==NULL) )
		return false;

	if(pairsNum<5)
		return false;

	vector<Sample> samplingResult;

	Sample bestSample;
	float minD = 99999999.f;

	bool findBest = false;
	
	Sampling( samplingResult, pairsNum, 5 );

	int nSam = samplingResult.size();
	for(int i=0; i<nSam; i++)
	{
		float I1xy[2] = {0}, I2xy[2] = {0};
		
		bool isValid = true;

		for(int p=0; p<2; p++)
		{
			float m431 = 0, m521= 0, m421 = 0, m531 = 0, m532 = 0, m432 = 0;
			float x[6], y[6];
			if(p==0)
			{
				for(int j=0; j<5; j++)
				{
					int indexSam = samplingResult[i].subSample[j];
					x[j+1] = pMatchPoints1[indexSam].x;
					y[j+1] = pMatchPoints1[indexSam].y;
				}
			}
			else
			{
				for(int j=0; j<5; j++)
				{
					int indexSam = samplingResult[i].subSample[j];
					x[j+1] = pMatchPoints2[indexSam].x;
					y[j+1] = pMatchPoints2[indexSam].y;
				}
			}

			float a431[9] = {x[4], x[3], x[1],
							 y[4], y[3], y[1],
							  1,    1,   1 };
			float a521[9] = {x[5], x[2], x[1],
							 y[5], y[2], y[1],
							  1,    1,   1 };
			float a421[9] = {x[4], x[2], x[1],
							 y[4], y[2], y[1],
							  1,    1,   1 };
			float a531[9] = {x[5], x[3], x[1],
							 y[5], y[3], y[1],
							  1,    1,   1 };
			float a532[9] = {x[5], x[3], x[2],
							 y[5], y[3], y[2],
							  1,    1,   1 };
			float a432[9] = {x[4], x[3], x[2],
							 y[4], y[3], y[2],
							  1,    1,   1 };
			// ---------------------------------------------------
			float small_num = 5.f;
			float a541[9] = {x[5], x[4], x[1],
							y[5], y[4], y[1],
							1,    1,   1 };
			float m541 = 0;
			CalDet33( a541, m541 );
			if(abs(m541)<small_num)
			{
				isValid = false;
				break;
			}

			float a543[9] = {x[5], x[4], x[3],
							y[5], y[4], y[3],
							1,    1,   1 };
			float m543 = 0;
			CalDet33( a543, m543 );
			if(abs(m543)<small_num)
			{
				isValid = false;
				break;
			}

			float a542[9] = {x[5], x[4], x[2],
							y[5], y[4], y[2],
							1,    1,   1 };
			float m542 = 0;
			CalDet33( a542, m542 );
			if(abs(m542)<small_num)
			{
				isValid = false;
				break;
			}

			float a321[9] = {x[3], x[2], x[1],
							y[3], y[2], y[1],
							1,    1,   1 };
			float m321 = 0;
			CalDet33( a321, m321 );
			if(abs(m321)<small_num)
			{
				isValid = false;
				break;
			}
			// --------------------------------------------------

			CalDet33( a431, m431 );
			if(abs(m431)<small_num)
			{
				isValid = false;
				break;
			}
			CalDet33( a521, m521 );
			if(abs(m521)<small_num)
			{
				isValid = false;
				break;
			}
			CalDet33( a421, m421 );
			if(abs(m421)<small_num)
			{
				isValid = false;
				break;
			}

			CalDet33( a531, m531 );
			if(abs(m531)<small_num)
			{
				isValid = false;
				break;
			}
			CalDet33( a532, m532 );
			if(abs(m532)<small_num)
			{
				isValid = false;
				break;
			}
			CalDet33( a432, m432 );
			if(abs(m432)<small_num)
			{
				isValid = false;
				break;
			}

			if( (m431==0) || (m521==0) || (m421==0) ||
				(m531==0) || (m532==0) || (m432==0) )
			{
				isValid = false;
			}
			else
			{
				I1xy[p] = m431 * m521 / (m421*m531);
				I2xy[p] = m421 * m532 / (m432*m521);
			}
		}
		if(isValid)
		{
			float D = 0;
			DistanceOfTwoPoints( I1xy[0], I2xy[0], I1xy[1], I2xy[1], D);

			if( (D<minD) && (D>0) )
			{
				minD = D;
				bestSample = samplingResult[i];

				findBest = true;
			}
		}
	}

	if(findBest)
	{
		for(int i=0; i<5; i++)
		{
			int index = bestSample.subSample[i];
			best5Pairs1[i].x = pMatchPoints1[index].x;
			best5Pairs1[i].y = pMatchPoints1[index].y;

			best5Pairs2[i].x = pMatchPoints2[index].x;
			best5Pairs2[i].y = pMatchPoints2[index].y;
		}
	}
	
	return findBest;
}

bool CMosaicHarris::EvaluateMatchQuality( BitmapImage* pSrc1, BitmapImage* pSrc2, 
										 SfPoint* pCorner1, int cornerNum1, 
										 SfPoint* pCorner2, int cornerNum2, 
										 int winR, 
										 float projectionMat[9], float &fScore )
{
	return true;
}

// 试图寻找更好的投影变换参数
bool CMosaicHarris::FindBetterParameter( BitmapImage* pSrc1, BitmapImage* pSrc2, 
										SfPoint* pCorner1, int cornerNum1, 
										SfPoint* pCorner2, int cornerNum2, 
										float projectMatSrc[9], float projectMatDst[9], 
										int winR, int sampleTimes )
{
	if( (pSrc1==NULL) || (pSrc2==NULL) || (pCorner1==NULL) || (pCorner2==NULL) )
		return false;

	//随机采样
	srand((unsigned)time(0));

	const int MIN_SAMPLING_NUM = 4;
	//SfPoint samPoint1Best[MIN_SAMPLING_NUM];
	//SfPoint samPoint2Best[MIN_SAMPLING_NUM];

	float maxScore = 0;
	float bestProjectMat[9] = {0};
	memcpy( bestProjectMat, projectMatSrc, sizeof(float)*9 );
	EvaluateMatchQuality( pSrc1, pSrc2, pCorner1, cornerNum1, 
		pCorner2, cornerNum2, winR, bestProjectMat, maxScore );

	int ptNum = (int)m_vecInnerPoints1.size();

	int realSamTimes = 0;
	const int maxTimes = 1000;
	for ( int t=0; t<sampleTimes; )
	{
		realSamTimes++;

		if(realSamTimes>=maxTimes)
		{
			break;
		}

		int aSampleIndex[MIN_SAMPLING_NUM];

		SfPoint samPoint1[MIN_SAMPLING_NUM];
		SfPoint samPoint2[MIN_SAMPLING_NUM];

		do
		{
			aSampleIndex[0] = rand()%ptNum;
			aSampleIndex[1] = rand()%ptNum;
			aSampleIndex[2] = rand()%ptNum;
			aSampleIndex[3] = rand()%ptNum;
		}
		while( ( aSampleIndex[0]==aSampleIndex[1] ) || 
			( aSampleIndex[0]==aSampleIndex[2] ) ||
			( aSampleIndex[0]==aSampleIndex[3] ) ||
			( aSampleIndex[1]==aSampleIndex[2] ) ||
			( aSampleIndex[1]==aSampleIndex[3] ) ||
			( aSampleIndex[2]==aSampleIndex[3] ) ); // 要求采样4个不同的点

		// 采样
		samPoint1[0] = m_vecInnerPoints1[aSampleIndex[0]];
		samPoint1[1] = m_vecInnerPoints1[aSampleIndex[1]];
		samPoint1[2] = m_vecInnerPoints1[aSampleIndex[2]];
		samPoint1[3] = m_vecInnerPoints1[aSampleIndex[3]];

		samPoint2[0] = m_vecInnerPoints2[aSampleIndex[0]];
		samPoint2[1] = m_vecInnerPoints2[aSampleIndex[1]];
		samPoint2[2] = m_vecInnerPoints2[aSampleIndex[2]];
		samPoint2[3] = m_vecInnerPoints2[aSampleIndex[3]];

		float aProjectMat2[9] = {0};
		SolveProjectMatrix3( samPoint1, samPoint2, MIN_SAMPLING_NUM, aProjectMat2 );
		if(aProjectMat2[8]>5)
		{
			continue;
		}

		//float motion[9], motion0[9];
		ProjectMat motion, motion0;
		memcpy( motion0.m, aProjectMat2, sizeof(float)*9 );
		NonlinearLeastSquareProjection(  samPoint1, samPoint2, MIN_SAMPLING_NUM, 
			motion.m, motion0.m, 1e-6f );

#ifdef _DEBUG
		BitmapImage* pResultImage = NULL;
		Merge( pSrc1, pSrc2, pResultImage, aProjectMat2 );
		//SaveImageATL( pResultImage, _T("c:\\tryMatch.bmp") );
		ReleaseBitmap8U( pResultImage );

		ShowMatch( pSrc1, pSrc2, samPoint1, samPoint2, 4, _T("c:\\match-try.bmp") );
#endif

		float currentScore = 0;
		//EvaluateMatchQuality( pSrc1, pSrc2, pCorner1, cornerNum1, 
		//	pCorner2, cornerNum2, winR, aProjectMat2, currentScore );

		// 用非线性最小二乘的结果
		EvaluateMatchQuality( pSrc1, pSrc2, pCorner1, cornerNum1, 
			pCorner2, cornerNum2, winR, motion.m, currentScore );

		if(currentScore>maxScore)
		{
			maxScore = currentScore;
			memcpy( bestProjectMat, aProjectMat2, sizeof(float)*9 );

#ifdef _DEBUG
			cout<<"better maxScore "<<maxScore<<endl;
#endif
		}

		t++; // 采样数
	}

#ifdef _DEBUG
	cout<<"maxScore "<<maxScore<<endl;
#endif

	memcpy( projectMatDst, bestProjectMat, sizeof(float)*9 );

	return true;
}

// 线匹配
bool CMosaicHarris::LineCorrespondence( int *pSingleLine, pool::BitmapImage* pTargetLPImage,
									   int minDx, int maxDx,
									   float &maxScore, int &maxScorePos )
{

	return true;
}

bool CMosaicHarris::LineCorrespondence( int *pSingleLine11, int *pSingleLine12,
									   pool::BitmapImage* pTargetLPImage,
									   int minDx, int maxDx,
									   float &maxScore, int &maxScorePos )
{

	return true;
}

// 功能：由第2张图片和第1张图片之间的变换关系，计算出一个变换矩阵H'，以使得图片1和图片2之间只有仿射变换关系
void EliminateProjection(float H12[9], float H_[9])
{
	float h6_, h7_, h0, h1, h3, h4, h6, h7;
	h0 = H12[0]; h3 = H12[3]; h6 = H12[6];
	h1 = H12[1]; h4 = H12[4]; h7 = H12[7];
	float A[4] = {h0-1, h3, h1, h4-1}, invA[4];
	InverseMatrix( A, 2, invA );
	h6_ = -invA[0] * h6 - invA[1] * h7;
	h7_ = -invA[2] * h6 - invA[3] * h7;

	memset( H_, 0, sizeof(float)*9 );
	H_[0] = 1; H_[4] = 1; H_[8] = 1;
	H_[6] = h6_; H_[7] = h7_;
	// 1   0   0 
	// 0   1   0
	// h6_ h7_ 1
}

// 功能：用最基本的Harris角点+NCC方法，拼接多幅图像
// 假设：相邻的两幅图像有足够大的重叠区
// 要求输入的图像的通道数全部相同（全部为彩色或全为灰度图）
// 2012-5-12
pool::BitmapImage* MosaicMultiImages(BitmapImage* pImages[], int imagesNum, 
									 int &errorCode,
									 int maxFeatruesNum, int winR, 
									 float minScore,
									 float ransacDist/**/,
									 int searchR/**/)
{	


	return NULL;
}

// 功能：用最基本的Harris角点+NCC方法，拼接多幅图像
// 假设：相邻的两幅图像有足够大的重叠区
// 要求输入的图像的通道数全部相同（全部为彩色或全为灰度图）
// 2012-5-12
IplImage* MosaicMultiImages3(IplImage** pImages, int imagesNum, 
									 int &errorCode,
									 int maxFeatruesNum, int winR, 
									 float minScore,
									 float ransacDist, /**/
									 int searchR, /**/
									 int referenceType, 
									 int blending )
{	

	return NULL;
}


// 功能:计算新加入的图像和已有的拼接图像的重叠类型
// pMergedImg-------------已经融合的大图
// pQuadrangleCorners-----即将加入的图像的4个顶点坐标
int CalOverlapType2(IplImage* pMergedImg, SfPoint* pQuadrangleCorners0, int &overlapType)
{
	// 对4个点进行分析，确定4个点的位置关系
	// 0 1
	// 3 2
	float massX = 0, massY = 0;
	for(int n=0; n<4; n++)
	{
		massX += pQuadrangleCorners0[n].x;
		massY += pQuadrangleCorners0[n].y;
	}
	massX /= 4;
	massY /= 4;
	float angle[4];
	for(int n=0; n<4; n++)
	{
		SfPoint vecMass_Pt(0,0);
		vecMass_Pt.x = pQuadrangleCorners0[n].x - massX;
		vecMass_Pt.y = pQuadrangleCorners0[n].y - massY;
		AngleofPoint360(vecMass_Pt.x, vecMass_Pt.y, angle[n]);
	}
	float minAngle, maxAngle;
	unsigned int minIndex, maxIndex;
	FindMinValue(angle, 4, minAngle, minIndex);
	FindMaxValue(angle, 4, maxAngle, maxIndex);
	SfPoint pQuadrangleCorners[4];
	pQuadrangleCorners[2] = pQuadrangleCorners0[minIndex]; // 最小的角度对应点2
	pQuadrangleCorners[1] = pQuadrangleCorners0[maxIndex]; // 最大的角度对应点1
	int resNum[2], nRes = 0;
	for(int n=0; n<4; n++)
	{
		if( (n!=maxIndex) && (n!=minIndex) )
		{
			resNum[nRes] = n;
			nRes++;
		}
	}
	if(angle[resNum[0]] > angle[resNum[1]])
	{
		pQuadrangleCorners[0] = pQuadrangleCorners0[resNum[0]];
		pQuadrangleCorners[3] = pQuadrangleCorners0[resNum[1]];
	}
	else
	{
		pQuadrangleCorners[0] = pQuadrangleCorners0[resNum[1]];
		pQuadrangleCorners[3] = pQuadrangleCorners0[resNum[0]];
	}

	int grayCorners[4] = {0};
	int wsMerged = pMergedImg->widthStep;
	for(int n=0; n<4; n++)
	{
		int x32I = (int)pQuadrangleCorners[n].x;
		int y32I = (int)pQuadrangleCorners[n].y;
		grayCorners[n] = *((unsigned char*)pMergedImg->imageData + y32I*wsMerged + x32I*3);
	}
	if( (grayCorners[0]==0)&&(grayCorners[1]==0)&&(grayCorners[2]>0)&&(grayCorners[3]>0) )
	{
		overlapType = 1;
	}
	if( (grayCorners[0]==0)&&(grayCorners[1]>0)&&(grayCorners[2]>0)&&(grayCorners[3]>0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]==0)&&(grayCorners[2]>0)&&(grayCorners[3]>0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]==0)&&(grayCorners[2]==0)&&(grayCorners[3]==0) )
	{
		overlapType = 2;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]==0)&&(grayCorners[2]==0)&&(grayCorners[3]>0) )
	{
		overlapType = 3;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]==0)&&(grayCorners[2]>0)&&(grayCorners[3]>0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]>0)&&(grayCorners[2]==0)&&(grayCorners[3]>0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]==0)&&(grayCorners[1]==0)&&(grayCorners[2]==0)&&(grayCorners[3]>0) )
	{
		overlapType = 4;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]>0)&&(grayCorners[2]==0)&&(grayCorners[3]==0) )
	{
		overlapType = 5;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]>0)&&(grayCorners[2]>0)&&(grayCorners[3]==0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]>0)&&(grayCorners[2]==0)&&(grayCorners[3]>0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]==0)&&(grayCorners[1]>0)&&(grayCorners[2]==0)&&(grayCorners[3]==0) )
	{
		overlapType = 6;
	}
	if( (grayCorners[0]==0)&&(grayCorners[1]>0)&&(grayCorners[2]>0)&&(grayCorners[3]==0) )
	{
		overlapType = 7;
	}
	if( (grayCorners[0]==0)&&(grayCorners[1]>0)&&(grayCorners[2]>0)&&(grayCorners[3]>0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]>0)&&(grayCorners[2]>0)&&(grayCorners[3]==0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]==0)&&(grayCorners[1]==0)&&(grayCorners[2]>0)&&(grayCorners[3]==0) )
	{
		overlapType = 8;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]>0)&&(grayCorners[2]>0)&&(grayCorners[3]>0) )
	{
		overlapType = 9;
	}

	return 0;
}

// 功能:计算新加入的图像和已有的拼接图像的重叠类型
// pMergedImg-------------已经融合的大图
// pQuadrangleCorners-----即将加入的图像的4个顶点坐标
int CalOverlapType2(BitmapImage* pMergedImg, SfPoint* pQuadrangleCorners0, int &overlapType)
{
	// 对4个点进行分析，确定4个点的位置关系
	// 0 1
	// 3 2
	float massX = 0, massY = 0;
	for(int n=0; n<4; n++)
	{
		massX += pQuadrangleCorners0[n].x;
		massY += pQuadrangleCorners0[n].y;
	}
	massX /= 4;
	massY /= 4;
	float angle[4];
	for(int n=0; n<4; n++)
	{
		SfPoint vecMass_Pt(0,0);
		vecMass_Pt.x = pQuadrangleCorners0[n].x - massX;
		vecMass_Pt.y = pQuadrangleCorners0[n].y - massY;
		AngleofPoint360(vecMass_Pt.x, vecMass_Pt.y, angle[n]);
	}
	float minAngle, maxAngle;
	unsigned int minIndex, maxIndex;
	FindMinValue(angle, 4, minAngle, minIndex);
	FindMaxValue(angle, 4, maxAngle, maxIndex);
	SfPoint pQuadrangleCorners[4];
	pQuadrangleCorners[2] = pQuadrangleCorners0[minIndex]; // 最小的角度对应点2
	pQuadrangleCorners[1] = pQuadrangleCorners0[maxIndex]; // 最大的角度对应点1
	int resNum[2], nRes = 0;
	for(int n=0; n<4; n++)
	{
		if( (n!=maxIndex) && (n!=minIndex) )
		{
			resNum[nRes] = n;
			nRes++;
		}
	}
	if(angle[resNum[0]] > angle[resNum[1]])
	{
		pQuadrangleCorners[0] = pQuadrangleCorners0[resNum[0]];
		pQuadrangleCorners[3] = pQuadrangleCorners0[resNum[1]];
	}
	else
	{
		pQuadrangleCorners[0] = pQuadrangleCorners0[resNum[1]];
		pQuadrangleCorners[3] = pQuadrangleCorners0[resNum[0]];
	}

	int grayCorners[4] = {0};
	int wsMerged = pMergedImg->widthStep;
	for(int n=0; n<4; n++)
	{
		int x32I = (int)pQuadrangleCorners[n].x;
		int y32I = (int)pQuadrangleCorners[n].y;
		grayCorners[n] = *(pMergedImg->imageData + y32I*wsMerged + x32I*3);
	}
	if( (grayCorners[0]==0)&&(grayCorners[1]==0)&&(grayCorners[2]>0)&&(grayCorners[3]>0) )
	{
		overlapType = 1;
	}
	if( (grayCorners[0]==0)&&(grayCorners[1]>0)&&(grayCorners[2]>0)&&(grayCorners[3]>0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]==0)&&(grayCorners[2]>0)&&(grayCorners[3]>0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]==0)&&(grayCorners[2]==0)&&(grayCorners[3]==0) )
	{
		overlapType = 2;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]==0)&&(grayCorners[2]==0)&&(grayCorners[3]>0) )
	{
		overlapType = 3;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]==0)&&(grayCorners[2]>0)&&(grayCorners[3]>0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]>0)&&(grayCorners[2]==0)&&(grayCorners[3]>0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]==0)&&(grayCorners[1]==0)&&(grayCorners[2]==0)&&(grayCorners[3]>0) )
	{
		overlapType = 4;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]>0)&&(grayCorners[2]==0)&&(grayCorners[3]==0) )
	{
		overlapType = 5;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]>0)&&(grayCorners[2]>0)&&(grayCorners[3]==0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]>0)&&(grayCorners[2]==0)&&(grayCorners[3]>0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]==0)&&(grayCorners[1]>0)&&(grayCorners[2]==0)&&(grayCorners[3]==0) )
	{
		overlapType = 6;
	}
	if( (grayCorners[0]==0)&&(grayCorners[1]>0)&&(grayCorners[2]>0)&&(grayCorners[3]==0) )
	{
		overlapType = 7;
	}
	if( (grayCorners[0]==0)&&(grayCorners[1]>0)&&(grayCorners[2]>0)&&(grayCorners[3]>0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]>0)&&(grayCorners[2]>0)&&(grayCorners[3]==0) )
	{
		overlapType = 9;
	}
	if( (grayCorners[0]==0)&&(grayCorners[1]==0)&&(grayCorners[2]>0)&&(grayCorners[3]==0) )
	{
		overlapType = 8;
	}
	if( (grayCorners[0]>0)&&(grayCorners[1]>0)&&(grayCorners[2]>0)&&(grayCorners[3]>0) )
	{
		overlapType = 9;
	}
	
	return 0;
}

// 计算重叠类型
int CalOverlapType(SfPoint currentImageSizeBig, SfPoint currentImageSizeSmall,
				   int begBoxX, int begBoxY, int endBoxX, int endBoxY)
{
	int overlapType = 0;
	if( (begBoxX<currentImageSizeBig.x) &&
		(endBoxX>currentImageSizeBig.x) &&
		(begBoxY<currentImageSizeBig.y) &&
		(endBoxY>currentImageSizeBig.y) )
	{
		return 2;
	}
	if( (begBoxX<currentImageSizeBig.x) &&
		(endBoxX>currentImageSizeBig.x) &&
		(begBoxY<currentImageSizeSmall.y) &&
		(endBoxY>currentImageSizeSmall.y) &&
		(endBoxY<currentImageSizeBig.y) )
	{
		return 4;
	}
	if( (begBoxX>currentImageSizeSmall.x) &&
		(endBoxX<currentImageSizeBig.x) &&
		(begBoxY<currentImageSizeSmall.y) &&
		(endBoxY>currentImageSizeSmall.y) &&
		(endBoxY<currentImageSizeBig.y) )
	{
		return 1;
	}
	if( (begBoxX>currentImageSizeSmall.x) &&
		(endBoxX<currentImageSizeBig.x) &&
		(begBoxY>currentImageSizeSmall.y) &&
		(endBoxY<currentImageSizeBig.y) )
	{
		return 9;
	}
	if( (begBoxX>currentImageSizeSmall.x) &&
		(begBoxX<currentImageSizeBig.x) &&
		(endBoxX>currentImageSizeBig.x) &&
		(begBoxY>currentImageSizeSmall.y) &&
		(endBoxY<currentImageSizeBig.y) )
	{
		return 3;
	}
	
	return -1;
}

// 以中间的图像为准，选择全局变换单应矩阵
int SelectHomographyMiddle(float hMiddle[9], float A[9])
{
	InverseMatrix( hMiddle, 3, A, 1e-12f );
	
	return 0;
}

// 给定单应矩阵, 对图像进行变换 (投影变换，透视变换)
int ImageProjectionTransform(BitmapImage* pImage, BitmapImage* &pResult, float h[9])
{
	if(pImage==NULL)
		return -1;

	int nChannels = pImage->nChannels;

	//确定新图像的大小
	float maxX = -1<<29, maxY = -1<<29, minX = 1<<29, minY = 1<<29;		
	SfPoint corner[4];
	for(int n=0; n<1; n++)
	{
		// 对图像的各个顶点进行变换
		corner[0].x = 0;
		corner[0].y = 0;
		corner[1].x = float(pImage->width-1);
		corner[1].y = 0;
		corner[2].x = float(pImage->width-1);
		corner[2].y = float(pImage->height-1);
		corner[3].x = 0; 
		corner[3].y = float(pImage->height-1);

		for(int i=0; i<4; i++)
		{
			float xSrc = corner[i].x, ySrc = corner[i].y;
			float xDst, yDst;
			float* pM = h;
			xDst = (xSrc*pM[0] + ySrc*pM[1] + pM[2]) / (xSrc*pM[6] + ySrc*pM[7] + pM[8]);
			yDst = (xSrc*pM[3] + ySrc*pM[4] + pM[5]) / (xSrc*pM[6] + ySrc*pM[7] + pM[8]);

			if(xDst>maxX)
				maxX = xDst;
			if(xDst<minX)
				minX = xDst;
			if(yDst>maxY)
				maxY = yDst;
			if(yDst<minY)
				minY = yDst;
		}
	}
	// 新的图像宽度
	int newWidth = int(maxX - minX + 1.5f);
	int newHeight = int(maxY - minY + 1.5f);

	pResult = CreateBitmap8U(newWidth, newHeight, nChannels);
	int wsNew = pResult->widthStep;
	float dx = -minX, dy = -minY;
	ZeroImage(pResult);

	cout<<"dx "<<dx<<" dy "<<dy<<endl;

	float* pM = h;
	float pInvM[9];
	InverseMatrix(pM, 3, pInvM); // 求变换矩阵的逆矩阵

	for(int yDst=0; yDst<newHeight; yDst++)
	{
		unsigned char* pRowDst = pResult->imageData + yDst*wsNew;
		for(int xDst=0; xDst<newWidth; xDst++)
		{
			float xDst32F = xDst - dx, yDst32F = yDst - dy;
			float xSrc32F, ySrc32F;
			xSrc32F = (xDst32F*pInvM[0] + yDst32F*pInvM[1] + pInvM[2]) / (xDst32F*pInvM[6] + yDst32F*pInvM[7] + pInvM[8]);
			ySrc32F = (xDst32F*pInvM[3] + yDst32F*pInvM[4] + pInvM[5]) / (xDst32F*pInvM[6] + yDst32F*pInvM[7] + pInvM[8]);

			int srcW_1 = pImage->width - 1, srcH_1 = pImage->height-1, wsSrc = pImage->widthStep;
			//双线性插值
			int ySrc = int(ySrc32F);
			int xSrc = int(xSrc32F);
			// 双线性插值
			if( (xSrc32F>=0) && (xSrc32F<srcW_1) && (ySrc32F>=0) && (ySrc32F<srcH_1) )
			{
				float p = ySrc32F - ySrc;
				float q = xSrc32F - xSrc;

				if(nChannels==1)
				{
					unsigned char* pSrcTemp = pImage->imageData + ySrc*wsSrc + xSrc;

					unsigned char gray = unsigned char((unsigned char)*(pSrcTemp)*(1-p)*(1-q) + 
						(unsigned char)*(pSrcTemp + 1)*(1-p)*q + 
						(unsigned char)*(pSrcTemp + wsSrc)*p*(1-q) + 
						(unsigned char)*(pSrcTemp + wsSrc + 1)*p*q );
					if(*(pRowDst + xDst) >0 )
					{
						*(pRowDst + xDst) = int( (gray + *(pRowDst + xDst))*0.5f );
					}
					else
					{
						*(pRowDst + xDst) = gray;
					}
				}
				else if(3==nChannels)
				{
					float weightR = -1, weightG = -1, weightB = -1;
					int i = ySrc;
					int j = xSrc;
					{	//B
						unsigned char* pTempB = pImage->imageData + i*wsSrc+ 3*j;
						unsigned char grayB1, grayB2, grayB3, grayB4;
						grayB1 = (unsigned char)*(pTempB);
						grayB2 = (unsigned char)*(pTempB + 3);
						grayB3 = (unsigned char)*(pTempB + wsSrc);
						grayB4 = (unsigned char)*(pTempB + wsSrc + 3);

						unsigned char gray = unsigned char( grayB1*(1-p)*(1-q) + 
							grayB2*(1-p)*q + 
							grayB3*p*(1-q) + 
							grayB4*p*q );
						pRowDst[3*xDst] = gray;
					}
					{	//G
						unsigned char* pTempG = pImage->imageData + i*wsSrc+ 3*j + 1;
						unsigned char grayG1, grayG2, grayG3, grayG4;
						grayG1 = (unsigned char)*(pTempG);
						grayG2 = (unsigned char)*(pTempG + 3);
						grayG3 = (unsigned char)*(pTempG + wsSrc);
						grayG4 = (unsigned char)*(pTempG + wsSrc + 3);

						unsigned char gray = unsigned char( grayG1*(1-p)*(1-q) + 
							grayG2*(1-p)*q + 
							grayG3*p*(1-q) + 
							grayG4*p*q );
						pRowDst[3*xDst+1] = gray;
					}
					{	//R		
						unsigned char* pTempR = pImage->imageData + i*wsSrc+ 3*j + 2;
						unsigned char grayR1, grayR2, grayR3, grayR4;
						grayR1 = (unsigned char)*(pTempR);
						grayR2 = (unsigned char)*(pTempR + 3);
						grayR3 = (unsigned char)*(pTempR + wsSrc);
						grayR4 = (unsigned char)*(pTempR + wsSrc + 3);

						unsigned char gray = unsigned char( grayR1*(1-p)*(1-q) + 
							grayR2*(1-p)*q + 
							grayR3*p*(1-q) + 
							grayR4*p*q );
							pRowDst[3*xDst+2] = gray;
					}
				}
			}
		}
	}

	return 0;
}

// 距离图
int FindMasksByDistMap(IplImage** pMasks, 
					   _IN int nImages, _IN Rectangle4Points* pRects, _IN cv::Point* pTL, int rectW, int rectH)
{
	cout<<"number of valid images "<<nImages<<endl;

	if(pMasks[0]->nChannels>1)
	{
		return -2;
	}
	if( (NULL==pMasks) || (NULL==pRects) )
		return -1;

	// 计算距离图
	vector<float*> vecDistMap;
	for(int n=0; n<nImages; n++)
	{
		float A[4], B[4], C[4];
		LineOf2Points1(A[0], B[0], C[0], pRects[n].pt[0].x, pRects[n].pt[0].y, pRects[n].pt[1].x, pRects[n].pt[1].y);
		LineOf2Points1(A[1], B[1], C[1], pRects[n].pt[1].x, pRects[n].pt[1].y, pRects[n].pt[2].x, pRects[n].pt[2].y);
		LineOf2Points1(A[2], B[2], C[2], pRects[n].pt[2].x, pRects[n].pt[2].y, pRects[n].pt[3].x, pRects[n].pt[3].y);
		LineOf2Points1(A[3], B[3], C[3], pRects[n].pt[3].x, pRects[n].pt[3].y, pRects[n].pt[0].x, pRects[n].pt[0].y);

		float inv_A2_plus_B2[4];
		for(int i=0; i<4; i++)
		{
			inv_A2_plus_B2[i] = 1/sqrt(A[i]*A[i] + B[i]*B[i]);
		}

		int w = pMasks[n]->width, h = pMasks[n]->height, ws = pMasks[n]->widthStep;
		float* pMap = new float[ws * h];
		memset(pMap, 0, ws*h*sizeof(float));
		
		float maxDist = 0;
		for(int r=0; r<h; r++)
		{
			unsigned char* pMaskRow = (unsigned char*)pMasks[n]->imageData + ws * r;
			float *pMapRow = pMap + r * ws;
			for(int c=0; c<w; c++)
			{
				if(0==pMaskRow[c])
				{
					continue;
				}
				// 计算点到直线的距离
				float dist[4];
				float minDist = 1<<29;
				for(int i=0; i<4; i++)
				{
					//DistanceOfPointToABCLine((float)c, (float)r, A[i], B[i], C[i], dist[i]);
					//
					dist[i] = abs(A[i]*c + B[i]*r + C[i]) * inv_A2_plus_B2[i];

					if(dist[i]<minDist)
					{
						minDist = dist[i];
					}
				}
				pMapRow[c] = minDist;
				if(minDist>maxDist)
				{
					maxDist = minDist;
				}
			}
		}
		for(int r=0; r<h; r++)
		{
			float *pMapRow = pMap + r * ws;
			for(int c=0; c<w; c++)
			{
				pMapRow[c] /= maxDist; // 距离图归一化
			}
		}
		vecDistMap.push_back(pMap);
	}

	for(int n=0; n<nImages; n++)
	{
		cvZero(pMasks[n]);
	}

	// 根据距离图, 更新mask
	for(int r=0; r<rectH; r++)
	{
		for(int c=0; c<rectW; c++)
		{
			if( (c==302) && (r==999) )
			{
				c = c;
			}
			int maxIdx = -1;
			float maxDist = 0;
			for(int n=0; n<nImages; n++)
			{
				int w = pMasks[n]->width, h = pMasks[n]->height, ws = pMasks[n]->widthStep;
				int yC = r-pTL[n].y, xC = c-pTL[n].x;
				if( (yC>=0) && (yC<h) && (xC>=0) && (xC<w) )
				{
					float currentDist = *(vecDistMap[n] + yC*ws + xC);
					if(currentDist>maxDist)
					{
						maxDist = currentDist;
						maxIdx = n;
					}
				}
			}
			if(maxIdx>=0)
			{
				int yC = r-pTL[maxIdx].y, xC = c-pTL[maxIdx].x;
				*(pMasks[maxIdx]->imageData + pMasks[maxIdx]->widthStep * yC + xC) = 255;
			}
		}
	}

	// 释放内存
	for(int n=0; n<nImages; n++)
	{
		delete[] vecDistMap[n]; vecDistMap[n] = NULL;
	}

	return 0;
}

// 求四边形的面积
int AreaOfQuadrangle(SfPoint corner[4], float &area)
{
	float v02x = corner[0].x - corner[2].x; // 对角线
	float v02y = corner[0].y - corner[2].y;
	float v13x = corner[1].x - corner[3].x; // 对角线
	float v13y = corner[1].y - corner[3].y;
	
	float L1 = sqrt(v02x*v02x + v02y*v02y);
	float L2 = sqrt(v13x*v13x + v13y*v13y);
	//if(IsPointOnLineSegmentOfTwoPoints())

	// 4点是否在同一直线上
	float L01, L02, L12;
	float L03, L23;
	DistanceOfTwoPoints(corner[0].x, corner[0].y, corner[1].x, corner[1].y, L01);
	DistanceOfTwoPoints(corner[0].x, corner[0].y, corner[2].x, corner[2].y, L02);
	DistanceOfTwoPoints(corner[1].x, corner[1].y, corner[2].x, corner[2].y, L12);

	DistanceOfTwoPoints(corner[0].x, corner[0].y, corner[2].x, corner[2].y, L02);
	DistanceOfTwoPoints(corner[0].x, corner[0].y, corner[3].x, corner[3].y, L03);
	DistanceOfTwoPoints(corner[2].x, corner[2].y, corner[3].x, corner[3].y, L23);

	float P1 = (L01 + L02 + L12) * 0.5f;
	float S1 = sqrt(P1*(P1-L01)*(P1-L02)*(P1-L12));
	
	float P2 = (L02 + L03 + L23) * 0.5f;
	float S2 = sqrt(P2*(P2-L02)*(P2-L03)*(P2-L23));

	if( (abs(S1)<1e-4) && (abs(S2)<1e-4) ) // 如果两个三角形的面积都为0
	{
		area = 0;
		return 0;
	}

	if(L1*L2>0)
	{
		float cosTheta = (v02x*v13x + v02y*v13y)/(L1*L2);
		float theta = acos(cosTheta);
		if(theta<0)
			theta = theta + 3.1415926f;

		area = 0.5*L1*L2*sin(theta);
	}
	else
		area = 0;
	
	return 0;
}

// 判断1个点是否落在2点构成的线段上
bool IsPointOnLineSegmentOfTwoPoints(SfPoint pt, SfPoint pt1, SfPoint pt2 )
{
	float dist12;
	DistanceOfTwoPoints(pt1.x, pt1.y, pt2.x, pt2.y, dist12);

	float dist01, dist02;
	DistanceOfTwoPoints(pt.x, pt.y, pt1.x, pt1.y, dist01);

	DistanceOfTwoPoints(pt.x, pt.y, pt2.x, pt2.y, dist02);

	if(abs(dist01+dist02-dist12)<1e-4)
		return true;
	else
		return false;
}

// 求两个4边形所有的交点
int GetAllIntersecPoints(SfPoint corner1[4], SfPoint corner2[4], vector<SfPoint> &vecIntersecPoints)
{
	for(int n=0; n<4; n++)
	{
		vecIntersecPoints.push_back(corner1[n]);
		vecIntersecPoints.push_back(corner2[n]);
	}

	int ind1[4] = {0,1,2,3};
	int ind2[4] = {1,2,3,0};

	for(int n1=0; n1<4;n1++)
	{
		//直线1
		float A1, B1, c1, rho1, theta1;
		LineOf2Points1(A1, B1, c1, 
			corner1[ind1[n1]].x, corner1[ind1[n1]].y,
			corner1[ind2[n1]].x, corner1[ind2[n1]].y);
		ABCToPolar(A1, B1, c1, rho1, theta1);

		for(int n2=0; n2<4; n2++)
		{
			//直线2
			float A2, B2, c2, rho2, theta2;
			LineOf2Points1(A2, B2, c2, 
				corner2[ind1[n2]].x, corner2[ind1[n2]].y,
				corner2[ind2[n2]].x, corner2[ind2[n2]].y);
			ABCToPolar(A2, B2, c2, rho2, theta2);

			//直线的交点
			SfPoint point;
			IntersectionPointOf2PolarLines(rho1, theta1, rho2, theta2, point);

			// 判断交点是否在直线上
			bool onLine1 = IsPointOnLineSegmentOfTwoPoints(point, corner1[ind1[n1]], corner1[ind2[n1]]);
			
			bool onLine2 = IsPointOnLineSegmentOfTwoPoints(point, corner2[ind1[n2]], corner2[ind2[n2]]);
			if(onLine1&&onLine2)
			{
				vecIntersecPoints.push_back(point);
			}
		}
	}

	return 0;
}

// 判断一个点是否在四边形内
int IsPointInQuadrangle(SfPoint point, SfPoint corner[4])
{
	int ind1[4] = {0,1,2,3};
	int ind2[4] = {1,2,3,0};

	bool outer = false;

	float area4 = 0;
	AreaOfQuadrangle(corner, area4);

	float accArea = 0 ;
	for(int n1=0; n1<4;n1++)
	{
		float area;
		SfPoint corner2[4];
		corner2[0] = corner[ind1[n1]];
		corner2[1] = corner[ind2[n1]];
		corner2[2] = point;
		corner2[3] = point;
		
		AreaOfQuadrangle(corner2, area);
		accArea += area;
	}

	if( abs(area4-accArea)<0.2 )
		return true;
	else 
		return false;
}

// 获取同时落在2个四边形内的点
int GetPointsInOverlapRegion(SfPoint corner1[4], SfPoint corner2[4],
							 vector<SfPoint> vecIntersecPoints, vector<SfPoint> &vecPointsOverlap0)
{
	vector<SfPoint> vecPointsOverlap;
	float cx = 0, cy = 0;
	for(int i=0; i<vecIntersecPoints.size(); i++)
	{
		bool inner1 = IsPointInQuadrangle(vecIntersecPoints[i], corner1);
		bool inner2 = IsPointInQuadrangle(vecIntersecPoints[i], corner2);
		if(inner1&&inner2)
		{
			vecPointsOverlap.push_back(vecIntersecPoints[i]);
			cx += vecIntersecPoints[i].x;
			cy += vecIntersecPoints[i].y;
		}
	}

	cx /= vecPointsOverlap.size();
	cy /= vecPointsOverlap.size();

	vector<Distance> vecAng;
	for(int i=0; i<vecPointsOverlap.size(); i++)
	{
		Distance angleArc;
		angleArc.seq = i;
		AngleofPoint360(vecPointsOverlap[i].x-cx, vecPointsOverlap[i].y-cy, angleArc.dist);
		vecAng.push_back(angleArc);
	}

	std::sort(vecAng.begin(), vecAng.end());

	for(int i=0; i<vecPointsOverlap.size(); i++)
	{
		vecPointsOverlap0.push_back(vecPointsOverlap[vecAng[i].seq]);
	}

	return 0;
}

// overlapT = 0.8
int ResampleByOverlap(IplImage** pImages, int imagesNum, float overlapT,
					  ProjectMat* pImgT, vector<int> &vecAbandonInd)
{
	vecAbandonInd.clear();
	for(int num=0; num<imagesNum; num++)
	{
		vecAbandonInd.push_back(1);
	}
	int nSatisfied = 0;
	SfPoint pQuadrangleCorners1[4]; // 4边形的顶点
	SfPoint pQuadrangleCorners2[4]; // 4边形的顶点
	SfPoint corner[4];
	vecAbandonInd[0] = 1; nSatisfied++;
	for(int n1=1; n1<imagesNum; n1++)
	{
		//cout<<"n1 "<<n1<<endl;
		bool satisfied = true;

		if(pImgT[n1].m[8]==0)
		{
			continue;
		}
		// 对图像的各个顶点进行变换
		corner[0].x = 0;
		corner[0].y = 0;
		corner[1].x = float(pImages[n1]->width-1);
		corner[1].y = 0;
		corner[2].x = float(pImages[n1]->width-1);
		corner[2].y = float(pImages[n1]->height-1);
		corner[3].x = 0; 
		corner[3].y = float(pImages[n1]->height-1);
		float boxMaxX = -1<<29, boxMaxY = -1<<29, boxMinX = 1<<29, boxMinY = 1<<29;
		for(int i=0; i<4; i++)
		{
			float xSrc = corner[i].x, ySrc = corner[i].y;
			float xDst, yDst;
			float* pM = pImgT[n1].m;
			xDst = (xSrc*pM[0] + ySrc*pM[1] + pM[2]) / 
				(xSrc*pM[6] + ySrc*pM[7] + pM[8]);
			yDst = (xSrc*pM[3] + ySrc*pM[4] + pM[5]) / 
				(xSrc*pM[6] + ySrc*pM[7] + pM[8]);

			pQuadrangleCorners1[i].x = xDst;
			pQuadrangleCorners1[i].y = yDst;
		}

		//cout<<"AreaOfQuadrangle"<<endl;
		// 求四边形的面积
		float area1 = 0;
		AreaOfQuadrangle(pQuadrangleCorners1, area1);

		for(int n2=0; n2<n1; n2++)
		{
			if(vecAbandonInd[n2]==0)
				continue;

			if(pImgT[n2].m[8]==0)
			{
				continue;
			}

			//cout<<"n2 "<<n2<<endl;

			// 对图像的各个顶点进行变换
			corner[0].x = 0;
			corner[0].y = 0;
			corner[1].x = float(pImages[n2]->width-1);
			corner[1].y = 0;
			corner[2].x = float(pImages[n2]->width-1);
			corner[2].y = float(pImages[n2]->height-1);
			corner[3].x = 0; 
			corner[3].y = float(pImages[n2]->height-1);
			float boxMaxX = -1<<29, boxMaxY = -1<<29, boxMinX = 1<<29, boxMinY = 1<<29;
			for(int i=0; i<4; i++)
			{
				float xSrc = corner[i].x, ySrc = corner[i].y;
				float xDst, yDst;
				float* pM = pImgT[n2].m;
				xDst = (xSrc*pM[0] + ySrc*pM[1] + pM[2]) / 
					(xSrc*pM[6] + ySrc*pM[7] + pM[8]);
				yDst = (xSrc*pM[3] + ySrc*pM[4] + pM[5]) / 
					(xSrc*pM[6] + ySrc*pM[7] + pM[8]);

				pQuadrangleCorners2[i].x = xDst;
				pQuadrangleCorners2[i].y = yDst;
			}
			
			//cout<<"GetAllIntersecPoints"<<endl;

			// 求出所有线段的交点和顶点
			vector<SfPoint> vecIntersecPoints;
			GetAllIntersecPoints(pQuadrangleCorners1, pQuadrangleCorners2, vecIntersecPoints);

			//cout<<"GetPointsInOverlapRegion"<<endl;

			// 求出同时落在两个四边形内的所有交点
			vector<SfPoint> vecPointsOverlap;
			GetPointsInOverlapRegion(pQuadrangleCorners1, pQuadrangleCorners2,
				vecIntersecPoints, vecPointsOverlap);

			// 计算两个四边形的重叠区域的面积
			if(vecPointsOverlap.size()==3)
			{
				SfPoint temp = vecPointsOverlap[2];
				vecPointsOverlap.push_back(temp);
			}

			//cout<<"AreaOfQuadrangle"<<endl;

			float area2 = 0;
			if(vecPointsOverlap.size()==4)
				AreaOfQuadrangle(&vecPointsOverlap[0], area2);

			// 如果
			float areaRatio = area2/area1;
			if( areaRatio>overlapT )
			{
				satisfied = false;
				break;
			}
		}
		if(satisfied==false)
		{
			vecAbandonInd[n1] = 0;
		}
	}

	//cout<<"vecAbandonInd"<<endl;
	vecAbandonInd[imagesNum-1] = 1;

	return 0;
}

// 功能：拉普拉斯金字塔图像融合
// pImgT为每张图像的变换参数
IplImage* LaplacianPyramidBlending(IplImage** pImages, int imagesNum, 
								   ProjectMat* pImgT, int band, float resScale)
{
	cout<<"LaplacianPyramidBlending..."<<endl;

	if(NULL==pImages)
		return NULL;

	int nChannels = pImages[0]->nChannels;
	IplImage* pMosaicResult = NULL;

	float scale = resScale;
	for(int i=0; i<imagesNum; i++)
	{
		for(int j=0; j<6; j++)
		{
			pImgT[i].m[j] *= scale;
		}
	}

	// 删除一些重叠很大的图像
	//int ResampleByOverlap()
	vector<int> vecAbandonInd;
	ResampleByOverlap(pImages, imagesNum, 0.7f,
		pImgT, 
		vecAbandonInd);

	//确定新图像的大小
	float maxX = 0, maxY = 0, minX = 0, minY = 0;		
	SfPoint* pBegBox = new SfPoint[imagesNum+4];
	SfPoint* pEndBox = new SfPoint[imagesNum+4];
	SfPoint* pQuadrangleCorners = new SfPoint[imagesNum*4]; // 4边形的顶点
	SfPoint corner[4];
	for(int n=0; n<imagesNum; n++)
	{
		if(vecAbandonInd[n]==0)
			continue;
		
		if(pImgT[n].m[8]==0)
			continue;

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
			xDst = (xSrc*pM[0] + ySrc*pM[1] + pM[2]) / 
				(xSrc*pM[6] + ySrc*pM[7] + pM[8]);
			yDst = (xSrc*pM[3] + ySrc*pM[4] + pM[5]) / 
				(xSrc*pM[6] + ySrc*pM[7] + pM[8]);
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

	float dGx = -minX, dGy = -minY;
	// 新的图像宽度
	int newWidth = int(maxX - minX + 1.5f);
	int newHeight = int(maxY - minY + 1.5f);
	cout<<"newWidth "<<newWidth<<" newHeight "<<newHeight<<endl;

	detail::MultiBandBlender blender(false, band);
	detail::GraphCutSeamFinder seam_finder_;

	blender.prepare( Rect(0, 0, newWidth, newHeight ) );
	vector<cv::Mat> vecImagesS;
	vector<cv::Mat> images_warped_f;
	vector<cv::Point> vecCorners;
	vector<IplImage*> vecMask;
	vector<Rectangle4Points> vecRectPoints;
	int nValid = 0;
	for(int n=0; n<imagesNum; n++)
	{
		if(vecAbandonInd[n]==0)
			continue;

		if(pImgT[n].m[8]==0)
			continue;

		float begBoxX32F = pBegBox[n].x + dGx;
		float begBoxY32F = pBegBox[n].y + dGy;
		float endBoxX32F = pEndBox[n].x + dGx;
		float endBoxY32F = pEndBox[n].y + dGy;

		int begBoxX = int(begBoxX32F);
		int begBoxY = int(begBoxY32F);
		int endBoxX = int(endBoxX32F + 0.5f);
		int endBoxY = int(endBoxY32F + 0.5f);

		float sx = begBoxX - begBoxX32F;
		float sy = begBoxY - begBoxY32F;

		int w0 = pImages[n]->width, h0 = pImages[n]->height;
		SfPoint ptOri[4] = {	SfPoint(0,0),		SfPoint(w0-1,0), 
								SfPoint(w0-1,h0-1),	SfPoint(0,h0-1) };
		vecRectPoints.push_back(Rectangle4Points());
		for(int i=0; i<4; i++)
		{
			SfPoint temp;
			ApplyProjectMat9(ptOri[i].x, ptOri[i].y, temp.x, temp.y, pImgT[n].m);
			vecRectPoints[nValid].pt[i].x = temp.x + dGx + sx - begBoxX;
			vecRectPoints[nValid].pt[i].y = temp.y + dGy + sy - begBoxY;
		}

		int wChip = endBoxX - begBoxX + 1;
		int hChip = endBoxY - begBoxY + 1;

		IplImage* pChipImage = cvCreateImage(cvSize(wChip, hChip), 8, pImages[0]->nChannels);
		IplImage* pMask = cvCreateImage(cvSize(wChip, hChip), 8, 1);
		memset(pMask->imageData, 255, pMask->widthStep*pMask->height);

		int wsNew = pChipImage->widthStep;
		float pInvM[9];
		InverseMatrix(pImgT[n].m, 3, pInvM, 1e-12f);
		// xDst = T(xSrc) + dGx + sx - begBox
		for(int yDst=0; yDst<hChip; yDst++)
		{
			unsigned char* pRowDst = (unsigned char*)pChipImage->imageData + yDst*wsNew;
			for(int xDst=0; xDst<wChip; xDst++)
			{
				float xSrc32F, ySrc32F;
				float xTemp = xDst - dGx - sx + begBoxX;
				float yTemp = yDst - dGy - sy + begBoxY;
				// ApplyProjectMat2()
				xSrc32F = (xTemp*pInvM[0] + yTemp*pInvM[1] + pInvM[2]) / 
					(xTemp*pInvM[6] + yTemp*pInvM[7] + pInvM[8]);
				ySrc32F = (xTemp*pInvM[3] + yTemp*pInvM[4] + pInvM[5]) / 
					(xTemp*pInvM[6] + yTemp*pInvM[7] + pInvM[8]);

				int srcW_1 = pImages[n]->width - 1, srcH_1 = pImages[n]->height-1, 
					wsSrc = pImages[n]->widthStep;
				//双线性插值
				int ySrc = int(ySrc32F);
				int xSrc = int(xSrc32F);
				// 双线性插值
				if( (xSrc32F>=0) && (xSrc32F<srcW_1) && (ySrc32F>=0) && (ySrc32F<srcH_1) )
				{
					float p = ySrc32F - ySrc;
					float q = xSrc32F - xSrc;

					int nChannels = pImages[n]->nChannels;
					const IplImage* pImage = pImages[n];
					if(nChannels==1)
					{
						unsigned char* pSrcTemp = (unsigned char*)pImage->imageData + ySrc*wsSrc + xSrc;

						unsigned char gray = unsigned char((unsigned char)*(pSrcTemp)*(1-p)*(1-q) + 
							(unsigned char)*(pSrcTemp + 1)*(1-p)*q + 
							(unsigned char)*(pSrcTemp + wsSrc)*p*(1-q) + 
							(unsigned char)*(pSrcTemp + wsSrc + 1)*p*q );
						if(*(pRowDst + xDst) >0 )
						{
							*(pRowDst + xDst) = int( (gray + *(pRowDst + xDst))*0.5f );
						}
						else
						{
							*(pRowDst + xDst) = gray;
						}
					}
					else if(3==nChannels)
					{
						float weightR = -1, weightG = -1, weightB = -1;
						int i = ySrc;
						int j = xSrc;
						{	//B
							unsigned char* pTempB = (unsigned char*)pImage->imageData + i*wsSrc+ 3*j;
							unsigned char grayB1, grayB2, grayB3, grayB4;
							grayB1 = (unsigned char)*(pTempB);
							grayB2 = (unsigned char)*(pTempB + 3);
							grayB3 = (unsigned char)*(pTempB + wsSrc);
							grayB4 = (unsigned char)*(pTempB + wsSrc + 3);

							unsigned char gray = unsigned char( grayB1*(1-p)*(1-q) + 
								grayB2*(1-p)*q + 
								grayB3*p*(1-q) + 
								grayB4*p*q );
							pRowDst[3*xDst] = gray;
						}
						{	//G
							unsigned char* pTempG = (unsigned char*)pImage->imageData + i*wsSrc+ 3*j + 1;
							unsigned char grayG1, grayG2, grayG3, grayG4;
							grayG1 = (unsigned char)*(pTempG);
							grayG2 = (unsigned char)*(pTempG + 3);
							grayG3 = (unsigned char)*(pTempG + wsSrc);
							grayG4 = (unsigned char)*(pTempG + wsSrc + 3);

							unsigned char gray = unsigned char( grayG1*(1-p)*(1-q) + 
								grayG2*(1-p)*q + 
								grayG3*p*(1-q) + 
								grayG4*p*q );
							pRowDst[3*xDst+1] = gray;
						}
						{	//R		
							unsigned char* pTempR = (unsigned char*)pImage->imageData + i*wsSrc+ 3*j + 2;
							unsigned char grayR1, grayR2, grayR3, grayR4;
							grayR1 = (unsigned char)*(pTempR);
							grayR2 = (unsigned char)*(pTempR + 3);
							grayR3 = (unsigned char)*(pTempR + wsSrc);
							grayR4 = (unsigned char)*(pTempR + wsSrc + 3);

							unsigned char gray = unsigned char( grayR1*(1-p)*(1-q) + 
								grayR2*(1-p)*q + 
								grayR3*p*(1-q) + 
								grayR4*p*q );
							pRowDst[3*xDst+2] = gray;
						}
					}
				}
				else
				{
					*(pMask->imageData + yDst*pMask->widthStep + xDst) = 0;
				}
			}
		}
		
		Mat pCurrentImg(pChipImage);
		Mat temp;
		vecImagesS.push_back(temp);
		pCurrentImg.convertTo(vecImagesS[nValid], CV_16S);
		vecMask.push_back(pMask);

		vecCorners.push_back( cv::Point(begBoxX, begBoxY) );

		cvReleaseImage(&pChipImage);
		nValid++;
	}

	//seam_finder_.find(images_warped_f, vecCorners, vecMask);
	// 释放原始图像
	for(int n=0; n<imagesNum; n++)
	{
		cvReleaseImage(&pImages[n]); // 11-28 blending之前释放源图像
	}

	//cout<<"newWidth "<<newWidth<<" newHeight "<<newHeight<<endl;
	cout<<"find mask"<<endl;
	FindMasksByDistMap(&vecMask[0], (int)vecMask.size(), &vecRectPoints[0], &vecCorners[0], 
		newWidth, newHeight);

	cout<<"feed"<<endl;
	vector<Mat> vecMaskMat;
	for(int n=0; n<(int)vecMask.size(); n++)
	{
		blender.feed( vecImagesS[n], vecMask[n], vecCorners[n] );
	}	

	cout<<"start blending"<<endl;
	Mat result_s, result_mask;
	blender.blend(result_s, result_mask);
	
	Mat result;
	result_s.convertTo(result, CV_8U);
	
	// 释放内存
	for(int n=0; n<(int)vecMask.size(); n++)
	{
		cvReleaseImage(& vecMask[n]);
	}
	delete[] pBegBox; pBegBox = NULL;
	delete[] pEndBox; pEndBox = NULL;
	delete[] pQuadrangleCorners; pQuadrangleCorners = NULL;

	int wRes, hRes;
	wRes = result.cols;
	hRes = result.rows;
	pMosaicResult = cvCreateImage(cvSize(wRes, hRes), 8, nChannels);
	int wsRes = pMosaicResult->widthStep;
	int step0 = result.step[0];
	int step1 = result.step[1];
	for(int row = 0; row<hRes; row++)
	{
		memcpy(pMosaicResult->imageData + wsRes * row, (char*)result.data + step0 * row, step0);
	}
	
	return pMosaicResult;
}

// 功能：利用计算出的各幅图像相对于第一幅图像的变换关系，对图像进行重采样，拼成一张大图
// 说明：以第一张图为基准图
// pImgT为每张图像相对于第1张图像的变换关系
IplImage* MergeMultiImages2(IplImage** pImages, int imagesNum, ProjectMat* pImgT, int blending, int referenceType)
{
	IplImage* pResult = NULL;

	//PyramidBlending(pImages, imagesNum, pImgT, blending);
	//return NULL;
	//if(blending==2) // 拉普拉斯金字塔融合还没有完成
	//	blending = 1; // 线性加权融合

	//cout<<"merge images"<<endl;
	if(pImgT==NULL)
		return NULL;
	if(imagesNum<=1)
		return NULL;

	int nChannels = pImages[0]->nChannels;

	float A[9] = {1,0,0, 0,1,0, 0,0,1};
	//KeepOriginalImageInfo2( pImages[0]->width, pImages[0]->height, 
	//					   pImages[imagesNum-1]->width, pImages[imagesNum-1]->height, 
	//					   pImgT[0].m, pImgT[imagesNum-1].m, 
	//					   A);

	if(referenceType==1) // 使得第一站图像和最后一张图像基本一致
	{
		SelectHomographyExaustive(pImages[0]->width, pImages[0]->height, 
								pImages[imagesNum-1]->width, pImages[imagesNum-1]->height, 
								pImgT[0].m, pImgT[imagesNum-1].m, 
								A);
	}
	else if(referenceType==2)
	{
		// 以中心为基准
		InverseMatrix(pImgT[imagesNum/2].m, 3, A, 1e-12f);
	}

	//if(imagesNum>15)
	//{
	//float A15[9] = {2.5317547,
	//		0.84433836	,
	//		-384.13373	,
	//		0.60114294	,
	//		2.7892795	,
	//		-350.24445	,
	//		0.0010596381,
	//		0.0027135320,
	//		1};

	//float H15_inv[9];
	//InverseMatrix(pImgT[15].m, 3, H15_inv, 1e-12f);
	//MulMatrix( A15, 3, 3, H15_inv,3, 3, A  );
	//}		
	//float A0[9] = {    1.0000,    0.1502,  -31.3274,
	//	0,    1.5043, -105.1802,
	//	0,    0.0004,    0.9099};
	//float H0_inv[9];
	//InverseMatrix(pImgT[0].m, 3, H0_inv, 1e-12f);
	//MulMatrix( A0, 3, 3, H0_inv,3, 3, A  );

	if(1)
	{
		//for(int n=0; n<9; n++)
		//{
		//	cout<<A[n]<<" ";
		//}
		//cout<<endl;
		for(int n=0; n<imagesNum; n++)
		{
			float temp[9];
			MulMatrix(A, 3, 3, pImgT[n].m, 3, 3, temp);
			memcpy( pImgT[n].m, temp, sizeof(temp) );
		}
	}

	if(blending==2) // 多频带图像融合
	{
		int band = 5;
		pResult = LaplacianPyramidBlending(pImages, imagesNum, pImgT, band);
		return pResult;
	}

	//// 
	//memset(pImgT[0].m, 0, sizeof(float)*9);
	//pImgT[0].m[0] = 1; 
	//pImgT[0].m[4] = 1;
	//pImgT[0].m[8] = 1;

	//确定新图像的大小
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
	// 新的图像宽度
	int newWidth = int(maxX - minX + 1.5f);
	int newHeight = int(maxY - minY + 1.5f);

	//cout<<"newWidth "<<newWidth<<" newHeight "<<newHeight<<endl;

	float dx = -minX, dy = -minY;
	// 找出每叠加一幅图之后的图像的尺寸
	SfPoint *pSizeImagesBig = new SfPoint[imagesNum];
	SfPoint *pSizeImagesSmall = new SfPoint[imagesNum];
	for(int n=0; n<imagesNum; n++)
	{
		for(int i=0; i<4; i++)
		{
			pQuadrangleCorners[n*4+i].x += dx;
			pQuadrangleCorners[n*4+i].y += dy;
		}
		pSizeImagesBig[n].x = 0; pSizeImagesBig[n].y = 0;
		pSizeImagesSmall[n].x = 1<<29; pSizeImagesSmall[n].y = 1<<29;
		for(int i=0; i<=n; i++)
		{
			if(pEndBox[i].x + dx>pSizeImagesBig[n].x)
			{
				pSizeImagesBig[n].x = pEndBox[i].x + dx;
			}
			if(pEndBox[i].y + dy>pSizeImagesBig[n].y)
			{
				pSizeImagesBig[n].y = pEndBox[i].y + dy;
			}

			if(pBegBox[i].x + dx<pSizeImagesSmall[n].x)
			{
				pSizeImagesSmall[n].x = pBegBox[i].x + dx;
			}
			if(pBegBox[i].y + dy<pSizeImagesSmall[n].y)
			{
				pSizeImagesSmall[n].y = pBegBox[i].y + dy;
			}
		}
	}

	//cout<<"create new image memory"<<endl;
	int wsNew = 0;
	double MAX_NEW_SIZE = 10000.0*10000.0;
	double newSize = (double)newWidth*(double)newHeight;
	if(newSize<MAX_NEW_SIZE) // 如果图像的尺寸超过最大尺寸, 则认为拼接失败
	{
		//cout<<"start create new image"<<endl;
		pResult = cvCreateImage( cvSize(newWidth, newHeight), 8, pImages[0]->nChannels );
		cvZero(pResult);
		wsNew = pResult->widthStep;
	}
	else
	{
		cout<<"create new image failed"<<endl;
	}

	if(pResult)
	{
		for(int n=0; n<imagesNum; n++)
		{
			int begBoxX = int(pBegBox[n].x + dx);
			int begBoxY = int(pBegBox[n].y + dy);
			int endBoxX = int(pEndBox[n].x + dx + 0.5f);
			int endBoxY = int(pEndBox[n].y + dy + 0.5f);

			float h_toBox[9] = {0};
			// 计算重叠类型
			int overlapType = -1;
			if(n>0)
			{
				//overlapType = CalOverlapType(pSizeImagesBig[n-1], pSizeImagesSmall[n-1], begBoxX, begBoxY, endBoxX, endBoxY);
				// 计算重叠类型
				// pMergedImg-------------已经融合的大图
				// pQuadrangleCorners-----即将加入的图像的4个顶点坐标
				CalOverlapType2(pResult, pQuadrangleCorners+n*4, overlapType);
				//cout<<n<<" overlayType "<<overlapType<<endl;

				SfPoint boxPoints[4];
				boxPoints[0].x = (float)begBoxX; boxPoints[0].y = (float)begBoxY;
				boxPoints[1].x = (float)endBoxX; boxPoints[1].y = (float)begBoxY;
				boxPoints[2].x = (float)endBoxX; boxPoints[2].y = (float)endBoxY;
				boxPoints[3].x = (float)begBoxX; boxPoints[3].y = (float)endBoxY;
				//SolvePseudoAffine( boxPoints, pQuadrangleCorners+n*4, 4, k_toBox );
				SolveProjectMatrix3(boxPoints, pQuadrangleCorners+n*4, 4, h_toBox);
				ProjectMat temp, fineH;
				memcpy(temp.m, h_toBox, sizeof(float)*9);
				NonlinearLeastSquareProjection(boxPoints, pQuadrangleCorners+n*4, 4, fineH.m, temp.m, 1e-9f );
				memcpy(h_toBox, fineH.m, sizeof(float)*9);
			}

			float* pM = pImgT[n].m;
			float pInvM[9];
			InverseMatrix(pM, 3, pInvM); // 求变换矩阵的逆矩阵

			//if(n!=imagesNum/2)
			//{
			//	continue;
			//}
			int boxW = endBoxX-begBoxX+1;
			int boxH = endBoxY-begBoxY+1;
			BitmapImage* pWeightImg = CreateBitmap8U( boxW, boxH, 1 );
			int wsWtImg = pWeightImg->widthStep;
			float* pWeightR = new float[boxW*boxH];
			ZeroImage(pWeightImg);
			const int NON_WEIGHT_PIXEL = 255, WEIGHT_PIXEL = 128;

			for(int yDst=begBoxY; yDst<=endBoxY; yDst++)
			{
				unsigned char* pRowDst = (unsigned char*)pResult->imageData + yDst*wsNew;
				for(int xDst=begBoxX; xDst<=endBoxX; xDst++)
				{
					*(pWeightR + (yDst-begBoxY)*boxW + xDst-begBoxX) = -1;
					if( (xDst==699) && (yDst==484) )
					{
						xDst = xDst;
					}
					float xDst32F = xDst - dx, yDst32F = yDst - dy;
					float xSrc32F, ySrc32F;
					xSrc32F = (xDst32F*pInvM[0] + yDst32F*pInvM[1] + pInvM[2]) / (xDst32F*pInvM[6] + yDst32F*pInvM[7] + pInvM[8]);
					ySrc32F = (xDst32F*pInvM[3] + yDst32F*pInvM[4] + pInvM[5]) / (xDst32F*pInvM[6] + yDst32F*pInvM[7] + pInvM[8]);

					int srcW_1 = pImages[n]->width - 1, srcH_1 = pImages[n]->height-1, wsSrc = pImages[n]->widthStep;
					//双线性插值
					int ySrc = int(ySrc32F);
					int xSrc = int(xSrc32F);
					// 双线性插值
					if( (xSrc32F>=0) && (xSrc32F<srcW_1) && (ySrc32F>=0) && (ySrc32F<srcH_1) )
					{
						float p = ySrc32F - ySrc;
						float q = xSrc32F - xSrc;

						// blending
						float weight1 = 0.0f, weight2 = 1;
						if (blending==1) // 线性加权平均
						{
							float xDstTrans = (float)xDst, yDstTrans = (float)yDst;
							//ApplyPseudoAffine(xDst, yDst, xDstTrans, yDstTrans, k_toBox); // 把当前点表换到标准的矩形中
							ApplyProjectMat2(xDst, yDst, xDstTrans, yDstTrans, h_toBox);
							if(1==overlapType)
							{
								float overlapW = float(endBoxX - begBoxX);
								float overlapH = endBoxY - pSizeImagesSmall[n-1].y;

								float mX = 0.5f * (endBoxX + begBoxX);
								float dmX = abs(xDstTrans-mX);
								float dY = yDstTrans - pSizeImagesSmall[n-1].y;
								float w1 = 1, w2 = 0, w3 = 0, w4 = 0;
								float p = dmX / (0.5f*overlapW);
								float q = dY / overlapH;
								weight2 = (1-p)*(1-q)*w1;
							}
							else if(2==overlapType)
							{
								float overlapW = pSizeImagesBig[n-1].x - begBoxX;
								float overlapH = pSizeImagesBig[n-1].y - begBoxY;
								float overlapX = xDstTrans - begBoxX;
								float overlapY = yDstTrans - begBoxY;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								float w1 = 0, w2 = 0, w3 = 1, w4 = 0;
								weight2 = (1-p) * (1-q) * w1 + p * (1-q) * w4 + p * q* w3 + (1-p) * q * w2;
							}
							else if(3==overlapType)
							{
								float overlapW = pSizeImagesBig[n-1].x - begBoxX;
								float overlapH = (float)(endBoxY - begBoxY);
								float mY = 0.5f * (endBoxY + begBoxY);
								float dmY = abs(yDstTrans - mY);
								float dX = pSizeImagesBig[n-1].x - xDstTrans;
								float p = dX / overlapW;
								float q = dmY / (0.5f*overlapH);
								float w1 = 1;
								weight2 = (1-p)*(1-q)*w1;
							}
							else if(4==overlapType)
							{
								float w1 = 0, w2 = 0, w3 = 0, w4 = 1;
								float overlapW = pSizeImagesBig[n-1].x - begBoxX;
								float overlapH = endBoxY - pSizeImagesSmall[n-1].y;

								float overlapX = (float)(xDstTrans - begBoxX);
								float overlapY = yDstTrans - pSizeImagesSmall[n-1].y;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								weight2 = (1-p) * (1-q) * w1 + p * (1-q) * w4 + p * q* w3 + (1-p) * q * w2;
							}
							else if(5==overlapType)
							{
								float w1 = 1;
								float overlapW = (float)(endBoxX - begBoxX);
								float overlapH = pSizeImagesBig[n-1].y - begBoxY;
								float mX = 0.5f * (endBoxX + begBoxX);
								float overlapX = (float)(xDstTrans - mX);
								float overlapY = yDstTrans - begBoxY;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								weight2 = (1-p) * (1-q) * w1 ;
							}
							else if(6==overlapType)
							{
								float overlapW = endBoxX - pSizeImagesSmall[n-1].x;
								float overlapH = pSizeImagesBig[n-1].y - begBoxY;
								float overlapX = xDstTrans - pSizeImagesSmall[n-1].x;
								float overlapY = yDstTrans - begBoxY;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								float w1 = 1;
								weight2 = (1-p)*q*w1;
							}
							else if(7==overlapType)
							{
								float overlapW = endBoxX - pSizeImagesSmall[n-1].x;
								float overlapH = (float)(endBoxY - begBoxY);
								float mY = 0.5f * (endBoxY + begBoxY);
								float overlapX = xDstTrans - pSizeImagesSmall[n-1].x;
								float dmY = abs(yDstTrans - mY);
								float p = overlapX / overlapW;
								float q = dmY / (0.5f*overlapH);
								float w3 = 1;
								weight2 = (1-p)*(1-q)*w3;
							}
							else if(8==overlapType)
							{
								float overlapW = endBoxX - pSizeImagesSmall[n-1].x;
								float overlapH = endBoxY - pSizeImagesSmall[n-1].y;
								float overlapX = xDstTrans - pSizeImagesSmall[n-1].x;
								float overlapY = yDstTrans - pSizeImagesSmall[n-1].y;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								float w1 = 1;
								weight2 = (1-p)*(1-q)*w1;
							}
							else if(9==overlapType)
							{
								int overlapW = endBoxX - begBoxX;
								int overlapH = endBoxY - begBoxY;
								float mX = 0.5f * (endBoxX + begBoxX);
								float mY = 0.5f * (endBoxY + begBoxY);
								float dmX = abs(xDstTrans - mX);
								float dmY = abs(yDstTrans - mY);
								weight2 = (0.5f*overlapW-dmX) * (0.5f*overlapH-dmY) / (0.25f * overlapW * overlapH);
								//weight1 = 1 - weight2;
							}
						}
						//if( (weight2>=0) && (weight2<=1) )
						//{
						//	*(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = 255;
						//}
						//else
						//{
						//	*(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = 128;
						//}
						weight1 = 1 - weight2;
						if( (weight1<0) || (weight2<0) || (weight1>1) || (weight2>1))
						{
							weight1 = 0.5f; weight2 = 0.5f;
						}
						if(nChannels==1)
						{
							unsigned char* pSrcTemp = (unsigned char*)pImages[n]->imageData + ySrc*wsSrc + xSrc;

							unsigned char gray = unsigned char((unsigned char)*(pSrcTemp)*(1-p)*(1-q) + 
								(unsigned char)*(pSrcTemp + 1)*(1-p)*q + 
								(unsigned char)*(pSrcTemp + wsSrc)*p*(1-q) + 
								(unsigned char)*(pSrcTemp + wsSrc + 1)*p*q );
							if(*(pRowDst + xDst) >0 )
							{
								*(pRowDst + xDst) = int( (gray + *(pRowDst + xDst))*0.5f );
							}
							else
							{
								*(pRowDst + xDst) = gray;
							}
						}
						else if(3==nChannels)
						{
							bool bMosImagePixelValid = false;
							if(( pRowDst[3*xDst] >0 ) || ( pRowDst[3*xDst+1] >0 ) || ( pRowDst[3*xDst+2] >0 ))
							{
								bMosImagePixelValid = true;
							}

							float weightR = -1, weightG = -1, weightB = -1;
							int i = ySrc;
							int j = xSrc;
							{	//B
								unsigned char* pTempB = (unsigned char*)pImages[n]->imageData + i*wsSrc+ 3*j;
								unsigned char grayB1, grayB2, grayB3, grayB4;
								grayB1 = (unsigned char)*(pTempB);
								grayB2 = (unsigned char)*(pTempB + 3);
								grayB3 = (unsigned char)*(pTempB + wsSrc);
								grayB4 = (unsigned char)*(pTempB + wsSrc + 3);

								unsigned char gray = unsigned char( grayB1*(1-p)*(1-q) + 
									grayB2*(1-p)*q + 
									grayB3*p*(1-q) + 
									grayB4*p*q );
								if( bMosImagePixelValid )
								{
									pRowDst[3*xDst] = int(weight1*pRowDst[3*xDst] + weight2 * gray);
									*(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = WEIGHT_PIXEL;//NON_WEIGHT_PIXEL
									if(gray==0)
									{
										weightB = 1.0f;
									}
									else
									{
										weightB = (float)pRowDst[3*xDst] / gray; // 图像2灰度变化比例
									}
								}
								else
								{
									pRowDst[3*xDst] = gray;
									*(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = NON_WEIGHT_PIXEL; // NON_WEIGHT_PIXEL
								}
							}
							{	//G
								unsigned char* pTempG = (unsigned char*)pImages[n]->imageData + i*wsSrc+ 3*j + 1;
								unsigned char grayG1, grayG2, grayG3, grayG4;
								grayG1 = (unsigned char)*(pTempG);
								grayG2 = (unsigned char)*(pTempG + 3);
								grayG3 = (unsigned char)*(pTempG + wsSrc);
								grayG4 = (unsigned char)*(pTempG + wsSrc + 3);

								unsigned char gray = unsigned char( grayG1*(1-p)*(1-q) + 
									grayG2*(1-p)*q + 
									grayG3*p*(1-q) + 
									grayG4*p*q );
								if( bMosImagePixelValid )
								{
									pRowDst[3*xDst+1] = int(weight1*pRowDst[3*xDst+1] + weight2 * gray);
									*(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = WEIGHT_PIXEL;
									if(gray==0)
									{
										weightG = 1.0f;
									}
									else
									{
										weightG = (float)pRowDst[3*xDst+1] / gray; // 图像2灰度变化比例
									}
								}
								else
								{
									pRowDst[3*xDst+1] = gray;
									*(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = NON_WEIGHT_PIXEL;
								}
							}
							{	//R		
								unsigned char* pTempR = (unsigned char*)pImages[n]->imageData + i*wsSrc+ 3*j + 2;
								unsigned char grayR1, grayR2, grayR3, grayR4;
								grayR1 = (unsigned char)*(pTempR);
								grayR2 = (unsigned char)*(pTempR + 3);
								grayR3 = (unsigned char)*(pTempR + wsSrc);
								grayR4 = (unsigned char)*(pTempR + wsSrc + 3);

								unsigned char gray = unsigned char( grayR1*(1-p)*(1-q) + 
									grayR2*(1-p)*q + 
									grayR3*p*(1-q) + 
									grayR4*p*q );
								if( bMosImagePixelValid )
								{
									pRowDst[3*xDst+2] = int(weight1*pRowDst[3*xDst+2] + weight2 * gray);
									*(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = WEIGHT_PIXEL;
									if(gray==0)
									{
										weightR = 1.0f;
									}
									else
									{
										weightR = (float)pRowDst[3*xDst+2] / gray; // 图像2灰度变化比例
									}
								}
								else
								{
									pRowDst[3*xDst+2] = gray;
									*(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = NON_WEIGHT_PIXEL;
								}
							}
							*(pWeightR + (yDst-begBoxY)*boxW + xDst-begBoxX) = (weightR + weightG + weightB)*0.33333f;
						}
					}
				}
			}

			float *pFullWeightR = NULL, *pFullWeightR_Smoothed = NULL;
			if((n>0) && (blending==1))
			{
				pFullWeightR = new float[boxW*boxH];
				pFullWeightR_Smoothed = new float[boxW*boxH];
				//FillWeight(pWeightR, pFullWeightR, boxW, boxH, boxW);
				FillWeightSparse(pWeightR, pFullWeightR, boxW, boxH, boxW);
				//SmoothWeight( pFullWeightR, pFullWeightR_Smoothed, boxW, boxH, boxW,
				//	pWeightImg, NON_WEIGHT_PIXEL );
				//SmoothWeight_MidVal( pFullWeightR, pFullWeightR_Smoothed, boxW, boxH, boxW,
				//	pWeightImg, NON_WEIGHT_PIXEL );

				// 处理没有weight的像素
				for(int yDst=begBoxY; yDst<=endBoxY; yDst++)
				{
					unsigned char* pRowDst = (unsigned char*)pResult->imageData + yDst*wsNew;
					for(int xDst=begBoxX; xDst<=endBoxX; xDst++)
					{
						if( (xDst==376) && (yDst==7) )
						{
							xDst = xDst;
						}
						float xDst32F = xDst - dx, yDst32F = yDst - dy;
						float xSrc32F, ySrc32F;
						xSrc32F = (xDst32F*pInvM[0] + yDst32F*pInvM[1] + pInvM[2]) / (xDst32F*pInvM[6] + yDst32F*pInvM[7] + pInvM[8]);
						ySrc32F = (xDst32F*pInvM[3] + yDst32F*pInvM[4] + pInvM[5]) / (xDst32F*pInvM[6] + yDst32F*pInvM[7] + pInvM[8]);

						int srcW_1 = pImages[n]->width - 1, srcH_1 = pImages[n]->height-1, wsSrc = pImages[n]->widthStep;
						//双线性插值
						int ySrc = int(ySrc32F);
						int xSrc = int(xSrc32F);
						// 双线性插值
						if( (xSrc32F>=0) && (xSrc32F<srcW_1) && (ySrc32F>=0) && (ySrc32F<srcH_1) )
						{
							int xBox = xDst-begBoxX, yBox = yDst-begBoxY;
							if( pWeightImg->imageData[yBox*wsWtImg + xBox]==NON_WEIGHT_PIXEL )
							{
								//float kPixel = pFullWeightR[yBox*boxW+xBox];
								float kPixel = pFullWeightR[yBox*boxW+xBox];
								if(kPixel<0)
								{
									continue;
								}
								if( (kPixel<=0) || (kPixel>2) )
								{
									kPixel = kPixel;
								}
								pRowDst[3*xDst+0] = Min((int)(pRowDst[3*xDst+0] * kPixel),255);
								pRowDst[3*xDst+1] = Min((int)(pRowDst[3*xDst+1] * kPixel),255);
								pRowDst[3*xDst+2] = Min((int)(pRowDst[3*xDst+2] * kPixel),255);
							}
						}
					}
				}
			}

#ifdef _DEBUG
			//SaveImageATL( pResult, "c:\\result.bmp" );
			//SaveImageATL( pWeightImg, "c:\\weighted-pixel.bmp" );			
			//COutData::OutData2Txt(pWeightR, boxW, boxH, boxW, "c:\\weight.txt");
			//COutData::OutData2Txt(pFullWeightR, boxW, boxH, boxW, "c:\\full-weight.txt");
			//COutData::OutData2Txt(pFullWeightR_Smoothed, boxW, boxH, boxW, "c:\\full-weight_smooth.txt");
#endif

			ReleaseBitmap8U(pWeightImg);

			delete[] pWeightR; pWeightR = NULL;
			delete[] pFullWeightR; pFullWeightR = NULL;
			delete[] pFullWeightR_Smoothed; pFullWeightR_Smoothed = NULL;

			//===================
			//float* pM = pImgT[n].m;
			//int w = pImages[n]->width, h = pImages[n]->height, ws = pImages[n]->widthStep;
			//for(int ySrc=0; ySrc<h; ySrc++)
			//{
			//	for(int xSrc=0; xSrc<w; xSrc++)
			//	{
			//		if( (xSrc==674) && (ySrc==190) )
			//			xSrc = xSrc;
			//		if( (xSrc==486) && (ySrc==8) )
			//			xSrc = xSrc;
			//		float xDst = 0, yDst = 0;
			//		xDst = (xSrc*pM[0] + ySrc*pM[1] + pM[2]) / (xSrc*pM[6] + ySrc*pM[7] + pM[8]);
			//		yDst = (xSrc*pM[3] + ySrc*pM[4] + pM[5]) / (xSrc*pM[6] + ySrc*pM[7] + pM[8]);
			//		xDst += dx;
			//		yDst += dy;
			//		*(pResult->imageData + (int)yDst*wsNew + (int)xDst*3) = *(pImages[n]->imageData + ySrc*ws + xSrc*3);
			//		*(pResult->imageData + (int)yDst*wsNew + (int)xDst*3+1) = *(pImages[n]->imageData + ySrc*ws + xSrc*3+1);
			//		*(pResult->imageData + (int)yDst*wsNew + (int)xDst*3+2) = *(pImages[n]->imageData + ySrc*ws + xSrc*3+2);
			//	}
			//}

		}
	}
	delete[] pSizeImagesBig; pSizeImagesBig = NULL;
	delete[] pSizeImagesSmall; pSizeImagesSmall = NULL;
	delete[] pQuadrangleCorners; pQuadrangleCorners = NULL;
	delete[] pBegBox; pBegBox = NULL;
	delete[] pEndBox; pEndBox = NULL;

	//cout<<"return Merge"<<endl;

	return pResult;
}

// 功能：利用计算出的各幅图像相对于第一幅图像的变换关系，对图像进行重采样，拼成一张大图
// 说明：以第一张图为基准图
// pImgT为每张图像相对于第1张图像的变换关系
BitmapImage* MergeMultiImages(const BitmapImage* pImages[], int imagesNum, ProjectMat* pImgT, int blending, int referenceType)
{
	//PyramidBlending(pImages, imagesNum, pImgT, blending);
	//return NULL;
	if(blending==2) // 拉普拉斯金字塔融合还没有完成
		blending = 1; // 线性加权融合

	//cout<<"merge images"<<endl;
	if(pImgT==NULL)
		return NULL;
	if(imagesNum<=1)
		return NULL;

	int nChannels = pImages[0]->nChannels;

	float A[9] = {1,0,0, 0,1,0, 0,0,1};
	//KeepOriginalImageInfo2( pImages[0]->width, pImages[0]->height, 
	//					   pImages[imagesNum-1]->width, pImages[imagesNum-1]->height, 
	//					   pImgT[0].m, pImgT[imagesNum-1].m, 
	//					   A);

	if(referenceType==1)
	{
		SelectHomographyExaustive(pImages[0]->width, pImages[0]->height, 
									pImages[imagesNum-1]->width, pImages[imagesNum-1]->height, 
									pImgT[0].m, pImgT[imagesNum-1].m, 
									A);	
		
	}
	else if(referenceType==2)
	{
		// 以中心为基准
		InverseMatrix(pImgT[imagesNum/2].m, 3, A, 1e-12f);
	}

	//if(imagesNum>15)
	//{
		//float A15[9] = {2.5317547,
		//		0.84433836	,
		//		-384.13373	,
		//		0.60114294	,
		//		2.7892795	,
		//		-350.24445	,
		//		0.0010596381,
		//		0.0027135320,
		//		1};

		//float H15_inv[9];
		//InverseMatrix(pImgT[15].m, 3, H15_inv, 1e-12f);
		//MulMatrix( A15, 3, 3, H15_inv,3, 3, A  );
	//}		
	//float A0[9] = {    1.0000,    0.1502,  -31.3274,
	//	0,    1.5043, -105.1802,
	//	0,    0.0004,    0.9099};
	//float H0_inv[9];
	//InverseMatrix(pImgT[0].m, 3, H0_inv, 1e-12f);
	//MulMatrix( A0, 3, 3, H0_inv,3, 3, A  );

	if(1)
	{
		//for(int n=0; n<9; n++)
		//{
		//	cout<<A[n]<<" ";
		//}
		//cout<<endl;
		for(int n=0; n<imagesNum; n++)
		{
			float temp[9];
			MulMatrix(A, 3, 3, pImgT[n].m, 3, 3, temp);
			memcpy( pImgT[n].m, temp, sizeof(temp) );
		}
	}

	//// 
	//memset(pImgT[0].m, 0, sizeof(float)*9);
	//pImgT[0].m[0] = 1; 
	//pImgT[0].m[4] = 1;
	//pImgT[0].m[8] = 1;

	//确定新图像的大小
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
	// 新的图像宽度
	int newWidth = int(maxX - minX + 1.5f);
	int newHeight = int(maxY - minY + 1.5f);

	//cout<<"newWidth "<<newWidth<<" newHeight "<<newHeight<<endl;

	float dx = -minX, dy = -minY;
	// 找出每叠加一幅图之后的图像的尺寸
	SfPoint *pSizeImagesBig = new SfPoint[imagesNum];
	SfPoint *pSizeImagesSmall = new SfPoint[imagesNum];
	for(int n=0; n<imagesNum; n++)
	{
		for(int i=0; i<4; i++)
		{
			pQuadrangleCorners[n*4+i].x += dx;
			pQuadrangleCorners[n*4+i].y += dy;
		}
		pSizeImagesBig[n].x = 0; pSizeImagesBig[n].y = 0;
		pSizeImagesSmall[n].x = 1<<29; pSizeImagesSmall[n].y = 1<<29;
		for(int i=0; i<=n; i++)
		{
			if(pEndBox[i].x + dx>pSizeImagesBig[n].x)
			{
				pSizeImagesBig[n].x = pEndBox[i].x + dx;
			}
			if(pEndBox[i].y + dy>pSizeImagesBig[n].y)
			{
				pSizeImagesBig[n].y = pEndBox[i].y + dy;
			}

			if(pBegBox[i].x + dx<pSizeImagesSmall[n].x)
			{
				pSizeImagesSmall[n].x = pBegBox[i].x + dx;
			}
			if(pBegBox[i].y + dy<pSizeImagesSmall[n].y)
			{
				pSizeImagesSmall[n].y = pBegBox[i].y + dy;
			}
		}
	}

	//cout<<"create new image memory"<<endl;
	BitmapImage* pResult = NULL;
	int wsNew = 0;
	double MAX_NEW_SIZE = 10000.0*10000.0;
	double newSize = (double)newWidth*(double)newHeight;
	if(newSize<MAX_NEW_SIZE) // 如果图像的尺寸超过最大尺寸, 则认为拼接失败
	{
		//cout<<"start create new image"<<endl;
		pResult = CreateBitmap8U( newWidth, newHeight, pImages[0]->nChannels );
		ZeroImage(pResult);
		wsNew = pResult->widthStep;
	}
	else
	{
		cout<<"create new image failed"<<endl;
	}

	if(pResult)
	{
		for(int n=0; n<imagesNum; n++)
		{
			int begBoxX = int(pBegBox[n].x + dx);
			int begBoxY = int(pBegBox[n].y + dy);
			int endBoxX = int(pEndBox[n].x + dx + 0.5f);
			int endBoxY = int(pEndBox[n].y + dy + 0.5f);

			float h_toBox[9] = {0};
			// 计算重叠类型
			int overlapType = -1;
			if(n>0)
			{
				//overlapType = CalOverlapType(pSizeImagesBig[n-1], pSizeImagesSmall[n-1], begBoxX, begBoxY, endBoxX, endBoxY);
				// 计算重叠类型
				// pMergedImg-------------已经融合的大图
				// pQuadrangleCorners-----即将加入的图像的4个顶点坐标
				CalOverlapType2(pResult, pQuadrangleCorners+n*4, overlapType);
				//cout<<n<<" overlayType "<<overlapType<<endl;

				SfPoint boxPoints[4];
				boxPoints[0].x = (float)begBoxX; boxPoints[0].y = (float)begBoxY;
				boxPoints[1].x = (float)endBoxX; boxPoints[1].y = (float)begBoxY;
				boxPoints[2].x = (float)endBoxX; boxPoints[2].y = (float)endBoxY;
				boxPoints[3].x = (float)begBoxX; boxPoints[3].y = (float)endBoxY;
				//SolvePseudoAffine( boxPoints, pQuadrangleCorners+n*4, 4, k_toBox );
				SolveProjectMatrix3(boxPoints, pQuadrangleCorners+n*4, 4, h_toBox);
				ProjectMat temp, fineH;
				memcpy(temp.m, h_toBox, sizeof(float)*9);
				NonlinearLeastSquareProjection(boxPoints, pQuadrangleCorners+n*4, 4, fineH.m, temp.m, 1e-9f );
				memcpy(h_toBox, fineH.m, sizeof(float)*9);
			}

			float* pM = pImgT[n].m;
			float pInvM[9];
			InverseMatrix(pM, 3, pInvM); // 求变换矩阵的逆矩阵

			//if(n!=imagesNum/2)
			//{
			//	continue;
			//}
			int boxW = endBoxX-begBoxX+1;
			int boxH = endBoxY-begBoxY+1;
			BitmapImage* pWeightImg = CreateBitmap8U( boxW, boxH, 1 );
			int wsWtImg = pWeightImg->widthStep;
			float* pWeightR = new float[boxW*boxH];
			ZeroImage(pWeightImg);
			const int NON_WEIGHT_PIXEL = 255, WEIGHT_PIXEL = 128;

			for(int yDst=begBoxY; yDst<=endBoxY; yDst++)
			{
				unsigned char* pRowDst = pResult->imageData + yDst*wsNew;
				for(int xDst=begBoxX; xDst<=endBoxX; xDst++)
				{
					*(pWeightR + (yDst-begBoxY)*boxW + xDst-begBoxX) = -1;
					if( (xDst==699) && (yDst==484) )
					{
						xDst = xDst;
					}
					float xDst32F = xDst - dx, yDst32F = yDst - dy;
					float xSrc32F, ySrc32F;
					xSrc32F = (xDst32F*pInvM[0] + yDst32F*pInvM[1] + pInvM[2]) / (xDst32F*pInvM[6] + yDst32F*pInvM[7] + pInvM[8]);
					ySrc32F = (xDst32F*pInvM[3] + yDst32F*pInvM[4] + pInvM[5]) / (xDst32F*pInvM[6] + yDst32F*pInvM[7] + pInvM[8]);
					
					int srcW_1 = pImages[n]->width - 1, srcH_1 = pImages[n]->height-1, wsSrc = pImages[n]->widthStep;
					//双线性插值
					int ySrc = int(ySrc32F);
					int xSrc = int(xSrc32F);
					// 双线性插值
					if( (xSrc32F>=0) && (xSrc32F<srcW_1) && (ySrc32F>=0) && (ySrc32F<srcH_1) )
					{
						float p = ySrc32F - ySrc;
						float q = xSrc32F - xSrc;

						// blending
						float weight1 = 0.0f, weight2 = 1;
						if (blending==1) // 线性加权平均
						{
							float xDstTrans = (float)xDst, yDstTrans = (float)yDst;
							//ApplyPseudoAffine(xDst, yDst, xDstTrans, yDstTrans, k_toBox); // 把当前点表换到标准的矩形中
							ApplyProjectMat2(xDst, yDst, xDstTrans, yDstTrans, h_toBox);
							if(1==overlapType)
							{
								float overlapW = float(endBoxX - begBoxX);
								float overlapH = endBoxY - pSizeImagesSmall[n-1].y;

								float mX = 0.5f * (endBoxX + begBoxX);
								float dmX = abs(xDstTrans-mX);
								float dY = yDstTrans - pSizeImagesSmall[n-1].y;
								float w1 = 1, w2 = 0, w3 = 0, w4 = 0;
								float p = dmX / (0.5f*overlapW);
								float q = dY / overlapH;
								weight2 = (1-p)*(1-q)*w1;
							}
							else if(2==overlapType)
							{
								float overlapW = pSizeImagesBig[n-1].x - begBoxX;
								float overlapH = pSizeImagesBig[n-1].y - begBoxY;
								float overlapX = xDstTrans - begBoxX;
								float overlapY = yDstTrans - begBoxY;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								float w1 = 0, w2 = 0, w3 = 1, w4 = 0;
								weight2 = (1-p) * (1-q) * w1 + p * (1-q) * w4 + p * q* w3 + (1-p) * q * w2;
							}
							else if(3==overlapType)
							{
								float overlapW = pSizeImagesBig[n-1].x - begBoxX;
								float overlapH = (float)(endBoxY - begBoxY);
								float mY = 0.5f * (endBoxY + begBoxY);
								float dmY = abs(yDstTrans - mY);
								float dX = pSizeImagesBig[n-1].x - xDstTrans;
								float p = dX / overlapW;
								float q = dmY / (0.5f*overlapH);
								float w1 = 1;
								weight2 = (1-p)*(1-q)*w1;
							}
							else if(4==overlapType)
							{
								float w1 = 0, w2 = 0, w3 = 0, w4 = 1;
								float overlapW = pSizeImagesBig[n-1].x - begBoxX;
								float overlapH = endBoxY - pSizeImagesSmall[n-1].y;

								float overlapX = (float)(xDstTrans - begBoxX);
								float overlapY = yDstTrans - pSizeImagesSmall[n-1].y;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								weight2 = (1-p) * (1-q) * w1 + p * (1-q) * w4 + p * q* w3 + (1-p) * q * w2;
							}
							else if(5==overlapType)
							{
								float w1 = 1;
								float overlapW = (float)(endBoxX - begBoxX);
								float overlapH = pSizeImagesBig[n-1].y - begBoxY;
								float mX = 0.5f * (endBoxX + begBoxX);
								float overlapX = (float)(xDstTrans - mX);
								float overlapY = yDstTrans - begBoxY;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								weight2 = (1-p) * (1-q) * w1 ;
							}
							else if(6==overlapType)
							{
								float overlapW = endBoxX - pSizeImagesSmall[n-1].x;
								float overlapH = pSizeImagesBig[n-1].y - begBoxY;
								float overlapX = xDstTrans - pSizeImagesSmall[n-1].x;
								float overlapY = yDstTrans - begBoxY;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								float w1 = 1;
								weight2 = (1-p)*q*w1;
							}
							else if(7==overlapType)
							{
								float overlapW = endBoxX - pSizeImagesSmall[n-1].x;
								float overlapH = (float)(endBoxY - begBoxY);
								float mY = 0.5f * (endBoxY + begBoxY);
								float overlapX = xDstTrans - pSizeImagesSmall[n-1].x;
								float dmY = abs(yDstTrans - mY);
								float p = overlapX / overlapW;
								float q = dmY / (0.5f*overlapH);
								float w3 = 1;
								weight2 = (1-p)*(1-q)*w3;
							}
							else if(8==overlapType)
							{
								float overlapW = endBoxX - pSizeImagesSmall[n-1].x;
								float overlapH = endBoxY - pSizeImagesSmall[n-1].y;
								float overlapX = xDstTrans - pSizeImagesSmall[n-1].x;
								float overlapY = yDstTrans - pSizeImagesSmall[n-1].y;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								float w1 = 1;
								weight2 = (1-p)*(1-q)*w1;
							}
							else if(9==overlapType)
							{
								int overlapW = endBoxX - begBoxX;
								int overlapH = endBoxY - begBoxY;
								float mX = 0.5f * (endBoxX + begBoxX);
								float mY = 0.5f * (endBoxY + begBoxY);
								float dmX = abs(xDstTrans - mX);
								float dmY = abs(yDstTrans - mY);
								weight2 = (0.5f*overlapW-dmX) * (0.5f*overlapH-dmY) / (0.25f * overlapW * overlapH);
								//weight1 = 1 - weight2;
							}
						}
						//if( (weight2>=0) && (weight2<=1) )
						//{
						//	*(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = 255;
						//}
						//else
						//{
						//	*(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = 128;
						//}
						weight1 = 1 - weight2;
						if( (weight1<0) || (weight2<0) || (weight1>1) || (weight2>1))
						{
							weight1 = 0.5f; weight2 = 0.5f;
						}
						if(nChannels==1)
						{
							unsigned char* pSrcTemp = pImages[n]->imageData + ySrc*wsSrc + xSrc;

							unsigned char gray = unsigned char((unsigned char)*(pSrcTemp)*(1-p)*(1-q) + 
								(unsigned char)*(pSrcTemp + 1)*(1-p)*q + 
								(unsigned char)*(pSrcTemp + wsSrc)*p*(1-q) + 
								(unsigned char)*(pSrcTemp + wsSrc + 1)*p*q );
							 if(*(pRowDst + xDst) >0 )
							 {
								*(pRowDst + xDst) = int( (gray + *(pRowDst + xDst))*0.5f );
							 }
							 else
							 {
								*(pRowDst + xDst) = gray;
							 }
						}
						else if(3==nChannels)
						{
							bool bMosImagePixelValid = false;
							if(( pRowDst[3*xDst] >0 ) || ( pRowDst[3*xDst+1] >0 ) || ( pRowDst[3*xDst+2] >0 ))
							{
								bMosImagePixelValid = true;
							}

							float weightR = -1, weightG = -1, weightB = -1;
							int i = ySrc;
							int j = xSrc;
							{	//B
								unsigned char* pTempB = pImages[n]->imageData + i*wsSrc+ 3*j;
								unsigned char grayB1, grayB2, grayB3, grayB4;
								grayB1 = (unsigned char)*(pTempB);
								grayB2 = (unsigned char)*(pTempB + 3);
								grayB3 = (unsigned char)*(pTempB + wsSrc);
								grayB4 = (unsigned char)*(pTempB + wsSrc + 3);
								
								unsigned char gray = unsigned char( grayB1*(1-p)*(1-q) + 
									grayB2*(1-p)*q + 
									grayB3*p*(1-q) + 
									grayB4*p*q );
								if( bMosImagePixelValid )
								 {
									 pRowDst[3*xDst] = int(weight1*pRowDst[3*xDst] + weight2 * gray);
									 *(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = WEIGHT_PIXEL;//NON_WEIGHT_PIXEL
									 if(gray==0)
									 {
										weightB = 1.0f;
									 }
									 else
									 {
										 weightB = (float)pRowDst[3*xDst] / gray; // 图像2灰度变化比例
									 }
								 }
								 else
								 {
									 pRowDst[3*xDst] = gray;
									 *(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = NON_WEIGHT_PIXEL; // NON_WEIGHT_PIXEL
								 }
							}
							{	//G
								unsigned char* pTempG = pImages[n]->imageData + i*wsSrc+ 3*j + 1;
								unsigned char grayG1, grayG2, grayG3, grayG4;
								grayG1 = (unsigned char)*(pTempG);
								grayG2 = (unsigned char)*(pTempG + 3);
								grayG3 = (unsigned char)*(pTempG + wsSrc);
								grayG4 = (unsigned char)*(pTempG + wsSrc + 3);

								unsigned char gray = unsigned char( grayG1*(1-p)*(1-q) + 
									grayG2*(1-p)*q + 
									grayG3*p*(1-q) + 
									grayG4*p*q );
								if( bMosImagePixelValid )
								{
									pRowDst[3*xDst+1] = int(weight1*pRowDst[3*xDst+1] + weight2 * gray);
									*(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = WEIGHT_PIXEL;
									if(gray==0)
									{
										weightG = 1.0f;
									}
									else
									{
										weightG = (float)pRowDst[3*xDst+1] / gray; // 图像2灰度变化比例
									}
								}
								else
								{
									pRowDst[3*xDst+1] = gray;
									*(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = NON_WEIGHT_PIXEL;
								}
							}
							{	//R		
								unsigned char* pTempR = pImages[n]->imageData + i*wsSrc+ 3*j + 2;
								unsigned char grayR1, grayR2, grayR3, grayR4;
								grayR1 = (unsigned char)*(pTempR);
								grayR2 = (unsigned char)*(pTempR + 3);
								grayR3 = (unsigned char)*(pTempR + wsSrc);
								grayR4 = (unsigned char)*(pTempR + wsSrc + 3);

								unsigned char gray = unsigned char( grayR1*(1-p)*(1-q) + 
									grayR2*(1-p)*q + 
									grayR3*p*(1-q) + 
									grayR4*p*q );
								if( bMosImagePixelValid )
								{
									pRowDst[3*xDst+2] = int(weight1*pRowDst[3*xDst+2] + weight2 * gray);
									*(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = WEIGHT_PIXEL;
									if(gray==0)
									{
										weightR = 1.0f;
									}
									else
									{
										weightR = (float)pRowDst[3*xDst+2] / gray; // 图像2灰度变化比例
									}
								}
								else
								{
									pRowDst[3*xDst+2] = gray;
									*(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = NON_WEIGHT_PIXEL;
								}
							}
							*(pWeightR + (yDst-begBoxY)*boxW + xDst-begBoxX) = (weightR + weightG + weightB)*0.33333f;
						}
					}
				}
			}

			float *pFullWeightR = NULL, *pFullWeightR_Smoothed = NULL;
			if((n>0) && (blending==1))
			{
				pFullWeightR = new float[boxW*boxH];
				pFullWeightR_Smoothed = new float[boxW*boxH];
				//FillWeight(pWeightR, pFullWeightR, boxW, boxH, boxW);
				FillWeightSparse(pWeightR, pFullWeightR, boxW, boxH, boxW);
				//SmoothWeight( pFullWeightR, pFullWeightR_Smoothed, boxW, boxH, boxW,
				//	pWeightImg, NON_WEIGHT_PIXEL );
 				//SmoothWeight_MidVal( pFullWeightR, pFullWeightR_Smoothed, boxW, boxH, boxW,
 				//	pWeightImg, NON_WEIGHT_PIXEL );

				// 处理没有weight的像素
				for(int yDst=begBoxY; yDst<=endBoxY; yDst++)
				{
					unsigned char* pRowDst = pResult->imageData + yDst*wsNew;
					for(int xDst=begBoxX; xDst<=endBoxX; xDst++)
					{
						if( (xDst==376) && (yDst==7) )
						{
							xDst = xDst;
						}
						float xDst32F = xDst - dx, yDst32F = yDst - dy;
						float xSrc32F, ySrc32F;
						xSrc32F = (xDst32F*pInvM[0] + yDst32F*pInvM[1] + pInvM[2]) / (xDst32F*pInvM[6] + yDst32F*pInvM[7] + pInvM[8]);
						ySrc32F = (xDst32F*pInvM[3] + yDst32F*pInvM[4] + pInvM[5]) / (xDst32F*pInvM[6] + yDst32F*pInvM[7] + pInvM[8]);

						int srcW_1 = pImages[n]->width - 1, srcH_1 = pImages[n]->height-1, wsSrc = pImages[n]->widthStep;
						//双线性插值
						int ySrc = int(ySrc32F);
						int xSrc = int(xSrc32F);
						// 双线性插值
						if( (xSrc32F>=0) && (xSrc32F<srcW_1) && (ySrc32F>=0) && (ySrc32F<srcH_1) )
						{
							int xBox = xDst-begBoxX, yBox = yDst-begBoxY;
							if( pWeightImg->imageData[yBox*wsWtImg + xBox]==NON_WEIGHT_PIXEL )
							{
								//float kPixel = pFullWeightR[yBox*boxW+xBox];
								float kPixel = pFullWeightR[yBox*boxW+xBox];
								if(kPixel<0)
								{
									continue;
								}
								if( (kPixel<=0) || (kPixel>2) )
								{
									kPixel = kPixel;
								}
								pRowDst[3*xDst+0] = Min((int)(pRowDst[3*xDst+0] * kPixel),255);
								pRowDst[3*xDst+1] = Min((int)(pRowDst[3*xDst+1] * kPixel),255);
								pRowDst[3*xDst+2] = Min((int)(pRowDst[3*xDst+2] * kPixel),255);
							}
						}
					}
				}
			}

	#ifdef _DEBUG
			//SaveImageATL( pResult, "c:\\result.bmp" );
			//SaveImageATL( pWeightImg, "c:\\weighted-pixel.bmp" );			
			//COutData::OutData2Txt(pWeightR, boxW, boxH, boxW, "c:\\weight.txt");
			//COutData::OutData2Txt(pFullWeightR, boxW, boxH, boxW, "c:\\full-weight.txt");
			//COutData::OutData2Txt(pFullWeightR_Smoothed, boxW, boxH, boxW, "c:\\full-weight_smooth.txt");
	#endif
			
			ReleaseBitmap8U(pWeightImg);

			delete[] pWeightR; pWeightR = NULL;
			delete[] pFullWeightR; pFullWeightR = NULL;
			delete[] pFullWeightR_Smoothed; pFullWeightR_Smoothed = NULL;

			//===================
			//float* pM = pImgT[n].m;
			//int w = pImages[n]->width, h = pImages[n]->height, ws = pImages[n]->widthStep;
			//for(int ySrc=0; ySrc<h; ySrc++)
			//{
			//	for(int xSrc=0; xSrc<w; xSrc++)
			//	{
			//		if( (xSrc==674) && (ySrc==190) )
			//			xSrc = xSrc;
			//		if( (xSrc==486) && (ySrc==8) )
			//			xSrc = xSrc;
			//		float xDst = 0, yDst = 0;
			//		xDst = (xSrc*pM[0] + ySrc*pM[1] + pM[2]) / (xSrc*pM[6] + ySrc*pM[7] + pM[8]);
			//		yDst = (xSrc*pM[3] + ySrc*pM[4] + pM[5]) / (xSrc*pM[6] + ySrc*pM[7] + pM[8]);
			//		xDst += dx;
			//		yDst += dy;
			//		*(pResult->imageData + (int)yDst*wsNew + (int)xDst*3) = *(pImages[n]->imageData + ySrc*ws + xSrc*3);
			//		*(pResult->imageData + (int)yDst*wsNew + (int)xDst*3+1) = *(pImages[n]->imageData + ySrc*ws + xSrc*3+1);
			//		*(pResult->imageData + (int)yDst*wsNew + (int)xDst*3+2) = *(pImages[n]->imageData + ySrc*ws + xSrc*3+2);
			//	}
			//}

		}
	}
	delete[] pSizeImagesBig; pSizeImagesBig = NULL;
	delete[] pSizeImagesSmall; pSizeImagesSmall = NULL;
	delete[] pQuadrangleCorners; pQuadrangleCorners = NULL;
	delete[] pBegBox; pBegBox = NULL;
	delete[] pEndBox; pEndBox = NULL;
	
	//cout<<"return Merge"<<endl;

	return pResult;
}

// 用基本的Harris+NCC, 匹配两幅图像
int CMosaicHarris::MosaicTwoImages(BitmapImage* pSrc1, BitmapImage* pSrc2, float aProjectMat[9],
					int maxFeatruesNum, int winR, 
					float minScore,
					float ransacDist/**/,
					int searchR/**/,
					CornerDescriptor* pCorner1, CornerDescriptor* pCorner2, int nCorner1, int nCorner2)
{
	bool isMosaicSuccess = false;
	// 两两拼接
	if(pSrc1->nChannels==3)// 彩色图
	{
		BitmapImage* pGray1 = NULL, *pGray2 = NULL;
		int w1 = pSrc1->width;
		int h1 = pSrc1->height;
		int w2 = pSrc2->width;
		int h2 = pSrc2->height;
		pGray1 = CreateBitmap8U( w1, h1, 1 );
		pGray2 = CreateBitmap8U( w2, h2, 1 );
		// 先变成灰度图
		ColorToGray1(pSrc1, pGray1);
		ColorToGray1(pSrc2, pGray2);
		//SaveImageATL(pGray1, "c:\\gray1.bmp");
		//SaveImageATL(pGray2, "c:\\gray2.bmp");

		// 拼接
		isMosaicSuccess = MosaicHarrisDescriptor( pGray1, pGray2, aProjectMat, 
			pCorner1, pCorner2, nCorner1, nCorner2 );
		ReleaseBitmap8U(pGray1);
		ReleaseBitmap8U(pGray2);
	}
	else if(1==pSrc1->nChannels) // 灰度图
	{
		isMosaicSuccess = MosaicHarrisDescriptor( pSrc1, pSrc2, aProjectMat,
			pCorner1, pCorner2, nCorner1, nCorner2);
	}

	if(!isMosaicSuccess)
		return -1;
	
	return 0;
}

// 功能：计算目标函数的值
int CalSumOfDistError(int w1, int h1, int w2, int h2, 
						   float* pH1, float *pH2, float *A,
						   double &error2)
{
	// 8个变量
	int X1[4] = { 0, w1-1, w1-1, 0 };
	int Y1[4] = { 0, 0,    h1-1, h1-1 };
	int X2[4] = { 0, w2-1, w2-1, 0 };
	int Y2[4] = { 0, 0,    h2-1, h2-1 };

	const int EDGE_NUM = 4;	
	float C[EDGE_NUM*2];

	float weight[2] = {1, 1};

	int tabI[EDGE_NUM] = { 1, 3, 0, 1 };
	int tabJ[EDGE_NUM] = { 2, 0, 2, 3 };
	float D[EDGE_NUM*2];
	for(int n=0; n<EDGE_NUM*2; n++)
	{
		float x1, y1, x2, y2;
		if(n<EDGE_NUM)
		{
			int i = tabI[n], j = tabJ[n];
			x1 = (float)X1[i]; y1 = (float)Y1[i];
			x2 = (float)X1[j]; y2 = (float)Y1[j];
		}
		else
		{
			int i = tabI[n-EDGE_NUM], j = tabJ[n-EDGE_NUM];
			x1 = (float)X2[i]; y1 = (float)Y2[i];
			x2 = (float)X2[j]; y2 = (float)Y2[j];
		}
		DistanceOfTwoPoints( x1, y1, x2, y2, D[n] );
	}

	float DBar[EDGE_NUM*2];

	int nData = 0;

	int nMaxIter = 1;

	nData = 0;

	error2 = 0;
	// 计算雅克比矩阵
	for(int n=0; n<EDGE_NUM*2;n++)
	{
		float currentW = 0;
		float* h = NULL;
		float xi, yi, xj, yj;
		int i = 0, j = 0;
		if(n<EDGE_NUM)
		{
			currentW = weight[0];
			h = pH1;
			i = tabI[n]; j = tabJ[n];
			xi = (float)X1[i]; yi = (float)Y1[i];
			xj = (float)X1[j]; yj = (float)Y1[j]; 
		}
		else
		{
			currentW = weight[1];
			h = pH2;
			i = tabI[n-EDGE_NUM]; j = tabJ[n-EDGE_NUM];
			xi = (float)X2[i]; yi = (float)Y2[i];
			xj = (float)X2[j]; yj = (float)Y2[j];
		}

		float xi_ = 0, yi_ = 0, xj_ = 0, yj_ = 0;
		float m[9];
		MulMatrix( A, 3, 3, h, 3, 3, m );

		float m6m7m8_i = (m[6]*xi + m[7]*yi + m[8]);
		float m6m7m8_j = (m[6]*xj + m[7]*yj + m[8]);
		xi_ = (m[0] * xi + m[1]*yi + m[2]) / m6m7m8_i;
		yi_ = (m[3] * xi + m[4]*yi + m[5]) / m6m7m8_i;

		xj_ = (m[0] * xj + m[1]*yj + m[2]) / m6m7m8_j;
		yj_ = (m[3] * xj + m[4]*yj + m[5]) / m6m7m8_j;

		DBar[n] = sqrt( (xi_-xj_)*(xi_-xj_) + (yi_-yj_)*(yi_-yj_) );

		C[nData] = DBar[n] - D[n];

		error2 += C[nData] * C[nData] * currentW;

		nData++;
	}
	return 0;
}

int SelectHomographyExaustive( int w1, int h1, int w2, int h2, 
							  float* pH1, float *pH2, float *A)
{
	// 给定初值
	memset(A, 0, sizeof(float)*9 );
	A[0] = 1; A[4] = 1; A[8] = 1;

	double minError2 = 0;
	CalSumOfDistError(w1, h1, w2, h2, 
		pH1, pH2, A, minError2);

	cout<<"初始误差 "<<minError2<<endl;

	float h6Best = 0, h7Best = 0;

	float h6Max = 0.001f;
	float h6Min = -h6Max;
	float h7Min = h6Min, h7Max = h6Max;

	int minSkewAng = -30; //单位为度
	int maxSkewAng = -minSkewAng;
	float nSamlingSkew = 15;
	float stepSkew = (maxSkewAng-minSkewAng) / nSamlingSkew;

	float minSx = 0.5f;
	float maxSx = 2.0f;
	int nSamplingS = 10;
	float stepScale = (maxSx - minSx) / nSamplingS;
	
	int nSampling = 20;
	float step32F = (h6Max - h6Min) / nSampling;
	float bestA[9] = {0}, AT[9];
	memcpy( bestA, A, sizeof(bestA) );

	for(int nSkew = 0; nSkew <nSamlingSkew; nSkew++) // 歪斜角度
	{
		cout<<"nSkew "<<nSkew<<endl;
		// 处理歪斜
		float skewAng = (minSkewAng + nSkew * stepSkew)/180*pool::pi;
		float sinAng = sin(skewAng);
		float cosAng = cos(skewAng);
		for(int nSx =0; nSx<nSamplingS; nSx++)
		{
			float sx = minSx + nSx*stepScale;
			for(int nSy =0; nSy<nSamplingS; nSy++)
			{
				memcpy( AT, A, sizeof(bestA) );

				AT[1] = -sinAng;
				AT[4] = cosAng;
				float sy = minSx + nSy*stepScale;
				AT[0] *= sx;
				AT[1] *= sy; 
				AT[3] *= sx;
				AT[4] *= sy;
				
		for(int n6=0; n6<nSampling; n6++) // h6
		{
			float h6 = h6Min + n6 * step32F;
			for(int n7=0; n7<nSampling; n7++) // h7
			{
				float h7 = h7Min + n7 * step32F;
				AT[6] = A[6] + h6;
				AT[7] = A[7] + h7;
				// 计算目标函数的值
				double error2 = 0;
				CalSumOfDistError(w1, h1, w2, h2, 
					pH1, pH2, AT, error2);
				if(error2<minError2)
				{
					//cout<<"smaller error "<<error2<<endl;
					minError2 = error2;
					memcpy( bestA, AT, sizeof(bestA) );
				}
			}
		}
			}
		}
	}

	memcpy( A, bestA, sizeof(bestA) );
	
	return 0;
}

// 函数功能：角点匹配,NCC
// 自由匹配(无约束)
// fSimilarityThreshold, 相似度阈值
// winRadius, 窗口半径
bool MatchHarrisFeature(BitmapImage* pSrc1, BitmapImage* pSrc2,
						CornerDescriptor* pCorner1, const int cornerNum1, 
						CornerDescriptor* pCorner2, const int cornerNum2,
						std::vector<CornerDescriptor> &vecMatchedPoints1, 
						std::vector<CornerDescriptor> &vecMatchedPoints2,
						const int winRadius,
						const float fSimilarityThreshold, int localSearchR, float *pH)
{
	return true;
}

// 功能: 补全图像2的灰度变化比例
// pSrc指向图像2的灰度变化比例, 没有填充的单位格的比例值为0
// pSrc和pDst具有相同的尺寸
int FillWeight(float* pSrcW, float* pDstW, int w, int h, int ws)
{
	if( (NULL==pSrcW) || (NULL==pDstW) )
		return -1;

	float *pTemp = new float[ws*h];

	float *pSrc = pSrcW;
	float *pDst = pDstW;

	memcpy( pDst, pSrc, ws*h*sizeof(float) );

	for(int y=0; y<h; y++)
	{
		for(int x=0; x<w; x++)
		{
			float currentW = *(pSrc + y*ws + x);
			if( currentW>0 )
			{
				// 检查4个方向
				// (1) 向上
				for(int yUp = y; yUp<h-1; yUp++)
				{
					if( *(pSrc + (yUp+1)*ws + x)<0 )
					{
						*(pDst + (yUp+1)*ws + x) = currentW;
					}
					else
					{
						break;
					}
				}
				// (3) 向下
				for(int yD = y; yD>0; yD--)
				{
					if( *(pSrc + (yD-1)*ws + x)<0 )
					{
						*(pDst + (yD-1)*ws + x) = currentW;
					}
					else
					{
						break;
					}
				}
			}
		}
	}

	memcpy( pTemp, pDst, ws*h*sizeof(float) );

	pSrc = pTemp;
	pDst = pDstW;

	for(int y=0; y<h; y++)
	{
		for(int x=0; x<w; x++)
		{
			float currentW = *(pSrc + y*ws + x);
			if( currentW>0 )
			{
				// (2) 向右
				for(int xR = x; xR<w-1; xR++)
				{
					if( *(pSrc + y*ws + xR+1)<0 )
					{
						*(pDst + y*ws + xR+1) = currentW;
					}
					else
					{
						break;
					}
				}
				// (4) 向左
				for(int xL = x; xL>0; xL--)
				{
					if( *(pSrc + y*ws + xL-1)<0 )
					{
						*(pDst + y*ws + xL-1) = currentW;
					}
					else
					{
						break;
					}
				}
			}
		}
	}
	memcpy( pDstW, pDst, ws*h*sizeof(float) );

	delete[] pTemp; pTemp = NULL;

	return 0;
}

// 功能: 补全图像2的灰度变化比例
// pSrc指向图像2的灰度变化比例, 没有填充的单位格的比例值为0
// pSrc和pDst具有相同的尺寸
int FillWeightSparse(float* pSrcW, float* pDstW, int w, int h, int ws)
{

	return 0;
}

// 对灰度变化比例进行平滑
int SmoothWeight( float *pSrcW32F, float *pSmoothed32F, int w, int h, int ws,
				 BitmapImage* pWeightImg, const int NON_WEIGHT_PIXEL )
{
	int wsImg = pWeightImg->widthStep;
	int w_1 = w - 1;
	int h_1 = h - 1;

	memcpy( pSmoothed32F, pSrcW32F, sizeof(float)*ws*h );

	const int R = 4;
	for(int y=0; y<h; y++)
	{
		for(int x=0; x<w; x++)
		{
			if( *(pWeightImg->imageData + y*wsImg + x)==NON_WEIGHT_PIXEL )
			{
				// 均值滤波
				int nValid = 0;
				float meanVal = 0;
				for(int p=-R; p<=R; p++)
				{
					int y2 = y + p;
					if( (y2<0)||(y2>h_1) )
					{
						continue;
					}
					for(int q=-R; q<=R; q++)
					{
						int x2 = x + q;
						if( (x2<0)||(x2>w_1) )
						{
							continue;
						}
						float currentWeight = *(pSrcW32F + y2*ws + x2);
						if(currentWeight>0)
						{
							meanVal += currentWeight;
							nValid++;
						}
					}
				}
				*(pSmoothed32F + y*ws + x) = meanVal / nValid;
			}
		}
	}
	
	return 0;
}

// 对灰度变化比例进行平滑, 用中值滤波
int SmoothWeight_MidVal( float *pSrcW32F, float *pSmoothed32F, int w, int h, int ws,
				 BitmapImage* pWeightImg, const int NON_WEIGHT_PIXEL )
{
	int wsImg = pWeightImg->widthStep;
	int w_1 = w - 1;
	int h_1 = h - 1;

	memcpy( pSmoothed32F, pSrcW32F, sizeof(float)*ws*h );
	vector<float> vecWeight;
	const int R = 4;
	for(int y=0; y<h; y++)
	{
		for(int x=0; x<w; x++)
		{
			if( *(pWeightImg->imageData + y*wsImg + x)==NON_WEIGHT_PIXEL )
			{
				// 均值滤波
				int nValid = 0;
				float meanVal = 0;
				for(int p=-R; p<=R; p++)
				{
					int y2 = y + p;
					if( (y2<0)||(y2>h_1) )
					{
						continue;
					}
					for(int q=-R; q<=R; q++)
					{
						int x2 = x + q;
						if( (x2<0)||(x2>w_1) )
						{
							continue;
						}
						float currentWeight = *(pSrcW32F + y2*ws + x2);
						if(currentWeight>0)
						{
							vecWeight.push_back(currentWeight);
						}
					}
				}
				int nVecW = (int)vecWeight.size();
				sort(vecWeight.begin(), vecWeight.end());
				*(pSmoothed32F + y*ws + x) = vecWeight[nVecW/2]; // 中值滤波
				vecWeight.clear();
			}
		}
	}

	return 0;
}

// 金字塔混合图像
int PyramidBlending(BitmapImage** pImages, int imagesNum, ProjectMat* pImgT, int blending)
{
	if(imagesNum>2)
		return -1;

	cout<<"merge images"<<endl;
	if(pImgT==NULL)
		return NULL;
	if(imagesNum<=1)
		return NULL;

	int nChannels = pImages[0]->nChannels;

	float A[9] = {1,0,0, 0,1,0, 0,0,1};
	//KeepOriginalImageInfo2( pImages[0]->width, pImages[0]->height, 
	//					   pImages[imagesNum-1]->width, pImages[imagesNum-1]->height, 
	//					   pImgT[0].m, pImgT[imagesNum-1].m, 
	//					   A);

	//SelectHomographyExaustive(pImages[0]->width, pImages[0]->height, 
	//							pImages[imagesNum-1]->width, pImages[imagesNum-1]->height, 
	//							pImgT[0].m, pImgT[imagesNum-1].m, 
	//							A);

	if(imagesNum>15)
	{
		float A15[9] = {1.7744670f,0.50448841f,-213.70569f,
			0.31337968f,	1.9825864f,	-194.38675f,
			0.00040634593f, 0.0015788968f, 1};
		float H15_inv[9];
		InverseMatrix(pImgT[15].m, 3, H15_inv, 1e-12f);
		MulMatrix( A15, 3, 3, H15_inv,3, 3, A  );
	}

	if(1)
	{
		for(int n=0; n<9; n++)
		{
			cout<<A[n]<<" ";
		}
		cout<<endl;
		for(int n=0; n<imagesNum; n++)
		{
			float temp[9];
			MulMatrix(A, 3, 3, pImgT[n].m, 3, 3, temp);
			memcpy( pImgT[n].m, temp, sizeof(temp) );
		}
	}

	//// 
	//memset(pImgT[0].m, 0, sizeof(float)*9);
	//pImgT[0].m[0] = 1; 
	//pImgT[0].m[4] = 1;
	//pImgT[0].m[8] = 1;

	//确定新图像的大小
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
	// 新的图像宽度
	int newWidth = int(maxX - minX + 1.5f);
	int newHeight = int(maxY - minY + 1.5f);

	cout<<"newWidth "<<newWidth<<" newHeight "<<newHeight<<endl;

	float dx = -minX, dy = -minY;
	// 找出每叠加一幅图之后的图像的尺寸
	SfPoint *pSizeImagesBig = new SfPoint[imagesNum];
	SfPoint *pSizeImagesSmall = new SfPoint[imagesNum];
	for(int n=0; n<imagesNum; n++)
	{
		for(int i=0; i<4; i++)
		{
			pQuadrangleCorners[n*4+i].x += dx;
			pQuadrangleCorners[n*4+i].y += dy;
		}
		pSizeImagesBig[n].x = 0; pSizeImagesBig[n].y = 0;
		pSizeImagesSmall[n].x = 1<<29; pSizeImagesSmall[n].y = 1<<29;
		for(int i=0; i<=n; i++)
		{
			if(pEndBox[i].x + dx>pSizeImagesBig[n].x)
			{
				pSizeImagesBig[n].x = pEndBox[i].x + dx;
			}
			if(pEndBox[i].y + dy>pSizeImagesBig[n].y)
			{
				pSizeImagesBig[n].y = pEndBox[i].y + dy;
			}

			if(pBegBox[i].x + dx<pSizeImagesSmall[n].x)
			{
				pSizeImagesSmall[n].x = pBegBox[i].x + dx;
			}
			if(pBegBox[i].y + dy<pSizeImagesSmall[n].y)
			{
				pSizeImagesSmall[n].y = pBegBox[i].y + dy;
			}
		}
	}
	
	cout<<"create new image memory"<<endl;
	BitmapImage* pResult = NULL;
	int wsNew = 0;
	double MAX_NEW_SIZE = 10000.0*10000.0;
	double newSize = (double)newWidth*(double)newHeight;
	if(newSize<MAX_NEW_SIZE) // 如果图像的尺寸超过最大尺寸, 则认为拼接失败
	{
		pResult = CreateBitmap8U( newWidth, newHeight, pImages[0]->nChannels );
		ZeroImage(pResult);
		wsNew = pResult->widthStep;
	}
	
	if(pResult)
	{
		for(int n=0; n<imagesNum; n++)
		{
			int begBoxX = int(pBegBox[n].x + dx);
			int begBoxY = int(pBegBox[n].y + dy);
			int endBoxX = int(pEndBox[n].x + dx + 0.5f);
			int endBoxY = int(pEndBox[n].y + dy + 0.5f);

			float h_toBox[9] = {0};
			// 计算重叠类型
			int overlapType = -1;
			if(n>0)
			{
				//overlapType = CalOverlapType(pSizeImagesBig[n-1], pSizeImagesSmall[n-1], begBoxX, begBoxY, endBoxX, endBoxY);
				// 计算重叠类型
				// pMergedImg-------------已经融合的大图
				// pQuadrangleCorners-----即将加入的图像的4个顶点坐标
				CalOverlapType2(pResult, pQuadrangleCorners+n*4, overlapType);
				cout<<n<<" overlayType "<<overlapType<<endl;

				SfPoint boxPoints[4];
				boxPoints[0].x = (float)begBoxX; boxPoints[0].y = (float)begBoxY;
				boxPoints[1].x = (float)endBoxX; boxPoints[1].y = (float)begBoxY;
				boxPoints[2].x = (float)endBoxX; boxPoints[2].y = (float)endBoxY;
				boxPoints[3].x = (float)begBoxX; boxPoints[3].y = (float)endBoxY;
				//SolvePseudoAffine( boxPoints, pQuadrangleCorners+n*4, 4, k_toBox );
				SolveProjectMatrix3(boxPoints, pQuadrangleCorners+n*4, 4, h_toBox);
				ProjectMat temp, fineH;
				memcpy(temp.m, h_toBox, sizeof(float)*9);
				NonlinearLeastSquareProjection(boxPoints, pQuadrangleCorners+n*4, 4, fineH.m, temp.m, 1e-9f );
				memcpy(h_toBox, fineH.m, sizeof(float)*9);
			}

			float* pM = pImgT[n].m;
			float pInvM[9];
			InverseMatrix(pM, 3, pInvM); // 求变换矩阵的逆矩阵

			//if(n!=imagesNum/2)
			//{
			//	continue;
			//}
			int boxW = endBoxX - begBoxX + 1;
			int boxH = endBoxY - begBoxY + 1;
			BitmapImage* pWeightImg = CreateBitmap8U( boxW, boxH, 1 );
			int boxW0 = pSizeImagesBig[n-1].x-pSizeImagesSmall[n-1].x+1;
			int boxH0 = pSizeImagesBig[n-1].y-pSizeImagesSmall[n-1].y+1;
			BitmapImage* pWeightImg0 = CreateBitmap8U( boxW0, boxH0, 1);
			ZeroImage(pWeightImg0);
			for(int y=0; y<boxH0; y++)
			{
				for(int x=0; x<boxW0; x++)
				{
					if( *(pResult->imageData + y*pResult->widthStep+x*3)>0)
					{
						*(pWeightImg0->imageData + y*pWeightImg0->widthStep + x) = 255;
					}
				}
			}
			int wsWtImg = pWeightImg->widthStep;
			float* pWeightR = new float[boxW*boxH];
			ZeroImage(pWeightImg);
			const int NON_WEIGHT_PIXEL = 255, WEIGHT_PIXEL = 128;

			for(int yDst=begBoxY; yDst<=endBoxY; yDst++)
			{
				unsigned char* pRowDst = pResult->imageData + yDst*wsNew;
				for(int xDst=begBoxX; xDst<=endBoxX; xDst++)
				{
					*(pWeightR + (yDst-begBoxY)*boxW + xDst-begBoxX) = -1;
					if( (xDst==203) && (yDst==140) )
					{
						xDst = xDst;
					}
					float xDst32F = xDst - dx, yDst32F = yDst - dy;
					float xSrc32F, ySrc32F;
					xSrc32F = (xDst32F*pInvM[0] + yDst32F*pInvM[1] + pInvM[2]) / (xDst32F*pInvM[6] + yDst32F*pInvM[7] + pInvM[8]);
					ySrc32F = (xDst32F*pInvM[3] + yDst32F*pInvM[4] + pInvM[5]) / (xDst32F*pInvM[6] + yDst32F*pInvM[7] + pInvM[8]);

					int srcW_1 = pImages[n]->width - 1, srcH_1 = pImages[n]->height-1, wsSrc = pImages[n]->widthStep;
					//双线性插值
					int ySrc = int(ySrc32F);
					int xSrc = int(xSrc32F);
					// 双线性插值
					if( (xSrc32F>=0) && (xSrc32F<srcW_1) && (ySrc32F>=0) && (ySrc32F<srcH_1) )
					{
						float p = ySrc32F - ySrc;
						float q = xSrc32F - xSrc;

						// blending
						float weight1 = 0.0f, weight2 = 1;
						if (blending==1) // 线性加权平均
						{
							float xDstTrans = (float)xDst, yDstTrans = (float)yDst;
							//ApplyPseudoAffine(xDst, yDst, xDstTrans, yDstTrans, k_toBox); // 把当前点表换到标准的矩形中
							ApplyProjectMat2(xDst, yDst, xDstTrans, yDstTrans, h_toBox);
							if(1==overlapType)
							{
								float overlapW = float(endBoxX - begBoxX);
								float overlapH = endBoxY - pSizeImagesSmall[n-1].y;

								float mX = 0.5f * (endBoxX + begBoxX);
								float dmX = abs(xDstTrans-mX);
								float dY = yDstTrans - pSizeImagesSmall[n-1].y;
								float w1 = 1, w2 = 0, w3 = 0, w4 = 0;
								float p = dmX / (0.5f*overlapW);
								float q = dY / overlapH;
								weight2 = (1-p)*(1-q)*w1;
							}
							else if(2==overlapType)
							{
								float overlapW = pSizeImagesBig[n-1].x - begBoxX;
								float overlapH = pSizeImagesBig[n-1].y - begBoxY;
								float overlapX = xDstTrans - begBoxX;
								float overlapY = yDstTrans - begBoxY;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								float w1 = 0, w2 = 0, w3 = 1, w4 = 0;
								weight2 = (1-p) * (1-q) * w1 + p * (1-q) * w4 + p * q* w3 + (1-p) * q * w2;
							}
							else if(3==overlapType)
							{
								float overlapW = pSizeImagesBig[n-1].x - begBoxX;
								float overlapH = (float)(endBoxY - begBoxY);
								float mY = 0.5f * (endBoxY + begBoxY);
								float dmY = abs(yDstTrans - mY);
								float dX = pSizeImagesBig[n-1].x - xDstTrans;
								float p = dX / overlapW;
								float q = dmY / (0.5f*overlapH);
								float w1 = 1;
								weight2 = (1-p)*(1-q)*w1;
							}
							else if(4==overlapType)
							{
								float w1 = 0, w2 = 0, w3 = 0, w4 = 1;
								float overlapW = pSizeImagesBig[n-1].x - begBoxX;
								float overlapH = endBoxY - pSizeImagesSmall[n-1].y;

								float overlapX = (float)(xDstTrans - begBoxX);
								float overlapY = yDstTrans - pSizeImagesSmall[n-1].y;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								weight2 = (1-p) * (1-q) * w1 + p * (1-q) * w4 + p * q* w3 + (1-p) * q * w2;
							}
							else if(5==overlapType)
							{
								float w1 = 1;
								float overlapW = (float)(endBoxX - begBoxX);
								float overlapH = pSizeImagesBig[n-1].y - begBoxY;
								float mX = 0.5f * (endBoxX + begBoxX);
								float overlapX = (float)(xDstTrans - mX);
								float overlapY = yDstTrans - begBoxY;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								weight2 = (1-p) * (1-q) * w1 ;
							}
							else if(6==overlapType)
							{
								float overlapW = endBoxX - pSizeImagesSmall[n-1].x;
								float overlapH = pSizeImagesBig[n-1].y - begBoxY;
								float overlapX = xDstTrans - pSizeImagesSmall[n-1].x;
								float overlapY = yDstTrans - begBoxY;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								float w1 = 1;
								weight2 = p*(1-q)*w1;
							}
							else if(7==overlapType)
							{
								float overlapW = endBoxX - pSizeImagesSmall[n-1].x;
								float overlapH = (float)(endBoxY - begBoxY);
								float mY = 0.5f * (endBoxY + begBoxY);
								float overlapX = xDstTrans - pSizeImagesSmall[n-1].x;
								float dmY = abs(yDstTrans - mY);
								float p = overlapX / overlapW;
								float q = dmY / (0.5f*overlapH);
								float w3 = 1;
								weight2 = (1-p)*(1-q)*w3;
							}
							else if(8==overlapType)
							{
								float overlapW = endBoxX - pSizeImagesSmall[n-1].x;
								float overlapH = endBoxY - pSizeImagesSmall[n-1].y;
								float overlapX = xDstTrans - pSizeImagesSmall[n-1].x;
								float overlapY = yDstTrans - pSizeImagesSmall[n-1].y;
								float p = overlapX/overlapW;
								float q = overlapY/overlapH;
								float w1 = 1;
								weight2 = (1-p)*(1-q)*w1;
							}
							else if(9==overlapType)
							{
								int overlapW = endBoxX - begBoxX;
								int overlapH = endBoxY - begBoxY;
								float mX = 0.5f * (endBoxX + begBoxX);
								float mY = 0.5f * (endBoxY + begBoxY);
								float dmX = abs(xDstTrans - mX);
								float dmY = abs(yDstTrans - mY);
								weight2 = (0.5f*overlapW-dmX) * (0.5f*overlapH-dmY) / (0.25f * overlapW * overlapH);
								//weight1 = 1 - weight2;
							}
							weight1 = 1 - weight2;
							if( (weight1<0) || (weight2<0) || (weight1>1) || (weight2>1))
							{
								weight1 = 0.5f; weight2 = 0.5f;
							}
							if(nChannels==1)
							{
								unsigned char* pSrcTemp = pImages[n]->imageData + ySrc*wsSrc + xSrc;

								unsigned char gray = unsigned char((unsigned char)*(pSrcTemp)*(1-p)*(1-q) + 
									(unsigned char)*(pSrcTemp + 1)*(1-p)*q + 
									(unsigned char)*(pSrcTemp + wsSrc)*p*(1-q) + 
									(unsigned char)*(pSrcTemp + wsSrc + 1)*p*q );
								if(*(pRowDst + xDst) >0 )
							 {
								 *(pRowDst + xDst) = int( (gray + *(pRowDst + xDst))*0.5f );
							 }
							 else
							 {
								 *(pRowDst + xDst) = gray;
							 }
							}
							else if(3==nChannels)
							{
								float weightR = -1, weightG = -1, weightB = -1;
								int i = ySrc;
								int j = xSrc;
								{	//B
									unsigned char* pTempB = pImages[n]->imageData + i*wsSrc+ 3*j;
									unsigned char grayB1, grayB2, grayB3, grayB4;
									grayB1 = (unsigned char)*(pTempB);
									grayB2 = (unsigned char)*(pTempB + 3);
									grayB3 = (unsigned char)*(pTempB + wsSrc);
									grayB4 = (unsigned char)*(pTempB + wsSrc + 3);

									unsigned char gray = unsigned char( grayB1*(1-p)*(1-q) + 
										grayB2*(1-p)*q + 
										grayB3*p*(1-q) + 
										grayB4*p*q );
									if( pRowDst[3*xDst] >0 )
									 {
										 pRowDst[3*xDst] = int(weight1*pRowDst[3*xDst] + weight2 * gray);
	
										 if(weight2>weight1)
										 {
											 *(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = 255;
										 }
							
										 if( (xDst<boxW0) || (yDst<boxH0) )
										 {
											 if(weight1>weight2)
											 {
												*(pWeightImg0->imageData + yDst*pWeightImg0->widthStep + xDst) = 255;
											 }
											 else
											 {
												 *(pWeightImg0->imageData + yDst*pWeightImg0->widthStep + xDst) = 0;
											 }
										 }
									 }
									 else
									 {
										 pRowDst[3*xDst] = gray;
										 *(pWeightImg->imageData + (yDst-begBoxY)*pWeightImg->widthStep + xDst-begBoxX) = 255; // NON_WEIGHT_PIXEL
									 }
								}
							}
						}
					}
				}
			}
			//SaveImageATL(pWeightImg, "c:\\weightImg.bmp");
			//SaveImageATL(pWeightImg0, "c:\\weightImg0.bmp");
			if(n>0)
			{
				// pyramid blending
				int level = 3;
				float sigma = 1;
				
			}
		}
	}

	
	

	return 0;
}

