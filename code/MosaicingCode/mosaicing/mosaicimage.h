/*

*/
#pragma once
#include "Bitmap.h"
//#include "LogPolar.h"
#include "Similarity.h"
#include "OutData.h"
#include "convert.h"
#include "usingAtlImage.h"
//#include "DrawFigure.h"
#include "ImageGeometry.h"
#include "opencv2\opencv.hpp"
#include "mvMath.h"
#include "MosaicWithoutPos.h"
using namespace pool;

// 用4点表示一个矩形
struct Rectangle4Points
{
	SfPoint pt[4];
};

enum TRANSFORM_TYPE
{
	AFFINE_MODEL = 0, // 仿射变换模型
	PROJECT_MODEL = 1, // 投影变换模型
	EUCLIDEAN_MODEL = 2 // 欧式变换模型
};

struct SampleIndexs
{
	std::vector<int> elem;
};

struct MatchPoints
{
	float x;
	float y;
	float score;
	int selfNum;
	int yourNum;

	bool   operator <  (const MatchPoints& mp)  const   //升序排序时必须写的函数
	{   
		return score<mp.score; 
	}
};

struct ProjectionData1000
{
	int data[1000];
};

class CMosaicHarris
{
public:

	CMosaicHarris();

	~CMosaicHarris();

	// pSrc1，原图1
	// pSrc2，原图2
	// aProjectTransform,，投影变换矩阵
	bool Mosaic(BitmapImage* pSrc1, BitmapImage* pSrc2, float aProjectTransform[9]);

	// Harris拼接
	bool MosaicHarris(BitmapImage* pSrc1, BitmapImage* pSrc2, float aProjectMat[9],
			SfPoint* pCorner1 = NULL, SfPoint *pCorner2 = NULL,
			int nCorner1 = 0, int nCorner2 = 0);

	// 函数功能：图像融合
	bool Merge( BitmapImage* pSrc1, BitmapImage* pSrc2, 
			   BitmapImage* &pResult, 
			   float aProjectMat[9]);

	// 灰度图转彩色图
	static bool GrayToColor2(BitmapImage* pSrcGray, BitmapImage* pDstColor);


	// 彩色图转灰度图
	// Gray = R*0.299 + G*0.587  + B*0.114;
	//static bool ColorToGray1(BitmapImage *pSrcColor, BitmapImage *pDstGray);

	// 填充Image2
	// strMethod, "bilinear"
	bool FillImage( BitmapImage* pDstImage, BitmapImage* pSrcImage, 
		InterestRegion2 region,
		int dx, int dy,
		float aProjectMat[9], string strMethod );

	// **********************************************************************
	// 函数功能：剔除误匹配 + 投影矩阵计算
	// **********************************************************************
	template<class PointType>
	bool Ransac(BitmapImage* pSrc1, BitmapImage* pSrc2,
				const std::vector<PointType> &vecMatchedPoints1, 
				const std::vector<PointType> &vecMatchedPoints2,
				float aProjectMat[9],
				float fRansacDist = 1, int sampleTimes = 1000)
	{
		cout<<vecMatchedPoints1.size()<<" "<<vecMatchedPoints2.size()<<endl;
		if( (pSrc1==NULL) || (pSrc2==NULL) )
			return false;
		if(vecMatchedPoints1.empty())
		{
			return false;
		}
		if((int)vecMatchedPoints1.size()<=0)
			return false;

		m_vecInnerPoints1.clear();
		m_vecInnerPoints2.clear();

		bool isSuccess = true;

		int MIN_SAMPLING_NUM = 4;
		if(m_transformType==AFFINE_MODEL)
		{
			MIN_SAMPLING_NUM = 3;
		}

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
		srand( (unsigned)time(0) );

		SfPoint samPoint1Best[4];
		SfPoint samPoint2Best[4];

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

			int aSampleIndex[4];

			PointType samPoint1[4];
			PointType samPoint2[4];

			if(m_transformType==PROJECT_MODEL)
			{
				do
				{
					for(int i=0; i<MIN_SAMPLING_NUM; i++)
					{
						aSampleIndex[i] = rand()%ptNum;
					}
				}
				while( ( aSampleIndex[0]==aSampleIndex[1] ) || 
					( aSampleIndex[0]==aSampleIndex[2] ) ||
					( aSampleIndex[0]==aSampleIndex[3] ) ||
					( aSampleIndex[1]==aSampleIndex[2] ) ||
					( aSampleIndex[1]==aSampleIndex[3] ) ||
					( aSampleIndex[2]==aSampleIndex[3] ) ); // 要求采样4个不同的点
			}
			else if(m_transformType==AFFINE_MODEL)
			{
				do
				{
					for(int i=0; i<MIN_SAMPLING_NUM; i++)
					{
						aSampleIndex[i] = rand()%ptNum;
					}
				}
				while( ( aSampleIndex[0]==aSampleIndex[1] ) || 
					( aSampleIndex[0]==aSampleIndex[2] ) ||
					( aSampleIndex[1]==aSampleIndex[2] ) ); // 要求采样3个不同的点
			}

			//// 判断该采样组是否已经使用过
			//SampleIndexs currentSamIndexs;
			//currentSamIndexs.elem.push_back( aSampleIndex[0] );
			//currentSamIndexs.elem.push_back( aSampleIndex[1] );
			//currentSamIndexs.elem.push_back( aSampleIndex[2] );
			//currentSamIndexs.elem.push_back( aSampleIndex[3] );
			//sort(currentSamIndexs.elem.begin(), currentSamIndexs.elem.end()); // 排序

			//bool isCurrentSamIndexUsed = false;
			//for(int i=0; i<(int)vecHaveSampled.size(); i++)
			//{
			//	if( ( vecHaveSampled[i].elem[0]==currentSamIndexs.elem[0] ) &&
			//		( vecHaveSampled[i].elem[1]==currentSamIndexs.elem[1] ) &&
			//		( vecHaveSampled[i].elem[2]==currentSamIndexs.elem[2] ) &&
			//		( vecHaveSampled[i].elem[3]==currentSamIndexs.elem[3] ) )
			//	{
			//		isCurrentSamIndexUsed = true;
			//	}
			//}
			//if(isCurrentSamIndexUsed)
			//	continue;
			//vecHaveSampled.push_back( currentSamIndexs );
			//// 判断该采样组是否已经使用过结束...

			// 采样
			for(int i=0; i<MIN_SAMPLING_NUM; i++)
			{
				samPoint1[i] = vecMatchedPoints1[aSampleIndex[i]];
				samPoint2[i] = vecMatchedPoints2[aSampleIndex[i]];
			}

			float aProjectMat2[9] = {0};
			if(m_transformType==PROJECT_MODEL)
			{
				SolveProjectMatrix2( samPoint1, samPoint2, MIN_SAMPLING_NUM, aProjectMat2 ); // 解透视变换模型
				if(aProjectMat2[8]>5)
				{
					continue;
				}
				else if( (aProjectMat2[8]<5) && (aProjectMat2[8]>0.01f) )
				{
					int i = 0;
					float fineH[9];
					// 用非线性最小二乘重算
					NonlinearLeastSquareProjection( samPoint1, samPoint2, MIN_SAMPLING_NUM,
						fineH, aProjectMat2, 1e-10f );
					memcpy(aProjectMat2, fineH, sizeof(fineH) );
				}
			}
			else if(m_transformType==AFFINE_MODEL)
			{
				SolveAffineMotion9( samPoint1, samPoint2, MIN_SAMPLING_NUM, aProjectMat2 ); // 解仿射变换模型
			}
			else if(m_transformType==EUCLIDEAN_MODEL)
			{
				//NonlinearLeastSquare3D_32F()
			}

			memcpy( pProjectMat[t].m, aProjectMat2, sizeof(aProjectMat2) );

			// 计算这个假设的支持度
			int currentSupport = 0;
			for(int i=0; i<ptNum; i++)
			{
				SfPoint point1Bar;
				ApplyProjectMat2( vecMatchedPoints2[i].x, vecMatchedPoints2[i].y, point1Bar.x, point1Bar.y, pProjectMat[t].m );

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

				memcpy( samPoint1Best, samPoint1, sizeof(samPoint1Best) );
				memcpy( samPoint2Best, samPoint2, sizeof(samPoint2Best) );

				if(maxSupport*fInvPtNum>0.99f)
				{
					break;
				}
			}

			t++; // 采样数
		}

		int currentSupport = 0;
		for(int i=0; i<ptNum; i++)
		{
			PointType point1Bar;
			ApplyProjectMat3( vecMatchedPoints2[i], point1Bar, pProjectMat[maxSupportIndex].m );

			float fDistSquare = 0;
			DistanceSquareOfTwoPoints( point1Bar.x, point1Bar.y, 
				vecMatchedPoints1[i].x, vecMatchedPoints1[i].y,
				fDistSquare);
			if(fDistSquare<fRansacDistSquare)
			{
				SfPoint temp1, temp2;
				temp1.x = vecMatchedPoints1[i].x;
				temp1.y = vecMatchedPoints1[i].y;
				temp1.id = vecMatchedPoints1[i].id;
				temp2.x = vecMatchedPoints2[i].x;
				temp2.y = vecMatchedPoints2[i].y;
				temp2.id = vecMatchedPoints2[i].id;
				m_vecInnerPoints1.push_back( temp1 ); // 得到内点
				m_vecInnerPoints2.push_back( temp2 );
			}
		}

#ifdef _DEBUG
		//ShowFeaturePoints( pSrc1, m_vecInnerPoints1, "inner1", "c:\\inner1.bmp" );
		//ShowFeaturePoints( pSrc2, m_vecInnerPoints2, "inner2", "c:\\inner2.bmp" );
#endif

		if(!m_vecInnerPoints1.empty())
		{
			//cout<<"InnerPointsNum "<<m_vecInnerPoints1.size()<<endl;

			if(m_transformType==PROJECT_MODEL)
			{
				// 用内点重新计算投影矩阵
				if( !SolveProjectMatrix2( &m_vecInnerPoints1[0], &m_vecInnerPoints2[0], (int)m_vecInnerPoints1.size(), aProjectMat) )
				{
					isSuccess = false;
				}
			}
			if(m_transformType==AFFINE_MODEL)
			{
				// 用内点重新计算仿射矩阵
				if( !SolveAffineMotion9( &m_vecInnerPoints1[0], &m_vecInnerPoints2[0], (int)m_vecInnerPoints1.size(), aProjectMat) )
				{
					isSuccess = false;
				}
			}
		}
		else
		{
			isSuccess = false;
		}

		if(isSuccess)
		{
			ProjectMat motion, motion0;
			if(m_transformType==PROJECT_MODEL)
			{
				memcpy( motion0.m, pProjectMat[maxSupportIndex].m, sizeof(float)*9 );
				// 非线性最小二乘
				NonlinearLeastSquareProjection( &m_vecInnerPoints1[0], &m_vecInnerPoints2[0], (int)m_vecInnerPoints1.size(),
					motion.m, motion0.m, 1e-10f );
				memcpy( aProjectMat, motion.m, sizeof(float)*9 );
			}

#ifdef _DEBUG
			//ShowMatch( pSrc1, pSrc2, &m_vecInnerPoints1[0], &m_vecInnerPoints2[0], (int)m_vecInnerPoints1.size(),
				//_T("c:\\match-ransac.bmp") );
			//ShowMatchPairs( pSrc1, &m_vecInnerPoints1[0], &m_vecInnerPoints2[0], (int)m_vecInnerPoints1.size(), aProjectMat, "c:\\match-pairs.bmp" );
#endif

			int resultSupport = 0;
//#if 0
//			for(int i=0; i<(int)m_vecInnerPoints2.size(); i++)
//			{
//				SfPoint point1Bar;
//				ApplyProjectMat3( m_vecInnerPoints2[i], point1Bar, aProjectMat );
//
//				float fDistSquare = 0;
//				DistanceSquareOfTwoPoints( point1Bar.x, point1Bar.y, 
//					m_vecInnerPoints1[i].x, m_vecInnerPoints1[i].y,
//					fDistSquare);
//				if(fDistSquare<fRansacDistSquare)
//				{
//					resultSupport++;
//				}
//			}
//			cout<<"resultSupport  "<<resultSupport<<endl;
//#endif
		}

//#if 0
//		for(int i=0; i<9; i++)
//		{
//			aProjectMat[i] = pProjectMat[maxSupportIndex].m[i];
//		}
//#endif

		delete[] pProjectMat; pProjectMat = NULL;

		if(m_vecInnerPoints1.empty())
		{
			return false;
		}

		if( (int)m_vecInnerPoints1.size()<4 )
		{
			return false;
		}

		return true;
	}

	// 函数功能：提取harris角点特征
	// max_corners_num
	bool HarrisFeature( BitmapImage *pSrcImage, 
		SfPoint* pCorners, int &cornerNum, 
		const int max_corners_num );

	//函数功能：计算相关度
	static float CalCorelaton2(BitmapImage *im1, BitmapImage *im2,
		int x1, int y1, int x2, int y2,
		int winRadius);

	//// 显示特征点
	//static bool ShowFeaturePoints(BitmapImage* pGrayImage, const std::vector<CvPoint2D32f> &vecPoints, 
	//	char* pName,
	//	char* pSavePath = NULL);


	static bool ShowFeaturePoints(BitmapImage* pGrayImage, SfPoint* pPoints, int ptNum, 
					TCHAR* pName);

	static bool ShowFeaturePoints(BitmapImage* pGrayImage, CornerDescriptor* pPoints, int ptNum, 
		TCHAR* pName);

	// 选择最好的特征（还没有发现这个方法很管用）
	bool SelectBestMatchPairs_ProjectiveInvariant(SfPoint* pMatchPoints1, SfPoint* pMatchPoints2, int pairsNum,
							SfPoint best5Pairs1[5], SfPoint best5Pairs2[5]);

	// 评价匹配的质量
	bool EvaluateMatchQuality( BitmapImage* pSrc1, BitmapImage* pSrc2, 
								SfPoint* pCorner1, int cornerNum1, SfPoint* pCorner2, int cornerNum2,
								int winR,
								float projectionMat[9], float &fScore );

	// 试图寻找更好的投影变换参数
	bool FindBetterParameter(BitmapImage* pSrc1, BitmapImage* pSrc2, 
			SfPoint* pCorner1, int cornerNum1, SfPoint* pCorner2, int cornerNum2, 
			float projectMatSrc[9], float projectMatDst[9],
			int winR, int sampleTimes);

	// 用基本的Harris+NCC匹配两幅图像
	int MosaicTwoImages(BitmapImage* pSrc1, BitmapImage* pSrc2, float aProjectMat[9],
			int maxFeatruesNum, int winR, 
			float minScore,
			float ransacDist/**/,
			int searchR/**/,
			CornerDescriptor* pCorner1, CornerDescriptor* pCorner2, int nCorner1, int nCorner2);

	// Harris拼接
	bool MosaicHarrisDescriptor(BitmapImage* pSrc1, BitmapImage* pSrc2, float aProjectMat[9],
			//SfPoint* pCorner1, SfPoint* pCorner2, 
			CornerDescriptor* pDescriptor1 /*= NULL*/, CornerDescriptor *pDescriptor2 /*= NULL*/,
			int nCorner1, int nCorner2);

	//// 显示特征点
	//static bool ShowFeaturePoints(IplImage* pGrayImage, CvPoint2D32f* pPoints, int ptNum, char* pName);

	int m_matchMethod;

	int m_maxFeatureNum;

	int m_winRadius;

	float m_minScore;

	bool m_using1D_Method;

	int m_maxRotate;

	int m_maxScale;

private:
	float m_ransacDist;
public:
	float RansacDist() const { return m_ransacDist; }
	void RansacDist(float val) { m_ransacDist = val; }


public:
	// 分成内点和外点
	std::vector<SfPoint> m_vecInnerPoints1, m_vecInnerPoints2;
	int m_transformType; // 几何变换模型
private:
	SfPoint *m_aCorners1;
	SfPoint *m_aCorners2;

	SfPoint *m_aCorners1_32f;
	SfPoint *m_aCorners2_32f;

	//EllipseParam* m_pEllipseParam1;
	//EllipseParam* m_pEllipseParam2;

public:
	//void SetEllipseParam1(EllipseParam* pEllipseParam1) 
	//{ m_pEllipseParam1 = pEllipseParam1; }
	//void SetEllipseParam2(EllipseParam* pEllipseParam2) 
	//{ m_pEllipseParam2 = pEllipseParam2; }
	
public:
	////// 函数功能：角点匹配
	////// 自由匹配(无约束)
	////// fSimilarityThreshold, 相似度阈值
	////// winRadius, 窗口半径
	//bool MatchFeaturePoints(IplImage* pSrc1, IplImage* pSrc2,
	//	CvPoint2D32f* pCorner1, const int cornerNum1, 
	//	CvPoint2D32f* pCorner2, const int cornerNum2,
	//	std::vector<CvPoint2D32f> &vecMatchedPoints1, 
	//	std::vector<CvPoint2D32f> &vecMatchedPoints2,
	//	const int winRadius,
	//	const float fSimilarityThreshold);

	bool LineCorrespondence(int *pSingleLine, pool::BitmapImage* pTargetLPImage, 
			int minDx, int maxDx,
			float &maxScore, int &maxScorePos);

	// 线匹配 双线匹配
	bool LineCorrespondence( int *pSingleLine11, int *pSingleLine12,
			pool::BitmapImage* pTargetLPImage,
			int minDx, int maxDx,
			float &maxScore, int &maxScorePos );


	// 2011-07-01
	// 线匹配，为了解决大的透视变换
	template<class PointType>
	bool MatchFeaturePointsLogPolarLineCorrespondence(BitmapImage* pSrc1, BitmapImage* pSrc2,
				PointType* pCorner1, const int cornerNum1, 
				PointType* pCorner2, const int cornerNum2,
				std::vector<PointType> &vecMatchedPoints1, 
				std::vector<PointType> &vecMatchedPoints2,
				const int winRadius,
				const float fSimilarityThreshold,
				float* pProjectionMat = NULL,
				int localSearchR = 5 )
	{

		return true;
	}


	// 函数功能：角点匹配,NCC
	// 自由匹配(无约束)
	// fSimilarityThreshold, 相似度阈值
	// winRadius, 窗口半径
	template<class T1>
	bool MatchFeaturePoints(BitmapImage* pSrc1, BitmapImage* pSrc2,
		T1* pCorner1, const int cornerNum1, 
		T1* pCorner2, const int cornerNum2,
		std::vector<T1> &vecMatchedPoints1, 
		std::vector<T1> &vecMatchedPoints2,
		const int winRadius,
		const float fSimilarityThreshold)
	{
		if(pCorner1==NULL)
			return false;
		if(pCorner2==NULL)
			return false;

#ifdef _DEBUG
		ShowFeaturePoints( pSrc1, pCorner1, cornerNum1, _T("c:\\corner1.bmp") );
		ShowFeaturePoints( pSrc2, pCorner2, cornerNum2, _T("c:\\corner2.bmp") );
#endif

		// 获取图像尺寸
		int w1, h1, ws1;
		GetImageSize( pSrc1, w1, h1, ws1 );
		int w2, h2, ws2;
		GetImageSize( pSrc2, w2, h2, ws2 );

		bool* pMatchedFlag1 = new bool[cornerNum1];
		bool* pMatchedFlag2 = new bool[cornerNum2];
		memset( pMatchedFlag1, 0, cornerNum1 );
		memset( pMatchedFlag2, 0, cornerNum2 );

		std::vector<int> vecMatchedPoints11;
		std::vector<int> vecMatchedPoints12;

		// 从点集1向点集2匹配
		for(int i1 = 0; i1<cornerNum1; i1++)
		{
			int x1 = 0, y1 = 0;
			x1 = (int)(pCorner1[i1].x + 0.5f);
			y1 = (int)(pCorner1[i1].y + 0.5f);
			// 边界判断
			if( (( x1-winRadius )<0) || (( x1+winRadius )>w1-1) )
				continue;
			if( (( y1-winRadius )<0) || (( y1+winRadius )>h1-1) )
				continue;

			float fMaxSimilarity = 0;
			int maxIndex2 = 0;

			for(int i2 = 0; i2<cornerNum2; i2++)
			{
				int x2 = 0, y2 = 0;
				x2 = (int)(pCorner2[i2].x + 0.5f);
				y2 = (int)(pCorner2[i2].y + 0.5f);
				// 边界判断
				if( (( x2-winRadius )<0) || (( x2+winRadius )>w2-1) )
					continue;
				if( (( y2-winRadius )<0) || (( y2+winRadius )>h2-1) )
					continue;

				float fSimilarity = 0; 
				fSimilarity = CalCorelaton2( pSrc1, pSrc2, 
					x1, y1,
					x2, y2,
					winRadius);

				if( fSimilarity>fMaxSimilarity )
				{
					fMaxSimilarity = fSimilarity;
					maxIndex2 = i2;
				}
			}
			if(fMaxSimilarity>fSimilarityThreshold)
			{
				//vecMatchedPoints11.push_back( i1 );
				//vecMatchedPoints12.push_back( maxIndex2 );

				vecMatchedPoints1.push_back( pCorner1[i1] );
				vecMatchedPoints2.push_back( pCorner2[maxIndex2] );
			}
		}

		//// 从点集2向点集1匹配
		//std::vector<int> vecMatchedPoints21;
		//std::vector<int> vecMatchedPoints22;
		//for(int i2 = 0; i2<cornerNum2; i2++)
		//{
		//	int x2 = 0, y2 = 0;
		//	x2 = (int)(pCorner2[i2].x + 0.5f);
		//	y2 = (int)(pCorner2[i2].y + 0.5f);
		//	// 边界判断
		//	if( (( x2-winRadius )<0) || (( x2+winRadius )>w2-1) )
		//		continue;
		//	if( (( y2-winRadius )<0) || (( y2+winRadius )>h2-1) )
		//		continue;

		//	float fMaxSimilarity = 0;
		//	int maxIndex1 = 0;

		//	for(int i1 = 0; i1<cornerNum1; i1++)
		//	{
		//		int x1 = 0, y1 = 0;
		//		x1 = (int)(pCorner1[i1].x + 0.5f);
		//		y1 = (int)(pCorner1[i1].y + 0.5f);
		//		// 边界判断
		//		if( (( x1-winRadius )<0) || (( x1+winRadius )>w1-1) )
		//			continue;
		//		if( (( y1-winRadius )<0) || (( y1+winRadius )>h1-1) )
		//			continue;

		//		float fSimilarity = 0; 
		//		fSimilarity = CalCorelaton2( pSrc1, pSrc2, 
		//			x1, y1,
		//			x2, y2,
		//			winRadius); // 没必要做两次匹配

		//		if( fSimilarity>fMaxSimilarity )
		//		{
		//			fMaxSimilarity = fSimilarity;
		//			maxIndex1 = i1;
		//		}
		//	}
		//	if(fMaxSimilarity>fSimilarityThreshold)
		//	{
		//		vecMatchedPoints21.push_back( maxIndex1 );
		//		vecMatchedPoints22.push_back( i2 );
		//	}
		//}

#ifdef _DEBUG
		//std::vector<T1> m11, m12, m21, m22;
		//for(int i=0; i<(int)vecMatchedPoints11.size(); i++)
		//{
		//	m11.push_back( pCorner1[vecMatchedPoints11[i]] );
		//	m12.push_back( pCorner2[vecMatchedPoints12[i]] );
		//}
		//ShowFeaturePoints( pSrc1, m11, "由前向后1" );
		//ShowFeaturePoints( pSrc2, m12, "由前向后2" );

		//for(int i=0; i<(int)vecMatchedPoints21.size(); i++)
		//{
		//	m21.push_back( pCorner1[vecMatchedPoints21[i]] );
		//	m22.push_back( pCorner2[vecMatchedPoints22[i]] );
		//}
		//ShowFeaturePoints( pSrc1, m21, "由后向前1" );
		//ShowFeaturePoints( pSrc2, m22, "由后向前2" );
#endif

		//// 验证同一个点，在前后两次匹配中，是否匹配到了相同的点
		//for(int i=0; i<(int)vecMatchedPoints11.size(); i++)
		//{
		//	int i11 = 0, i12 = 0;
		//	i11 = vecMatchedPoints11[i];
		//	i12 = vecMatchedPoints12[i];

		//	for(int j=0; j<(int)vecMatchedPoints22.size(); j++)
		//	{
		//		int i21 = 0, i22 = 0;
		//		i21 = vecMatchedPoints21[j];
		//		i22 = vecMatchedPoints22[j];
		//		if( i12==i22 )
		//		{
		//			if( i21==i11 )
		//			{
		//				vecMatchedPoints1.push_back( pCorner1[i11] );
		//				vecMatchedPoints2.push_back( pCorner2[i12] );
		//				break;
		//			}
		//		}
		//	}
		//}


		delete[] pMatchedFlag1; pMatchedFlag1 = NULL;
		delete[] pMatchedFlag2; pMatchedFlag2 = NULL;

#ifdef _DEBUG
		//ShowFeaturePoints( pSrc1, vecMatchedPoints1, "match1" );
		//ShowFeaturePoints( pSrc2, vecMatchedPoints2, "match2" );
#endif

		return true;
	}

	// 函数功能：基于Log Polar变换的角点特征匹配
	// 自由匹配(无约束)
	// fSimilarityThreshold, 相似度阈值
	// winRadius, 窗口半径
	template<class T1>
	bool MatchFeaturePointsLogPolar(BitmapImage* pSrc1, BitmapImage* pSrc2,
			T1* pCorner1, const int cornerNum1, 
			T1* pCorner2, const int cornerNum2,
			std::vector<T1> &vecMatchedPoints1, 
			std::vector<T1> &vecMatchedPoints2,
			const int winRadius,
			const float fSimilarityThreshold)
	{
#ifdef _DEBUG
#endif
		ShowFeaturePoints( pSrc1, pCorner1, cornerNum1, _T("c:\\corner1.bmp") );
		ShowFeaturePoints( pSrc2, pCorner2, cornerNum2, _T("c:\\corner2.bmp") );

		CLogPolar logPolar;
		logPolar.CalStep((float)winRadius);
		logPolar.MaxScale(2);

		vector<MatchPoints> vecMatchPoint_Pre;

		vector<BitmapImage*> vecTargetLP;
		for(int i=0; i<cornerNum2; i++)
		{
			vecTargetLP.push_back( NULL );
		}

		for(int n1=0; n1<cornerNum1; n1++)
		{
			// 从图1截取一个圆
			BitmapImage* pModelLP = NULL;
			logPolar.LogPolarTransform2( pSrc1, pModelLP, (float)winRadius, 
				(float)pCorner1[n1].x, (float)pCorner1[n1].y );
			// 
			if(pModelLP==NULL)
			{
				continue;
			}

			logPolar.CalMinMaxDx( pModelLP->width );

			cout<<n1<<endl;

			float globalMaxScore = 0;
			SfPoint pt1, pt2;
			pt1 = pCorner1[n1];
			int maxN2 = 0;
			for(int n2=0; n2<cornerNum2; n2++)
			{
				if(vecTargetLP[n2]==NULL)
				{
					logPolar.LogPolarTransform2( pSrc2, vecTargetLP[n2], (float)winRadius, 
						(float)pCorner2[n2].x, (float)pCorner2[n2].y );
				}

				if(vecTargetLP[n2]==NULL)
				{
					continue;
				}

				logPolar.m_saveFlag = false;
				if( (pCorner1[n1].x==179) && 
					(pCorner1[n1].y==188) && 
					(pCorner2[n2].x==185) && 
					(pCorner2[n2].y==206) )
				{
					//logPolar.m_saveFlag = true;
				}

				int maxX, maxY;
				float maxScore = 0;
				logPolar.Correlation( vecTargetLP[n2], pModelLP, maxX, maxY, maxScore, 
					logPolar.m_minDx, logPolar.m_maxDx);

				if(maxScore>globalMaxScore) // globalMaxScore竟然大于1?
				{
					globalMaxScore = maxScore;
					pt2 = pCorner2[n2];
					maxN2 = n2;
				}
			}
			if(globalMaxScore>fSimilarityThreshold)
			{
				//vecMatchedPoints1.push_back( pt1 );
				//vecMatchedPoints2.push_back( pt2 );
				MatchPoints temp;
				temp.score = globalMaxScore;
				temp.selfNum = n1;
				temp.yourNum = maxN2;
				vecMatchPoint_Pre.push_back(temp);
			}
			ReleaseBitmap8U( pModelLP );
		}

		// 初次匹配个数
		int pre_matchNum = (int)vecMatchPoint_Pre.size();

		int sSet2MatchNum[2000] = {0}; // 点集2中匹配到了的点
		for(int i=0; i<pre_matchNum; i++)
		{
			sSet2MatchNum[vecMatchPoint_Pre[i].yourNum]++;
		}

		int method1 = 0;
		if(method1)
		{
			//// 这个方法不行!
			//for(int i=0; i<cornerNum2; i++)
			//{
			//	if(sSet2MatchNum[i]<=0)
			//	{
			//		continue;
			//	}

			//	int set2Num = i;
			//	float maxScore = 0;
			//	int maxSet1Num = 0;
			//	for(int j=0; j<pre_matchNum; j++)
			//	{
			//		if( vecMatchPoint_Pre[j].yourNum==set2Num )
			//		{
			//			if( vecMatchPoint_Pre[j].score>maxScore)
			//			{
			//				maxSet1Num = vecMatchPoint_Pre[j].selfNum;
			//				maxScore = vecMatchPoint_Pre[j].score;
			//			}
			//		}
			//	}

			//	vecMatchedPoints1.push_back( pCorner1[maxSet1Num] );
			//	vecMatchedPoints2.push_back( pCorner2[set2Num] );
			//}
		}
		else
		{
			if(pre_matchNum>cornerNum1/2)
			{
				sort( vecMatchPoint_Pre.rbegin(), vecMatchPoint_Pre.rend() );
				int pre_matchNumHalf = cornerNum1>>1;
				for(int j=0; j<pre_matchNumHalf; j++)
				{
					vecMatchedPoints1.push_back( pCorner1[vecMatchPoint_Pre[j].selfNum] );
					vecMatchedPoints2.push_back( pCorner2[vecMatchPoint_Pre[j].yourNum] );
				}
			}
			else
			{
				for(int j=0; j<pre_matchNum; j++)
				{
					vecMatchedPoints1.push_back( pCorner1[vecMatchPoint_Pre[j].selfNum] );
					vecMatchedPoints2.push_back( pCorner2[vecMatchPoint_Pre[j].yourNum] );
				}
			}
		}

		for(int i=0; i<cornerNum2; i++)
		{
			ReleaseBitmap8U( vecTargetLP[i] );
		}

		return true;
	}

	// 函数功能：基于Log Polar变换的角点特征匹配(一维投影)
	// 自由匹配(无约束)
	// fSimilarityThreshold, 相似度阈值
	// winRadius, 窗口半径
	template<class T1>
	bool MatchFeaturePointsLogPolarProjection(BitmapImage* pSrc1, BitmapImage* pSrc2,
			T1* pCorner1, const int cornerNum1, 
			T1* pCorner2, const int cornerNum2,
			std::vector<T1> &vecMatchedPoints1, 
			std::vector<T1> &vecMatchedPoints2,
			const int winRadius,
			const float fSimilarityThreshold)
	{
#ifdef _DEBUG
#endif
		ShowFeaturePoints( pSrc1, pCorner1, cornerNum1, _T("c:\\corner1.bmp") );
		ShowFeaturePoints( pSrc2, pCorner2, cornerNum2, _T("c:\\corner2.bmp") );

		CLogPolar logPolar;
		logPolar.CalStep((float)winRadius);
		logPolar.MaxScale(2);

		vector<MatchPoints> vecMatchPoint_Pre;
		ProjectionData1000* pProjection2 = new ProjectionData1000[cornerNum2];
		memset( pProjection2, 0, cornerNum2*sizeof(ProjectionData1000) );

		int bigThanThNum = 0;

		vector<int*> vecSumTableM(cornerNum1), vecSumTableM2(cornerNum1);
		memset(&vecSumTableM[0], 0, cornerNum1*sizeof(int));
		memset(&vecSumTableM2[0], 0, cornerNum1*sizeof(int));
		
		vector<BitmapImage*> vecTargetLP;
		for(int i=0; i<cornerNum2; i++)
		{
			vecTargetLP.push_back( NULL );
		}

		for(int n1=0; n1<cornerNum1; n1++)
		{
			// 从图1截取一个圆 log-polar变换
			BitmapImage* pModelLP = NULL;
			logPolar.LogPolarTransform2( pSrc1, pModelLP, (float)winRadius, 
				(float)pCorner1[n1].x, (float)pCorner1[n1].y );
			// 
			int wLP = 0;
			int projection1[1000] = {0};
			if(pModelLP!=NULL)
			{
				wLP = pModelLP->width;

				// 计算在theta轴上的投影

				// 计算在logR轴上的投影
				
				Projection1D( pModelLP, projection1, "X" );

				logPolar.CalMinMaxDx( pModelLP->width );

				cout<<n1<<endl;				
			}
			float globalMaxScore = 0;
			SfPoint pt1, pt2;
			pt1 = pCorner1[n1];
			int maxN2 = 0;
			for(int n2=0; n2<cornerNum2; n2++)
			{
				BitmapImage *pTargetLP = NULL;
				if(n1==0)
				{
					// log-polar变换
					logPolar.LogPolarTransform2( pSrc2, pTargetLP, (float)winRadius, 
						(float)pCorner2[n2].x, (float)pCorner2[n2].y );

					// 计算在theta轴上的投影
					if(pTargetLP)
					{
						// 计算在logR轴上的投影
						Projection1D( pTargetLP, pProjection2[n2].data, "X" );
					}

					vecTargetLP[n2] = pTargetLP;
				}
				//if(pTargetLP==NULL)
				//{
				//	continue;
				//}
				//ReleaseBitmap8U(pTargetLP);

				logPolar.m_saveFlag = false;
				if( (pCorner1[n1].x==939) && 
					(pCorner1[n1].y==322) && 
					(pCorner2[n2].x==435) && 
					(pCorner2[n2].y==247) )
				{
					//logPolar.m_saveFlag = true;
					n2 = n2;

					//SaveImageATL( pModelLP, _T("c:\\modelLP.bmp") );
					//SaveImageATL( vecTargetLP[n2], _T("c:\\targetLP.bmp") );
				}

				// theta轴一维相关

				// logR轴一维相关
				float maxScore = 0;
				int maxDx = 0;
				logPolar.Correlation1D( pProjection2[n2].data, projection1, wLP, 
					logPolar.m_minDx, logPolar.m_maxDx, maxScore, maxDx );

				int maxX = 0,  maxY = 0;
				//logPolar.Correlation( vecTargetLP[n2], pModelLP, maxX, maxY, maxScore, maxDx, maxDx );
				logPolar.Correlation2DFixedDx( vecTargetLP[n2], pModelLP, maxX, maxY, maxScore, 
					maxDx );

				//logPolar.Correlation2DFixedDx_1D_Projection( vecTargetLP[n2], pModelLP, maxX, maxY, maxScore, 
				//	maxDx );

				if(maxScore>globalMaxScore) // globalMaxScore竟然大于1?
				{
					globalMaxScore = maxScore;
					pt2 = pCorner2[n2];
					maxN2 = n2;

					//SaveImageATL( pModelLP, _T("c:\\modelLP.bmp") );
					//SaveImageATL( vecTargetLP[n2], _T("c:\\targetLP.bmp") );
					n2 = n2;
				}				
				if(maxScore>fSimilarityThreshold)
				{
					bigThanThNum++;
				}
			}
			if(globalMaxScore>fSimilarityThreshold)
			{
				//vecMatchedPoints1.push_back( pt1 );
				//vecMatchedPoints2.push_back( pt2 );
				MatchPoints temp;
				temp.score = globalMaxScore;
				temp.selfNum = n1;
				temp.yourNum = maxN2;
				vecMatchPoint_Pre.push_back(temp);
			}
			ReleaseBitmap8U( pModelLP );
		}

		// 初次匹配个数
		int pre_matchNum = 0;
		if(!vecMatchPoint_Pre.empty())
		{
			pre_matchNum = (int)vecMatchPoint_Pre.size();
		}

		cout<<"pre_matchNum "<<pre_matchNum<<endl;
		cout<<"bigThanThNum "<<bigThanThNum<<endl;

		int sSet2MatchNum[2000] = {0}; // 点集2中匹配到了的点
		for(int i=0; i<pre_matchNum; i++)
		{
			sSet2MatchNum[vecMatchPoint_Pre[i].yourNum]++;
		}

		int method1 = 0;
		if(method1)
		{
			// 这个方法不行!
			for(int i=0; i<cornerNum2; i++)
			{
				if(sSet2MatchNum[i]<=0)
				{
					continue;
				}

				int set2Num = i;
				float maxScore = 0;
				int maxSet1Num = 0;
				for(int j=0; j<pre_matchNum; j++)
				{
					if( vecMatchPoint_Pre[j].yourNum==set2Num )
					{
						if( vecMatchPoint_Pre[j].score>maxScore)
						{
							maxSet1Num = vecMatchPoint_Pre[j].selfNum;
							maxScore = vecMatchPoint_Pre[j].score;
						}
					}
				}

				vecMatchedPoints1.push_back( pCorner1[maxSet1Num] );
				vecMatchedPoints2.push_back( pCorner2[set2Num] );
			}
		}
		else
		{
			if(pre_matchNum>cornerNum1/2)
			{
				sort( vecMatchPoint_Pre.rbegin(), vecMatchPoint_Pre.rend() );
				int pre_matchNumHalf = cornerNum1>>1;
				for(int j=0; j<pre_matchNumHalf; j++)
				{
					vecMatchedPoints1.push_back( pCorner1[vecMatchPoint_Pre[j].selfNum] );
					vecMatchedPoints2.push_back( pCorner2[vecMatchPoint_Pre[j].yourNum] );
				}
			}
			else
			{
				for(int j=0; j<pre_matchNum; j++)
				{
					vecMatchedPoints1.push_back( pCorner1[vecMatchPoint_Pre[j].selfNum] );
					vecMatchedPoints2.push_back( pCorner2[vecMatchPoint_Pre[j].yourNum] );
				}
			}
		}

		// 释放内存
		for(int i=0; i<cornerNum2; i++)
		{
			ReleaseBitmap8U( vecTargetLP[i] );
		}
		
		delete[] pProjection2; pProjection2 = NULL;

		return true;
	};

	// 函数功能：基于Log Polar变换的角点特征匹配(一维投影)
	// 自由匹配(无约束)
	// fSimilarityThreshold, 相似度阈值
	// winRadius, 窗口半径
	template<class T1>
	bool MatchFeaturePointsLogPolarProjectionSumTable(BitmapImage* pSrc1, BitmapImage* pSrc2,
				T1* pCorner1, const int cornerNum1, 
				T1* pCorner2, const int cornerNum2,
				std::vector<T1> &vecMatchedPoints1, 
				std::vector<T1> &vecMatchedPoints2,
				const int winRadius,
				const float fSimilarityThreshold,
				float* pProjectionMat = NULL,
				int localSearchR = 5 )
	{
#ifdef _DEBUG
		ShowFeaturePoints( pSrc1, pCorner1, cornerNum1, _T("c:\\corner1.bmp") );
		ShowFeaturePoints( pSrc2, pCorner2, cornerNum2, _T("c:\\corner2.bmp") );
#endif
		pool::BitmapImage* pSrc1Color = CreateBitmap8U( pSrc1->width, pSrc1->height, 3 );
		GrayToColor( pSrc1, pSrc1Color );
		pool::BitmapImage* pSrc2Color = CreateBitmap8U( pSrc2->width, pSrc2->height, 3 );
		GrayToColor( pSrc2, pSrc2Color );

		CLogPolar logPolar;
		logPolar.CalStep((float)winRadius);
		logPolar.MaxScale((float)m_maxScale);
		logPolar.MaxRotate((float)m_maxRotate);

		vector<MatchPoints> vecMatchPoint_Pre;
		ProjectionData1000* pProjection2 = new ProjectionData1000[cornerNum2];
		memset( pProjection2, 0, cornerNum2*sizeof(ProjectionData1000) );

		vector<int*> vecSubTableT2(cornerNum2);
		memset( &vecSubTableT2[0], 0, cornerNum2*sizeof(int*) );

		int bigThanThNum = 0;

		int* pSumTableWidenM = NULL, *pSumTableWidenM2 = NULL;
		//memset(&vecSumTableM[0], 0, cornerNum1*sizeof(int));
		//memset(&vecSumTableM2[0], 0, cornerNum1*sizeof(int));

		vector<BitmapImage*> vecTargetLP;
		for(int i=0; i<cornerNum2; i++)
		{
			vecTargetLP.push_back( NULL );
		}

		vector<int> vecMaxDx;

		BitmapImage *pModelLPWiden = NULL;
		for(int n1=0; n1<cornerNum1; n1++)
		{
			//cout<<n1<<endl;
			//n1 = 82;
			// 从图1截取一个圆 log-polar变换
			float zoom = 1;
			BitmapImage* pModelLP = NULL;
			if(m_pEllipseParam1==false)
			{
				logPolar.LogPolarTransform2( pSrc1, pModelLP, (float)winRadius, 
					(float)pCorner1[n1].x, (float)pCorner1[n1].y );
			}
			else
			{
				logPolar.LogPolarTransform2( pSrc1, pModelLP, (float)winRadius, 
					(float)pCorner1[n1].x, (float)pCorner1[n1].y,
					&m_pEllipseParam1[n1] );

				Color color(0, 0, 255);
				GrayToColor( pSrc1, pSrc1Color );
				float a = m_pEllipseParam1[n1].a;
				float b = m_pEllipseParam1[n1].b;
				float c = m_pEllipseParam1[n1].c;
				float d = m_pEllipseParam1[n1].d;

				char pCirclePath[300];
				sprintf_s( pCirclePath, "c:\\circle\\%dM_circle.bmp", n1 );
				EllipseToCircle(pSrc1, (float)pCorner1[n1].x, (float)pCorner1[n1].y, 
					a, b, c, d, 
					zoom,
					(float)winRadius, pCirclePath);
				
				CDraw::DrawEllipse( pSrc1Color, (int)pCorner1[n1].x, (int)pCorner1[n1].y,
									a, b, c, d, 
									zoom,
									(float)winRadius, color, 1);

				CDraw::DrawCircle( pSrc1Color, (int)pCorner1[n1].x, (int)pCorner1[n1].y,
									(int)winRadius, color, 1 );

				char pEllipsePath[300];
				sprintf_s( pEllipsePath, "c:\\circle\\%dM_ellipse.bmp", n1 );
				//SaveImageATL( pSrc1Color, pEllipsePath );
				//MessageBox(NULL, _T("ellipse"), _T("caption"), 0);
			}
			char savePath1[300] = {NULL}; 
			TCHAR savePathT1[300];      
			sprintf_s( savePath1, "c:\\lp1\\%d_%d_%d.bmp", n1, (int)pCorner1[n1].x, (int)pCorner1[n1].y);  
			unsigned lenOfWideCharStr = ::MultiByteToWideChar(CP_ACP, 0, savePath1, -1, NULL, 0); //include '\0'    
			::MultiByteToWideChar(CP_ACP, 0, savePath1, -1, savePathT1, lenOfWideCharStr);  
			//SaveImageATL( pModelLP, savePathT1 );

			// 
			int wLP = 0;
			int projection1[1000] = {0};
			if(pModelLP)
			{
				wLP = pModelLP->width;
				// 计算在logR轴上的投影

				Projection1D( pModelLP, projection1, "X" );

				logPolar.CalMinMaxDx( pModelLP->width );
				logPolar.CalMinMaxDth();

				if(pSumTableWidenM==NULL)
				{
					pSumTableWidenM = new int[pModelLP->widthStep*pModelLP->height*2];
					pSumTableWidenM2 = new int[pModelLP->widthStep*pModelLP->height*2];
				}
				int w, h, ws;
				GetImageSize( pModelLP, w, h, ws );

				// 加宽
				if(pModelLPWiden==NULL)
				{
					pModelLPWiden = CreateBitmap8U( w, h*2, 1 );
				}

				memcpy( pModelLPWiden->imageData, pModelLP->imageData, ws*h );
				memcpy( pModelLPWiden->imageData + ws*h, pModelLP->imageData, ws*h );

				CalSumTable( pModelLPWiden, pSumTableWidenM );
				//vecSumTableM.push_back(pSumTableM);
				CalSumTable2( pModelLPWiden, pSumTableWidenM2 );
				//vecSumTableM2.push_back(pSumTableM2);

				//COutData::OutImage2Txt(pModelLP, "c:\\modelLP.txt");
				//COutData::OutData2Txt(pSumTableM, w, h, ws, "c:\\SumTableM.txt");
				//COutData::OutData2Txt(pSumTableM2, w, h, ws, "c:\\SumTableM2.txt");
				//cout<<n1<<endl;
			}
			float globalMaxScore = 0;
			SfPoint pt1, pt2;
			pt1 = pCorner1[n1];
			int maxN2 = 0;

			InterestRegion32F currentPointNeighborhood;
			if(pProjectionMat)
			{
				//float x1_in_image2 = 0, y1_in_image2 = 0;
				//ApplyProjectMat2Inv(pCorner1[n1].x, pCorner1[n1].y, x1_in_image2, y1_in_image2, pProjectionMat);

				//currentPointNeighborhood.xbeg = x1_in_image2 - localSearchR;
				//currentPointNeighborhood.xend = x1_in_image2 + localSearchR;
				//currentPointNeighborhood.ybeg = y1_in_image2 - localSearchR;
				//currentPointNeighborhood.yend = y1_in_image2 + localSearchR;
				currentPointNeighborhood.xbeg = pCorner1[n1].x - localSearchR;
				currentPointNeighborhood.xend = pCorner1[n1].x + localSearchR;
				currentPointNeighborhood.ybeg = pCorner1[n1].y - localSearchR;
				currentPointNeighborhood.yend = pCorner1[n1].y + localSearchR;
			}
			for(int n2=0; n2<cornerNum2; n2++)
			{
				//cout<<n2<<" of "<<cornerNum2<<endl;
				//n2 = 83;
				BitmapImage *pTargetLP = NULL;
				if(n1==0)
				{
					if(m_pEllipseParam2==false)
					{
						// log-polar变换
						logPolar.LogPolarTransform2( pSrc2, pTargetLP, (float)winRadius, 
							(float)pCorner2[n2].x, (float)pCorner2[n2].y );
					}
					else
					{
						// log-polar变换
						logPolar.LogPolarTransform2( pSrc2, pTargetLP, (float)winRadius, 
							(float)pCorner2[n2].x, (float)pCorner2[n2].y,
							&m_pEllipseParam2[n2] );

						Color color(0, 0, 255);
						GrayToColor( pSrc2, pSrc2Color );
						float a = m_pEllipseParam2[n2].a;
						float b = m_pEllipseParam2[n2].b;
						float c = m_pEllipseParam2[n2].c;
						float d = m_pEllipseParam2[n2].d;
						char pCirclePath[300];
						sprintf_s( pCirclePath, "c:\\circle2\\%dM_circle.bmp", n2 );
						EllipseToCircle(pSrc2, (float)pCorner2[n2].x, (float)pCorner2[n2].y, 
							a, b, c, d, 
							zoom,
							(float)winRadius, pCirclePath);

						CDraw::DrawEllipse( pSrc2Color, (int)pCorner2[n2].x, (int)pCorner2[n2].y,
							a, b, c, d, 
							zoom,
							(float)winRadius, color, 1);

						CDraw::DrawCircle( pSrc1Color, (int)pCorner1[n1].x, (int)pCorner1[n1].y,
											(int)winRadius, color, 1 );

						char pEllipsePath[300];
						sprintf_s( pEllipsePath, "c:\\circle2\\%dT_ellipse.bmp", n2 );
						//SaveImageATL( pSrc2Color, pEllipsePath );
						//MessageBox(NULL, _T("ellipse"), _T("caption"), 0);
					}
					char savePath2[300] = {NULL}; 
					TCHAR savePathT2[300];      
					sprintf_s( savePath2, "c:\\lp2\\%d_%d_%d.bmp", n2, (int)pCorner2[n2].x, (int)pCorner2[n2].y);
					unsigned lenOfWideCharStr = ::MultiByteToWideChar(CP_ACP, 0, savePath2, -1, NULL, 0); //include '\0'    
					::MultiByteToWideChar(CP_ACP, 0, savePath2, -1, savePathT2, lenOfWideCharStr); 
					//SaveImageATL( pTargetLP, savePathT2 );

					if(pTargetLP)
					{
						int* pSumTableT2 = new int[pTargetLP->widthStep*pTargetLP->height];
						CalSumTable2( pTargetLP, pSumTableT2 );
						vecSubTableT2[n2] = pSumTableT2;

						// 计算在logR轴上的投影
						Projection1D( pTargetLP, pProjection2[n2].data, "X" );
					}

					vecTargetLP[n2] = pTargetLP;
				}

				if(pProjectionMat)
				{
					float x2_in_image1 = 0, y2_in_image1 = 0;
					ApplyProjectMat2((float)pCorner2[n2].x, (float)pCorner2[n2].y, x2_in_image1, y2_in_image1, 
							pProjectionMat);
					if( (currentPointNeighborhood.xbeg<x2_in_image1) &&
						(x2_in_image1<currentPointNeighborhood.xend) && 
						(currentPointNeighborhood.ybeg<y2_in_image1) &&
						(y2_in_image1<currentPointNeighborhood.yend) ) 
					{
						// go on
					}
					else
					{
						continue;
					}
				}
#ifdef _DEBUG
#endif
				//if( (pCorner2[n2].x==435) && (pCorner2[n2].y==260) )
				//{
				//	if( (pCorner1[n1].x==537) && (pCorner1[n1].y==372) )
				//	{
				//if( (pCorner2[n2].x==189) && (pCorner2[n2].y==115) )
				//{
				//	if( (pCorner1[n1].x==123) && (pCorner1[n1].y==102) )
				//	{
				if( (pCorner1[n1].x==302) && (pCorner1[n1].y==253) )
				{
					if( (pCorner2[n2].x==373) && (pCorner2[n2].y==293) )
					{
				//if( (pCorner1[n1].x==337) && (pCorner1[n1].y==225) )
				//{
				//	if( (pCorner2[n2].x==386) && (pCorner2[n2].y==283) )
				//	{
						//SaveImageATL( vecTargetLP[n2], _T("c:\\targetLP.bmp") );
						//SaveImageATL( pModelLP, _T("c:\\modelLP.bmp") );

						//
						COutData::OutData2Txt(pProjection2[n2].data, wLP, "c:\\targetProjection.txt");
						COutData::OutData2Txt(projection1, wLP, "c:\\modelProjection.txt");
					}
				}

				logPolar.m_saveFlag = false;

				// logR轴一维相关
				float maxScore = 0;
				int maxDx = 0;
				logPolar.Correlation1D( pProjection2[n2].data, projection1, wLP, 
					logPolar.m_minDx, logPolar.m_maxDx, maxScore, maxDx );
				if(maxScore<fSimilarityThreshold)
				{
					continue;
				}
				vecMaxDx.push_back(maxDx);

				int maxX = 0,  maxY = 0;
				//logPolar.Correlation( vecTargetLP[n2], pModelLP, maxX, maxY, maxScore, maxDx, maxDx );

				if(m_using1D_Method)
				{
					float maxScore1 = maxScore;

					// 1D投影法(可用2011-5-22)
					logPolar.Correlation2DFixedDx_1D_Projection( vecTargetLP[n2], pModelLP, 
						maxX, maxY, 
						maxScore, 
						maxDx );

					maxScore = (maxScore + maxScore1) * 0.5f;
				}
				else
				{
					//// 二维相关
					maxScore = globalMaxScore;
					logPolar.Correlation2DFixedDx( vecTargetLP[n2], pModelLP, pModelLPWiden,
						maxX, maxY, maxScore, 
						maxDx,
						pSumTableWidenM, pSumTableWidenM2,
						vecSubTableT2[n2]);
				}

				if(maxScore>globalMaxScore) // globalMaxScore竟然大于1?
				{
					globalMaxScore = maxScore;
					pt2 = pCorner2[n2];
					maxN2 = n2;

					//SaveImageATL( pModelLP, _T("c:\\modelLP.bmp") );
					//SaveImageATL( vecTargetLP[n2], _T("c:\\targetLP.bmp") );
					n2 = n2;
				}				
				if(maxScore>fSimilarityThreshold)
				{
					bigThanThNum++;
				}
			}
			if(globalMaxScore>fSimilarityThreshold)
			{
				//vecMatchedPoints1.push_back( pt1 );
				//vecMatchedPoints2.push_back( pt2 );
				MatchPoints temp;
				temp.score = globalMaxScore;
				temp.selfNum = n1;
				temp.yourNum = maxN2;
				vecMatchPoint_Pre.push_back(temp);

				static int nMatchImage = 0;
				char saveNameM[300], saveNameT[300];
				TCHAR saveNameM_T[300], saveNameT_T[300];
				sprintf_s( saveNameM, "c:\\matchResult\\%d_M.bmp", nMatchImage );
				sprintf_s( saveNameT, "c:\\matchResult\\%d_T.bmp", nMatchImage );
				nMatchImage++;
				charTocharT( saveNameM, saveNameM_T );
				charTocharT( saveNameT, saveNameT_T );
				//SaveImageATL( pModelLP, saveNameM_T );
				//SaveImageATL( vecTargetLP[maxN2], saveNameT_T );
			}
			ReleaseBitmap8U( pModelLP );
		}

		// 初次匹配个数
		int pre_matchNum = 0;
		if(!vecMatchPoint_Pre.empty())
		{
			pre_matchNum = (int)vecMatchPoint_Pre.size();
		}

		cout<<"pre_matchNum "<<pre_matchNum<<endl;
		//cout<<"bigThanThNum "<<bigThanThNum<<endl;

		int sSet2MatchNum[2000] = {0}; // 点集2中匹配到了的点
		for(int i=0; i<pre_matchNum; i++)
		{
			sSet2MatchNum[vecMatchPoint_Pre[i].yourNum]++;
		}

		if(pre_matchNum>cornerNum1/2)
		{
			sort( vecMatchPoint_Pre.rbegin(), vecMatchPoint_Pre.rend() );
			int pre_matchNumHalf = cornerNum1>>1;
			for(int j=0; j<pre_matchNumHalf; j++)
			{
				vecMatchedPoints1.push_back( pCorner1[vecMatchPoint_Pre[j].selfNum] );
				vecMatchedPoints2.push_back( pCorner2[vecMatchPoint_Pre[j].yourNum] );
			}
		}
		else
		{
			for(int j=0; j<pre_matchNum; j++)
			{
				vecMatchedPoints1.push_back( pCorner1[vecMatchPoint_Pre[j].selfNum] );
				vecMatchedPoints2.push_back( pCorner2[vecMatchPoint_Pre[j].yourNum] );
			}
		}

		// 释放内存
		for(int i=0; i<cornerNum2; i++)
		{
			ReleaseBitmap8U( vecTargetLP[i] );

			delete[] vecSubTableT2[i];
		}
		ReleaseBitmap8U( pModelLPWiden );
		delete[] pSumTableWidenM;
		delete[] pSumTableWidenM2;
		delete[] pProjection2; pProjection2 = NULL;
		ReleaseBitmap8U(pSrc1Color);
		ReleaseBitmap8U(pSrc2Color);

		return true;
	}
};

template<class PointType>
int ShowMatchPairs(const BitmapImage *pSrc1, const PointType* pMatch1, const PointType *pMatch2, int matchNum, float h[9],
				   char* pPath = NULL)
{
	BitmapImage* pMatchResultColor = CreateBitmap8U( pSrc1->width, pSrc1->height, 3 );
	GrayToColor( pSrc1, pMatchResultColor );

	Color blue(255, 0, 0), red(0, 0, 255);
	int length1 = 3, length2 = 2;
	for(int n=0; n<matchNum; n++)
	{
		CDraw::DrawCross(pMatchResultColor, int(pMatch1[n].x+0.5f), int(pMatch1[n].y+0.5f), length1, red);

		PointType pt1_bar;
		ApplyProjectMat2(pMatch2[n].x, pMatch2[n].y, pt1_bar.x, pt1_bar.y, h);
		CDraw::DrawCross(pMatchResultColor, int(pt1_bar.x+0.5f), int(pt1_bar.y+0.5f), length2, blue);
	}
	SaveImageATL( pMatchResultColor, pPath );

	ReleaseBitmap8U( pMatchResultColor );

	return 0;
}

template<class PointType>
bool ShowMatch(BitmapImage *pSrc1, BitmapImage *pSrc2, PointType* pMatch1, PointType *pMatch2, int matchNum, TCHAR* pPath = NULL)
//bool ShowMatch(BitmapImage *pSrc1, BitmapImage *pSrc2, SfPoint* pMatch1, SfPoint *pMatch2, int matchNum, TCHAR *pPath)
{
	if(pSrc1==NULL)
		return false;
	if(pSrc2==NULL)
		return false;
	if(pMatch1==NULL)
		return false;
	if(pMatch2==NULL)
		return false;

	int w1, h1, ws1;
	int w2, h2, ws2;
	GetImageSize( pSrc1, w1, h1, ws1 );
	GetImageSize( pSrc2, w2, h2, ws2 );

	BitmapImage* pMatchResult = NULL;
	if(pSrc1->nChannels==1)
	{
		pMatchResult = CreateBitmap8U( w1+w2, Max(h1,h2), 1 );
	}
	else
	{
		pMatchResult = CreateBitmap8U( w1+w2, Max(h1,h2), 3 );
	}

	int wsR = pMatchResult->widthStep;
	ZeroImage( pMatchResult );

	//SaveImageATL(pSrc1, "c:\\showMatch.bmp");

	for(int y=0; y<h1; y++)
	{
		memcpy(pMatchResult->imageData + y*wsR,        pSrc1->imageData + y*ws1, ws1);
	}
	for(int y=0; y<h2; y++)
	{
		memcpy(pMatchResult->imageData + y*wsR + ws1,   pSrc2->imageData + y*ws2, ws2);
	}

	BitmapImage* pMatchResultColor = NULL;
	if(pSrc1->nChannels==1)
	{
		pMatchResultColor = CreateBitmap8U( w1+w2, Max(h1,h2), 3 );
		GrayToColor( pMatchResult, pMatchResultColor );
	}
	else
	{
		pMatchResultColor = CloneBitmap8U(pMatchResult);
	}

	pool::Color color(0, 0, 255);

	for(int i=0; i<matchNum; i++)
	{
		IntPoint ptBeg, ptEnd;
		ptBeg.x = (int)pMatch1[i].x;
		ptBeg.y = (int)pMatch1[i].y;

		ptEnd.x = (int)pMatch2[i].x + w1;
		ptEnd.y = (int)pMatch2[i].y;

		//CDraw::DrawLine( pMatchResultColor, ptBeg, ptEnd, color );
	}

	if(pPath==NULL)
	{
		//SaveImageATL( pMatchResultColor, _T("c:\\matchResult.bmp") );
	}
	else
	{
		//SaveImageATL( pMatchResultColor, pPath );
	}

	ReleaseBitmap8U( pMatchResult );
	ReleaseBitmap8U( pMatchResultColor );

	return true;
}

// 功能：选择点集的外接矩形上的4个点，输出它们的索引号
template<class TypePoint>
int Select4Points(TypePoint *pPoints, int nPoints, int boxPointsIndex[4])
{
	TypePoint LD, RU;
	LD.x = pPoints[0].x; LD.y = pPoints[0].y;
	RU.x = pPoints[0].x; RU.y = pPoints[0].y;
	int nMinX = 0, nMinY = 0, nMaxX = 0, nMaxY = 0;
	for(int i=0; i<nPoints; i++)
	{
		if(pPoints[i].x<LD.x)
		{
			nMinX = i;
			LD.x = pPoints[i].x;
		}
		if(pPoints[i].y<LD.y)
		{
			nMinY = i;
			LD.y = pPoints[i].y;
		}
		if(pPoints[i].x>RU.x)
		{
			nMaxX = i;
			RU.x = pPoints[i].x;
		}
		if(pPoints[i].y>RU.y)
		{
			nMaxY = i;
			RU.y = pPoints[i].y;
		}
	}

	boxPointsIndex[0] = nMinX;
	boxPointsIndex[1] = nMinY;
	boxPointsIndex[2] = nMaxX;
	boxPointsIndex[3] = nMaxY;

	return 0;
}

// 最小二乘精匹配？
template<class PointType, class T1>
int RefineMatchLeastSquare(BitmapImage* pSrc1, BitmapImage* pSrc2,
						   int winR,
						   PointType* pInnerPoints1, PointType* pInnerPoints2, int nPairs, T1 H0[0], T1 refinedH[9])
{
	if(pInnerPoints1==NULL)
		return -1;

	int boxPointIndex[4] = {-1};
	Select4Points(pInnerPoints1, nPairs, boxPointIndex);

	PointType finePoints1[2000], finePoints2[2000];

	//for(int n=0; n<4; n++)
	int nFine = 0;
	for(int n=0; n<nPairs; n++)
	{
		//int indexPt = boxPointIndex[n];
		int indexPt = n;

		IntPoint begPt, endPt;
		begPt.x = (int)(pInnerPoints1[indexPt].x - winR);
		begPt.y = (int)(pInnerPoints1[indexPt].y - winR);
		endPt.x = (int)(pInnerPoints1[indexPt].x + winR);
		endPt.y = (int)(pInnerPoints1[indexPt].y + winR);
		BitmapImage* pModel1 = CutPatchImage(pSrc1, begPt, endPt);
#ifdef _DEBUG
		SaveImageATL(pModel1, "c:\\cut-model.bmp");
#endif
		AffineParam32F param0, fineRes;
		param0.a = 1;
		param0.b = 0;
		param0.c = 0;
		param0.d = 1;
		param0.e = pInnerPoints2[indexPt].x;
		param0.f = pInnerPoints2[indexPt].y;
		param0.p = 1;
		param0.q = 0;
		// 用最小二乘找到更精确的点2的坐标
		if( !LeastSquareMatch(pSrc2, pModel1, param0, fineRes, 20, 1e-6f, 1e-6f, 3) )
		{
			finePoints1[nFine] = pInnerPoints1[indexPt]; // 假定点1是准确的
			finePoints2[nFine].x = fineRes.e;
			finePoints2[nFine].y = fineRes.f;

			nFine++;
		}
		ReleaseBitmap8U(pModel1);
	}

	// 用精确的点重新计算单应矩阵
	SolveProjectMatrix2( finePoints1, finePoints2, nFine, refinedH );

	return 0;
}

// 
template<class PointType>
bool Ransac2D(const std::vector<PointType> &vecMatchedPoints1, 
			  const std::vector<PointType> &vecMatchedPoints2,
			  std::vector<PointType> &vecInnerPoints1, 
			  std::vector<PointType> &vecInnerPoints2,
			  float aProjectMat[9],
			  float fRansacDist = 1, int sampleTimes = 1000)
{
	int transformType = PROJECT_MODEL;

	if(vecMatchedPoints1.empty())
	{
		return false;
	}
	if((int)vecMatchedPoints1.size()<=0)
		return false;

	vecInnerPoints1.clear();
	vecInnerPoints2.clear();

	bool isSuccess = true;

	int MIN_SAMPLING_NUM = 4;
	if(transformType==AFFINE_MODEL)
	{
		MIN_SAMPLING_NUM = 3;
	}

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
	srand( (unsigned)time(0) );

	SfPoint samPoint1Best[4];
	SfPoint samPoint2Best[4];

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

		int aSampleIndex[4];

		PointType samPoint1[4];
		PointType samPoint2[4];

		if(transformType==PROJECT_MODEL)
		{
			do
			{
				for(int i=0; i<MIN_SAMPLING_NUM; i++)
				{
					aSampleIndex[i] = rand()%ptNum;
				}
			}
			while( ( aSampleIndex[0]==aSampleIndex[1] ) || 
				( aSampleIndex[0]==aSampleIndex[2] ) ||
				( aSampleIndex[0]==aSampleIndex[3] ) ||
				( aSampleIndex[1]==aSampleIndex[2] ) ||
				( aSampleIndex[1]==aSampleIndex[3] ) ||
				( aSampleIndex[2]==aSampleIndex[3] ) ); // 要求采样4个不同的点
		}
		else if(transformType==AFFINE_MODEL)
		{
			do
			{
				for(int i=0; i<MIN_SAMPLING_NUM; i++)
				{
					aSampleIndex[i] = rand()%ptNum;
				}
			}
			while( ( aSampleIndex[0]==aSampleIndex[1] ) || 
				( aSampleIndex[0]==aSampleIndex[2] ) ||
				( aSampleIndex[1]==aSampleIndex[2] ) ); // 要求采样3个不同的点
		}

		//// 判断该采样组是否已经使用过
		//SampleIndexs currentSamIndexs;
		//currentSamIndexs.elem.push_back( aSampleIndex[0] );
		//currentSamIndexs.elem.push_back( aSampleIndex[1] );
		//currentSamIndexs.elem.push_back( aSampleIndex[2] );
		//currentSamIndexs.elem.push_back( aSampleIndex[3] );
		//sort(currentSamIndexs.elem.begin(), currentSamIndexs.elem.end()); // 排序

		//bool isCurrentSamIndexUsed = false;
		//for(int i=0; i<(int)vecHaveSampled.size(); i++)
		//{
		//	if( ( vecHaveSampled[i].elem[0]==currentSamIndexs.elem[0] ) &&
		//		( vecHaveSampled[i].elem[1]==currentSamIndexs.elem[1] ) &&
		//		( vecHaveSampled[i].elem[2]==currentSamIndexs.elem[2] ) &&
		//		( vecHaveSampled[i].elem[3]==currentSamIndexs.elem[3] ) )
		//	{
		//		isCurrentSamIndexUsed = true;
		//	}
		//}
		//if(isCurrentSamIndexUsed)
		//	continue;
		//vecHaveSampled.push_back( currentSamIndexs );
		//// 判断该采样组是否已经使用过结束...

		// 采样
		for(int i=0; i<MIN_SAMPLING_NUM; i++)
		{
			samPoint1[i] = vecMatchedPoints1[aSampleIndex[i]];
			samPoint2[i] = vecMatchedPoints2[aSampleIndex[i]];
		}

		float aProjectMat2[9] = {0};
		if(transformType==PROJECT_MODEL)
		{
			SolveHomographyMatrix( samPoint1, samPoint2, MIN_SAMPLING_NUM, aProjectMat2 ); // 解透视变换模型
			if(aProjectMat2[8]>5)
			{
				continue;
			}
			else if( (aProjectMat2[8]<5) && (aProjectMat2[8]>0.01f) )
			{
				int i = 0;
				float fineH[9];
				// 用非线性最小二乘重算
				NonlinearLeastSquareProjection2( samPoint1, samPoint2, MIN_SAMPLING_NUM,
					fineH, aProjectMat2, 1e-10f );
				memcpy(aProjectMat2, fineH, sizeof(fineH) );
			}
		}
		else if(transformType==AFFINE_MODEL)
		{
			SolveAffineMotion9( samPoint1, samPoint2, MIN_SAMPLING_NUM, aProjectMat2 ); // 解仿射变换模型
		}
		else if(transformType==EUCLIDEAN_MODEL)
		{
			//NonlinearLeastSquare3D_32F()
		}

		memcpy( pProjectMat[t].m, aProjectMat2, sizeof(aProjectMat2) );

		// 计算这个假设的支持度
		int currentSupport = 0;
		for(int i=0; i<ptNum; i++)
		{
			SfPoint point1Bar;
			ApplyProjectMat2( vecMatchedPoints2[i].x, vecMatchedPoints2[i].y, point1Bar.x, point1Bar.y, pProjectMat[t].m );

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

			memcpy( samPoint1Best, samPoint1, sizeof(samPoint1Best) );
			memcpy( samPoint2Best, samPoint2, sizeof(samPoint2Best) );

			if(maxSupport*fInvPtNum>0.99f)
			{
				break;
			}
		}

		t++; // 采样数
	}

	int currentSupport = 0;
	for(int i=0; i<ptNum; i++)
	{
		PointType point1Bar;
		ApplyProjectMat3( vecMatchedPoints2[i], point1Bar, pProjectMat[maxSupportIndex].m );

		float fDistSquare = 0;
		DistanceSquareOfTwoPoints( point1Bar.x, point1Bar.y, 
			vecMatchedPoints1[i].x, vecMatchedPoints1[i].y,
			fDistSquare);
		if(fDistSquare<fRansacDistSquare)
		{
			SfPoint temp1, temp2;
			temp1.x = vecMatchedPoints1[i].x;
			temp1.y = vecMatchedPoints1[i].y;
			temp1.id = vecMatchedPoints1[i].id;
			temp2.x = vecMatchedPoints2[i].x;
			temp2.y = vecMatchedPoints2[i].y;
			temp2.id = vecMatchedPoints2[i].id;
			vecInnerPoints1.push_back( temp1 ); // 得到内点
			vecInnerPoints2.push_back( temp2 );
		}
	}

#ifdef _DEBUG
	//ShowFeaturePoints( pSrc1, m_vecInnerPoints1, "inner1", "c:\\inner1.bmp" );
	//ShowFeaturePoints( pSrc2, m_vecInnerPoints2, "inner2", "c:\\inner2.bmp" );
#endif

	if(!vecInnerPoints1.empty())
	{
		//cout<<"InnerPointsNum "<<m_vecInnerPoints1.size()<<endl;

		if(transformType==PROJECT_MODEL)
		{
			// 用内点重新计算投影矩阵
			if( !SolveHomographyMatrix( &vecInnerPoints1[0], &vecInnerPoints2[0], (int)vecInnerPoints1.size(), aProjectMat) )
			{
				isSuccess = false;
			}
		}
		if(transformType==AFFINE_MODEL)
		{
			// 用内点重新计算仿射矩阵
			if( !SolveAffineMotion9( &vecInnerPoints1[0], &vecInnerPoints2[0], (int)vecInnerPoints1.size(), aProjectMat) )
			{
				isSuccess = false;
			}
		}
	}
	else
	{
		isSuccess = false;
	}

	if(isSuccess)
	{
		ProjectMat motion, motion0;
		if(transformType==PROJECT_MODEL)
		{
			memcpy( motion0.m, pProjectMat[maxSupportIndex].m, sizeof(float)*9 );
			// 非线性最小二乘
			NonlinearLeastSquareProjection2( &vecInnerPoints1[0], &vecInnerPoints2[0], (int)vecInnerPoints1.size(),
				motion.m, motion0.m, 1e-10f );
			memcpy( aProjectMat, motion.m, sizeof(float)*9 );
		}

#ifdef _DEBUG
		//ShowMatch( pSrc1, pSrc2, &m_vecInnerPoints1[0], &m_vecInnerPoints2[0], (int)m_vecInnerPoints1.size(),
		//_T("c:\\match-ransac.bmp") );
		//ShowMatchPairs( pSrc1, &m_vecInnerPoints1[0], &m_vecInnerPoints2[0], (int)m_vecInnerPoints1.size(), aProjectMat, "c:\\match-pairs.bmp" );
#endif

		int resultSupport = 0;
		//#if 0
		//			for(int i=0; i<(int)m_vecInnerPoints2.size(); i++)
		//			{
		//				SfPoint point1Bar;
		//				ApplyProjectMat3( m_vecInnerPoints2[i], point1Bar, aProjectMat );
		//
		//				float fDistSquare = 0;
		//				DistanceSquareOfTwoPoints( point1Bar.x, point1Bar.y, 
		//					m_vecInnerPoints1[i].x, m_vecInnerPoints1[i].y,
		//					fDistSquare);
		//				if(fDistSquare<fRansacDistSquare)
		//				{
		//					resultSupport++;
		//				}
		//			}
		//			cout<<"resultSupport  "<<resultSupport<<endl;
		//#endif
	}

	//#if 0
	//		for(int i=0; i<9; i++)
	//		{
	//			aProjectMat[i] = pProjectMat[maxSupportIndex].m[i];
	//		}
	//#endif

	delete[] pProjectMat; pProjectMat = NULL;

	if(vecInnerPoints1.empty())
	{
		return false;
	}

	if( (int)vecInnerPoints1.size()<4 )
	{
		return false;
	}

	return true;
}


// 彩色图转灰度图
// Gray = R*0.299 + G*0.587  + B*0.114;
bool ColorToGray1(BitmapImage *pSrcColor, BitmapImage *pDstGray);

// 功能：计算目标函数的值
int CalSumOfDistError(int w1, int h1, int w2, int h2, 
						   float* pH1, float *pH2, float *A,
						   double &error2);

int SelectHomographyExaustive( int w1, int h1, int w2, int h2, 
							  float* pH1, float *pH2, float *A);

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
						const float fSimilarityThreshold, int localSearchR = 15, float *pH = NULL);

// 功能：利用计算出的各幅图像相对于第一幅图像的变换关系，对图像进行重采样，拼成一张大图
// 说明：以第一张图为基准图
// pImgT为每张图像相对于第1张图像的变换关系
BitmapImage* MergeMultiImages(const BitmapImage* pImages[], int imagesNum, ProjectMat* pImgT, int blending, int referenceType);

// 功能：利用计算出的各幅图像相对于第一幅图像的变换关系，对图像进行重采样，拼成一张大图
// 说明：以第一张图为基准图
// pImgT为每张图像相对于第1张图像的变换关系
IplImage* MergeMultiImages2(IplImage** pImages, int imagesNum, ProjectMat* pImgT, int blending, int referenceType);

// 功能：拉普拉斯金字塔图像融合
// pImgT为每张图像的变换参数
IplImage* LaplacianPyramidBlending(IplImage** pImages, int imagesNum, ProjectMat* pImgT, int band = 5, float resScale = 1);

struct PosWeight 
{
	int x, y;
	float z;
};
// 功能: 补全图像2的灰度变化比例
// pSrc指向图像2的灰度变化比例, 没有填充的单位格的比例值为0
// pSrc和pDst具有相同的尺寸
int FillWeight(float* pSrcW, float* pDstW, int w, int h, int ws);

int FillWeightSparse(float* pSrcW, float* pDstW, int w, int h, int ws);

// 对灰度变化比例进行平滑
int SmoothWeight( float *pSrcW32F, float *pSmoothed32F, int w, int h, int ws,
				 BitmapImage* pWeightImg, const int NON_WEIGHT_PIXEL );

// 对灰度变化比例进行平滑, 用中值滤波
int SmoothWeight_MidVal( float *pSrcW32F, float *pSmoothed32F, int w, int h, int ws,
						BitmapImage* pWeightImg, const int NON_WEIGHT_PIXEL );

// 金字塔混合图像
int PyramidBlending(BitmapImage** pImages, int imagesNum, ProjectMat* pImgT, int blending);

// 给定单应矩阵, 对图像进行变换
int ImageProjectionTransform(BitmapImage* pImage, BitmapImage* &pResult, float h[9]);

int WriteMatchPairs(const vector<MatchPointPairs> &vecMatchPairs, char* pName);

int WriteMatchPairs_ASC2(const vector<MatchPointPairs> &vecMatchPairs, char* pName);

int LoadMatchPairs(vector<MatchPointPairs> &vecMatchPairs, char* pName);

// 计算单应矩阵
Mat CalHomoH_CV(vector<SfPoint> vecMatch1, vector<SfPoint> vecMatch2, 
				float ransacDist,
				vector<SfPoint> &inner1, vector<SfPoint> &inner2);

// 功能：拼接多幅图片
// 原理：Harris角点 + NCC + RANSAC + 单应矩阵
// 返回值：拼接的结果大图指针;  如果为NULL，表示拼接失败; 
// BitmapImage* pImages[], -----输入的图像指针数组
// int imagesNum, --------------待拼接的图像帧数
// int errorCode---------------------错误码, 输出, 0表示输入参数没有异常, -1表示输入的参数错误
// int maxFeatruesNum, ---------最大特征数(默认给400)
// int winR, -------------------NCC窗口大小(默认给15)
// float minScore,--------------NCC最小匹配得分(默认给0.7)
// float ransacDist/**/,--------RANSAC阈值(默认给2 pixel)
// int searchR = -1/**/---------限定特征匹配的搜索范围（如果不确定，给-1）
extern "C" __declspec(dllexport)
pool::BitmapImage* MosaicMultiImages(BitmapImage* pImages[], int imagesNum, 
									 int &errorCode,
									 int maxFeatruesNum, int winR, 
									 float minScore,
									 float ransacDist/**/,
									 int searchR /**/);

// 功能：拼接多幅图片
// 原理：Harris角点 + NCC + RANSAC + 单应矩阵
// 返回值：拼接的结果大图指针;  如果为NULL，表示拼接失败; 
// BitmapImage* pImages[], -----输入的图像指针数组
// int imagesNum, --------------待拼接的图像帧数
// int errorCode----------------错误码, 输出, 0表示输入参数没有异常, -1表示输入的参数错误
// int maxFeatruesNum, ---------最大特征数(默认给400)
// int winR, -------------------NCC窗口大小(默认给14)
// float minScore,--------------NCC最小匹配得分(默认给0.7)
// float ransacDist/**/,--------RANSAC阈值(默认给2 pixel)
// int searchR = -1/**/---------限定特征匹配的搜索范围（如果不确定，给-1）
// int referenceType ---------- 0以第一张图像为基准, 1自动计算, 2以中心为基准
// int blending		----------- 0不融合, 1线性加权融合, 2多波段融合
extern "C" __declspec(dllexport)
IplImage* MosaicMultiImages3(IplImage** pImages, int imagesNum, 
							 int &errorCode,
							 int maxFeatruesNum, int winR, 
							 float minScore,
							 float ransacDist/**/,
							 int searchR/**/,
							 int referenceType = 1, 
							 int blending = 2);



