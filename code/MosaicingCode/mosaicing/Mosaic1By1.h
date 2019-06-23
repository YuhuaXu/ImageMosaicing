#pragma once

#include "stdafx.h"
#include "opencv2\opencv.hpp"
#include "Bitmap.h"

#if defined (_WINDLL) || defined(_CONSOLE) || defined(_WINDOWS)
	#ifndef MV_MODULE_TYPE2
		#define MV_MODULE_TYPE2 _declspec(dllexport)
	#else
		#define MV_MODULE_TYPE2 _declspec(dllimport)
	#endif
#else
	#define MV_MODULE_TYPE2 
#endif

struct MosaicImageInfo
{
	IplImage* pSrc;
	ProjectMat T;
	int success;
	int emergency;
	MosaicImageInfo()
	{
		pSrc = NULL;
		success = 0;
		emergency = 0;
	}
};


// 图像拼接参数
struct MosaicParams
{
	// 当送进来的图像数等于mergeImagesNum时, 拼成一幅大图
	int mergeImagesNum; 

	// 出现异常情况时, 把前后邻近的imagesNumEmergency张图片拼接一幅图像
	int imagesNumEmergency; 

	// 最大特征点个数
	int maxFeatruesNum;

	// 匹配窗口的半径
	int winR; 

	// 限定搜索半径
	int searchR;

	// 最小匹配得分
	float minScore;

	// ransac距离阈值
	float ransacDist;

	// 图像融合方法,0不做处理, 1线性加权融合, 2拉普拉斯金字塔融合
	int blendingType;

	// 参考基准, 0以第一张图像为基准, 1程序自动选择基准
	int referenceType;

	int wResDynamic;

	int hResDynamic;

	MosaicParams()
	{
		this->mergeImagesNum = 10; // 
		this->imagesNumEmergency = 15;//
		this->maxFeatruesNum = 400;
		this->winR = 13;
		this->searchR = -1;
		this->minScore = 0.7f;
		this->ransacDist = 2.0f;
		this->blendingType = 1;
		this->referenceType = 1;
		this->wResDynamic = 768;
		this->hResDynamic = 1024;
	}
};

class MV_MODULE_TYPE2 CMosaic
{
public:
	// 设置拼接参数
	void SetParams(MosaicParams param);

	// 清空动态拼接池中的内容
	void ClearPool();

	//******************************************************************************
	// 功能：图像拼接, 实时动画效果, 每来一张图像都更新拼接图
	// 返回值含义: 0拼接成功, -1输入参数错误, -2拼接失败
	// pSrc为待拼接的图像
	//******************************************************************************
	int MosaicDynamic(const IplImage* pSrc);

	bool UpdateResult() const { return m_bUpdateResult; }
	void UpdateResult(bool val) { m_bUpdateResult = val; }

	// New
	IplImage* GetDynamicMosaicResult(bool bExternal); // 外部调用时, bExternal置true
public:
	//******************************************************************************
	// 功能: 图像拼接, 来一张拼一张, 非动画效果, 隔一段时间拼出一张大图
	// 返回值含义: 0拼接成功, -1输入参数错误, -2拼接失败
	// pSrc为待拼接的图像
	// pMosaicResult为拼接后的图像, 不允许在外部释放!
	// emergency为true时, 立刻拼接一幅图像
	//******************************************************************************
	int Mosaic(const IplImage* pSrc, IplImage* &pMosaicResult, const bool emergency);

	IplImage* GetMosaicResult(bool bExternal); // 外部调用时, bExternal置true
	
	//******************************************************************************
	// 功能: 异步方式的图像拼接
	// 返回值含义: 0拼接成功, -1输入参数错误, -2拼接失败
	// pSrc为待拼接的图像
	// pMosaicResult为拼接后的图像, 不允许在外部释放!
	// emergency为true时, 立刻拼接一幅图像
	//******************************************************************************
	int MosaicAsynchronous(const IplImage* pSrc);//, bool bNewTask);

	// 设置结束拼接线程标志
	void EndDynamicMosaic(bool val) { m_bEndDynamicMosaic = val; }
	// 查询结束拼接线程标志
	bool EndDynamicMosaic() const { return m_bEndDynamicMosaic; }

	CMosaic();

	~CMosaic();
private:
	// 功能：生成动态拼接图
	int GenerateDynamicPana(const IplImage* pImages[], int imagesNum, ProjectMat* pImgT);

	// 销毁资源
	void Destroy();

private:
	// 拼接参数
	MosaicParams m_params;

	bool m_bEndDynamicMosaic;
private:
	int m_nCorner1, m_nCorner2;
	IplImage* m_pGray1, *m_pGray2;

private:
	// 拼接的结果
	IplImage* m_pMosaicResult;

private:
	// 拼接图像更新标志, 为true时, 可以读取m_pMosaicResult
	bool m_bUpdateResult;

private:
	// 相对于上一张图像的单应变换矩阵
	float m_homoToPrev[9];

	pool::IntPoint leftUpPt;

	float* HomoToPrev() { return m_homoToPrev; }
};

void CvImage2BitmapImage(IplImage* pSrc, pool::BitmapImage* pDst);

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
IplImage* MosaicMultiImages2(IplImage* pImages[], int imagesNum, 
									 int &errorCode,
									 int maxFeatruesNum, int winR, 
									 float minScore,
									 float ransacDist/**/,
									 int searchR /**/);