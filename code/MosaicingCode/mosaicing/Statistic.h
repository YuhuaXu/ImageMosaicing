#pragma once

//#include "ati.h"
#include "Bitmap.h"

// 计算图像的信息熵
//bool CalInformationEntropy(IplImage* pSrcImage, float &fInfoEntropy);

// 计算图像的信息熵
bool CalInformationEntropy(pool::BitmapImage* pSrcImage, float &fInfoEntropy);

// 函数功能：求图像直方图
int CalHistogram(pool::BitmapImage *im, float hist[256]);

//// 计算栅格中的方差
//bool CalStdInGrid(IplImage* pSrc, 
//				 const int x, const int y, int gridSize, 
//				 float &fStdR, float &fStdG, float &fStdB);

// 由直方图求 均值 和 方差
int GetMeanDeviation2(float *Hist, float &mean, float &deviation);

// 由直方图求 均值 和 方差
int GetMeanDeviation(float *Hist, int maxLevel, float &mean, float &deviation);

// 由直方图求 均值 和 方差
int GetMeanDeviation(float *Hist, float &mean, float &deviation);

// 由直方图求 均值 和 方差
int GetMeanVariance(float *Hist, int maxLevel, float &mean, float &deviation);

// 求均值 和 标准差
template<class T, class T2>
int GetMeanAndSTD(std::vector<T>vecData, T2 &mean,T2 &stdValue)
{
	// std---标准差
	int ndata = (int)vecData.size();

	float N=0, M, D, acc;

	for(int u=0; u<ndata; u++)
	{
		N+=vecData[u]; // 总和
	}

	M = N/ndata; // 均值

	acc=0;
	for(int u=0;u<ndata;u++)
	{
		acc+=(vecData[u]-M)*(vecData[u]-M); // 求方差
	}
	D=acc/ndata; // 方差

	mean=M;
	stdValue = sqrt(D); // 求标准差
	return 1;
}

// 求均值 和 标准差
template<class T1, class T2>
int GetMeanAndSTD(const T1 *pdata, const int ndata,T2 &mean, T2 &fStd)
{
	// std---标准差

	T2 N=0, M = 0, D = 0;
	T2 acc = 0;

	for(int u=0; u<ndata; u++)
	{
		N += (T2)pdata[u]; // 总和
	}

	M = N/ndata; // 均值

	acc = 0;
	for(int u=0;u<ndata;u++)
	{
		acc += (pdata[u]-M)*(pdata[u]-M); // 求方差
	}
	D = acc/ndata; // 方差

	mean = M;
	fStd = sqrt(D); // 求标准差
	return 1;
}
 
// 求均值 和 标准差  (带加权)
// sum(pWeight)应该等于1
template<class T1, class T2>
int GetMeanAndSTD_Weight(const T1 *pdata, const int ndata, T2 &mean, T2 &fStd, T2 *pWeight0)
{
	if(pdata==NULL)
		return -1;

	T2* pWeight = new T2[ndata];
	memcpy( pWeight, pWeight0, ndata * sizeof(T2) );

	// 
	T2 sumOfWeight = 0;
	Accumulate( pWeight, ndata, sumOfWeight );

	// 求均值
	mean = 0;
	for(int u=0; u<ndata; u++)
	{
		pWeight[u] /= sumOfWeight; // 权重归一化
		mean += pWeight[u] * pdata[u];
	}

	T2 acc = 0;
	for(int u=0; u<ndata; u++)
	{
		acc += (pdata[u]-mean) * (pdata[u]-mean) * pWeight[u];
	}

	fStd = sqrt(acc); // 标准差

	delete pWeight; pWeight = NULL;

	return 1;
}

// 求均值 和 标准差
template<class T1>
int GetMeanAndSTD2(const T1 *pdata, const int ndata,float &mean, float &fStd)
{
	// std---标准差

	float N=0, M = 0, D = 0;
	float acc = 0;

	for(int u=0; u<ndata; u++)
	{
		N += pdata[u]; // 总和
	}

	M = N/ndata; // 均值

	acc = 0;
	for(int u=0;u<ndata;u++)
	{
		acc += (pdata[u]-M)*(pdata[u]-M); // 求方差
	}
	D = acc/ndata; // 方差

	mean = M;
	fStd = sqrt(D); // 求标准差
	return 1;
}

template<class T1>
bool Nomalize(T1* pWeight, int nOverlapData)
{
	if(pWeight==NULL)
		return false;

	T1 sumOfWeight = 0;
	Accumulate( pWeight, nOverlapData, sumOfWeight );

	if(sumOfWeight<=0)
		return false;

	for(int i=0; i<nOverlapData; i++)
	{
		pWeight[i] /= sumOfWeight;
	}

	return true;
}

// 生成高斯噪声,均值为0，方差为1
float GenerateGaussNoise();