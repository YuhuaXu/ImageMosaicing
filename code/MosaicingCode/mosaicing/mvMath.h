
#pragma once

#include "Point.h"
#include <vector>
#include "LeastSquare.h"

namespace pool
{
	#define Max(a,b)            (((a) > (b)) ? (a) : (b))
	#define Min(a,b)            (((a) < (b)) ? (a) : (b))
}

using namespace pool;

//求一个点在四象限中的角度（弧度），0 ~ 2*pi
template<class T1, class T2>
int AngleofPoint360(T1 x, T1 y, T2 &angleArc)
{
	if(x>=0)// 1，4象限
	{
		if(y>=0) // 1  x>0 y>0
		{
			if(x!=0)
			{
				angleArc = (T2)atan(y/x);
			}
			else//x==0
			{
				angleArc = pool::pi/2;
			}
		}
		else // 4  x>0  y<0
		{
			if(x!=0)
			{
				angleArc = 2*pool::pi + (T2)atan(y/x);
			}
			else//x==0
			{
				angleArc = 3*pool::pi/2;
			}
		}
	}
	else// 2，3象限
	{
		if(y>=0) // 2
		{
			if(x!=0)  // x<0 y>0
			{
				angleArc = pool::pi + (T2)atan(y/x);
			}
		}
		else //3  x<0  y<0
		{
			if(x!=0)
			{
				angleArc = pool::pi + (T2)atan(y/x); 
			}
		}
	}
	return 1;
}


// 累加
template<class T1, class T2>
bool Accumulate(T1 *pSrcData, int dataSize, T2 &accResult)
{
	if(!pSrcData)
		return false;

	accResult = 0;
	for(int i=0; i<dataSize; i++)
	{
		accResult += pSrcData[i];
	}

	return true;
}


// 计算2*2矩阵的特征值
template<class T1>
bool CalEigenvalue(T1* pData,
				   float &lamata1, float &lamata2)
{
	if(pData==NULL)
		return false;

	T1 m00, m01, m10, m11;
	m00 = pData[0];
	m01 = pData[1];
	m10 = pData[2];
	m11 = pData[3];

	float b = -m11 - m00;
	float c = m00*m11 - m01*m10;
	const float a = 1;
	float delta = b*b - 4*a*c;
	if(delta<0)
		return false;

	float sqrtDealta = sqrt(delta);

	lamata1 = (-b + sqrtDealta)*0.5f;
	lamata2 = (-b - sqrtDealta)*0.5f;

	return true;
}

// 计算3*3行列式
// a11-a12-a13
// a21-a22-a23
// a31-a32-a33
template<class T1>
bool CalDet33(T1* pSrc, T1 &det)
{
	if(pSrc==NULL)
		return false;
	T1  a11, a12, a13, 
		a21, a22, a23, 
		a31, a32, a33;
	a11 = pSrc[0];
	a12 = pSrc[1];
	a13 = pSrc[2];

	a21 = pSrc[3];
	a22 = pSrc[4];
	a23 = pSrc[5];

	a31 = pSrc[6];
	a32 = pSrc[7];
	a33 = pSrc[8];

	det = a11*a22*a33 + a12*a23*a31 + a13*a21*a32 - a11*a23*a32 - a12*a21*a33 - a13*a22*a31;

	return true;
}


// pSrc1 - pSrc2 = pDst
template <class T1>
int Sub(T1* pSrc1, T1* pSrc2, T1* pDst,int dataNum)
{
	if( ( pSrc1==NULL ) || ( pSrc2==NULL ) || ( pDst==NULL ) )
		return 0;
	for( int i=0; i<dataNum; i++ )
	{
		pDst[i] = pSrc1[i] - pSrc2[i];
	}
	return 1;
}


// pSrc1 + pSrc2 = pDst
template <class T1, class T2, class T3>
int Add(T1* pSrc1, T2* pSrc2, T3* pDst, int dataNum)
{
	if( ( pSrc1==NULL ) || ( pSrc2==NULL ) || ( pDst==NULL ) )
		return 0;

	for( int i=0; i<dataNum; i++ )
	{
		pDst[i] = (T3)(pSrc1[i] + pSrc2[i]);
	}
	return 1;
}


// pSrc1 + pSrc2 = pDst
template <class T1, class T2, class T3>
int Add(T1* pSrc1, T2 addedValue, T3* pDst, int dataNum)
{
	if( ( pSrc1==NULL ) || ( pDst==NULL ) )
		return 0;

	for( int i=0; i<dataNum; i++ )
	{
		pDst[i] = (T3)(pSrc1[i] + addedValue);
	}

	return 1;
}

template<class T1, class T2, class T3>
inline void DistanceOfTwoPoints(T1 x1, T1 y1, 
								T2 x2, T2 y2, 
								T3 &dist)
{
	dist = sqrt( (T3)(x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

template<class T1, class T2, class T3>
inline void DistanceOfTwoPoints3(T1 x1, T1 y1, T1 z1,
	T2 x2, T2 y2, T2 z2,
	T3 &dist)
{
	dist = sqrt( (T3)(x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2) );
}

template<class PointType3D, class T2>
inline void DistanceOfTwo3DPoints(PointType3D p1, PointType3D p2,
								 T2 &dist)
{
	dist = sqrt( (T2)(p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) +(p1.z-p2.z)*(p1.z-p2.z) );
}

template<class T1>
inline void DistanceSquareOfTwoPoints(T1 x1, T1 y1, T1 x2, T1 y2, T1 &distSquare)
{
	distSquare = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
}

// pSrc1 * pSrc2 = pDst
template <class T1>
int Multiply(T1* pSrc1, T1* pSrc2, T1* pDst,int dataNum)
{
	if( ( pSrc1==NULL ) || ( pSrc2==NULL ) || ( pDst==NULL ) )
		return 0;
	for( int i=0; i<dataNum; i++ )
	{
		pDst[i] = pSrc1[i] * pSrc2[i];
	}
}


// pSrc1 * pSrc2 = pDst
template <class T1, class T2, class T3>
int Multiply2(T1* pSrc1, T2* pSrc2, T3* pDst,int dataNum)
{
	if( ( pSrc1==NULL ) || ( pSrc2==NULL ) || ( pDst==NULL ) )
		return 0;
	for( int i=0; i<dataNum; i++ )
	{
		pDst[i] = pSrc1[i] * pSrc2[i];
	}
	return 1;
}


// pSrc1 * pSrc2 = pDst
template <class T1, class T2>
int Multiply(T1* pSrc, T1* pDst, T2 k, int dataNum)
{
	if( ( pSrc==NULL ) || ( pDst==NULL ) )
		return 0;
	for( int i=0; i<dataNum; i++ )
	{
		pDst[i] = pSrc[i] * k;
	}
	return 1;
}


// pSrc1 / pSrc2 = pDst
template <class T1>
int Divide(T1* pSrc1, T1* pSrc2, T1* pDst, int dataNum)
{
	if( ( pSrc1==NULL ) || ( pSrc2==NULL ) || ( pDst==NULL ) )
		return 0;
	for( int i=0; i<dataNum; i++ )
	{
		pDst[i] = pSrc1[i] / pSrc2[i];
	}
	return 1;
}


// pSrc1 / pSrc2 = pDst
template <class T1>
int Divide(T1* pSrc1, T1* pSrc2, T1* pDst, int dataNum, const T1 SMALL_NUMBER)
{
	if( ( pSrc1==NULL ) || ( pSrc2==NULL ) || ( pDst==NULL ) )
		return 0;
	for( int i=0; i<dataNum; i++ )
	{
		if( ( abs(pSrc1[i])<SMALL_NUMBER ) || (abs(pSrc2[i])<SMALL_NUMBER) )
		{
			pDst[i] = 0;
		}
		else
		{
			pDst[i] = pSrc1[i] / pSrc2[i];
		}

	}
	return 1;
}


// pSrc1 / pSrc2 = pDst
template <class T1, class T2>
int Divide(T1* pSrc1, T2 denominator, T1* pDst,int dataNum)
{
	if( ( pSrc1==NULL ) || ( pDst==NULL ) )
		return 0;
	if( denominator==0 )
		return 0;

	for( int i=0; i<dataNum; i++ )
	{
		pDst[i] = pSrc1[i] / denominator;
	}

	return 1;
}


template<class T, class T2, class T3>
void FindMaxValue(T2 arr[], int nArr,T &max_value, T3 &maxi )
{
	max_value=arr[0]; 
	maxi=0;
	for(int u=0; u<nArr; u++)
	{
		if(arr[u]>max_value)
		{
			max_value = (T)arr[u];
			maxi = u;
		}
	}

	return;
}

// 获取最大值和最小值
template<class T1>
bool FindMaxValue(T1* data, int w, int h, int stride, 
				 T1 &minVal, T1 &maxVal)
{
	if(data==NULL)
		return false;

	minVal = data[0];
	maxVal = data[0];

	// 求出最大灰度和最小灰度
	for (int y = 0; y < h; y++)
	{
		T1* pDataRow = data + stride*y;

		for (int x = 0; x < w; x++)
		{
			T1 curVal = pDataRow[x];

			// 最大值
			if (curVal > maxVal)
				maxVal = pDataRow[x];

			// 最小值
			if (curVal < minVal)
				minVal = pDataRow[x];
		}
	}

	return true;
}


template<class T, class T2>
void FindSecondMaxValue(T2 arr[], int nArr,T &secondMaxValue, int &secondMaxIndex )
{
	// 先找出最大值
	T max_value=arr[0]; 
	int maxi=0;
	for(int u=0; u<nArr; u++)
	{
		if(arr[u]>max_value)
		{
			max_value = (T)arr[u];
			maxi = u;
		}
	}

	secondMaxValue = arr[0];
	for(int u=0; u<nArr; u++)
	{
		if(arr[u]>secondMaxValue)
		{
			if(arr[u]<max_value)
			{
				secondMaxValue = (T)arr[u];
				secondMaxIndex = u;
			}
		}
	}

	return;
}


template<class T, class T2>
void FindSecondMinValue(T2 arr[], int nArr,T &secondMinValue, unsigned int &secondMinIndex )
{
	// 先找出最大值
	T min_value=arr[0]; 
	int mini=0;
	for(int u=0; u<nArr; u++)
	{
		if(arr[u]<min_value)
		{
			min_value = (T)arr[u];
			mini = u;
		}
	}

	secondMinValue = arr[0];
	for(int u=0; u<nArr; u++)
	{
		if(arr[u]<secondMinValue)
		{
			if(arr[u]>min_value)
			{
				secondMinValue = (T)arr[u];
				secondMinIndex = u;
			}
		}
	}

	return;
}


// 返回数组中的最小值，及最小值的标号
template<class T2, class T3, class T4>
void FindMinValue(T2 arr[], unsigned int nArr, T4 &min_value, T3 &minI )
{
	min_value = arr[0]; 
	minI = 0;
	for(unsigned int u=0; u<nArr; u++)
	{
		//cout<<arr[u]<<endl;
		if(arr[u]<min_value)
		{
			min_value = arr[u];
			minI = u;
		}
	}
	return;
}

template<class T1>
void FindBorder(pool::SfPoint* pPoints, int ptNum, 
				T1 &xMin, T1 &yMin, T1 &xMax, T1 &yMax)
{
	xMax = pPoints[0].x;
	xMin = pPoints[0].x;

	yMax = pPoints[0].y;
	yMin = pPoints[0].y;

	for(int u=0; u<ptNum; u++)
	{
		if (pPoints[u].x<xMin) 
		{
			xMin = (T1)pPoints[u].x;
		}
		if (pPoints[u].x>xMax) 
		{
			xMax = (T1)pPoints[u].x;
		}
		if (pPoints[u].y<yMin) 
		{
			yMin = (T1)pPoints[u].y;
		}
		if (pPoints[u].y>yMax) 
		{
			yMax = (T1)pPoints[u].y; // 找出X、Y的最大，最小值
		}
	}
	
	return;
}


template<class T1>
void FindBorder(std::vector<pool::IntPoint> vecPoints, int ptNum, 
				T1 &xMin, T1 &yMin, T1 &xMax, T1 &yMax)
{
	xMax = vecPoints[0].x;
	xMin = vecPoints[0].x;

	yMax = vecPoints[0].y;
	yMin = vecPoints[0].y;

	for(int u=0; u<ptNum; u++)
	{
		if (vecPoints[u].x<xMin) 
		{
			xMin = (T1)vecPoints[u].x;
		}
		if (vecPoints[u].x>xMax) 
		{
			xMax = (T1)vecPoints[u].x;
		}
		if (vecPoints[u].y<yMin) 
		{
			yMin = (T1)vecPoints[u].y;
		}
		if (vecPoints[u].y>yMax) 
		{
			yMax = (T1)vecPoints[u].y; // 找出X、Y的最大，最小值
		}
	}

	return;
}


template<class T1, class T2, class T3 >
inline void RangeCut(T1 src, T2 minVal, T2 maxVal, T3 &dst)
{
	if( src<minVal )
		dst = (T3)minVal;
	else if( src>maxVal )
		dst = (T3)maxVal;
	else
	{
		dst = (T3)src;
	}
}

// 在哪个象限中
// quadrant = 1, 2, 3, or 4
// 4  3  2
// 1  0  1
// 2  4  4
template<class T1>
void InWhichQuadrant(T1 gx, T1 gy, int &quadrant)
{
	const float T225 = tan(22.5f*3.14159f/180);
	const float T675 = tan(67.5f*3.14159f/180);

	//梯度方向的非最大抑制
	float tanTh = 0.f;
	if(gx==0)
	{
		tanTh = 99999.f;
	}
	else
	{
		tanTh = (float)gy/gx;
	}
	float absTanTh = abs(tanTh);

	if( absTanTh<=T225 )
	{
		//1	
		quadrant = 1;
	}
	else if( ( tanTh<T675 ) && ( tanTh>T225 ) )
	{
		//2
		quadrant = 2;
	}
	else if( absTanTh>=T675 )
	{
		//3
		quadrant = 3;
	}
	//else if( ( tanTh<-T225 ) || (tanTh>-T675) )
	else
	{
		//4
		quadrant = 4;
	}
}


// 冒泡排序
template<class T1>
void SortLowToHighBubbleUp( T1* pData, int dataSize )
{
	if(pData==NULL)
		return;

	for(int i=0; i<dataSize; i++)
	{
		for(int j=i; j<dataSize; j++)
		{
			if(pData[j]<pData[i])
			{
				swap( pData[i], pData[j] );
			}
		}
	}

	return;
}

// 循环移动
// shiftDirection----0为左移, 1为右移
template<class T1>
bool AnnularShift(T1* pData, int nData, int nShiftStep, int shiftDirection)
{
	if(pData==NULL)
		return false;

	if(shiftDirection==0) // 左移
	{
		while(nShiftStep>0)
		{
			T1 temp;
			temp = pData[0];

			for(int i=0; i<nData-1; i++)
			{
				pData[i] = pData[i+1];
			}
			pData[nData-1] = temp;
			nShiftStep--;
		}
	}
	else // 右移
	{
		while(nShiftStep>0)
		{
			T1 temp;
			temp = pData[nData-1];

			for(int i=nData-1; i>0; i--)
			{
				pData[i] = pData[i-1];
			}
			pData[0] = temp;
			nShiftStep--;
		}
	}


	return true;
}

//两条直线的交点
void IntersectionPointOf2PolarLines( float rho1, float theta1, 
									float rho2, float theta2, 
									pool::SfPoint &point);

////两条直线的交点
//void IntersectionPointOf2PolarLines( pool::PolarLine line1, 
//									pool::PolarLine line2, 
//									pool::SfPoint &point);

//点到极坐标直线的距离
void DistanceOfPointToPolarLine(float ptX, float ptY, PolarLine line, float &dist);

//点到直线的距离
void DistanceOfPointToABCLine(float ptX, float ptY, float a, float b, float c, float &dist);

template <class T1>
int SetValue(T1* pSrc, int nData, T1 val)
{
	for(int i=0; i<nData; i++)
	{
		pSrc[i] = val;
	}

	return 0;
}

template <class T1>
int Sign(T1 val)
{
	if(val>0)
	{
		return 1;
	}
	else if(val<0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

int GenerateRandomNumbers(int n, int N, vector<int> &randNums);

// 生成n个不同的随机数[最大数字为N
int GenerateLargeRandomNumbers6( int n, int N, vector<int> &randNums );

// 生成n个不同的随机数[最大数字为N
int GenerateLargeRandomNumbers(int n, int N, vector<int> &randNums);

// 生成n个不同的随机数[最大数字为N
// N不能超过999999
int GenerateLargeRandomNumbers6_woSeed( int n, int N, vector<int> &randNums );

// 三次方程的根
int RootOf3OrderEquation(double a, double b, double c, double d, double &x1, double &x2, double &x3);



