//#include "stdafx.h"
#include "imageMath.h"
#include "ImageIO.h"
#include "Matrix.h"
#include "LeastSquare.h"
#include "usingCV24.h"

//求一个点在四象限中的角度（弧度），0 ~ 2*pi
int AngleofPoint360(float x, float y, float &angleArc)
{
	if(x>=0)// 1，4象限
	{
		if(y>=0) // 1  x>0 y>0
		{
			if(x!=0)
			{
				angleArc = (float)atan(y/x);
			}
			else//x==0
			{
				angleArc = pi/2;
			}
		}
		else // 4  x>0  y<0
		{
			if(x!=0)
			{
				angleArc = 2*pi + (float)atan(y/x);
			}
			else//x==0
			{
				angleArc = 3*pi/2;
			}
		}
	}
	else// 2，3象限
	{
		if(y>=0) // 2
		{
			if(x!=0)  // x<0 y>0
			{
				angleArc = pi + (float)atan(y/x);
			}
		}
		else //3  x<0  y<0
		{
			if(x!=0)
			{
				angleArc = pi + (float)atan(y/x); 
			}
		}
	}
	return 1;
}

// 线性变换 
bool LinearTransform(const BitmapImage *pSrc, BitmapImage *pDst, 
					 const float fK, const float fB)
{
	if( (pSrc==NULL) || (pDst==NULL) )
		return false;

	int w, h, stride;
	GetImageSize( pSrc, w, h, stride );

	for (int y = 0; y < h; y++)
	{
		unsigned char *pSrcRow = pSrc->imageData + y*stride;
		unsigned char* pDstRow = pDst->imageData + y*stride;

		for (int x = 0; x < w; x++)
		{
			float fGrayTemp = fK * pSrcRow[x] + fB + 0.5f;

			int grayTemp = (int)fGrayTemp;

			if (grayTemp < 0)
				pDstRow[x] = 0;
			else if (grayTemp > 255)
				pDstRow[x] = 255;
			else
				pDstRow[x] = (byte)grayTemp;
		}
	}
	return true;
}

void LineOf2Points1(float &a,float &b,float &c,
				   float x1,float y1,float x2,float y2)
{
	if( fabs(x1-x2)<0.000001 ) // 如果x1等于x2
	{
		a=1.0f;
		b=0;
		c=-x1;
	}
	else // x1不等于x2
	{
		a=(y1-y2)/(x1-x2);
		b=-1.0f;
		c=y1-a*x1;
	}
}

//计算 1/sqrt(x)  1/sqrt(x)----开根号的倒数快速求解方法
float InvSqrt (float x)
{
	float xhalf = 0.5f*x;
	int i = *(int*)&x;
	i = 0x5f3759df - (i >> 1);        // 计算第一个近似根
	x = *(float*)&i;
	x = x*(1.5f - xhalf*x*x);       // 牛顿迭代法
	return x;
}

//void Apply4DMotion(pool::DfPoint point1, pool::Gesture4D* motion, pool::DfPoint &point1Bar)
//{
//	point1Bar.x = point1.x
//}

// Point1 = R*Point0 + T;
// Point0 = R^-1*(Point1-T)
void Point0ToPoint1(float x0,float y0,
						   float &x1,float &y1,
						   float sina, float cosa, float dx, float dy)
{
	x1 = cosa*x0 - sina*y0 + dx;

	y1 = sina*x0 + cosa*y0 + dy;
}

// Point1 = R*Point0 + T;
// Point0 = R^-1*(Point1-T)
void Point1ToPoint0(float x1,float y1,
						   float &x0,float &y0,
						   float sina, float cosa, float dx, float dy)
{
	x0 = cosa*(x1-dx) + sina*(y1-dy);

	y0 = -sina*(x1-dx) + cosa*(y1-dy);
}

// 直线表示 ax+by+c=0  转到  rho = x*cos(theta)+y*sin(theta)
void ABCToPolar(float a,float b,float c,
				float &rho,float &theta)
{
	// 变成极坐标表示,
	float a1,b1;
	float xc,yc;
	rho=abs(a*0 +b*0 +c)/sqrt(a*a + b*b); // 极坐标的距离
	if (b==0)// 原直线垂直
	{ 
		a1=0; b1=1;
		yc=0;
		xc=-c/a;
	}
	if (a==0) // 原直线水平
	{ 
		a1=1;
		b1=0;
		xc=0;
		yc=-c/b;
	}
	if( (a!=0) && (b!=0) ) 
	{
		a1=-1.0f/a;

		xc=-c/(a-a1);
		yc=a1*xc;
	}
	AngleofPoint(xc, yc, theta);
	if(theta<0)
	{
		theta = theta + 2*pi; // theta 
	}
}

pool::fPoint TimeRotationMatrix2D(pool::fPoint src, float cosa, float sina)
{
	pool::fPoint dst;
	dst.x=cosa*src.x - sina*src.y;
	dst.y=sina*src.x + cosa*src.y;
	return dst;
}

pool::SfPoint TimeRotationMatrix2D(pool::SfPoint src, float cosa, float sina)
{
	pool::SfPoint dst;
	dst.x=cosa*src.x - sina*src.y;
	dst.y=sina*src.x + cosa*src.y;
	return dst;
}


bool And2(unsigned char* pSrc1, int w1, int h1, int ws1, 
		 unsigned char* pSrc2,
		 unsigned char* pDst)
{
	if(pSrc1==NULL)
		return false;
	if(pSrc2==NULL)
		return false;
	
	for(int y=0; y<h1; y++)
	{
		unsigned char* pRow1 = pSrc1 + y*ws1;
		unsigned char* pRow2 = pSrc2 + y*ws1;
		unsigned char* pDstRow = pDst + y*ws1;
		for(int x=0; x<w1; x++)
		{
			if( (pRow1[x]) && (pRow2[x]) )
			{
				pDstRow[x] = (unsigned char)255;
			}
			else
			{
				pDstRow[x] = 0;
			}
		}
	}

	return true;
}


//计算重心
int CalCenterOfGravity(const vector<pool::sfPoint> &pointSet, float &centerX,float &centerY)
{
	double accX=0, accY=0;

	int setSize;
	setSize = pointSet.size();

	if( setSize<=0 )
	{
		return 0;
	}

	if( setSize <= 0 )
	{
		return 0;
	}

	for( int u=0; u<setSize; u++ )
	{
		accX += pointSet[u].x;
		accY += pointSet[u].y;
	}

	centerX = (float)(accX/setSize);
	centerY = (float)(accY/setSize);

	return 1;
}


//计算重心
int CalCenterOfGravity(const vector<pool::Point> &pointSet, float &centerX,float &centerY)
{
	double accX=0, accY=0;

	int setSize = 0;
	setSize = (int)pointSet.size();

	if( setSize<=0 )
	{
		return 0;
	}

	if( setSize <= 0 )
	{
		return 0;
	}

	for( int u=0; u<setSize; u++ )
	{
		accX += pointSet[u].x;
		accY += pointSet[u].y;
	}

	centerX = (float)accX/setSize;
	centerY = (float)accY/setSize;

	return 1;
}


//计算重心
int CalCenterOfGravity( pool::Point *pointSet, int setSize, float &centerX, float &centerY)
{
	if(pointSet==NULL)
	{
		return 0;
	}

	if( setSize <= 0 )
	{
		return 0;
	}

	double dfAccX=0, dfAccY=0;
	for( int u=0; u<setSize; u++ )
	{
		dfAccX += pointSet[u].x;
		dfAccY += pointSet[u].y;
	}

	centerX = (float)dfAccX/setSize;
	centerY = (float)dfAccY/setSize;

	return 1;
}


// 函数功能：计算一张二值图的非零点的个数
int CalValidPixel(pool::BitmapImage *src,int &counter)
{
	if(src==NULL)
	{
		counter=0;
		return 0;
	}

	int u,v;
	counter=0;
	// 输入的图像应为二值图
	int height,width,widthstep;
	height=src->height;
	width=src->width;
	widthstep=src->widthStep;
	unsigned char *temp=NULL;
	for(u=0;u<height;u++) 
	{
		temp=src->imageData+u*widthstep;
		for(v=0;v<width;v++) 
		{
			if((unsigned char)*(temp+v)>0 )
			{
				counter++;
			}
		}
	}
	return 1;
}

// 求一堆点里面的4个端点的坐标  
void Find4EndPoints(pool::Point *pdata,int npt, sfPoint endpoint[4])
{
	int maxi=0,mini=0;
	int maxv=0,minv=9999;
	// x方向
	for(int u=0;u<npt;u++)
	{
		if(pdata[u].x>maxv)
		{
			maxv=pdata[u].x;
			maxi=u;
		}
		if(pdata[u].x<minv)
		{
			minv=pdata[u].x;
			mini=u;
		}
	}
	endpoint[0].x=(float)pdata[maxi].x;
	endpoint[0].y=(float)pdata[maxi].y;
	endpoint[1].x=(float)pdata[mini].x;
	endpoint[1].y=(float)pdata[mini].y;

	// y方向
	maxv=0,minv=9999;
	for(int u=0;u<npt;u++)
	{
		if(pdata[u].y>maxv)
		{
			maxv=pdata[u].y;
			maxi=u;
		}
		if(pdata[u].y<minv)
		{
			minv=pdata[u].y;
			mini=u;
		}
	}
	endpoint[2].x=(float)pdata[maxi].x;
	endpoint[2].y=(float)pdata[maxi].y;
	endpoint[3].x=(float)pdata[mini].x;
	endpoint[3].y=(float)pdata[mini].y;
}

bool SolveProjectMatrix_API( pool::SfPoint *vecPoints1,
							pool::SfPoint *vecPoints2, int pairsNum,
							float aProjectMat[9])
{
	return SolveProjectMatrix3(vecPoints1, vecPoints2, pairsNum, aProjectMat );
}

//两条直线的交点
void IntersectionPointOf2PolarLines( float rho1, float theta1, 
									float rho2, float theta2, 
									pool::SfPoint &point)
{
	float det;//行列式

	det = sin(theta2)*cos(theta1) - sin(theta1)*cos(theta2);

	point.x=( rho1*sin(theta2) - rho2*sin(theta1) )/det;

	point.y=( -rho1*cos(theta2) + rho2*cos(theta1) )/det;

	return ;
}

//点到极坐标直线的距离
void DistanceOfPointToPolarLine(float ptX, float ptY, PolarLine line, float &dist)
{
	dist = abs( ( ptX*cos(line.th) + ptY*sin(line.th)  ) - line.p );

	return;
}

//点到直线的距离
void DistanceOfPointToABCLine(float x, float y, float a, float b, float c, float &dist)
{
	dist = abs(a*x + b*y + c) / sqrt(a*a + b*b);
}

bool IntersectionPointsOfTwoCircles( float x1, float y1, float r1, 
									float x2, float y2, float r2, 
									float &intersectX1, float &intersectY1, 
									float &intersectX2, float &intersectY2 )
{
	
	return true;
}

// 遍历采样
// 从N个数中抽取所有n个数的组合
// N = sampleSize
// subSampleSize = n
bool Sampling(vector<Sample> &samplingResult, int sampleSize, int subSampleSize)
{
	if(sampleSize<=1)
		return false;

	const int MAX_SUB_SAMPLE_SIZE = 10;// 最大子样本维数
	vector<Sample> vecSampleing[MAX_SUB_SAMPLE_SIZE];

	for(int i=0; i<subSampleSize; i++)
	{

		if(i==0)
		{
			for(int j=0; j<sampleSize; j++)
			{
				Sample temp;
				temp.subSample[i] = j;
				vecSampleing[i].push_back( temp );
			}
		}
		else
		{
			int lastSampleLevelNum = int(vecSampleing[i-1].size());
			for(int p=0; p<lastSampleLevelNum; p++)
			{
				for(int j=0; j<sampleSize; j++)
				{
					if(j>vecSampleing[i-1][p].subSample[i-1])
					{
						Sample temp;
						for(int q=0; q<i; q++)
						{
							temp.subSample[q] = vecSampleing[i-1][p].subSample[q];
						}
						temp.subSample[i] = j;
						vecSampleing[i].push_back(temp);
					}
				}
			}		
		}
	}
	int sampleNumTotal = vecSampleing[subSampleSize-1].size();
	for(int i=0; i<sampleNumTotal; i++)
	{
		samplingResult.push_back( vecSampleing[subSampleSize-1][i] );
	}

	return true;
}

// 解投影矩阵(8个未知数，实际解的时候是8个)
// 方法：线性最小二乘
// point2---->point1的变换

// 获得特征向量
// 特征向量存入fM(降序排列), 按行存储
// 特征值存入fV
// CvMat* M = cvCreateMat(N, N, CV_64FC1);
// CvMat* E = cvCreateMat(N, N, CV_64FC1);
// CvMat* I = cvCreateMat(N, 1, CV_64FC1);
int GetEigenMat(double* k, int N, 
				double *fM, // 特征向量
				double *fV, // 特征值
				CvMat* M, 
				CvMat* E,
				CvMat* I
				)
{
	if(fM==NULL)
		return -1;

	//CvMat* M = cvCreateMat(N, N, CV_64FC1);

	cvZero(M);
	for(int i = 0; i < N; i++)
		for(int j = 0; j < N; j++)
			cvmSet(M, i, j, k[i*N+j]);

	//CvMat* E = cvCreateMat(N, N, CV_64FC1);
	cvZero(E);

	//CvMat* I = cvCreateMat(N, 1, CV_64FC1);
	cvZero(I);

	//从矩阵M中获取特征向量存入E(降序排列)，特征值存入I
	cvEigenVV(M, E, I);

	//取前两个最大的特征向量
	for(int i = 0; i < N; i++)
	{
		for(int j = 0; j < N; j++)
		{
			fM[i*N+j] = cvmGet(E,i,j);
		}
		fV[i] = cvmGet(I, i, 0); // 特征值
	}


	//cvReleaseMat(&M);
	//cvReleaseMat(&E);
	//cvReleaseMat(&I);

	return 0;
} 

int InverseMatrixCv(float *src, int dim, float *dst)
{
	CvMat* M = cvCreateMat(dim, dim, CV_32F);
	CvMat* invM = cvCreateMat(dim, dim, CV_32F);

	for(int r=0; r<dim; r++)
	{
		for(int c=0; c<dim; c++)
		{
			cvSetReal2D(M, r, c, src[r*dim+c]);
		}
	}

	cvInvert(M, invM);

	for(int r=0; r<dim; r++)
	{
		for(int c=0; c<dim; c++)
		{
			dst[r*dim+c] = cvGetReal2D(invM, r, c);
		}
	}

	cvReleaseMat(&M);
	cvReleaseMat(&invM);

	return 0;
}

int InverseMatrixCv64(double *src, int dim, double *dst)
{
	CvMat* M =	  cvCreateMat(dim, dim, CV_64F);
	CvMat* invM = cvCreateMat(dim, dim, CV_64F);

	for(int r=0; r<dim; r++)
	{
		for(int c=0; c<dim; c++)
		{
			cvSetReal2D(M, r, c, src[r*dim+c]);
		}
	}

	cvInvert(M, invM);

	for(int r=0; r<dim; r++)
	{
		for(int c=0; c<dim; c++)
		{
			dst[r*dim+c] = cvGetReal2D(invM, r, c);
		}
	}

	cvReleaseMat(&M);
	cvReleaseMat(&invM);

	return 0;
}

void GammaCorrection(IplImage* src, IplImage* dst, float gamma)
{
	int w = src->width;
	int h = src->height;

	for(int r=0; r<h; r++)
	{
		for(int c=0; c<w; c++)
		{
			int val = *((unsigned char*)src->imageData + r*w + c);
			float val32f = val/255.0f;

			val32f = pow( (float)val32f, gamma );

			*((unsigned char*)src->imageData + r*w + c) = 255*val32f;
		}
	}

	return;
}

// 生成n个不同的随机数[最大数字为N]
int GenerateRandomNumbers( int n, int N, vector<int> &randNums )
{
	vector<byte> mask(N);
	memset(&mask[0], 0, N);

	srand((unsigned)time(0)); // 种子

	for(int i=0; i<n;)
	{
		int val = (float)rand() /(RAND_MAX+1) * N;

		if(val>=N)
			val = 0;

		if(mask[val]==0) // valid
		{
			randNums.push_back(val);
			mask[val] = 1;
			i++;
		}
	}

	return 0;
}

int GetRandNumber2(int nBitMax)
{
	int res = 0;
	int t = 1;
	for(int i = 0 ; i<nBitMax; i++)
	{
		int vi = rand()%10;
		vi = vi*t;
		t = t * 10;
		res = res + vi;
	}

	return res;
}

// 生成n个不同的随机数[最大数字为N
// N不能超过999999
int GenerateLargeRandomNumbers6( int n, int N, vector<int> &randNums )
{
	vector<byte> mask(N);
	memset(&mask[0], 0, N);

	srand((unsigned)time(0)); // 种子

	int nBit = 6;
	float MAX_NUM6 = 999999.0;

	for(int i=0; i<n;)
	{
		//int val = (float)rand() /RAND_MAX * N;
		int val = GetRandNumber2(nBit) / (MAX_NUM6+1) * N;
		if(val>=N)
			val = 0;

		if(mask[val]==0) // valid
		{
			randNums.push_back(val);
			mask[val] = 1;
			i++;
		}
	}

	return 0;
}

// 生成n个不同的随机数[最大数字为N
int GenerateLargeRandomNumbers(int n, int N, vector<int> &randNums)
{
	vector<byte> mask(N);
	memset(&mask[0], 0, N);

	srand((unsigned)time(0)); // 种子

	// 计算N的位数
	int Ntemp = N;
	int nBit = 0;
	float MAX_NUM = 0;
	int level = 1;
	while (Ntemp)
	{
		nBit++;
		Ntemp /= 10;
		level *= 10;
	}
	MAX_NUM = level - 1;

	for (int i = 0; i<n;)
	{
		int val = GetRandNumber2(nBit) / (MAX_NUM + 1) * N;
		if (val >= N)
			val = 0;

		if (mask[val] == 0) // valid
		{
			randNums.push_back(val);
			mask[val] = 1;
			i++;
		}
	}

	return 0;
}

// 生成n个不同的随机数[最大数字为N
// N不能超过999999
int GenerateLargeRandomNumbers6_woSeed( int n, int N, vector<int> &randNums )
{
	vector<byte> mask(N);
	memset(&mask[0], 0, N);

	//srand((unsigned)time(0)); // 种子

	int nBit = 6;
	float MAX_NUM6 = 999999.0;

	for(int i=0; i<n;)
	{
		//int val = (float)rand() /RAND_MAX * N;
		int val = GetRandNumber2(nBit) / (MAX_NUM6+1) * N;
		if(val>=N)
			val = 0;

		if(mask[val]==0) // valid
		{
			randNums.push_back(val);
			mask[val] = 1;
			i++;
		}
	}

	return 0;
}

// 三次方程的根
int RootOf3OrderEquation(double a, double b, double c, double d, double &x1, double &x2, double &x3)
{
	double A;
	double B;
	double C;
	double delt;
	x1 = 0;
	x2 = 0;
	x3 = 0;

	A = b * b - 3 * a * c;
	B = b * c - 9 * a * d;
	C = c * c - 3 * b * d;

	delt = B * B - 4 * A * C;

	//--------------------------------------------

	if ( (abs(A)<1e-8) && (abs(B)<1e-8) )
	{
		x1 = -b / (3 * a);
		x2 = x1;
		x3 = x1;

		return 0;
	}

	if(delt>0)
	{
		double y1 = A * b + 3 * a * (-B + sqrt(delt)) / 2.0;

		double y2 = A * b + 3 * a * ((-B - sqrt(delt)) / 2.0);

		double pow1 = Sign(y1)*pow(abs(y1), 1.0/3.0);

		double pow2 = Sign(y2)*pow(abs(y2), 1.0/3.0);

		x1 = ( -b - pow1 - pow2 ) / (3*a);

		x2 = x3 = x1;

		return 0;
	}

	if (abs(delt)<1e-9)
	{
		if (A != 0)
		{
			double K = B / A;
			x1 = -b / a + K;
			x2 = -K / 2;
			x3 = x2;
			return 0;
		}
	}

	if (delt < 0)
	{
		if(A>0)
		{
			double sqrtA = sqrt(A);
			double T = (2 * A * b - 3 * a * B) / (2 * sqrt(A*A*A));
			double st = acos(T);
			double cos_st_3 = cos(st / 3);
			double sin_st_3  = sin(st / 3);
			x1 = (-b - 2 * sqrtA * cos_st_3) / (3 * a);
			x2 = (-b + sqrtA * (cos_st_3 + 1.7320508075 * sin_st_3)) / (3 * a);
			x3 = (-b + sqrtA * (cos_st_3 -  1.7320508075 * sin_st_3)) / (3 * a);
		}

		return 0;
	}

	return 0;
}