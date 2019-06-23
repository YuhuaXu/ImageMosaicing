
#pragma once

#include "Point.h"
#include "Bitmap.h"
#include "Matrix.h"
#include "mvMath.h"
namespace pool
{
	struct Gesture 
	{
		double th;
		double tx;
		double ty;
		Gesture()
		{
			th=0;
			tx=0;
			ty=0;
		}
	};


	struct Gesture3D
	{
		double th;
		double tx;
		double ty;
		Gesture3D()
		{
			th=0;
			tx=0;
			ty=0;
		}
		Gesture3D(double th, double tx, double ty)
		{
			this->th = th;
			this->tx = tx;
			this->ty = ty;
		}
	};

	struct Gesture3D32F
	{
		float th;
		float tx;
		float ty;
		Gesture3D32F()
		{
			th=0;
			tx=0;
			ty=0;
		}
		Gesture3D32F(float th, float tx, float ty)
		{
			this->th = th;
			this->tx = tx;
			this->ty = ty;
		}
	};


	struct Gesture5D
	{
		double sx;
		double sy;
		double th;
		double tx;
		double ty;

		Gesture5D()
		{
			sx = 0;
			sy = 0;
			th = 0;
			tx = 0;
			ty = 0;
		}
	};


	struct Gesture4D
	{
		double sx;
		double th;
		double tx;
		double ty;

		Gesture4D()
		{
			sx = 0;
			th = 0;
			tx = 0;
			ty = 0;
		}
	};

	struct Gesture4D32F
	{
		float sx;
		float th;
		float tx;
		float ty;

		Gesture4D32F()
		{
			sx = 0;
			th = 0;
			tx = 0;
			ty = 0;
		}
	};

	// 假设Y轴没有歪斜，X轴有歪斜
	// sx,sy------尺度
	// theta------X轴的歪斜角度
	// beta-------刚性变换的旋转角
	// tx,ty------平移
	struct AffineXParam32F
	{
		float sx, sy, theta, beta, tx, ty;

		AffineXParam32F(float sx, float sy, float theta, float beta, float tx, float ty)
		{
			this->sx = sx;
			this->sy = sy;
			this->theta = theta;
			this->beta = beta;
			this->tx = tx;
			this->ty = ty;
		}
		AffineXParam32F()
		{
			
		}
	};

	// 假设Y轴没有歪斜，X轴有歪斜
	// sx,sy------尺度
	// theta------X轴的歪斜角度
	// beta-------刚性变换的旋转角
	// tx,ty------平移
	struct AffineXParam64F
	{
		double sx, sy, theta, beta, tx, ty;

		AffineXParam64F(double sx, double sy, double theta, double beta, double tx, double ty)
		{
			this->sx = sx;
			this->sy = sy;
			this->theta = theta;
			this->beta = beta;
			this->tx = tx;
			this->ty = ty;
		}
		AffineXParam64F()
		{

		}
	};
}
void Apply5DGesture(const pool::DfPoint point0, const pool::Gesture5D motion5D, 
					pool::DfPoint &point1 );

void Apply3DGesture(const pool::DfPoint* point0, const pool::Gesture3D motion3D, 
					pool::DfPoint* point1);


//非线性最小二乘法，求解 平移矩阵 和 旋转矩阵
int NonlinearLeastSquare3D(pool::DfPoint *matchsort1, pool::DfPoint *matchsort0, int N, 
						pool::Gesture3D &motion, pool::Gesture3D w0,
						double stop_x, double stop_y, double stop_th);

//非线性最小二乘法，求解 平移矩阵 和 旋转矩阵
int NonlinearLeastSquare3D_32F(pool::SfPoint *matchsort1, pool::SfPoint *matchsort0, int N, 
						pool::Gesture3D &motion, pool::Gesture3D w0,
						float stop_x, float stop_y, float stop_th);

// 估计两个对应好的点集之间的尺度
template <class PointType, class T1>
int EstimateScale(PointType *matchsort1, PointType *matchsort0, int N, T1 &scale)
{
	int nValid = 0;
	scale = 0;
	for(int n=0; n<N; n++)
	{
		for(int m=n+1; m<N; m++)
		{
			T1 dist1, dist0;
			DistanceOfTwoPoints(matchsort0[n].x, matchsort0[n].y, matchsort0[m].x, matchsort0[m].y, dist0);
			DistanceOfTwoPoints(matchsort1[n].x, matchsort1[n].y, matchsort1[m].x, matchsort1[m].y, dist1);
			if( (dist0>1e-6f)&&(dist1>1e-6f) )
			{
				scale += dist1/dist0;
				nValid++;
			}
		}
	}

	if(nValid>0)
	{
		scale /= nValid;
	}

	return 0;
}

// 函数功能：非线性最小二乘法，求解 平移矩阵 和 旋转矩阵，以及缩放系数
// matchsort1---固定
// Without CV
//int NonlinearLeastSquare4D(pool::DfPoint *matchsort1, pool::DfPoint *matchsort0, int N, pool::Gesture4D &motion, pool::Gesture4D w0,
//	double stop_sx, double stop_x, double stop_y, double stop_th );
template <class PointType, class GestureType, class T1>
int NonlinearLeastSquare4D(PointType *matchsort1, PointType *matchsort0, int N, 
						   GestureType &motion, GestureType w0,
						   T1 stop_sx, T1 stop_x, T1 stop_y, T1 stop_th )
{
	if(N<2)
	{
		std::cout<<"Error: nPairs must >2 in NonlinearLeastSquare(). scaleX, scaleY"<<std::endl;
		return 0;
	}
	if( (matchsort0==NULL) || (matchsort1==NULL) )
	{
		std::cout<<"Input pointers can not be null in NolinearLeastSquare()."<<std::endl;
		return 0;
	}

	int tMax = 15; // 最大迭代次数
	T1 xi1, yi1, xi0, yi0;

	GestureType w;

	//const int nMax = 16;
	//if(N>nMax)
	//	return 0;
	CMemoryPool memPool;
	
	const int DIMENSION = 4; // 变量的维数

	// T1 aJ[nMax*2*dimension];
	T1 *aJ = (T1*)memPool.Operate( NULL, N*2*DIMENSION*sizeof(T1) );

	//T1 aC[nMax*2*1];
	T1 *aC = (T1*)memPool.Operate( NULL, N*2*1*sizeof(T1) );

	T1 aJTEMP1[DIMENSION*DIMENSION];
	T1 aJTEMP2[DIMENSION*DIMENSION];

	//T1 aJT[dimension*2*nMax];
	T1* aJT = (T1*)memPool.Operate( NULL, DIMENSION*2*N*sizeof(T1) );

	//T1 aJLast[dimension*2*nMax];
	T1* aJLast = (T1*)memPool.Operate( NULL, DIMENSION*2*N*sizeof(T1) );

	T1 aDeltaX[DIMENSION];

	for(int t=0; t<tMax; t++) // 迭代计算
	{
		if(t==0)
		{
			w = w0;
		}
#ifdef _DEBUG
		double meanErrorDist = 0;
#endif
		T1 dfCosA = cos(w.th);
		T1 dfSinA = sin(w.th);
		for(int i=0; i<N; i++)
		{
			xi1 = matchsort1[i].x;
			yi1 = matchsort1[i].y;

			xi0 = matchsort0[i].x;
			yi0 = matchsort0[i].y;

			// 
			aJ[i*8 + 0] = xi0*dfCosA - yi0*dfSinA;
			aJ[i*8 + 1] = 1.0;
			aJ[i*8 + 2] = 0.0;
			aJ[i*8 + 3] = -w.sx*xi0*dfSinA - w.sx*yi0*dfCosA;

			aJ[i*8 + 4] = xi0*dfSinA + yi0*dfCosA;
			aJ[i*8 + 5] = 0;
			aJ[i*8 + 6] = 1.0;
			aJ[i*8 + 7] = w.sx*xi0*dfCosA - w.sx*yi0*dfSinA;

			aC[i*2 + 0] = xi1 - (w.sx*xi0*dfCosA - w.sx*yi0*dfSinA + w.tx);
			aC[i*2 + 1] = yi1 - (w.sx*xi0*dfSinA + w.sx*yi0*dfCosA + w.ty);

#ifdef _DEBUG
			meanErrorDist += (aC[i*2 + 0] * aC[i*2 + 0] + aC[i*2 + 1] * aC[i*2 + 1]);
#endif
		}

#ifdef _DEBUG
		meanErrorDist = sqrt(meanErrorDist/N);
		cout<<"meanErrorDist "<<meanErrorDist<<endl;
#endif

		//cvTranspose( JO, JT );	//JT=J'转置
		TransposeMatrix( aJ, 2*N, DIMENSION, aJT );

		//cvGEMM(JT,JO , 1, NULL, 0, JTEMP1);	//JT*J   矩阵相乘
		MulMatrix( aJT, DIMENSION, 2*N, aJ, 2*N, DIMENSION, aJTEMP1 );

		//double det = cvInvert(JTEMP1, JTEMP1  );	// 矩阵求逆
		InverseMatrix( aJTEMP1, DIMENSION, aJTEMP2 );

		//cvGEMM(JTEMP1, JT , 1, NULL, 0, Jlast);	//Jlast---J*  相乘
		MulMatrix( aJTEMP2, DIMENSION, DIMENSION, aJT, DIMENSION, 2*N, aJLast );

		//cvGEMM(Jlast, CC , 1, NULL, 0, deltax);	//求delta_x
		MulMatrix( aJLast, DIMENSION, 2*N, aC, 2*N, 1, aDeltaX );

		T1 d_sx = 0, dtx=0, dty=0, dth=0;

		d_sx = aDeltaX[0];
		dtx = aDeltaX[1];
		dty = aDeltaX[2];
		dth = aDeltaX[3];

		//x(k)=x(k-1)+δx
		w.sx = w.sx + d_sx;
		w.tx = w.tx + dtx;
		w.ty = w.ty + dty;
		w.th = w.th + dth;

		if( (abs(d_sx)<stop_sx) && 
			(abs(dtx)<stop_x) && 
			(abs(dty)<stop_y) && 
			(abs(dth)<stop_th) )
		{
			break;
		}

	}

	motion.sx = w.sx;
	motion.th = w.th;
	motion.tx = w.tx;
	motion.ty = w.ty;

	memPool.Operate( aJ );
	memPool.Operate( aC );
	memPool.Operate( aJT );
	memPool.Operate( aJLast );

	return 1;
}

//非线性最小二乘法，求解 投影矩阵
template<class TypePoint, class T1>
int NonlinearLeastSquareProjection2(TypePoint *matchsort1, TypePoint *matchsort0, int N, 
								   T1* motion, T1 *motion0,
								   T1 stop_condition)
{
	if(N<4)
	{
		return 0;
	}
	if( (matchsort0==NULL) || (matchsort1==NULL) )
	{
		return 0;
	}

	int tmax = 15;
	T1 x1 = 0, y1 = 0, x2 = 0, y2 = 0;

	//ProjectMat w;
	T1 w[9];
	const int nMax = 10;

	const int dimension = 8;

	T1* pJ = new T1[N*2*dimension];
	T1 *pC = new T1[N*2*1];
	T1 *pJT = new T1[dimension*2*N];
	T1 aJTEMP1[dimension*dimension];
	T1 aJTEMP2[dimension*dimension];
	T1 *pJLast = new T1[dimension*2*N];
	T1 aDeltaX[dimension];
	double maxErrorDist = 0;

	for(int t=0; t<tmax; t++) // 迭代计算
	{
		if(t==0)
		{
			//w = w0;
			memcpy( w, motion0, sizeof(T1)*8 );
		}

#ifdef _DEBUG
		double meanErrorDist = 0;
#endif
		for(int i=0; i<N; i++)
		{
			x2 = (T1)matchsort1[i].x;
			y2 = (T1)matchsort1[i].y;

			x1 = (T1)matchsort0[i].x;
			y1 = (T1)matchsort0[i].y;

			T1 m6x1_m7y1_1 = w[6] * x1 + w[7] * y1 + 1;
			T1 m0x1_m2y1_m2 = w[0] * x1 + w[1] * y1 + w[2];
			T1 m3x1_m4y1_m5 = w[3] * x1 + w[4] * y1 + w[5];

			////求J(i)
			pJ[i*16 + 0] = x1 / m6x1_m7y1_1;
			pJ[i*16 + 1] = y1 / m6x1_m7y1_1;
			pJ[i*16 + 2] = 1 / m6x1_m7y1_1;

			pJ[i*16 + 3] = 0;
			pJ[i*16 + 4] = 0;
			pJ[i*16 + 5] = 0;

			pJ[i*16 + 6] = -x1*m0x1_m2y1_m2 / (m6x1_m7y1_1*m6x1_m7y1_1);
			pJ[i*16 + 7] = -y1*m0x1_m2y1_m2 / (m6x1_m7y1_1*m6x1_m7y1_1);
			//=============================================================
			pJ[i*16 + 8] = 0;
			pJ[i*16 + 9] = 0;
			pJ[i*16 + 10] = 0;

			pJ[i*16 + 11] = x1/m6x1_m7y1_1;
			pJ[i*16 + 12] = y1/m6x1_m7y1_1;
			pJ[i*16 + 13] = 1/m6x1_m7y1_1;

			pJ[i*16 + 14] = -x1 * m3x1_m4y1_m5 / (m6x1_m7y1_1*m6x1_m7y1_1);
			pJ[i*16 + 15] = -y1 * m3x1_m4y1_m5 / (m6x1_m7y1_1*m6x1_m7y1_1);

			pC[i*2 + 0] = x2 - m0x1_m2y1_m2 / m6x1_m7y1_1;
			pC[i*2 + 1] = y2 - m3x1_m4y1_m5 / m6x1_m7y1_1;

#ifdef _DEBUG
			meanErrorDist += (pC[i*2 + 0] * pC[i*2 + 0] + pC[i*2 + 1] * pC[i*2 + 1]);
#endif
		}

#ifdef _DEBUG
		meanErrorDist = sqrt(meanErrorDist/N);
		//cout<<"meanErrorDist "<<meanErrorDist<<endl;
#endif

		//cvTranspose(JO, JT );//JT=J'转置
		TransposeMatrix( pJ, 2*N, dimension, pJT );

		//cvGEMM(JT,JO , 1, NULL, 0, JTEMP1);//JT*J   矩阵相乘
		MulMatrix( pJT, dimension, 2*N, pJ, 2*N, dimension, aJTEMP1 );

		//float det=cvInvert(JTEMP1, JTEMP1  );// 矩阵求逆
		InverseMatrix( aJTEMP1, dimension, aJTEMP2 );

		//cvGEMM(JTEMP1, JT , 1, NULL, 0, Jlast);//Jlast---J*  相乘
		MulMatrix( aJTEMP2, dimension, dimension, pJT, dimension, 2*N, pJLast );

		//cvGEMM(Jlast, CC , 1, NULL, 0, deltax);//求delta_x
		MulMatrix( pJLast, dimension, 2*N, pC, 2*N, 1, aDeltaX );

		for(int i=0; i<8; i++)
		{
			w[i] += aDeltaX[i];
		}

		// 计算均方差
		T1 errorDist = 0;
		T1 errorDistMax = 0;
		double accError = 0, MSE = 0;
		for(int i=0; i<N; i++)
		{
			T1 xDst, yDst = 0;
			ApplyProjectMat2( matchsort0[i].x, matchsort0[i].y, xDst, yDst, w );
			float dist = 0;
			DistanceOfTwoPoints( matchsort1[i].x, matchsort1[i].y, xDst, yDst, dist );
			errorDist = dist;
			if(errorDist>errorDistMax)
			{
				errorDistMax = errorDist;
			}
			accError += errorDist*errorDist;
		}
		MSE = sqrt(accError / N);

		if( (abs(aDeltaX[0])<stop_condition) && 
			(abs(aDeltaX[1])<stop_condition) &&
			(abs(aDeltaX[2])<stop_condition) &&
			(abs(aDeltaX[3])<stop_condition) &&
			(abs(aDeltaX[4])<stop_condition) &&
			(abs(aDeltaX[5])<stop_condition) &&
			(abs(aDeltaX[6])<stop_condition) &&
			(abs(aDeltaX[7])<stop_condition) )
		{
			break;
		}
	}

	for(int i=0; i<9; i++)
	{
		motion[i] = w[i];
	}

	// 计算均方差
	T1 errorDist = 0;
	T1 errorDistMax = 0;
	for(int i=0; i<N; i++)
	{
		T1 xDst, yDst = 0;
		ApplyProjectMat2( matchsort0[i].x, matchsort0[i].y, xDst, yDst, motion );
		float dist = 0;
		DistanceOfTwoPoints( matchsort1[i].x, matchsort1[i].y, xDst, yDst, dist );
		errorDist = dist;
		if(errorDist>errorDistMax)
		{
			errorDistMax = errorDist;
		}
	}

#ifdef _DEBUG
#endif
	motion[8] = errorDistMax;

	delete[] pJ;
	//memPool.Operate( pJ );
	delete[] pC;
	//memPool.Operate( pC );
	delete[] pJT;
	//memPool.Operate( pJT );
	delete[] pJLast;
	//memPool.Operate( pJLast );

	return 1;
}

//非线性最小二乘法，求解 投影矩阵
template<class TypePoint, class T1>
int NonlinearLeastSquareProjection(TypePoint *matchsort1, TypePoint *matchsort0, int N, 
								   T1* motion, T1 *motion0,
								   T1 stop_condition)
{
	if(N<4)
	{
		return 0;
	}
	if( (matchsort0==NULL) || (matchsort1==NULL) )
	{
		return 0;
	}

	CMemoryPool memPool;

	int tmax = 15;
	T1 x1 = 0, y1 = 0, x2 = 0, y2 = 0;

	//ProjectMat w;
	T1 w[9];
	const int nMax = 10;

	const int dimension = 8;

	T1* pJ = (T1*)memPool.Operate( NULL, N*2*dimension*sizeof(T1) );
	T1 *pC = (T1*)memPool.Operate( NULL, N*2*1*sizeof(T1) );
	T1 *pJT = (T1*)memPool.Operate( NULL, dimension*2*N*sizeof(T1) );
	T1 aJTEMP1[dimension*dimension];
	T1 aJTEMP2[dimension*dimension];
	T1 *pJLast = (T1*)memPool.Operate( NULL, dimension*2*N*sizeof(T1));
	T1 aDeltaX[dimension];
	double maxErrorDist = 0;

	for(int t=0; t<tmax; t++) // 迭代计算
	{
		if(t==0)
		{
			//w = w0;
			memcpy( w, motion0, sizeof(T1)*8 );
		}

#ifdef _DEBUG
		double meanErrorDist = 0;
#endif
		for(int i=0; i<N; i++)
		{
			x2 = (T1)matchsort1[i].x;
			y2 = (T1)matchsort1[i].y;

			x1 = (T1)matchsort0[i].x;
			y1 = (T1)matchsort0[i].y;

			T1 m6x1_m7y1_1 = w[6] * x1 + w[7] * y1 + 1;
			T1 m0x1_m2y1_m2 = w[0] * x1 + w[1] * y1 + w[2];
			T1 m3x1_m4y1_m5 = w[3] * x1 + w[4] * y1 + w[5];

			////求J(i)
			pJ[i*16 + 0] = x1 / m6x1_m7y1_1;
			pJ[i*16 + 1] = y1 / m6x1_m7y1_1;
			pJ[i*16 + 2] = 1 / m6x1_m7y1_1;

			pJ[i*16 + 3] = 0;
			pJ[i*16 + 4] = 0;
			pJ[i*16 + 5] = 0;

			pJ[i*16 + 6] = -x1*m0x1_m2y1_m2 / (m6x1_m7y1_1*m6x1_m7y1_1);
			pJ[i*16 + 7] = -y1*m0x1_m2y1_m2 / (m6x1_m7y1_1*m6x1_m7y1_1);
			//=============================================================
			pJ[i*16 + 8] = 0;
			pJ[i*16 + 9] = 0;
			pJ[i*16 + 10] = 0;

			pJ[i*16 + 11] = x1/m6x1_m7y1_1;
			pJ[i*16 + 12] = y1/m6x1_m7y1_1;
			pJ[i*16 + 13] = 1/m6x1_m7y1_1;

			pJ[i*16 + 14] = -x1 * m3x1_m4y1_m5 / (m6x1_m7y1_1*m6x1_m7y1_1);
			pJ[i*16 + 15] = -y1 * m3x1_m4y1_m5 / (m6x1_m7y1_1*m6x1_m7y1_1);

			pC[i*2 + 0] = x2 - m0x1_m2y1_m2 / m6x1_m7y1_1;
			pC[i*2 + 1] = y2 - m3x1_m4y1_m5 / m6x1_m7y1_1;

#ifdef _DEBUG
			meanErrorDist += (pC[i*2 + 0] * pC[i*2 + 0] + pC[i*2 + 1] * pC[i*2 + 1]);
#endif
		}

#ifdef _DEBUG
		meanErrorDist = sqrt(meanErrorDist/N);
		//cout<<"meanErrorDist "<<meanErrorDist<<endl;
#endif

		//cvTranspose(JO, JT );//JT=J'转置
		TransposeMatrix( pJ, 2*N, dimension, pJT );

		//cvGEMM(JT,JO , 1, NULL, 0, JTEMP1);//JT*J   矩阵相乘
		MulMatrix( pJT, dimension, 2*N, pJ, 2*N, dimension, aJTEMP1 );

		//float det=cvInvert(JTEMP1, JTEMP1  );// 矩阵求逆
		InverseMatrix( aJTEMP1, dimension, aJTEMP2 );

		//cvGEMM(JTEMP1, JT , 1, NULL, 0, Jlast);//Jlast---J*  相乘
		MulMatrix( aJTEMP2, dimension, dimension, pJT, dimension, 2*N, pJLast );

		//cvGEMM(Jlast, CC , 1, NULL, 0, deltax);//求delta_x
		MulMatrix( pJLast, dimension, 2*N, pC, 2*N, 1, aDeltaX );

		for(int i=0; i<8; i++)
		{
			w[i] += aDeltaX[i];
		}

		// 计算均方差
		T1 errorDist = 0;
		T1 errorDistMax = 0;
		double accError = 0, MSE = 0;
		for(int i=0; i<N; i++)
		{
			T1 xDst, yDst = 0;
			ApplyProjectMat2( matchsort0[i].x, matchsort0[i].y, xDst, yDst, w );
			float dist = 0;
			DistanceOfTwoPoints( matchsort1[i].x, matchsort1[i].y, xDst, yDst, dist );
			errorDist = dist;
			if(errorDist>errorDistMax)
			{
				errorDistMax = errorDist;
			}
			accError += errorDist*errorDist;
		}
		MSE = sqrt(accError / N);

		if( (abs(aDeltaX[0])<stop_condition) && 
		    (abs(aDeltaX[1])<stop_condition) &&
			(abs(aDeltaX[2])<stop_condition) &&
			(abs(aDeltaX[3])<stop_condition) &&
			(abs(aDeltaX[4])<stop_condition) &&
			(abs(aDeltaX[5])<stop_condition) &&
			(abs(aDeltaX[6])<stop_condition) &&
			(abs(aDeltaX[7])<stop_condition) )
		{
			break;
		}
	}

	for(int i=0; i<9; i++)
	{
		motion[i] = w[i];
	}

	// 计算均方差
	T1 errorDist = 0;
	T1 errorDistMax = 0;
	for(int i=0; i<N; i++)
	{
		T1 xDst, yDst = 0;
		ApplyProjectMat2( matchsort0[i].x, matchsort0[i].y, xDst, yDst, motion );
		float dist = 0;
		DistanceOfTwoPoints( matchsort1[i].x, matchsort1[i].y, xDst, yDst, dist );
		errorDist = dist;
		if(errorDist>errorDistMax)
		{
			errorDistMax = errorDist;
		}
	}

#ifdef _DEBUG
#endif
	motion[8] = errorDistMax;

	//delete[] pJ;
	memPool.Operate( pJ );
	//delete[] pC;
	memPool.Operate( pC );
	//delete[] pJT;
	memPool.Operate( pJT );
	//delete[] pJLast;
	memPool.Operate( pJLast );

	return 1;
}

// 函数功能：非线性最小二乘法，求解 平移矩阵 和 旋转矩阵，以及缩放矩阵
// Without CV
int NonlinearLeastSquare5D_4Points(pool::DfPoint matchsort1[], pool::DfPoint matchsort0[], int N, 
			pool::Gesture5D &motion, 
			pool::Gesture5D w0,
			double stop_sx, double stop_sy, double stop_x, double stop_y, double stop_th );

// 函数功能：非线性最小二乘法，求解 平移矩阵 和 旋转矩阵，以及缩放矩阵
// Without CV
int NonlinearLeastSquare5D(pool::DfPoint* pMatchPoint1, pool::DfPoint* pMatchPoint0, int N, 
		pool::Gesture5D &motion, 
		pool::Gesture5D w0,
		double stop_sx = 1e-6, double stop_sy = 1e-6, 
		double stop_x = 1e-6, double stop_y = 1e-6, double stop_th = 1e-6 );

// 非最小二乘法拟合直线，极坐标表示【稳定版】
int FitLineNonLeastSquare(float A[], float B[], int N, PolarLine &linepa, PolarLine w0);

template<class T1>
void ApplyAffineX(const T1 x0, const T1 y0, const pool::AffineXParam32F motion, 
				  T1 &xDst, T1 &yDst)
{
	T1 sinTheta = 0, cosTheta = 0, sinBeta = 0, cosBeta = 0;
	sinTheta = sin(motion.theta); cosTheta = cos(motion.theta);
	sinBeta = sin(motion.beta); cosBeta = cos(motion.beta);
	T1 sx = motion.sx, sy = motion.sy;
	T1 tx = motion.tx, ty = motion.ty;

	xDst = sx*x0*cosTheta*cosBeta - sinBeta*(sy*y0-sx*x0*sinTheta) + tx;
	yDst = sx*x0*cosTheta*sinBeta + cosBeta*(sy*y0-sx*x0*sinTheta) + ty;	

	return;
}

// 解仿射变换, 6个参数,
// X轴歪斜
// RMSE------均方根误差
template<class TypePoint, class T1>
bool SoveAffineXSkew(TypePoint *matchsort1, TypePoint *matchsort0, int N, 
									pool::AffineXParam32F &motion, T1 &RMSE, pool::AffineXParam32F motion0,
									T1 stop_condition = (T1)1e-6 )
{
	if(N<3)
	{
		return 0;
	}
	if( (matchsort0==NULL) || (matchsort1==NULL) )
	{
		return false;
	}

	CMemoryPool memPool;

	int tmax = 15;

	//ProjectMat w;
	const int nMax = 100;

	const int dimension = 6;

	float* pJ = (float*)memPool.Operate(NULL, N*2*dimension*sizeof(float) );

	float *pC = (float*)memPool.Operate(NULL, N*2*1*sizeof(float) );

	float *pJT = (float*)memPool.Operate(NULL, dimension*2*N*sizeof(float) );

	float aJTEMP1[dimension*dimension];

	float aJTEMP2[dimension*dimension];

	float *pJLast = (float*)memPool.Operate(NULL, dimension*2*N*sizeof(float));

	float aDeltaX[dimension];

	double maxErrorDist = 0;

	for(int t=0; t<tmax; t++) // 迭代计算
	{
		if(t==0)
		{
			motion = motion0;
		}

#ifdef _DEBUG
		double meanErrorDist = 0;
#endif
		for(int i=0; i<N; i++)
		{
			T1 x3 = 0, y3 = 0, x0 = 0, y0 = 0;
			x3 = (T1)matchsort1[i].x;
			y3 = (T1)matchsort1[i].y;

			x0 = (T1)matchsort0[i].x;
			y0 = (T1)matchsort0[i].y;

			T1 sinTheta = 0, cosTheta = 0, sinBeta = 0, cosBeta = 0;
			sinTheta = sin(motion.theta); cosTheta = cos(motion.theta);
			sinBeta = sin(motion.beta); cosBeta = cos(motion.beta);
			T1 sx = motion.sx, sy = motion.sy;
			T1 tx = motion.tx, ty = motion.ty;
	
			////求J(i)
			pJ[i*12 + 0] = x0*cosTheta*cosBeta + sinBeta*x0*sinTheta;
			pJ[i*12 + 1] = -sinBeta*y0;
			pJ[i*12 + 2] = -sx*x0*sinTheta*cosBeta + sinBeta*sx*x0*cosTheta;
			pJ[i*12 + 3] = -sx*x0*cosTheta*sinBeta - cosBeta*(sy*y0-sx*x0*sinTheta);
			pJ[i*12 + 4] = 1;
			pJ[i*12 + 5] = 0;
			//================================================================
			pJ[i*12 + 6] = x0*cosTheta*sinBeta - cosBeta*x0*sinTheta;
			pJ[i*12 + 7] = cosBeta*y0;
			pJ[i*12 + 8] = -sx*x0*sinTheta*sinBeta - cosBeta*sx*x0*cosTheta;
			pJ[i*12 + 9] = sx*x0*cosTheta*cosBeta - sinBeta*(sy*y0-sx*x0*sinTheta);
			pJ[i*12 + 10] = 0;
			pJ[i*12 + 11] = 1;
			//=================================================================
			pC[i*2 + 0] = x3 - (sx*x0*cosTheta*cosBeta - sinBeta*(sy*y0-sx*x0*sinTheta) + tx);
			pC[i*2 + 1] = y3 - (sx*x0*cosTheta*sinBeta + cosBeta*(sy*y0-sx*x0*sinTheta) + ty);

#ifdef _DEBUG
			meanErrorDist += (pC[i*2 + 0] * pC[i*2 + 0] + pC[i*2 + 1] * pC[i*2 + 1]);
#endif

		}

#ifdef _DEBUG
		meanErrorDist = sqrt(meanErrorDist/N);
		if(meanErrorDist>maxErrorDist)
		{
			maxErrorDist = meanErrorDist;
		}
		//cout<<"meanErrorDist "<<meanErrorDist<<endl;
#endif

		//JT=J'转置
		TransposeMatrix( pJ, 2*N, dimension, pJT );

		//JT*J   矩阵相乘
		MulMatrix( pJT, dimension, 2*N, pJ, 2*N, dimension, aJTEMP1 );

		// 矩阵求逆
		InverseMatrix( aJTEMP1, dimension, aJTEMP2 );

		//Jlast---J*  相乘
		MulMatrix( aJTEMP2, dimension, dimension, pJT, dimension, 2*N, pJLast );

		//求delta_x
		MulMatrix( pJLast, dimension, 2*N, pC, 2*N, 1, aDeltaX );

		motion.sx += aDeltaX[0];
		motion.sy += aDeltaX[1];
		motion.theta += aDeltaX[2];
		motion.beta += aDeltaX[3];
		motion.tx += aDeltaX[4];
		motion.ty += aDeltaX[5];

		if( (abs(aDeltaX[0])<stop_condition) && 
			(abs(aDeltaX[1])<stop_condition) &&
			(abs(aDeltaX[2])<stop_condition) &&
			(abs(aDeltaX[3])<stop_condition) &&
			(abs(aDeltaX[4])<stop_condition) &&
			(abs(aDeltaX[5])<stop_condition) )
		{
			break;
		}
	}

	// 计算均方差
	RMSE = 0;
	{
		T1 sinTheta = 0, cosTheta = 0, sinBeta = 0, cosBeta = 0;
		sinTheta = sin(motion.theta); cosTheta = cos(motion.theta);
		sinBeta = sin(motion.beta); cosBeta = cos(motion.beta);
		T1 sx = motion.sx, sy = motion.sy;
		T1 tx = motion.tx, ty = motion.ty;

		float errorDist2 = 0;
		float errorDistMax2 = 0;
		for(int i=0; i<N; i++)
		{
			T1 x0 = matchsort0[i].x, y0 = matchsort0[i].y;
			T1 xDst, yDst = 0;
			xDst = sx*x0*cosTheta*cosBeta - sinBeta*(sy*y0-sx*x0*sinTheta) + tx;
			yDst = sx*x0*cosTheta*sinBeta + cosBeta*(sy*y0-sx*x0*sinTheta) + ty;

			float dist = 0;
			DistanceOfTwoPoints( matchsort1[i].x, matchsort1[i].y, xDst, yDst, dist );
			errorDist2 = dist * dist;
			RMSE += errorDist2;
			if(errorDist2>errorDistMax2)
			{
				errorDistMax2 = errorDist2;
			}
		}
		RMSE = sqrt(RMSE/N);
	}

	memPool.Operate( pJ );
	memPool.Operate( pC );
	memPool.Operate( pJT );
	memPool.Operate( pJLast );

	return true;
}