
#include "LeastSquare.h"
#include "Point.h"
#include "Matrix.h"
#include "math.h"
#include "mvMath.h"
#include <iostream>
#include "MemoryPool.h"

using namespace pool;

	void Apply5DGesture(const DfPoint point0, const Gesture5D motion5D, 
		DfPoint &point1 )
	{
		float sina = (float)sin(motion5D.th);
		float cosa = (float)cos(motion5D.th);
		point1.x = point0.x * motion5D.sx * cosa - point0.y * motion5D.sy * sina + motion5D.tx;
		point1.y = point0.x*motion5D.sx*sina + point0.y*motion5D.sy*cosa + motion5D.ty;
	}

	void Apply3DGesture(const DfPoint* point0, const Gesture3D motion3D, 
		DfPoint* point1)
	{
		float sina = (float)sin(motion3D.th);
		float cosa = (float)cos(motion3D.th);
		point1->x = point0->x*cosa - point0->y*sina + motion3D.tx;
		point1->y = point0->x*sina + point0->y*cosa + motion3D.ty;
	}



	//非线性最小二乘法，求解 平移矩阵 和 旋转矩阵
	int NonlinearLeastSquare3D(pool::DfPoint *matchsort1, pool::DfPoint *matchsort0, int N, 
							   pool::Gesture3D &motion, pool::Gesture3D w0,
							   double stop_x, double stop_y, double stop_th)
	{
		if(N<2)
		{
			std::cout<<"Error: nPairs must >2 in NonlinearLeastSquare()."<<std::endl;
			return 0;
		}
		if( (matchsort0==0) || (matchsort1==0) )
		{
			std::cout<<"Input pointers can not be null in NolinearLeastSquare()."<<std::endl;
			return 0;
		}

		int tmax=15;
		float xi1,zi1, xi0,zi0;

		pool::Gesture3D w;
		w.th=0;
		w.tx=0;
		w.ty=0;
		const int nMax = 100;

		const int dimension = 3;

		double aJ[nMax*2*dimension];

		double aC[nMax*2*1];

		double aJT[dimension*2*nMax];
		//CvMat *JT=NULL;
		//JT= cvCreateMat(3, 2*N, CV_64FC1);//T的转置----4*3N    2

		double aJTEMP1[dimension*dimension];
		double aJTEMP2[dimension*dimension];
		//CvMat *JTEMP1=NULL;
		//JTEMP1= cvCreateMat(3, 3, CV_64FC1); //                4

		double aJLast[3*2*nMax];
		//CvMat *Jlast=NULL;
		//Jlast= cvCreateMat(3, 2*N, CV_64FC1);    //           5

		double aDeltaX[3];
		//CvMat *deltax=NULL;
		//deltax= cvCreateMat(3, 1, CV_64FC1);//                6

		for(int t=0; t<tmax; t++) // 迭代计算
		{
			if(t==0)
			{
				w=w0;
			}

			for(int i=0; i<N; i++)
			{
				xi1=(float)matchsort1[i].x;
				zi1=(float)matchsort1[i].y;

				xi0=(float)matchsort0[i].x;
				zi0=(float)matchsort0[i].y;

				////求J(i)
				//J[i].j[0][0] = (double)(-sin(w.th)*xi0-cos(w.th)*zi0);
				//J[i].j[0][1] = 1.0;
				//J[i].j[0][2] = 0.0;
				aJ[i*6 + 0] = (-sin(w.th)*xi0-cos(w.th)*zi0);
				aJ[i*6 + 1] = 1.0;
				aJ[i*6 + 2] = 0.0;

				//J[i].j[1][0] = (double)cos(w.th)*xi0-sin(w.th)*zi0;
				//J[i].j[1][1] = 0.0;
				//J[i].j[1][2] = 1.0;
				aJ[i*6 + 3] = cos(w.th)*xi0-sin(w.th)*zi0;
				aJ[i*6 + 4] = 0.0;
				aJ[i*6 + 5] = 1.0;

				aC[i*2 + 0] = xi1-cos(w.th)*xi0+sin(w.th)*zi0-w.tx;
				aC[i*2 + 1] = zi1-sin(w.th)*xi0-cos(w.th)*zi0-w.ty;

				////求 C(i)
				//C[i].c[0] = xi1-cos(w.th)*xi0+sin(w.th)*zi0-w.tx;
				//C[i].c[1] = zi1-sin(w.th)*xi0-cos(w.th)*zi0-w.ty;
			}


			//cvTranspose(JO, JT );//JT=J'转置
			TransposeMatrix( aJ, 2*N, dimension, aJT );

			//cvGEMM(JT,JO , 1, NULL, 0, JTEMP1);//JT*J   矩阵相乘
			MulMatrix( aJT, dimension, 2*N, aJ, 2*N, dimension, aJTEMP1 );

			//float det=cvInvert(JTEMP1, JTEMP1  );// 矩阵求逆
			InverseMatrix( aJTEMP1, dimension, aJTEMP2 );

			//cvGEMM(JTEMP1, JT , 1, NULL, 0, Jlast);//Jlast---J*  相乘
			MulMatrix( aJTEMP2, dimension, dimension, aJT, dimension, 2*N, aJLast );

			//cvGEMM(Jlast, CC , 1, NULL, 0, deltax);//求delta_x
			MulMatrix( aJLast, dimension, 2*N, aC, 2*N, 1, aDeltaX );

			double dth = 0,dtx = 0,dtz = 0;
			//dth=cvmGet(deltax,0,0);
			//dtx=cvmGet(deltax,1,0);
			//dtz=cvmGet(deltax,2,0);

			dth = aDeltaX[0];
			dtx = aDeltaX[1];
			dtz = aDeltaX[2];

			//x(k)=x(k-1)+δx
			w.th=w.th+dth;
			w.tx=w.tx+dtx;
			w.ty=w.ty+dtz;

			if( (abs(dtx)<stop_x) && (abs(dtz)<stop_y) && (abs(dth)<stop_th) ){
				break;
			}
		}

		motion.th=w.th;
		motion.tx=w.tx;
		motion.ty=w.ty;

		return 1;
	}

	//非线性最小二乘法，求解 平移矩阵 和 旋转矩阵
	int NonlinearLeastSquare3D_32F(pool::SfPoint *matchsort1, pool::SfPoint *matchsort0, int N, 
					pool::Gesture3D &motion, pool::Gesture3D w0,
					float stop_x, float stop_y, float stop_th)
	{
		if(N<2)
		{
			std::cout<<"Error: nPairs must >2 in NonlinearLeastSquare()."<<std::endl;
			return 0;
		}
		if( (matchsort0==0) || (matchsort1==0) )
		{
			std::cout<<"Input pointers can not be null in NolinearLeastSquare()."<<std::endl;
			return 0;
		}

		CMemoryPool memPool;

		const int tmax = 15;
		float xi1,zi1, xi0,zi0;

		pool::Gesture3D w;
		w.th=0;
		w.tx=0;
		w.ty=0;

		//const int nMax = 10;

		const int dimension = 3;

		//float* pJ = new. float[N*2*dimension];
		float* pJ = (float*)memPool.Operate( NULL, N*2*dimension*sizeof(float) );

		//float *pC = new. float[N*2*1];
		float *pC = (float*)memPool.Operate( NULL, N*2*1*sizeof(float) );

		//float *pJT = new. float[dimension*2*N];
		float *pJT = (float*)memPool.Operate( NULL, dimension*2*N*sizeof(float) );

		float aJTEMP1[dimension*dimension];

		float aJTEMP2[dimension*dimension];

		//float *pJLast = new. float[3*2*N];
		float *pJLast = (float*)memPool.Operate( NULL, 3*2*N*sizeof(float));

		float aDeltaX[3];

		for(int t=0; t<tmax; t++) // 迭代计算
		{
			if(t==0)
			{
				w=w0;
			}

#ifdef _DEBUG
			double meanErrorDist = 0;
#endif
			for(int i=0; i<N; i++)
			{
				xi1=(float)matchsort1[i].x;
				zi1=(float)matchsort1[i].y;

				xi0=(float)matchsort0[i].x;
				zi0=(float)matchsort0[i].y;

				////求J(i)
				//J[i].j[0][0] = (double)(-sin(w.th)*xi0-cos(w.th)*zi0);
				//J[i].j[0][1] = 1.0;
				//J[i].j[0][2] = 0.0;
				pJ[i*6 + 0] = float(-sin(w.th)*xi0-cos(w.th)*zi0);
				pJ[i*6 + 1] = 1.0f;
				pJ[i*6 + 2] = 0.0f;

				//J[i].j[1][0] = (double)cos(w.th)*xi0-sin(w.th)*zi0;
				//J[i].j[1][1] = 0.0;
				//J[i].j[1][2] = 1.0;
				pJ[i*6 + 3] = float(cos(w.th)*xi0-sin(w.th)*zi0);
				pJ[i*6 + 4] = 0.0;
				pJ[i*6 + 5] = 1.0;

				pC[i*2 + 0] = float(xi1-cos(w.th)*xi0+sin(w.th)*zi0-w.tx);
				pC[i*2 + 1] = float(zi1-sin(w.th)*xi0-cos(w.th)*zi0-w.ty);

				////求 C(i)
				//C[i].c[0] = xi1-cos(w.th)*xi0+sin(w.th)*zi0-w.tx;
				//C[i].c[1] = zi1-sin(w.th)*xi0-cos(w.th)*zi0-w.ty;
#ifdef _DEBUG
				meanErrorDist += (pC[i*2 + 0] * pC[i*2 + 0] + pC[i*2 + 1] * pC[i*2 + 1]);
#endif
			}

#ifdef _DEBUG
			meanErrorDist = sqrt(meanErrorDist/N);
			cout<<"meanErrorDist "<<meanErrorDist<<endl;
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

			float dth = 0,dtx = 0,dtz = 0;

			dth = (float)aDeltaX[0];
			dtx = (float)aDeltaX[1];
			dtz = (float)aDeltaX[2];

			//x(k)=x(k-1)+δx
			w.th=w.th+dth;
			w.tx=w.tx+dtx;
			w.ty=w.ty+dtz;

			if( (abs(dtx)<stop_x) && (abs(dtz)<stop_y) && (abs(dth)<stop_th) ){
				break;
			}
		}

		motion.th=w.th;
		motion.tx=w.tx;
		motion.ty=w.ty;

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
									   double stop_sx, double stop_sy, double stop_x, double stop_y, double stop_th )
	{

		if( (N!=4)&&(N!=2))
			return 0;

		if(N==2)
		{
			pool::Gesture4D motion4D, initialMotion4D;
			initialMotion4D.sx = w0.sx;
			initialMotion4D.th = w0.th;
			initialMotion4D.tx = w0.tx;
			initialMotion4D.ty = w0.ty;
			int backValue =  NonlinearLeastSquare4D( matchsort1, matchsort0, N, 
				motion4D, initialMotion4D, 
				stop_sx, stop_x, stop_y, stop_th );

			motion.sx = motion4D.sx;
			motion.sy = motion4D.sx;
			motion.th = motion4D.th;
			motion.tx = motion4D.tx;
			motion.ty = motion4D.ty;

			return backValue;
		}

		int aSam[5][4] = { 0,1,2,0, 
			0,1,3,0,
			0,2,3,0,
			1,2,3,0,
			0,1,2,3 };

		int aSamNum[5] = {3,3,3,3,4};
		pool::Gesture5D aMotion[5];

		double dfMinMeanDistSquare = 9999999999;
		pool::DfPoint point1[4], point0[4];
		for(int s=0; s<5; s++)
		{
			for(int i=0; i<aSamNum[s]; i++)
			{
				int index = aSam[s][i];
				point0[i] = matchsort0[index];
				point1[i] = matchsort1[index];
			}

			int backValue = NonlinearLeastSquare5D( point1, point0, aSamNum[s], 
				aMotion[s], w0, stop_sx, stop_sy, stop_x, stop_y, stop_th );
			if(backValue==1)
			{
				double dfMeanDistSquare = 0;
				for(int i=0; i<aSamNum[s]; i++)
				{
					double dfDistSquare = 0;
					pool::DfPoint point1Estimate;
					Apply5DGesture( point0[i], aMotion[s], point1Estimate );
					//DfPoint errorEstimate;
					DistanceSquareOfTwoPoints( point1Estimate.x, point1Estimate.y,
						point1[i].x, point1[i].y, dfDistSquare );
					dfMeanDistSquare += dfDistSquare;
				}
				dfMeanDistSquare /= aSamNum[s];
				if(dfMeanDistSquare<dfMinMeanDistSquare)
				{
					motion = aMotion[s];
					dfMinMeanDistSquare = dfMeanDistSquare;
				}
			}
		}


		return 1;
	}


	// 函数功能：非线性最小二乘法，求解 平移矩阵 和 旋转矩阵，以及缩放矩阵
	// Without CV
	// 成功返回1
	int NonlinearLeastSquare5D(pool::DfPoint* pMatchPoint1, pool::DfPoint* pMatchPoint0, int N, 
							   pool::Gesture5D &motion, 
							   pool::Gesture5D w0,
							   double stop_sx, double stop_sy, double stop_x, double stop_y, double stop_th )
	{
		if(N<3)
		{
			std::cout<<"Error: nPairs must >3 in NonlinearLeastSquare(). scaleX, scaleY"<<std::endl;
			return 0;
		}
		if( (pMatchPoint0==NULL) || (pMatchPoint1==NULL) )
		{
			std::cout<<"Input pointers can not be null in NolinearLeastSquare()."<<std::endl;
			return 0;
		}

		int tMax = 15; // 最大迭代次数
		double xi1, yi1, xi0, yi0;

		pool::Gesture5D w;

		const int nMax = 20;

		double aJ[nMax*2*5];
		double aC[nMax*2*1];

		double aJTEMP1[5*5];
		double aJTEMP2[5*5];

		double aJT[5*2*nMax];

		double aJLast[5*2*nMax];

		double aDeltaX[5];

		for(int t=0; t<tMax; t++) // 迭代计算
		{
			if(t==0)
			{
				w=w0;
			}

			double dfCosA = cos(w.th);
			double dfSinA = sin(w.th);
			for(int i=0; i<N; i++)
			{
				xi1 = pMatchPoint1[i].x;
				yi1 = pMatchPoint1[i].y;

				xi0 = pMatchPoint0[i].x;
				yi0 = pMatchPoint0[i].y;

				// 
				aJ[i*10 + 0] = xi0*dfCosA;
				aJ[i*10 + 1] = -yi0*dfSinA;
				aJ[i*10 + 2] = 1.0;
				aJ[i*10 + 3] = 0.0;
				aJ[i*10 + 4] = -w.sx*xi0*dfSinA-w.sy*yi0*dfCosA;

				aJ[i*10 + 5] = xi0*dfSinA;
				aJ[i*10 + 6] = yi0*dfCosA;
				aJ[i*10 + 7] = 0;
				aJ[i*10 + 8] = 1.0;
				//aJ[i*10 + 9] = w.sx*xi0 - w.sy*yi0*dfSinA; // 08-01修改之前
				aJ[i*10 + 9] = w.sx*xi0*dfCosA - w.sy*yi0*dfSinA; // 2011-08-01修改

				aC[i*2 + 0] = xi1 - (w.sx*xi0*dfCosA - w.sy*yi0*dfSinA + w.tx);
				aC[i*2 + 1] = yi1 - (w.sx*xi0*dfSinA + w.sy*yi0*dfCosA + w.ty);
			}


			//cvTranspose( JO, JT );	//JT=J'转置
			TransposeMatrix( aJ, 2*N, 5, aJT );

			//cvGEMM(JT,JO , 1, NULL, 0, JTEMP1);	//JT*J   矩阵相乘
			MulMatrix( aJT, 5, 2*N, aJ, 2*N, 5, aJTEMP1 );

			//double det = cvInvert(JTEMP1, JTEMP1  );	// 矩阵求逆
			InverseMatrix( aJTEMP1, 5, aJTEMP2, 1e-12 );

			//cvGEMM(JTEMP1, JT , 1, NULL, 0, Jlast);	//Jlast---J*  相乘
			MulMatrix( aJTEMP2, 5, 5, aJT, 5, 2*N, aJLast );

			//cvGEMM(Jlast, CC , 1, NULL, 0, deltax);	//求delta_x
			MulMatrix( aJLast, 5, 2*N, aC, 2*N, 1, aDeltaX );

			double d_sx = 0, d_sy=0, dtx=0, dty=0, dth=0;

			d_sx = aDeltaX[0];
			d_sy = aDeltaX[1];
			dtx = aDeltaX[2];
			dty = aDeltaX[3];
			dth = aDeltaX[4];

			// CV_LU - 最佳主元选取的高斯消除法 
			// CV_SVD - 奇异值分解法 (SVD) 
			// CV_SVD_SYM - 对正定对称矩阵的 SVD 方法 
			//cvSolve(A, Y,K , CV_SVD );
			//cvSolve(JO, CC,deltax , CV_SVD );

			//x(k)=x(k-1)+δx
			w.sx = w.sx + d_sx;
			w.sy = w.sy + d_sy;
			w.tx = w.tx + dtx;
			w.ty = w.ty + dty;
			w.th = w.th + dth;

			if( (abs(d_sx)<stop_sx) && 
				(abs(d_sy)<stop_sy) && 
				(abs(dtx)<stop_x) && 
				(abs(dty)<stop_y) && 
				(abs(dth)<stop_th) )
			{
				break;
			}

		}

		motion.sx = w.sx;
		motion.sy = w.sy;
		motion.th = w.th;
		motion.tx = w.tx;
		motion.ty = w.ty;

		return 1;
	}

	// 非最小二乘法拟合直线，极坐标表示【未测试, 2011-01-11】
	int FitLineNonLeastSquare(float A[], float B[], int N, PolarLine &linepa, PolarLine w0)
	{
		if(N<2)
		{
			std::cout<<"Error: 2 points are need at least to fit a line.◎"<<std::endl;
			return 0;
		}

		CMemoryPool memPool;
	
		float acc2=0;

		int tmax=30; // 迭代次数
		float x,y;

		const int DIM = 2;

		// 线性方程: JX = C
		int rowJ = N, colJ = DIM;
		//float *pJ = new. float[rowJ*colJ];
		float *pJ = (float*)memPool.Operate( NULL, rowJ*colJ*sizeof(float));

		int rowC = N, colC = 1;
		//float *pC = new. float[rowC*colC];
		float *pC = (float*)memPool.Operate( NULL, rowC*colC*sizeof(float));

		float aX[2];

		PolarLine w;

		// 迭代
		for(int t=0; t<tmax; t++)
		{
			if(t==0)
			{
				// 初值
				w=w0;
			}
			acc2 = 0;
			float sina = 0, cosa = 0;
			sina = sin(w.th);
			cosa = cos(w.th);
			for(int i=0; i<N; i++)
			{
				x=A[i];
				y=B[i];

				//求J(i)
				//J[i].j[0][0]=1;
				//J[i].j[0][1]=float(x*sina-y*cosa);
				*(pJ + i*colJ + 0) = 1;
				*(pJ + i*colJ + 1) = float(x*sina-y*cosa);

				//求 C(i)
				//C[i].c[0]=float(w.p-x*cosa-y*sina);
				*(pC + i*colC + 0) = float(w.p-x*cosa-y*sina);
			}

			// 解线性方程
			SolveLinearLeastSquare( pJ, rowJ, colJ,
				pC, rowC, colC, 
				aX, 2, 1);

			float dth = 0, dp = 0;
			dp = aX[0];
			dth = aX[1];

			//x(k)=x(k-1)+δx
			w.th = w.th-dth;
			w.p = w.p-dp;
		}

		linepa.th = w.th; /**/
		linepa.p = w.p;

		//delete.[] pJ; pJ = NULL;
		memPool.Operate(pJ); pJ = NULL;
		//delete.[] pC; pC = NULL;
		memPool.Operate(pC); pC = NULL;

		return 1;
	}