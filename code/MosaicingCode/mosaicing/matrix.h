// 矩阵运算
#pragma once

//#include "ati.h"
#include "Point.h"
#include <vector>
#include <algorithm>
#include "math.h"
#include "MemoryPool.h"
#include "mvMath.h"

namespace pool
{
	struct Distance
	{
		float dist;
		int seq;

		bool   operator <  (const Distance& rhs)  const   //升序排序时必须写的函数
		{   
			return dist<rhs.dist; 
		}
	};
	
	struct SfMatrix
	{
		float* pMat;
		int row;
		int col;
	};


	struct DfMatrix
	{
		double* pMat;
		int row;
		int col;
	};


	struct IntMatrix
	{
		int* pMat;
		int row;
		int col;
	};

} // 命名空间结束

	// 函数功能：矩阵加法
	template <class T>
	int AddMatrix(T* pSrc1, int row1, int col1, 
		T* pSrc2, T* pDst)
	{
		if( (pSrc1==NULL) || (pSrc2==NULL) || (pDst==NULL) )
			return -1;

		int matSize = row1 * col1;
		for(int n=0; n<matSize; n++)
		{
			pDst[n] = pSrc1[n] + pSrc2[n];
		}

		return 0;
	}

	// Matrix Transpose
	// 矩阵转置   
	// 输入矩阵 row行col列， 转置矩阵 col行row列
	template <class T>
	int TransposeMatrix(T* pSrc, int row, int col,
						T* pDst)
	{
		if( ( pSrc==NULL ) || (pDst==NULL) )
			return 0;

		if( ( row<1 ) || (col<1) )
			return 0;

		int col2 = row;
		for(int r=0; r<row; r++)
		{
			for(int c=0; c<col; c++)
			{
				pDst[c*col2 + r] = pSrc[r*col + c];
			}
		}

		return 1;
	}

	// Matrix Multiplication
	// 相乘结果的矩阵大小为  row1---col2  
	template <class T>
	int MulMatrix(T* pSrc1, int row1, int col1, 
				  T* pSrc2, int row2, int col2, 
				  T* pDst )
	{
		if(col1!=row2)
			return -1;
		if( ( pSrc1==NULL ) || (pSrc2==NULL) || (pDst==NULL) )
			return -1;
		if( (pSrc1==pDst) || (pSrc2==pDst) )
			return -1;
		
		for(int r1 = 0; r1<row1; r1++)
		{
			for(int c2=0; c2<col2; c2++)
			{
				T dfAcc = 0;
				for(int r2=0; r2<row2; r2++) // row2==col1
				{
					dfAcc += pSrc1[r1*col1 + r2] * pSrc2[r2*col2 + c2];
				}
				pDst[r1*col2 + c2] = dfAcc;
			}
		}
		
		return 1;
	}

	// 相乘结果的矩阵大小为  row1---col2  
	template <class T1, class T2>
	int MulMatrix2(T1* pSrc, int row, int col, 
		T2 a, 
		T1* pDst )
	{
		for(int r = 0; r<row; r++)
		{
			for(int c=0; c<col; c++)
			{
				pDst[r*col + c] = pSrc[r*col + c] * a;
			}
		}

		return 0;
	}

	// 功能：矩阵求逆，方阵尺寸要小于10*10
	// m00 m01
	// m10 m11
	// ----------------------------
	// m00 m01 m10 m11
	// ----------------------------
	// pSrc指向1D数组，矩阵按行排列
	// pDst指向1D数据，矩阵按行排列
	template <class T>
	int InverseMatrix( const T *pSrc, const int order, 
					  T* pDst,
					  const T SMALL_NUMBER = 1e-6, int out = 0)
	{
		if( (order>13) || (order<2) )
			return -1;

		T pTemp[400];// 增广矩阵
		memset( pTemp, 0, order*order*2*sizeof(T) );
		int order2 = order*2;

		// 
		pool::Distance aFlag[169];

		for(int i=0; i<order; i++)
		{
			aFlag[i].dist = (float)i;
			aFlag[i].seq = 0;
			*( pTemp + i*order2 + order + i ) = 1;
			for(int j=0; j<order; j++)
			{
				*(pTemp + i*order2 + j) = *(pSrc + i*order + j);
			}
		}
		
		for(int i=0; i<order; i++)//列
		{
			T ei = 0;
			int rowI = 0;
			for(int j=0; j<order; j++) // 行
			{
				if(aFlag[j].seq==1)
				{
					continue;
				}
				if( abs(*(pTemp + j*order2 + i))>SMALL_NUMBER )
				{
					aFlag[j].seq = 1;
					ei = *(pTemp + j*order2 + i);
					rowI = j; // 该行的第i列元素不为0
					break;
				}
			}
			if(abs(ei)<SMALL_NUMBER)
				return 0;

			for(int c = 0; c<order2; c++)
			{
				*(pTemp + rowI*order2 + c) /= ei;
			}

			for(int j=0; j<order; j++)//行
			{
				if( j==rowI )
					continue;
				if( abs(*(pTemp + j*order2 + i))<SMALL_NUMBER )// 如果已经为0了
					continue;

				T ei2 = *(pTemp + j*order2 + i);

				for(int c = 0; c<order2; c++)
				{
					*(pTemp + j*order2 + c) += -ei2 * *(pTemp + rowI*order2 + c);
				}
				
			}
		}

		//for(int i=0; i<order; i++)
		//{
		//	for(int j=0; j<order; j++)
		//	{
		//		if( *(pTemp + i*order2 + j)==1 )
		//		{
		//			aFlag[i].dist = (float)j;
		//			aFlag[i].seq = i;
		//			break;
		//		}
		//	}
		//}

		// 注意, 某一行, 有可能要交换2次
		if(out)
		{
			//for(int i=0; i<order; i++)
			//{
			//	for(int j=0; j<order2; j++) // 列
			//	{
			//		cout<<*(pTemp + i*order2 + j)<<" ";
			//	}
			//	cout<<endl;
			//}
		}

		for(int r=0; r<order; r++)
		{
			int targetCol = -1;
			int targetRow = -1;
			// 搜索非零项
			for(int i=0; i<order; i++)
			{
				for(int j=0; j<order; j++) // 列
				{
					if( *(pTemp + i*order2 + j)==1 )
					{
						if(j==r)
						{
							targetCol = j;
							targetRow = i;
							i = j = order;
							break;
						}
					}
				}
			}

			if(out)
			{
				cout<<"r "<<r<<" targetRow "<<targetRow<<endl;
			}
			if(targetRow>=0)
			{
				// 交换第r行和第targetRow行
				if(r!=targetRow)
				{
					for(int j=0; j<order2; j++)
					{
						swap(*(pTemp + r*order2 + j), *(pTemp + targetRow*order2 + j));
					}
				}
			}
		}

		// 增广矩阵的左半部分就是逆矩阵
		for(int i=0; i<order; i++)
		{
			//int row = (int)aFlag[i].dist;
			//T* pDstRow = pDst + i*order;
			//T* pTempRow = pTemp + row*order2 + order;
			for(int j=0; j<order; j++)
			{
				*(pDst + i*order + j) = *(pTemp + i*order2 + order +j);
				//*(pDst + i*order + j) = *(pTemp + row*order2 + order +j);
				//*(pDstRow + j) = *(pTempRow +j);
			}
		}
		
		return 1;
	}

	// 3x3矩阵求逆
	template <class T>
	int InverseMatrix33(T *m, T *invA)
	{
		// 
		//invA = //[m(5)*m(9) - m(8)*m(6)   - (m(2)*m(9) - m(8)*m(3))       m(2)*m(6) - m(3)*m(5)
			//m(6)*m(7) - m(9)*m(4)          - (m(3)*m(7) - m(9)*m(1))       m(3)*m(4) - m(1)*m(6)
			//m(4)*m(8) - m(7)*m(5)          - (m(1)*m(8) - m(7)*m(2))       m(1)*m(5) - m(2)*m(4)];

		invA[0] = m[4]*m[8] - m[7]*m[5];	
		invA[1] = -(m[1]*m[8] - m[7]*m[2]);	
		invA[2] = m[1]*m[5] - m[2]*m[4];
		invA[3] = m[5]*m[6] - m[8]*m[3];	invA[4] = -(m[2]*m[6] - m[8]*m[0]);	invA[5] = m[2]*m[3] - m[0]*m[5];
		invA[6] = m[3]*m[7] - m[6]*m[4];	invA[7] = -(m[0]*m[7] - m[6]*m[1]);	invA[8] = m[0]*m[4] - m[1]*m[3];

		//detm = m[1]*m[5]*m[9] + m[2]*m[6]*m[7] + m[3]*m[4]*m[8] - m[3]*m[5]*m[7] - m[1]*m[6]*m[8] - m[2]*m[4]*m[9];

		T detm = m[0]*(m[4]*m[8] - m[5]*m[7]) + m[1]*(m[5]*m[6] - m[3]*m[8]) + m[2]*(m[3]*m[7] - m[4]*m[6]);

		T inv_detm = 0;
		if (detm!=0)
		{
			inv_detm = 1 / detm;
		}
		for (int i = 0; i < 9; i++)
		{
			invA[i] *= inv_detm;
		}

		//invA = 1 / det(m)*invA';

		return 0;
	}

	// AX = B
	// 线性最小二乘解方程组
	template<class T1>
	bool SolveLinearLeastSquare2(T1* pMatA, int rowA, int colA, 
								T1* pMatB, int rowB, int colB,
								T1* pX, int rowX, int colX )
	{
		if ((pMatA == NULL) || (pMatB == NULL) || (pX == NULL))
		{
			return false;
		}

		int rowAT = colA;
		int colAT = rowA;

		T1 *pMatAT = new T1[rowAT*colAT];

		T1* pInv_AT = new T1[colA*rowA];

		// A转置  2*pairsNum(rowA)   8 (colA)
		TransposeMatrix( pMatA, rowA, colA, pMatAT);

		// AT*A
		T1 aATA[20*20] = {0}; // 最大方阵<20
		MulMatrix( pMatAT, rowAT, colAT,
			pMatA, rowA, colA,
			aATA);

		T1 aATA_Inv[20*20] = {0};
		//if (colA == 3)
		//{
		//	InverseMatrix(aATA, colA, aATA_Inv, T1(1e-20));
		//	InverseMatrix33(aATA, aATA_Inv); // 求逆
		//	//for (int r = 0; r < 3; r++)
		//	//{
		//	//	for (int c = 0; c < 3; c++)
		//	//	{
		//	//		cout << aATA_Inv[r * 3 + c] << " ";
		//	//	}
		//	//}
		//	//cout << endl;
		//}
		//else
		{
			//InverseMatrixCv64(aATA, 3, aATA_Inv);
			InverseMatrix(aATA, colA, aATA_Inv, T1(1e-20)); // 求逆
			//if (colA == 3)
			//{
			//	for (int r = 0; r < 3; r++)
			//	{
			//		for (int c = 0; c < 3; c++)
			//		{
			//			cout << aATA_Inv[r * 3 + c] << " ";
			//		}
			//	}
			//	cout << endl << ".." << endl;
			//}
		}
		
		MulMatrix( aATA_Inv, colA, colA, 
			pMatAT, rowAT, colAT, 
			pInv_AT ); // inv(A'A) * A'

		MulMatrix( pInv_AT, colA, rowA, 
			pMatB, rowB, colB,
			pX); // inv(A'A) * A'* B

		delete[] pMatAT;
		delete[] pInv_AT;

		return true;
	}

	// AX = B
	// 线性最小二乘解方程组
	template<class T1>
	bool SolveLinearLeastSquare3(T1* pMatA, int rowA, int colA, 
		T1 lamada,
		T1* pMatB, int rowB, int colB,
		T1* pX, int rowX, int colX )
	{
		if( (pMatA==NULL) || (pMatB==NULL) || (pX==NULL) )
			return false;

		int rowAT = colA;
		int colAT = rowA;

		T1 *pMatAT = new T1[rowAT*colAT];

		T1* pInv_AT = new T1[colA*rowA];

		// A转置  2*pairsNum(rowA)   8 (colA)
		TransposeMatrix( pMatA, rowA, colA, pMatAT);

		// AT*A
		T1 aATA[20*20] = {0}; // 最大方阵<20
		MulMatrix( pMatAT, rowAT, colAT,
			pMatA, rowA, colA,
			aATA);

		for(int r=0; r<colA; r++)
		{
			aATA[r*colA + r] += lamada; // LM算法
		}

		T1 aATA_Inv[20*20] = {0};
		InverseMatrix( aATA, colA, aATA_Inv, T1(1e-20) ); // 求逆

		MulMatrix( aATA_Inv, colA, colA, 
			pMatAT, rowAT, colAT, 
			pInv_AT ); // inv(A'A) * A'

		MulMatrix( pInv_AT, colA, rowA, 
			pMatB, rowB, colB,
			pX); // inv(A'A) * A'* B

		delete[] pMatAT;
		delete[] pInv_AT;

		return true;
	}

	// AX = B
	// 线性最小二乘解方程组
	template<class T1>
	bool SolveLinearLeastSquare(T1* pMatA, int rowA, int colA, 
								T1* pMatB, int rowB, int colB,
								T1* pX, int rowX, int colX )
	{
		if( (pMatA==NULL) || (pMatB==NULL) || (pX==NULL) )
			return false;

		int rowAT = colA;
		int colAT = rowA;

		CMemoryPool memPool;

		//float *pMatAT = new. float[rowAT*colAT];
		T1 *pMatAT = NULL;
		pMatAT = (T1*)memPool.Operate( NULL, rowAT*colAT*sizeof(T1) ); // A的转置

		//memset(pMatAT, 0, rowAT*colAT*sizeof(T1));

		//float* pInv_AT = new. float[colA*rowA];
		T1* pInv_AT = (T1*)memPool.Operate( NULL, colA * rowA * sizeof(T1) ); // inv(A'A)A'

		// A转置  2*pairsNum(rowA)   8 (colA)
		TransposeMatrix( pMatA, rowA, colA, pMatAT);

		// AT*A
		T1 aATA[20*20] = {0}; // 最大方阵<20
		MulMatrix( pMatAT, rowAT, colAT,
			pMatA, rowA, colA,
			aATA);

		T1 aATA_Inv[20*20] = {0};
		InverseMatrix( aATA, colA, aATA_Inv, T1(1e-20) ); // 求逆

		MulMatrix( aATA_Inv, colA, colA, 
			pMatAT, rowAT, colAT, 
			pInv_AT ); // inv(A'A) * A'

		MulMatrix( pInv_AT, colA, rowA, 
			pMatB, rowB, colB,
			pX); // inv(A'A) * A'* B

		memPool.Operate( pMatAT ); pMatAT = NULL;
		memPool.Operate( pInv_AT ); pInv_AT = NULL;

		return true;
	}

	// LM方法
	template<class T1, class T2>
	bool LM_Method(T1* pMatA, int rowA, int colA, 
		T1* pMatB, int rowB, int colB,
		T1* pX, int rowX, int colX,
		T2 lamata)
	{
		if( (pMatA==NULL) || (pMatB==NULL) || (pX==NULL) )
			return false;
		if(lamata<0)
			return false;

		const int MAX_W = 20;

		int rowAT = colA;
		int colAT = rowA;

		CMemoryPool memPool;

		//float *pMatAT = new. float[rowAT*colAT];
		T1 *pMatAT = (T1*)memPool.Operate( NULL, rowAT*colAT*sizeof(T1) );

		memset(pMatAT, 0, rowAT*colAT*sizeof(float));

		//float* pInv_AT = new. float[colA*rowA];
		T1* pInv_AT = (T1*)memPool.Operate( NULL, colA*rowA*sizeof(T1) ); 

		// A转置  2*pairsNum(rowA)   8 (colA)
		TransposeMatrix( pMatA, rowA, colA, pMatAT);

		// AT*A
		T1 aATA[MAX_W*MAX_W] = {0}; // 最大方阵<20
		MulMatrix( pMatAT, rowAT, colAT,
			pMatA, rowA, colA,
			aATA);

		for(int i=0; i<colA; i++)
		{
			*(aATA + i*colA + i) += lamata; // lm方法
		}

		T1 aATA_Inv[MAX_W*MAX_W] = {0};
		InverseMatrix( aATA, colA, aATA_Inv, T1(1e-20) ); // 求逆

		MulMatrix( aATA_Inv, colA, colA, 
			pMatAT, rowAT, colAT, 
			pInv_AT );

		MulMatrix( pInv_AT, colA, rowA, 
			pMatB, rowB, colB,
			pX);

		//delete.[] pMatAT; pMatAT = NULL;
		memPool.Operate( pMatAT ); pMatAT = NULL;
		//delete.[] pInv_AT; pInv_AT = NULL;
		memPool.Operate( pInv_AT ); pInv_AT = NULL;

		return true;
	}

	// 求仿射运动参数(6个未知数)
	// maxError-----最大距离误差
	template<class PointType, class T1>
	bool SolveAffineMotion( PointType *vecPoints1,
		PointType *vecPoints2, int pairsNum,
		T1 aAffineMat[6],
		T1 &maxError)
	{
		if(pairsNum<3)
		{
			return false;
		}

		T1 aAffineMat2[6];

		int rowA = 2*pairsNum;
		int colA = 6;

		int rowAT = colA;
		int colAT = rowA;

		int rowB = 2*pairsNum;
		int colB = 1;

		int rowX = 6;
		int colX = 1;

		//CMemoryPool memPool;

		T1 *pMatA = new T1[rowA*colA];
		//T1 *pMatA = (T1*)memPool.Operate(NULL, rowA*colA*sizeof(T1));
		memset(pMatA, 0, rowA*colA*sizeof(T1));

		T1 *pMatB = new T1[rowB*colB];
		//T1 *pMatB = (T1*)memPool.Operate(NULL, rowB*colB*sizeof(T1));
		memset(pMatB, 0, rowB*colB*sizeof(T1));

		T1 *pMatAT = new T1[rowAT*colAT];
		//T1 *pMatAT = (T1*)memPool.Operate(NULL, rowAT*colAT*sizeof(T1));
		memset(pMatAT, 0, rowAT*colAT*sizeof(T1));

		// 按行填充A，B
		for(int row=0; row<pairsNum; row++)
		{
			// Y
			pMatA[ 2*row*6 + 0 ] = vecPoints2[row].x;
			pMatA[ 2*row*6 + 1 ] = vecPoints2[row].y;
			pMatA[ 2*row*6 + 4 ] = 1;

			pMatA[ (2*row+1)*6 + 2 ] = vecPoints2[row].x;
			pMatA[ (2*row+1)*6 + 3 ] = vecPoints2[row].y;
			pMatA[ (2*row+1)*6 + 5 ] = 1;

			pMatB[2*row] = vecPoints1[row].x;
			pMatB[2*row+1] = vecPoints1[row].y;
		}

		SolveLinearLeastSquare2( pMatA, rowA, colA, 
			pMatB, rowB, colB,
			aAffineMat2, rowX, colX );

		for(int i=0; i<6; i++)
		{
			aAffineMat[i] = (T1)aAffineMat2[i];
		}

		// 计算误差的均方根
		float sumError2 = 0, maxError2 = 0;
		for(int n=0; n<pairsNum; n++)
		{
			T1 x1Bar = 0, y1Bar = 0;
			ApplyAffineMat2(vecPoints2[n].x, vecPoints2[n].y, x1Bar, y1Bar, aAffineMat);
			T1 error2 = (x1Bar-vecPoints1[n].x)*(x1Bar-vecPoints1[n].x) + (y1Bar-vecPoints1[n].y)*(y1Bar-vecPoints1[n].y);
			sumError2 += error2;
			if(error2>maxError2)
			{
				maxError2 = error2;
			}
		}
		float meanError = sqrt(sumError2/pairsNum);
		maxError = sqrt(maxError2);

		//memPool.Operate(pMatA);
		//memPool.Operate(pMatB);
		//memPool.Operate(pMatAT);
		delete[] pMatA;
		delete[] pMatB;
		delete[] pMatAT;

		return true;
	}

	// 求仿射运动参数(6个未知数, 但写成单应矩阵的形式)
	template<class PointType, class T1>
	bool SolveAffineMotion9( PointType *pPoints1,
		PointType *pPoints2, int pairsNum,
		T1 affineMat9[9])
	{
		if( (NULL==pPoints1) || (pPoints2==NULL) || (pairsNum<3))
			return false;

		T1 affineMat6[6], maxDistError;
		SolveAffineMotion(pPoints1, pPoints2, pairsNum, affineMat6, maxDistError);
		memcpy( affineMat9, affineMat6, sizeof(affineMat6) );
		affineMat9[0] = affineMat6[0];
		affineMat9[1] = affineMat6[1];
		affineMat9[2] = affineMat6[4];
		affineMat9[3] = affineMat6[2];
		affineMat9[4] = affineMat6[3];
		affineMat9[5] = affineMat6[5];
		affineMat9[6] = 0; affineMat9[7] = 0; affineMat9[8] = maxDistError;
		return true;
	}

	// 解投影矩阵(8个未知数，实际解的时候是8个)
	// 方法：线性最小二乘
	// point2---->point1的变换
	template<class T1>
	bool SolveProjectMatrix3( pool::SfPoint *vecPoints1,
		pool::SfPoint *vecPoints2, int pairsNum,
		T1 aProjectMat[9])

	{
		if(pairsNum<4)
		{
			return false;
		}

		float aProjectMat2[9];

		int rowA = 2*pairsNum;
		int colA = 8;

		int rowAT = 8;
		int colAT = 2*pairsNum;

		int rowB = 2*pairsNum;
		int colB = 1;

		int rowX = 8;
		int colX = 1;

		CMemoryPool memPool;

		//float *pMatA = new. float[rowA*colA];
		float *pMatA = (float*)memPool.Operate( NULL, rowA*colA*sizeof(float));
		memset(pMatA, 0, rowA*colA*sizeof(float));

		//float *pMatB = new. float[rowB*colB];
		float *pMatB = (float*)memPool.Operate( NULL, rowB*colB*sizeof(float));
		memset(pMatB, 0, rowB*colB*sizeof(float));

		//float *pMatAT = new. float[rowAT*colAT];
		float *pMatAT = (float*)memPool.Operate( NULL, rowAT*colAT*sizeof(float));
		memset(pMatAT, 0, rowAT*colAT*sizeof(float));

		// 按行填充A，B
		for(int row=0; row<pairsNum; row++)
		{
			// Y
			pMatA[ 2*row*8 + 0 ] = vecPoints2[row].x;
			pMatA[ 2*row*8 + 1 ] = vecPoints2[row].y;
			pMatA[ 2*row*8 + 2 ] = 1;
			pMatA[ 2*row*8 + 6 ] = -vecPoints1[row].x*vecPoints2[row].x;
			pMatA[ 2*row*8 + 7 ] = -vecPoints1[row].x*vecPoints2[row].y;

			pMatA[ (2*row+1)*8 + 3 ] = vecPoints2[row].x;
			pMatA[ (2*row+1)*8 + 4 ] = vecPoints2[row].y;
			pMatA[ (2*row+1)*8 + 5 ] = 1;
			pMatA[ (2*row+1)*8 + 6 ] = -vecPoints1[row].y*vecPoints2[row].x;
			pMatA[ (2*row+1)*8 + 7 ] = -vecPoints1[row].y*vecPoints2[row].y;

			pMatB[2*row] = vecPoints1[row].x;
			pMatB[2*row+1] = vecPoints1[row].y;
		}

		SolveLinearLeastSquare( pMatA, rowA, colA, 
			pMatB, rowB, colB,
			aProjectMat2, rowX, colX );

		for(int i=0; i<9; i++)
		{
			aProjectMat[i] = (T1)aProjectMat2[i];
		}

#ifdef _DEBUG
#endif
		// 计算均方差
		float errorDist = 0;
		float errorDistMax = 0;
		for(int i=0; i<pairsNum; i++)
		{
			T1 xDst, yDst = 0;
			ApplyProjectMat2( vecPoints2[i].x, vecPoints2[i].y, xDst, yDst, aProjectMat );
			float dist = 0;
			DistanceOfTwoPoints( vecPoints1[i].x, vecPoints1[i].y, xDst, yDst, dist );
			errorDist = dist * dist;
			if(errorDist>errorDistMax)
			{
				errorDistMax = errorDist;
			}
		}

		aProjectMat[8] = errorDistMax;

		//errorDist /= pairsNum;
		//errorDist = sqrt( errorDist );
		//cout<<"errorDistMax "<<errorDistMax<<endl;

		memPool.Operate(pMatA);
		memPool.Operate(pMatB);
		memPool.Operate(pMatAT);

		return true;
	}

	// 解投影矩阵(8个未知数，实际解的时候是8个)
	// 方法：线性最小二乘
	// point2---->point1的变换
	template<class POINTTYPE, class T1>
	bool SolveHomographyMatrix( POINTTYPE *vecPoints1, POINTTYPE *vecPoints2, 
								int pairsNum,
								T1 aProjectMat[9])
	{
		if(pairsNum<4)
		{
			return false;
		}

		T1 aProjectMat2[9];

		int rowA = 2*pairsNum;
		int colA = 8;

		int rowAT = 8;
		int colAT = 2*pairsNum;

		int rowB = 2*pairsNum;
		int colB = 1;

		int rowX = 8;
		int colX = 1;

		T1 *pMatA = new T1[rowA*colA];
		memset(pMatA, 0, rowA*colA*sizeof(T1));

		T1 *pMatB = new T1[rowB*colB];
		memset(pMatB, 0, rowB*colB*sizeof(T1));

		T1 *pMatAT = new T1[rowAT*colAT];
		memset(pMatAT, 0, rowAT*colAT*sizeof(T1));

		// 按行填充A，B
		for(int row=0; row<pairsNum; row++)
		{
			// Y
			pMatA[ 2*row*8 + 0 ] = vecPoints2[row].x;
			pMatA[ 2*row*8 + 1 ] = vecPoints2[row].y;
			pMatA[ 2*row*8 + 2 ] = 1;
			pMatA[ 2*row*8 + 6 ] = -vecPoints1[row].x*vecPoints2[row].x;
			pMatA[ 2*row*8 + 7 ] = -vecPoints1[row].x*vecPoints2[row].y;

			pMatA[ (2*row+1)*8 + 3 ] = vecPoints2[row].x;
			pMatA[ (2*row+1)*8 + 4 ] = vecPoints2[row].y;
			pMatA[ (2*row+1)*8 + 5 ] = 1;
			pMatA[ (2*row+1)*8 + 6 ] = -vecPoints1[row].y*vecPoints2[row].x;
			pMatA[ (2*row+1)*8 + 7 ] = -vecPoints1[row].y*vecPoints2[row].y;

			pMatB[2*row] = vecPoints1[row].x;
			pMatB[2*row+1] = vecPoints1[row].y;
		}

		SolveLinearLeastSquare2( pMatA, rowA, colA, 
			pMatB, rowB, colB,
			aProjectMat2, rowX, colX );

		for(int i=0; i<9; i++)
		{
			aProjectMat[i] = (T1)aProjectMat2[i];
		}

#ifdef _DEBUG
#endif
		// 计算均方差
		double errorDist = 0;
		double errorDistMax = 0;
		double MSE = 0;
		for(int i=0; i<pairsNum; i++)
		{
			double xDst = 0, yDst = 0;
			ApplyProjectMat2( vecPoints2[i].x, vecPoints2[i].y, xDst, yDst, aProjectMat );
			double dist = 0;
			DistanceOfTwoPoints( vecPoints1[i].x, vecPoints1[i].y, xDst, yDst, dist );
			errorDist = dist;
			if(errorDist>errorDistMax)
			{
				errorDistMax = errorDist;
			}
			MSE += errorDist * errorDist;
		}
		MSE = sqrt(MSE/pairsNum);

		aProjectMat[8] = (T1)errorDistMax;

		//errorDist /= pairsNum;
		//errorDist = sqrt( errorDist );
		//cout<<"errorDistMax "<<errorDistMax<<endl;

		delete[] pMatA;
		delete[] pMatB;
		delete[] pMatAT;

		return true;
	}

	// 解投影矩阵(8个未知数，实际解的时候是8个)
	// 方法：线性最小二乘
	// point2---->point1的变换
	template<class POINTTYPE, class T1>
	bool SolveProjectMatrix2( POINTTYPE *vecPoints1,
		POINTTYPE *vecPoints2, int pairsNum,
		T1 aProjectMat[9])
	{
		if(pairsNum<4)
		{
			return false;
		}

		T1 aProjectMat2[9];

		int rowA = 2*pairsNum;
		int colA = 8;

		int rowAT = 8;
		int colAT = 2*pairsNum;

		int rowB = 2*pairsNum;
		int colB = 1;

		int rowX = 8;
		int colX = 1;

		CMemoryPool memPool;

		T1 *pMatA = (T1*)memPool.Operate(NULL, rowA*colA*sizeof(T1));
		memset(pMatA, 0, rowA*colA*sizeof(T1));

		T1 *pMatB = (T1*)memPool.Operate(NULL, rowB*colB*sizeof(T1));
		memset(pMatB, 0, rowB*colB*sizeof(T1));

		T1 *pMatAT = (T1*)memPool.Operate(NULL, rowAT*colAT*sizeof(T1));
		memset(pMatAT, 0, rowAT*colAT*sizeof(T1));

		// 按行填充A，B
		for(int row=0; row<pairsNum; row++)
		{
			// Y
			pMatA[ 2*row*8 + 0 ] = vecPoints2[row].x;
			pMatA[ 2*row*8 + 1 ] = vecPoints2[row].y;
			pMatA[ 2*row*8 + 2 ] = 1;
			pMatA[ 2*row*8 + 6 ] = -vecPoints1[row].x*vecPoints2[row].x;
			pMatA[ 2*row*8 + 7 ] = -vecPoints1[row].x*vecPoints2[row].y;

			pMatA[ (2*row+1)*8 + 3 ] = vecPoints2[row].x;
			pMatA[ (2*row+1)*8 + 4 ] = vecPoints2[row].y;
			pMatA[ (2*row+1)*8 + 5 ] = 1;
			pMatA[ (2*row+1)*8 + 6 ] = -vecPoints1[row].y*vecPoints2[row].x;
			pMatA[ (2*row+1)*8 + 7 ] = -vecPoints1[row].y*vecPoints2[row].y;

			pMatB[2*row] = vecPoints1[row].x;
			pMatB[2*row+1] = vecPoints1[row].y;
		}

		SolveLinearLeastSquare( pMatA, rowA, colA, 
			pMatB, rowB, colB,
			aProjectMat2, rowX, colX );

		for(int i=0; i<9; i++)
		{
			aProjectMat[i] = (T1)aProjectMat2[i];
		}

#ifdef _DEBUG
#endif
		// 计算均方差
		double errorDist = 0;
		double errorDistMax = 0;
		double MSE = 0;
		for(int i=0; i<pairsNum; i++)
		{
			double xDst = 0, yDst = 0;
			ApplyProjectMat2( vecPoints2[i].x, vecPoints2[i].y, xDst, yDst, aProjectMat );
			double dist = 0;
			DistanceOfTwoPoints( vecPoints1[i].x, vecPoints1[i].y, xDst, yDst, dist );
			errorDist = dist;
			if(errorDist>errorDistMax)
			{
				errorDistMax = errorDist;
			}
			MSE += errorDist * errorDist;
		}
		MSE = sqrt(MSE/pairsNum);

		aProjectMat[8] = (T1)errorDistMax;

		//errorDist /= pairsNum;
		//errorDist = sqrt( errorDist );
		//cout<<"errorDistMax "<<errorDistMax<<endl;

		memPool.Operate(pMatA);
		memPool.Operate(pMatB);
		memPool.Operate(pMatAT);

		return true;
	}

	template<class T1>
	inline void ApplyAffineMat(pool::SfPoint point1, pool::SfPoint &point1Bar, 
		T1 M[6])
	{
		float xSrc = point1.x;
		float ySrc = point1.y;

		point1Bar.x = (M[0]*xSrc + M[1]*ySrc + M[4]);

		point1Bar.y = (M[2]*xSrc + M[3]*ySrc + M[5]);
	}

	// M的维数必须要>=6
	template<class T1, class T2, class T3>
	inline void ApplyAffineMat2(T1 xSrc, T1 ySrc, 
		T2 &xDst, T2 &yDst, 
		T3* M)
	{
		xDst = (M[0]*xSrc + M[1]*ySrc + M[4]);
		yDst = (M[2]*xSrc + M[3]*ySrc + M[5]);
	}

	template<class TYPEPOINT, class T1>
	inline void ApplyProjectMat3(TYPEPOINT point1, TYPEPOINT &point1Bar, 
		T1 M[9])
	{
		T1 xSrc = point1.x;
		T1 ySrc = point1.y;

		point1Bar.x = (M[0]*xSrc + M[1]*ySrc + M[2]) / (M[6]*xSrc + M[7]*ySrc + 1);

		point1Bar.y = (M[3]*xSrc + M[4]*ySrc + M[5]) / (M[6]*xSrc + M[7]*ySrc + 1);
	}

	// M的维数必须要>=8
	template<class T1, class T2, class T3>
	inline void ApplyProjectMat9(T1 xSrc, T1 ySrc, 
		T2 &xDst, T2 &yDst, 
		T3* M)
	{
		T3 inv = 1 / (M[6]*xSrc + M[7]*ySrc + M[8]);

		xDst = (M[0]*xSrc + M[1]*ySrc + M[2]) * inv ;
		yDst = (M[3]*xSrc + M[4]*ySrc + M[5]) * inv;
	}

	// M的维数必须要>=8
	template<class T1, class T2, class T3>
	inline void ApplyProjectMat2(T1 xSrc, T1 ySrc, 
		T2 &xDst, T2 &yDst, 
		T3* M)
	{
		T3 inv = 1 / (M[6]*xSrc + M[7]*ySrc + 1);
		xDst = (M[0]*xSrc + M[1]*ySrc + M[2]) * inv ;

		yDst = (M[3]*xSrc + M[4]*ySrc + M[5]) * inv;
	}

	template<class T1, class T2, class T3>
	inline void ApplyProjectMat2Inv(T1 x1, T1 y1, 
		T2 &x2, T2 &y2, 
		T3 M[9])
	{
		float a1, b1, c1, a2, b2, c2;
		a1 = M[6]*x1 - M[0];
		b1 = M[7]*x1 - M[1];
		c1 = M[2] - x1;

		a2 = M[6]*y1 - M[3];
		b2 = M[7]*y1 - M[4];
		c2 = M[5] - y1;

		x2 = (b2*c1 - b1*c2) / (b2*a1 - b1*a2);
		y2 = (a2*c1 - a1*c2) / (a2*b1 - a1*b2);

	}

template<class PointType, class T1>
void ApplyPseudoAffine(PointType srcPoint, PointType dstPoint, T1 k[8])
{
	dstPoint.x2 = k[0]*srcPoint.x + k[1]*srcPoint.y + k[2]*srcPoint.x*srcPoint.y + k[3];
	dstPoint.y2 = k[4]*srcPoint.x + k[5]*srcPoint.y + k[6]*srcPoint.x*srcPoint.y + k[7];
}

template<class T1, class T2, class T3>
void ApplyPseudoAffine(T1 &xSrc, T1 &ySrc, T2 &xDst, T2 &yDst, T3 k[8])
{
	xDst = k[0]*xSrc + k[1]*ySrc + k[2]*xSrc*ySrc + k[3];
	yDst = k[4]*xSrc + k[5]*ySrc + k[6]*xSrc*ySrc + k[7];
}

// 解伪仿射变换矩阵
// 方法：线性最小二乘
// point2---->point1的变换
// x2 = k0*x + k1*y + k2*x*y + k3
// y2 = k4*x + k5*y + k6*x*y + k7
template<class PointType, class T1>
int SolvePseudoAffine( const PointType *pPoints1, const PointType *pPoints2, int pairsNum, T1 k[8])
{
	if( (NULL==pPoints1) || (NULL==pPoints2) )
		return -1;

	int rowA = pairsNum, colA = 4;
	int rowB = pairsNum, colB = 1;
	T1 *pA = new T1[rowA*colA];
	T1 *pB = new T1[rowB*rowB];
	for(int n=0; n<pairsNum; n++)
	{
		pA[n*4+0] = pPoints2[n].x;
		pA[n*4+1] = pPoints2[n].y;
		pA[n*4+2] = pPoints2[n].x * pPoints2[n].y;
		pA[n*4+3] = 1;

		pB[n] = pPoints1[n].x;
	}
	T1 k1[4];
	SolveLinearLeastSquare(pA, rowA, colA, pB, rowB, colB, k1, 4, 1);

	for(int n=0; n<pairsNum; n++)
	{
		pA[n*4+0] = pPoints2[n].x;
		pA[n*4+1] = pPoints2[n].y;
		pA[n*4+2] = pPoints2[n].x * pPoints2[n].y;
		pA[n*4+3] = 1;

		pB[n] = pPoints1[n].y;
	}
	T1 k2[4];
	SolveLinearLeastSquare(pA, rowA, colA, pB, rowB, colB, k2, 4, 1);
	memcpy(k, k1, sizeof(T1)*4);
	memcpy(k+4, k2, sizeof(T1)*4);

	delete[] pA; pA = NULL;
	delete[] pB; pB = NULL;

	return 0;
}

extern "C" MV_MODULE_TYPE
bool SolveProjectMatrix_API( pool::SfPoint *vecPoints1,
	pool::SfPoint *vecPoints2, int pairsNum,
	float aProjectMat[9]);

int InverseMatrixCv(float *src, int dim, float *dst);

int InverseMatrixCv64(double *src, int dim, double *dst);

