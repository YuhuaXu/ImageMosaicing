#include "using_cholmod.h"
#include "usingCV24.h"
#include "Bitmap.h"

cholmod_dense* cholmod_FeedDenseMat(CvMat* pA, cholmod_common &c)
{
	cholmod_dense* A;
	//cholmod_common c ;
	//cholmod_start (&c) ; /* start CHOLMOD */

	int row = pA->rows, col = pA->cols;
	int dRow = row;
	int xtype = CHOLMOD_REAL;
	A = cholmod_allocate_dense(row, col, dRow, xtype, &c);

	double* pVal = (double*)A->x;

	for(int r=0; r<row; r++)
	{
		for(int c=0; c<col; c++)
		{
			double val = cvGetReal2D(pA, r, c); // a
			*(pVal + r + c*dRow) = val;
		}
	}

	//cholmod_finish (&c) ; /* finish CHOLMOD */

	return A;
}

cholmod_sparse* cholmod_FeedSparseMat(pool::SparseMatrix pA, int symmetric, 
									  cholmod_common &common)
{
	if(pA.mat.empty())
		return NULL;

	if( (symmetric!=0) && (symmetric!=1) )
		return NULL;

	cholmod_sparse *A ;

	int row = pA.row, col = pA.col;
	int nNonzero = 0;
	// 计算pA中非零的个数
	for(int c=0; c<col; c++)
	{
		int nColElem = pA.mat[c].matCol.size();
		for(int i=0; i<nColElem; i++)
		{
			int r = pA.mat[c].matCol[i].iRow;
			if(symmetric==1)
			{
				if(r>c)
				{
					continue;
				}
			}		
			//double val = cvGetReal2D(pA, r, c); // a
			double val = pA.mat[c].matCol[i].val;
			if(val!=0)
			{
				nNonzero++;
			}
		}
	}

	int sorted = 1, packed = 1, stype = symmetric, xtype = CHOLMOD_REAL;

	A = cholmod_allocate_sparse(row, col, nNonzero, sorted, packed, stype, xtype, &common);

	int* pRowI = (int*)A->i;
	int* pColPtr = (int*)A->p;
	double* pNZVal = (double*)A->x;

	int nNZ = 0;
	pColPtr[0] = 0;
	for(int c=0; c<col; c++)
	{
		int nNZ_col = 0;
		int nColElem = pA.mat[c].matCol.size();
		for(int i=0; i<nColElem; i++)
		{
			int r = pA.mat[c].matCol[i].iRow;
			if(symmetric==1)
			{
				if(r>c)
				{
					break; // 只存上三角
				}
			}
			//double val = cvGetReal2D(pA, r, c);
			double val = pA.mat[c].matCol[i].val;
			if(val!=0)
			{
				pRowI[nNZ] = r;
				pNZVal[nNZ] = val;
				nNZ++;
				nNZ_col++;
			}
		}

		pColPtr[c+1] = nNZ_col + pColPtr[c];
	}

	return A;
}

cholmod_sparse* cholmod_FeedSparseMat(CvMat* pA, int symmetric, 
									  cholmod_common &c)
{
	if(pA==NULL)
		return NULL;
	if( (symmetric!=0) && (symmetric!=1) )
		return NULL;

	cholmod_sparse *A ;

	int row = pA->rows, col = pA->cols;
	int nNonzero = 0;
	// 计算pA中非零的个数
	for(int r=0; r<row; r++)
	{
		for(int c=0; c<col; c++)
		{
			if(symmetric==1)
			{
				if(r>c)
				{
					continue;
				}
			}
			double val = cvGetReal2D(pA, r, c); // a
			if(val!=0)
			{
				nNonzero++;
			}
		}
	}

	int sorted = 1, packed = 1, stype = symmetric, xtype = CHOLMOD_REAL;

	A = cholmod_allocate_sparse(row, col, nNonzero, sorted, packed, stype, xtype, &c);

	int* pRowI = (int*)A->i;
	int* pColPtr = (int*)A->p;
	double* pNZVal = (double*)A->x;

	int nNZ = 0;
	pColPtr[0] = 0;
	for(int c=0; c<col; c++)
	{
		int nNZ_col = 0;
		for(int r=0; r<row; r++)
		{
			if(symmetric==1)
			{
				if(r>c)
				{
					break; // 只存上三角
				}
			}
			double val = cvGetReal2D(pA, r, c);
			if(val!=0)
			{
				pRowI[nNZ] = r;
				pNZVal[nNZ] = val;
				nNZ++;
				nNZ_col++;
			}
		}
		
		pColPtr[c+1] = nNZ_col + pColPtr[c];
	}

	return A;
}

// 功能：求解稀疏线性系统
int SolveSparseSystem2(pool::SparseMatrix pA, pool::SparseMatrix pB, CvMat* pX)
{
	//if( (pA==NULL) || (pB==NULL) || (pX==NULL) )
	//	return -1;

	cholmod_sparse *X;
	cholmod_common c;
	cholmod_start (&c) ; /* start CHOLMOD */

	//CvMat* pAT = cvCreateMat(pA->cols, pA->rows, CV_64F);
	//CvMat* pATA = cvCreateMat(pA->cols, pA->cols, CV_64F);
	//CvMat* pATB = cvCreateMat(pAT->rows, pB->cols, CV_64F);

	//std::cout<<"start JT*J"<<std::endl;
	//cvTranspose(pA, pAT);
	//cvGEMM(pAT, pA, 1, NULL, 0, pATA);
	//cvGEMM(pAT, pB, 1, NULL, 0, pATB);

	std::cout<<"start cholmod"<<std::endl;
	int symmetric = 1;
	int nonsymmetric = 0;

	cholmod_sparse *A0 = cholmod_FeedSparseMat(pA, nonsymmetric, c);
	cholmod_sparse *B0 = cholmod_FeedSparseMat(pB, nonsymmetric, c);
	cout<<"A0->nzmax "<<A0->nzmax<<endl;
	cout<<"B0->nzmax "<<B0->nzmax<<endl;

	
	cholmod_sparse *A0T = cholmod_transpose(A0, 1, &c);
	//int stype = 1; // 对称

	cholmod_sparse *A = cholmod_ssmult(A0T, A0, symmetric, TRUE, TRUE, &c);
	cholmod_sparse *B = cholmod_ssmult(A0T, B0, nonsymmetric, TRUE, TRUE, &c);

	// AX = B

	cholmod_factor *L ;

	cholmod_print_sparse (A, "A", &c) ; /* print the matrix */

	if (A == NULL || A->stype == 0) /* A must be symmetric */
	{
		cholmod_free_sparse (&A, &c) ;
		cholmod_finish (&c) ;
		return -1;
	}

	L = cholmod_analyze (A, &c) ; /* analyze */

	//cholmod_print_sparse (L, "L", &c) ; /* print the matrix */
	cholmod_factorize (A, L, &c) ; /* factorize */
	X = cholmod_spsolve (CHOLMOD_A, L, B, &c) ; /* solve Ax=b */

	double *pXx = (double *)X->x;
	int *pRowI = (int*)X->i;
	cvZero(pX);
	for(int n=0; n<X->nzmax; n++)
	{
		cvSetReal2D(pX, pRowI[n], 0, pXx[n]);
	}

	//r = cholmod_copy_dense (B, &c) ; /* r = b */
	//cholmod_sdmult (A, 0, m1, one, x, r, &c) ; /* r = r-Ax */
	//printf ("norm(b-Ax) %8.1e\n",
	//	cholmod_norm_dense (r, 0, &c)) ; /* print norm(r) */

	cholmod_free_factor (&L, &c) ; /* free matrices */
	cholmod_free_sparse (&A, &c) ;
	cholmod_free_sparse (&B, &c) ;
	cholmod_free_sparse (&X, &c) ;

	cholmod_free_sparse (&B0, &c) ;
	cholmod_free_sparse (&A0, &c) ;
	cholmod_free_sparse (&A0T, &c) ;

	cholmod_finish (&c) ; /* finish CHOLMOD */

	//cvReleaseMat(&pAT);
	//cvReleaseMat(&pATA);
	//cvReleaseMat(&pATB);

	return 0;
}

// 功能：求解稀疏线性系统
// pA为对称的方阵
int SolveSparseSystem3(pool::SparseMatrix pA, pool::SparseMatrix pB, CvMat* pX)
{
	//if( (pA==NULL) || (pB==NULL) || (pX==NULL) )
	//	return -1;

	cholmod_sparse *X;
	cholmod_common c;
	cholmod_start (&c) ; /* start CHOLMOD */

	//CvMat* pAT = cvCreateMat(pA->cols, pA->rows, CV_64F);
	//CvMat* pATA = cvCreateMat(pA->cols, pA->cols, CV_64F);
	//CvMat* pATB = cvCreateMat(pAT->rows, pB->cols, CV_64F);

	//std::cout<<"start JT*J"<<std::endl;
	//cvTranspose(pA, pAT);
	//cvGEMM(pAT, pA, 1, NULL, 0, pATA);
	//cvGEMM(pAT, pB, 1, NULL, 0, pATB);

	std::cout<<"start cholmod"<<std::endl;
	int symmetric = 1;
	int nonsymmetric = 0;

	cholmod_sparse *A = cholmod_FeedSparseMat(pA, symmetric, c);
	cholmod_sparse *B = cholmod_FeedSparseMat(pB, nonsymmetric, c);
	cout<<"A->nzmax "<<A->nzmax<<endl;
	cout<<"B->nzmax "<<B->nzmax<<endl;


	//cholmod_sparse *A0T = cholmod_transpose(A0, 1, &c);
	//int stype = 1; // 对称

	//cholmod_sparse *A = cholmod_ssmult(A0T, A0, symmetric, TRUE, TRUE, &c);
	//cholmod_sparse *B = cholmod_ssmult(A0T, B0, nonsymmetric, TRUE, TRUE, &c);

	// AX = B

	cholmod_factor *L ;

	cholmod_print_sparse (A, "A", &c) ; /* print the matrix */

	if (A == NULL || A->stype == 0) /* A must be symmetric */
	{
		cholmod_free_sparse (&A, &c) ;
		cholmod_finish (&c) ;
		return -1;
	}

	L = cholmod_analyze (A, &c) ; /* analyze */

	//cholmod_print_sparse (L, "L", &c) ; /* print the matrix */
	cholmod_factorize (A, L, &c) ; /* factorize */
	X = cholmod_spsolve (CHOLMOD_A, L, B, &c) ; /* solve Ax=b */

	double *pXx = (double *)X->x;
	int *pRowI = (int*)X->i;
	cvZero(pX);
	for(int n=0; n<X->nzmax; n++)
	{
		cvSetReal2D(pX, pRowI[n], 0, pXx[n]);
	}

	//r = cholmod_copy_dense (B, &c) ; /* r = b */
	//cholmod_sdmult (A, 0, m1, one, x, r, &c) ; /* r = r-Ax */
	//printf ("norm(b-Ax) %8.1e\n",
	//	cholmod_norm_dense (r, 0, &c)) ; /* print norm(r) */

	cholmod_free_factor (&L, &c) ; /* free matrices */
	cholmod_free_sparse (&A, &c) ;
	cholmod_free_sparse (&B, &c) ;
	cholmod_free_sparse (&X, &c) ;

	//cholmod_free_sparse (&B0, &c) ;
	//cholmod_free_sparse (&A0, &c) ;
	//cholmod_free_sparse (&A0T, &c) ;

	cholmod_finish (&c) ; /* finish CHOLMOD */

	//cvReleaseMat(&pAT);
	//cvReleaseMat(&pATA);
	//cvReleaseMat(&pATB);

	return 0;
}

// 功能：求解稀疏线性系统
// pEye-----单位阵
int SolveSparseSystemLM(pool::SparseMatrix pA, pool::SparseMatrix pB, pool::SparseMatrix pEye, CvMat* pX, double beta)
{
	//if( (pA==NULL) || (pB==NULL) || (pX==NULL) )
	//	return -1;

	cholmod_sparse *X;
	cholmod_common c;
	cholmod_start (&c) ; /* start CHOLMOD */

	//CvMat* pAT = cvCreateMat(pA->cols, pA->rows, CV_64F);
	//CvMat* pATA = cvCreateMat(pA->cols, pA->cols, CV_64F);
	//CvMat* pATB = cvCreateMat(pAT->rows, pB->cols, CV_64F);

	//std::cout<<"start JT*J"<<std::endl;
	//cvTranspose(pA, pAT);
	//cvGEMM(pAT, pA, 1, NULL, 0, pATA);
	//cvGEMM(pAT, pB, 1, NULL, 0, pATB);

	//std::cout<<"start cholmod"<<std::endl;
	int symmetric = 1;
	int nonsymmetric = 0;

	cholmod_sparse *A0 = cholmod_FeedSparseMat(pA, nonsymmetric, c);
	cholmod_sparse *B0 = cholmod_FeedSparseMat(pB, nonsymmetric, c);
	cholmod_sparse *eye0 = cholmod_FeedSparseMat(pEye, symmetric, c);
	//cout<<"A0->nzmax "<<A0->nzmax<<endl;
	//cout<<"B0->nzmax "<<B0->nzmax<<endl;
	//cout<<"eye0->nzmax "<<eye0->nzmax<<endl;

	cholmod_sparse *A0T = cholmod_transpose(A0, 1, &c);
	int stype = 1; // 对称

	cholmod_sparse *ATA = cholmod_ssmult(A0T, A0, symmetric, TRUE, TRUE, &c);
	//cout<<"ATA->nzmax "<<ATA->nzmax<<endl;
	double alfa[2] = {1.0, 0};
	double beta2[2] = {beta, 0};
	//cholmod_print_sparse (ATA, "ATA", &c) ; /* print the matrix */
	//cout<<"ATA->stype "<<ATA->stype <<endl;
	//cholmod_print_sparse (eye0, "eye0", &c) ; /* print the matrix */
	eye0->stype = ATA->stype;
	cholmod_sparse *A = cholmod_add(ATA, eye0, alfa, beta2, TRUE, TRUE, &c); // LM算法

	//cout<<"A->nzmax "<<A->nzmax<<endl;

	cholmod_sparse *B = cholmod_ssmult(A0T, B0, nonsymmetric, TRUE, TRUE, &c);

	//A = cholmod_FeedSparseMat(pATA, symmetric, c);
	//B = cholmod_FeedDenseMat(pATB, c);

	cholmod_factor *L ;

	cholmod_print_sparse (A, "A", &c) ; /* print the matrix */

	//cholmod_print_dense (A, "A", &c) ; /* print the matrix */

	if (A == NULL || A->stype == 0) /* A must be symmetric */
	{
		cholmod_free_sparse (&A, &c) ;
		cholmod_finish (&c) ;
		return -1;
	}

	L = cholmod_analyze (A, &c) ; /* analyze */

	//cholmod_print_sparse (L, "L", &c) ; /* print the matrix */
	cholmod_factorize (A, L, &c) ; /* factorize */
	X = cholmod_spsolve (CHOLMOD_A, L, B, &c) ; /* solve Ax=b */

	double *pXx = (double *)X->x;
	int *pRowI = (int*)X->i;
	cvZero(pX);
	for(int n=0; n<X->nzmax; n++)
	{
		cvSetReal2D(pX, pRowI[n], 0, pXx[n]);
	}

	//r = cholmod_copy_dense (B, &c) ; /* r = b */
	//cholmod_sdmult (A, 0, m1, one, x, r, &c) ; /* r = r-Ax */
	//printf ("norm(b-Ax) %8.1e\n",
	//	cholmod_norm_dense (r, 0, &c)) ; /* print norm(r) */

	cholmod_free_factor (&L, &c) ; /* free matrices */
	cholmod_free_sparse (&A, &c) ;
	cholmod_free_sparse (&B, &c) ;
	cholmod_free_sparse (&X, &c) ;

	cholmod_free_sparse (&B0, &c) ;
	cholmod_free_sparse (&A0, &c) ;
	cholmod_free_sparse (&A0T, &c) ;

	cholmod_free_sparse (&ATA, &c) ;
	cholmod_free_sparse (&eye0, &c) ;

	cholmod_finish (&c) ; /* finish CHOLMOD */

	//cvReleaseMat(&pAT);
	//cvReleaseMat(&pATA);
	//cvReleaseMat(&pATB);

	return 0;
}

// 功能：求解稀疏线性系统
int SolveSparseSystem(CvMat* pA, CvMat* pB, CvMat* pX)
{
	if( (pA==NULL) || (pB==NULL) || (pX==NULL) )
		return -1;

	cholmod_sparse *A;
	cholmod_dense *X, *B;
	cholmod_common c;
	cholmod_start (&c) ; /* start CHOLMOD */

	CvMat* pAT = cvCreateMat(pA->cols, pA->rows, CV_64F);
	CvMat* pATA = cvCreateMat(pA->cols, pA->cols, CV_64F);
	CvMat* pATB = cvCreateMat(pAT->rows, pB->cols, CV_64F);

	std::cout<<"start JT*J"<<std::endl;
	cvTranspose(pA, pAT);
	cvGEMM(pAT, pA, 1, NULL, 0, pATA);
	cvGEMM(pAT, pB, 1, NULL, 0, pATB);

	//cout<<"pATA"<<endl;
	//cvPrintMatrix(pATA, "c:\\ata.txt", 1);
	//cout<<"pATB"<<endl;
	//cvPrintMatrix(pATB, "c:\\atb.txt");
	
	std::cout<<"start cholmod"<<std::endl;
	int symmetric = 1;
	A = cholmod_FeedSparseMat(pATA, symmetric, c);

	B = cholmod_FeedDenseMat(pATB, c);

	cholmod_factor *L ;

	cholmod_print_sparse (A, "A", &c) ; /* print the matrix */

	if (A == NULL || A->stype == 0) /* A must be symmetric */
	{
		cholmod_free_sparse (&A, &c) ;
		cholmod_finish (&c) ;
		return -1;
	}


	L = cholmod_analyze (A, &c) ; /* analyze */

	//cholmod_print_sparse (L, "L", &c) ; /* print the matrix */
	cholmod_factorize (A, L, &c) ; /* factorize */
	X = cholmod_solve (CHOLMOD_A, L, B, &c) ; /* solve Ax=b */
	double *pXx = (double *)X->x;
	for(int r=0; r<pX->rows; r++)
	{
		cvSetReal2D(pX, r, 0, pXx[r]);
	}

	double* pBx = (double *)B->x;
	int nzB = 0;
	for(int r=0; r<B->nzmax; r++)
	{
		double val = *(pBx + r);
		if(val!=0)
		{
			nzB++;
		}
	}
	cout<<"nzB "<<nzB<<endl;
	
	//r = cholmod_copy_dense (B, &c) ; /* r = b */
	//cholmod_sdmult (A, 0, m1, one, x, r, &c) ; /* r = r-Ax */
	//printf ("norm(b-Ax) %8.1e\n",
	//	cholmod_norm_dense (r, 0, &c)) ; /* print norm(r) */

	cholmod_free_factor (&L, &c) ; /* free matrices */
	cholmod_free_sparse (&A, &c) ;
	cholmod_free_dense (&X, &c) ;
	cholmod_free_dense (&B, &c) ;

	cholmod_finish (&c) ; /* finish CHOLMOD */

	cvReleaseMat(&pAT);
	cvReleaseMat(&pATA);
	cvReleaseMat(&pATB);

	return 0;
}

void TestCholmod()
{
	cholmod_sparse *A ;
	cholmod_dense *x, *b, *r ;
	cholmod_factor *L ;
	double one [2] = {1,0}, m1 [2] = {-1,0} ; /* basic scalars */
	cholmod_common c ;
	cholmod_start (&c) ; /* start CHOLMOD */

	int row = 10, col = 10, nNonzero = 11, sorted = 1, packed = 1, stype = 0, xtype = CHOLMOD_REAL;
	A = cholmod_allocate_sparse(row, col, nNonzero, sorted, packed, stype, xtype, &c);
	int* pI = (int*)A->i;
	pI[0] = 0;
	pI[1] = 1;

	//A = cholmod_read_sparse (stdin, &c) ; /* read in a matrix */ // 缺少库支持?

	cholmod_print_sparse (A, "A", &c) ; /* print the matrix */
	if (A == NULL || A->stype == 0) /* A must be symmetric */
	{
		cholmod_free_sparse (&A, &c) ;
		cholmod_finish (&c) ;
		return;
	}
	b = cholmod_ones (A->nrow, 1, A->xtype, &c) ; /* b = ones(n,1) */

	L = cholmod_analyze (A, &c) ; /* analyze */
	cholmod_factorize (A, L, &c) ; /* factorize */

	x = cholmod_solve (CHOLMOD_A, L, b, &c) ; /* solve Ax=b */

	r = cholmod_copy_dense (b, &c) ; /* r = b */

	cholmod_sdmult (A, 0, m1, one, x, r, &c) ; /* r = r-Ax */
	printf ("norm(b-Ax) %8.1e\n",
		cholmod_norm_dense (r, 0, &c)) ; /* print norm(r) */
	cholmod_free_factor (&L, &c) ; /* free matrices */
	cholmod_free_sparse (&A, &c) ;
	cholmod_free_dense (&r, &c) ;
	cholmod_free_dense (&x, &c) ;
	cholmod_free_dense (&b, &c) ;
	cholmod_finish (&c) ; /* finish CHOLMOD */
	return;
}
