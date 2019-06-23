
// 
/*#include "stdafx.h"*/
#include "Point.h"
#include "ImageGeometry.h"
#include "ImageMath.h"
#include "usingAtlImage.h"
#include "ImageFilter.h"
#include "mvMath.h"
using namespace pool;

// mirror, 0为X方向镜像, 1为Y方向镜像
bool ImageMirror( const BitmapImage* pSrcImage, BitmapImage* pDstImage, int mirrorType)
{
	if( (pSrcImage==NULL) || (pDstImage==NULL) )
		return false;
	int width=pSrcImage->width ;
	int height=pSrcImage->height ;

	if(mirrorType==0) // X方向镜像
	{
		for(int i=0; i<height; i++)
		{
			unsigned char* pSrcRow = pSrcImage->imageData +pSrcImage->widthStep*i;
			unsigned char* pDstRow = pDstImage->imageData +pDstImage->widthStep*i;
			for(int j=0; j<width; j++)
			{
				pDstRow[width-j-1]=pSrcRow[j];
			}
		}
	}
	else  // Y方向镜像
	{
		for(int i=0; i<height; i++)
		{
			unsigned char* pSrcRow = pSrcImage->imageData +pSrcImage->widthStep*(height-i-1);
			unsigned char* pDstRow = pDstImage->imageData +pDstImage->widthStep*i;
			for(int j=0; j<width; j++)
			{
				pDstRow[j]=pSrcRow[j];
			}
		}
	}

	return true;
}

bool ImageMirror(  pool::BitmapImage* pSrcImage, int mirrorType )
{
	if( pSrcImage==NULL )
		return false;
	int width=pSrcImage->width ;
	int height=pSrcImage->height;
	int widthStep = pSrcImage->widthStep;

	BitmapImage* pDstImage = CreateBitmap8U(width, height, pSrcImage->nChannels);
	ImageMirror( pSrcImage, pDstImage, mirrorType );

	memcpy( pSrcImage->imageData, pDstImage->imageData, widthStep * height );

	ReleaseBitmap8U( pDstImage );

	return true;
}

bool Translation( const IplImage*  pSrcImage, IplImage* _OUT pDstImage, 
				 const float  deltaX,  const float  deltaY)
{
	if( (pSrcImage==NULL) || (pDstImage==NULL) )
		return false;

	cvZero( pDstImage );

	int wDst, hDst, wsDst;
	CvGetImageSize( pDstImage, wDst, hDst, wsDst );
	int wSrc, hSrc, wsSrc;
	CvGetImageSize( pSrcImage, wSrc, hSrc, wsSrc );
	int hSrc_1 = hSrc - 1;
	int wSrc_1 = wSrc - 1;

	for(int i=0; i<hDst; i++) 
	{
		unsigned char* pRowDst = (unsigned char*)pDstImage->imageData + i*wsDst;
		float fSrcY = i - deltaY;
		if( (fSrcY<0)||(fSrcY>=hSrc_1) )
			continue;
		for(int j=0; j<wDst; j++)
		{
			float fSrcX = j - deltaX;
			if( (fSrcX<0)||(fSrcX>=wSrc_1) )
				continue;
			// 双线性插值
			int ySrc = int(fSrcY);
			int xSrc = int(fSrcX);

			float p = fSrcY - ySrc;

			float q = fSrcX - xSrc;

			unsigned char* pSrcTemp = (unsigned char*)pSrcImage->imageData + ySrc*wsSrc + xSrc;

			*(pRowDst + j) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
				*(pSrcTemp + 1)*(1-p)*q + 
				*(pSrcTemp + wsSrc)*p*(1-q) + 
				*(pSrcTemp + wsSrc + 1)*p*q );
		}
	}

	return true;
}

bool TranslationX( const IplImage*  pSrcImage, IplImage* _OUT pDstImage, 
				 const float  deltaX)
{
	if( (pSrcImage==NULL) || (pDstImage==NULL) )
		return false;

	cvZero( pDstImage );

	int wDst, hDst, wsDst;
	CvGetImageSize( pDstImage, wDst, hDst, wsDst );
	int wSrc, hSrc, wsSrc;
	CvGetImageSize( pSrcImage, wSrc, hSrc, wsSrc );
	int hSrc_1 = hSrc - 1;
	int wSrc_1 = wSrc - 1;

	for(int i=0; i<hDst; i++) 
	{
		unsigned char* pRowDst = (unsigned char*)pDstImage->imageData + i*wsDst;
		float fSrcY = i;

		if( (fSrcY<0)||(fSrcY>=hSrc_1) )
			continue;
		for(int j=0; j<wDst; j++)
		{
			float fSrcX = j - deltaX;
			if( (fSrcX<0)||(fSrcX>=wSrc_1) )
				continue;
			// 双线性插值
			int ySrc = int(fSrcY);
			int xSrc = int(fSrcX);

			float p = 0;

			float q = fSrcX - xSrc;

			unsigned char* pSrcTemp = (unsigned char*)pSrcImage->imageData + ySrc*wsSrc + xSrc;

			*(pRowDst + j) = unsigned char( *(pSrcTemp)*(1-q) + *(pSrcTemp + 1)*q);
		}
	}

	return true;
}

// 图像平移(未完成2011-01-17)
bool Translation(const BitmapImage* pSrcImage, BitmapImage* _OUT pDstImage, 
				 const float deltaX, const float deltaY)
{
	if( (pSrcImage==NULL) || (pDstImage==NULL) )
		return false;

	ZeroImage( pDstImage );

	int wDst, hDst, wsDst;
	GetImageSize( pDstImage, wDst, hDst, wsDst );
	int wSrc, hSrc, wsSrc;
	GetImageSize( pSrcImage, wSrc, hSrc, wsSrc );
	int hSrc_1 = hSrc - 1;
	int wSrc_1 = wSrc - 1;

	for(int i=0; i<hDst; i++)
	{
		unsigned char* pRowDst = pDstImage->imageData + i*wsDst;
		float fSrcY = i - deltaY;
		if( (fSrcY<0)||(fSrcY>=hSrc_1) )
			continue;
		for(int j=0; j<wDst; j++)
		{
			float fSrcX = j - deltaX;
			if( (fSrcX<0)||(fSrcX>=wSrc_1) )
				continue;
			// 双线性插值
			int ySrc = int(fSrcY);
			int xSrc = int(fSrcX);

			float p = fSrcY - ySrc;
			float q = fSrcX - xSrc;

			unsigned char* pSrcTemp = pSrcImage->imageData + ySrc*wsSrc + xSrc;

			*(pRowDst + j) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
				*(pSrcTemp + 1)*(1-p)*q + 
				*(pSrcTemp + wsSrc)*p*(1-q) + 
				*(pSrcTemp + wsSrc + 1)*p*q );
		}
	}

	return true;
}

//功能：围绕中心旋转
 pool::BitmapImage* RotateImageCenter2( pool::BitmapImage *pSrc, const float fAngle)
{
	if(!IsBitmapValid(pSrc))
		return false;
	
	 pool::BitmapImage* pDst = NULL;
	
	if( abs(fAngle) < 1e-6f )
	{
		pDst = CloneBitmap8U( pSrc  );
		return pDst;
	}
	else
	{
		int u = 0, v = 0;
		
		int width, height, widthstep;
		int width_1, height_1;

		width = pSrc->width;
		height = pSrc->height;
		widthstep = pSrc->widthStep;

		width_1 = width - 1;
		height_1 = height - 1;

		if( (width>9999) ||(height>9999) )
		{
			return false;
		}

		float cos_a, sin_a;
		float hist[256] = {0};
		float max1=0, maxi1=0;

		pool::SfPoint corner[4];
		corner[0].x=0.0f;		corner[0].y=0.0f;
		corner[1].x=(float)width;  corner[1].y=0.0f;
		corner[2].x=0.0f;		corner[2].y=(float)height;
		corner[3].x=(float)width;  corner[3].y=(float)height;
		float xmax=-9999, xmin=9999, ymax=-9999, ymin=9999;
		cos_a=(float)cos(fAngle);
		sin_a=(float)sin(fAngle);

		float fCenterX0 = 0, fCenterY0 = 0, fCenterX1 = 0, fCenterY1 = 0;
		//原图的中心点
		fCenterX0 = (width - 1)*0.5f;
		fCenterY0 = (height - 1)*0.5f;

		//要求：围绕中心点旋转

		for(u=0; u<4; u++)
		{
			corner[u] = TimeRotationMatrix2D(corner[u], cos_a, sin_a);
			if (corner[u].x<xmin) 
			{
				xmin=(float)corner[u].x;
			}
			if (corner[u].x>xmax) 
			{
				xmax=(float)corner[u].x;
			}
			if (corner[u].y<ymin) 
			{
				ymin=(float)corner[u].y;
			}
			if (corner[u].y>ymax) 
			{
				ymax=(float)corner[u].y;// 找出X、Y的最大，最小值
			}
		}
		int newwidth, newheight;
		newwidth = int(xmax-xmin+0.5f);
		newheight = int(ymax-ymin+0.5f);

		//如果原图的宽度（高度）为偶数（奇数），则旋转后的图像的宽度（高度）也应为偶数（奇数），要保证旋转中心不变
		if(width%2==0)//如果为偶数
		{
			newwidth = (newwidth+1)/2*2;
		}
		else//如果是奇数
		{
			if( newwidth%2==0 )
			{
				newwidth += 1;
			}
		}

		if(height%2==0)//如果为偶数
		{
			newheight = (newheight+1)/2*2;
		}
		else//如果是奇数
		{
			if( newheight%2==0 )
			{
				newheight += 1;
			}
		}

		//开辟内存
		//pDst = CreateImage(cvSize(newwidth, newheight), 8, 1);
		pDst = CreateBitmap8U( newwidth, newheight, 1);
		memset(pDst->imageData, 0, pDst->widthStep*pDst->height);

		int widthStep2 = pDst->widthStep;

		// 计算平移矩阵 (fOffsetX, fOffsetY)
		float fOffsetX, fOffsetY;
		fOffsetX = -fCenterX0*cos_a + fCenterY0*sin_a + (newwidth-1)*0.5f ;
		fOffsetY = -fCenterX0*sin_a - fCenterY0*cos_a + (newheight-1)*0.5f ;

		if( (newwidth>9999) ||(newheight>9999) )
		{
			return false;
		}

		float afPartX[10000] = {0};
		float afPartY[10000] = {0};

		cos_a = cos(-fAngle);
		sin_a = sin(-fAngle);

		// 建立表格，可以查表
		for (v=0; v<newwidth; v++) 
		{
			afPartX[v] = (v-fOffsetX)*cos_a;
			afPartY[v] = (v-fOffsetX)*sin_a;
		}

		unsigned char* pDstData = pDst->imageData;
		unsigned char* pSrcData = pSrc->imageData;

		//
		unsigned char* ptr = pDstData;
		for(int u=0; u<newheight; u++)
		{

			float ysina = (u-fOffsetY)*sin_a;
			float ycosa = (u-fOffsetY)*cos_a;

			for (v=0; v<newwidth; v++) 
			{
				float fPointtempX = afPartX[v] - ysina;
				float fPointtempY = afPartY[v] + ycosa;

				if( (fPointtempX>=0) && (fPointtempX<width_1) && (fPointtempY>=0) && (fPointtempY<height_1) )
				{
					int i = int(fPointtempY);
					int j = int(fPointtempX);

					unsigned char* pTemp = pSrcData + i*widthstep + j;
					//if( (uchar)*(pTemp)==0 )
					//	continue;

					// 浮点运算
					float p = fPointtempY-i;
					float q = fPointtempX-j;

					//// 原始版本
					//*(ptr + v) = char((uchar)*(pTemp)*(1-p)*(1-q) + 
					//	(uchar)*(pTemp + 1)*(1-p)*q + 
					//	(uchar)*(pTemp + widthstep)*p*(1-q) + 
					//	(uchar)*(pTemp + widthstep + 1)*p*q );
					//// 原始版本结束...

					// // 版本2

					unsigned char gray1 = (unsigned char)*(pTemp);
					unsigned char gray2 = (unsigned char)*(pTemp + 1);
					unsigned char gray3 = (unsigned char)*(pTemp + widthstep);
					unsigned char gray4 = (unsigned char)*(pTemp + widthstep + 1);
					*(ptr + v) = unsigned char( gray1*(1-p)*(1-q) + 
						gray2*(1-p)*q + 
						gray3*p*(1-q) + 
						gray4*p*q );
					// 版本2结束...
				}
			}
			ptr += widthStep2;
		}


	}

	return pDst;
}

//功能：围绕中心旋转
 IplImage* RotateImageCenterBilinear(IplImage *pSrc, const float fAngle)
{
	if(NULL==pSrc)
		return false;
	
	 IplImage* pDst = NULL;
	
	if( abs(fAngle) < 1e-6f )
	{
		pDst = cvCloneImage( pSrc  );
		return pDst;
	}
	else
	{
		int u = 0, v = 0;
		
		int width, height, widthstep;
		int width_1, height_1;

		width = pSrc->width;
		height = pSrc->height;
		widthstep = pSrc->widthStep;

		width_1 = width - 1;
		height_1 = height - 1;

		if( (width>9999) ||(height>9999) )
		{
			return false;
		}

		float cos_a, sin_a;
		float hist[256] = {0};
		float max1=0, maxi1=0;

		pool::SfPoint corner[4];
		corner[0].x=0.0f;		corner[0].y=0.0f;
		corner[1].x=(float)width;  corner[1].y=0.0f;
		corner[2].x=0.0f;		corner[2].y=(float)height;
		corner[3].x=(float)width;  corner[3].y=(float)height;
		float xmax=-9999, xmin=9999, ymax=-9999, ymin=9999;
		cos_a=(float)cos(fAngle);
		sin_a=(float)sin(fAngle);

		float fCenterX0 = 0, fCenterY0 = 0, fCenterX1 = 0, fCenterY1 = 0;
		//原图的中心点
		fCenterX0 = (width - 1)*0.5f;
		fCenterY0 = (height - 1)*0.5f;

		//要求：围绕中心点旋转

		for(u=0; u<4; u++)
		{
			corner[u] = TimeRotationMatrix2D(corner[u], cos_a, sin_a);
			if (corner[u].x<xmin) 
			{
				xmin=(float)corner[u].x;
			}
			if (corner[u].x>xmax) 
			{
				xmax=(float)corner[u].x;
			}
			if (corner[u].y<ymin) 
			{
				ymin=(float)corner[u].y;
			}
			if (corner[u].y>ymax) 
			{
				ymax=(float)corner[u].y;// 找出X、Y的最大，最小值
			}
		}
		int newwidth, newheight;
		newwidth = int(xmax-xmin+0.5f);
		newheight = int(ymax-ymin+0.5f);

		//如果原图的宽度（高度）为偶数（奇数），则旋转后的图像的宽度（高度）也应为偶数（奇数），要保证旋转中心不变
		if(width%2==0)//如果为偶数
		{
			newwidth = (newwidth+1)/2*2;
		}
		else//如果是奇数
		{
			if( newwidth%2==0 )
			{
				newwidth += 1;
			}
		}

		if(height%2==0)//如果为偶数
		{
			newheight = (newheight+1)/2*2;
		}
		else//如果是奇数
		{
			if( newheight%2==0 )
			{
				newheight += 1;
			}
		}

		//开辟内存
		//pDst = CreateImage(cvSize(newwidth, newheight), 8, 1);
		pDst = cvCreateImage( cvSize(newwidth, newheight), 8, 1);
		memset(pDst->imageData, 0, pDst->widthStep*pDst->height);

		int widthStep2 = pDst->widthStep;

		// 计算平移矩阵 (fOffsetX, fOffsetY)
		float fOffsetX, fOffsetY;
		fOffsetX = -fCenterX0*cos_a + fCenterY0*sin_a + (newwidth-1)*0.5f ;
		fOffsetY = -fCenterX0*sin_a - fCenterY0*cos_a + (newheight-1)*0.5f ;

		if( (newwidth>9999) ||(newheight>9999) )
		{
			return false;
		}

		float afPartX[10000] = {0};
		float afPartY[10000] = {0};

		cos_a = cos(-fAngle);
		sin_a = sin(-fAngle);

		// 建立表格，可以查表
		for (v=0; v<newwidth; v++) 
		{
			afPartX[v] = (v-fOffsetX)*cos_a;
			afPartY[v] = (v-fOffsetX)*sin_a;
		}

		unsigned char* pDstData = (unsigned char*)pDst->imageData;
		unsigned char* pSrcData = (unsigned char*)pSrc->imageData;

		//
		unsigned char* ptr = pDstData;
		for(int u=0; u<newheight; u++)
		{

			float ysina = (u-fOffsetY)*sin_a;
			float ycosa = (u-fOffsetY)*cos_a;

			for (v=0; v<newwidth; v++) 
			{
				float fPointtempX = afPartX[v] - ysina;
				float fPointtempY = afPartY[v] + ycosa;

				if( (fPointtempX>=0) && (fPointtempX<width_1) && (fPointtempY>=0) && (fPointtempY<height_1) )
				{
					int i = int(fPointtempY);
					int j = int(fPointtempX);

					unsigned char* pTemp = pSrcData + i*widthstep + j;
					//if( (uchar)*(pTemp)==0 )
					//	continue;

					// 浮点运算
					float p = fPointtempY-i;
					float q = fPointtempX-j;

					//// 原始版本
					//*(ptr + v) = char((uchar)*(pTemp)*(1-p)*(1-q) + 
					//	(uchar)*(pTemp + 1)*(1-p)*q + 
					//	(uchar)*(pTemp + widthstep)*p*(1-q) + 
					//	(uchar)*(pTemp + widthstep + 1)*p*q );
					//// 原始版本结束...

					// // 版本2

					unsigned char gray1 = (unsigned char)*(pTemp);
					unsigned char gray2 = (unsigned char)*(pTemp + 1);
					unsigned char gray3 = (unsigned char)*(pTemp + widthstep);
					unsigned char gray4 = (unsigned char)*(pTemp + widthstep + 1);
					*(ptr + v) = unsigned char( gray1*(1-p)*(1-q) + 
						gray2*(1-p)*q + 
						gray3*p*(1-q) + 
						gray4*p*q );
					// 版本2结束...
				}
			}
			ptr += widthStep2;
		}
	}

	return pDst;
}


float S3Fun(float x)
{
	float sx = 0;

	float absx = abs(x);

	if( (absx>=0) && (absx<1) )
	{
		sx = 1 - 2*absx*absx  + absx*absx*absx;
	}
	else if(absx>=2)
	{
		sx = 0;
	}
	else
	{
		sx = 4 - 8*absx + 5*absx*absx - absx*absx*absx;
	}

	return sx;
}

//功能：围绕中心旋转
 IplImage* RotateImageCenterCubic( IplImage *pSrc, const float fAngle)
{
	if(NULL==pSrc)
		return false;
	
	 IplImage* pDst = NULL;
	
	if( abs(fAngle) < 1e-8f )
	{
		pDst = cvCloneImage( pSrc  );
		return pDst;
	}
	else
	{
		int u = 0, v = 0;
		
		int width, height, widthstep;
		int width_1, height_1;

		width = pSrc->width;
		height = pSrc->height;
		widthstep = pSrc->widthStep;

		width_1 = width - 1;
		height_1 = height - 1;

		if( (width>9999) ||(height>9999) )
		{
			return false;
		}

		float cos_a, sin_a;
		float hist[256] = {0};
		float max1=0, maxi1=0;

		pool::SfPoint corner[4];
		corner[0].x=0.0f;		corner[0].y=0.0f;
		corner[1].x=(float)width;  corner[1].y=0.0f;
		corner[2].x=0.0f;		corner[2].y=(float)height;
		corner[3].x=(float)width;  corner[3].y=(float)height;
		float xmax=-9999, xmin=9999, ymax=-9999, ymin=9999;
		cos_a=(float)cos(fAngle);
		sin_a=(float)sin(fAngle);

		float fCenterX0 = 0, fCenterY0 = 0, fCenterX1 = 0, fCenterY1 = 0;
		//原图的中心点
		fCenterX0 = (width - 1)*0.5f;
		fCenterY0 = (height - 1)*0.5f;

		//要求：围绕中心点旋转

		for(u=0; u<4; u++)
		{
			corner[u] = TimeRotationMatrix2D(corner[u], cos_a, sin_a);
			if (corner[u].x<xmin) 
			{
				xmin=(float)corner[u].x;
			}
			if (corner[u].x>xmax) 
			{
				xmax=(float)corner[u].x;
			}
			if (corner[u].y<ymin) 
			{
				ymin=(float)corner[u].y;
			}
			if (corner[u].y>ymax) 
			{
				ymax=(float)corner[u].y;// 找出X、Y的最大，最小值
			}
		}
		int newwidth, newheight;
		newwidth = int(xmax-xmin+0.5f);
		newheight = int(ymax-ymin+0.5f);

		//如果原图的宽度（高度）为偶数（奇数），则旋转后的图像的宽度（高度）也应为偶数（奇数），要保证旋转中心不变
		if(width%2==0)//如果为偶数
		{
			newwidth = (newwidth+1)/2*2;
		}
		else//如果是奇数
		{
			if( newwidth%2==0 )
			{
				newwidth += 1;
			}
		}

		if(height%2==0)//如果为偶数
		{
			newheight = (newheight+1)/2*2;
		}
		else//如果是奇数
		{
			if( newheight%2==0 )
			{
				newheight += 1;
			}
		}

		//开辟内存
		pDst = cvCreateImage( cvSize(newwidth, newheight), 8, 1);
		memset(pDst->imageData, 0, pDst->widthStep*pDst->height);

		int widthStep2 = pDst->widthStep;

		// 计算平移矩阵 (fOffsetX, fOffsetY)
		float fOffsetX, fOffsetY;
		fOffsetX = -fCenterX0*cos_a + fCenterY0*sin_a + (newwidth-1)*0.5f ;
		fOffsetY = -fCenterX0*sin_a - fCenterY0*cos_a + (newheight-1)*0.5f ;

		if( (newwidth>9999) ||(newheight>9999) )
		{
			return false;
		}

		float afPartX[10000] = {0};
		float afPartY[10000] = {0};

		cos_a = cos(-fAngle);
		sin_a = sin(-fAngle);

		// 建立表格，可以查表
		for (v=0; v<newwidth; v++) 
		{
			afPartX[v] = (v-fOffsetX)*cos_a;
			afPartY[v] = (v-fOffsetX)*sin_a;
		}

		unsigned char* pDstData = (unsigned char*)pDst->imageData;
		unsigned char* pSrcData = (unsigned char*)pSrc->imageData;

		//
		unsigned char* ptr = pDstData;
		for(int u=0; u<newheight; u++)
		{

			float ysina = (u-fOffsetY)*sin_a;
			float ycosa = (u-fOffsetY)*cos_a;

			for (v=0; v<newwidth; v++) 
			{
				float fPointtempX = afPartX[v] - ysina;
				float fPointtempY = afPartY[v] + ycosa;

				if( (fPointtempX>=0) && (fPointtempX<width_1) && (fPointtempY>=0) && (fPointtempY<height_1) )
				{
					int i = int(fPointtempY);
					int j = int(fPointtempX);

					unsigned char* pTemp = pSrcData + i*widthstep + j;
					//if( (uchar)*(pTemp)==0 )
					//	continue;

					// 浮点运算
					float i32f = fPointtempY-i;
					float j32f = fPointtempX-j;

					//// 原始版本
					//*(ptr + v) = char((uchar)*(pTemp)*(1-p)*(1-q) + 
					//	(uchar)*(pTemp + 1)*(1-p)*q + 
					//	(uchar)*(pTemp + widthstep)*p*(1-q) + 
					//	(uchar)*(pTemp + widthstep + 1)*p*q );
					//// 原始版本结束...

					// // 版本2
					
					//unsigned char gray1 = (unsigned char)*(pTemp);
					//unsigned char gray2 = (unsigned char)*(pTemp + 1);
					//unsigned char gray3 = (unsigned char)*(pTemp + widthstep);
					//unsigned char gray4 = (unsigned char)*(pTemp + widthstep + 1);
					float gray[16];
					for(int p=0; p<4; p++)
					{
						int r = i + p - 1; //
						for(int q=0; q<4; q++)
						{
							int c = j + q - 1;
							gray[p*4+q] = pSrcData[r*widthstep+c];
						}
					}

					float sy[4], sx[4];
					for (int p=0; p<4; p++)
					{
						float up = i32f + 1 - p; // y
						sy[p] = S3Fun(up);

						float vp = j32f + 1 - p; // x
						sx[p] = S3Fun(vp);
					}

					float ab[4], grayDst;
					MulMatrix(sy, 1, 4, gray, 4, 4, ab);
					MulMatrix(ab, 1, 4, sx, 4, 1, &grayDst);

					if(grayDst<0) grayDst  = 0;
					if(grayDst>=255) grayDst = 255;
					*(ptr + v) = int(grayDst + 0.5f);
					// 版本2结束...
				}
			}
			ptr += widthStep2;
		}
	}

	return pDst;
}

// 函数功能：旋转 图像带来的坐标偏移
int RotateImageOffsetCenter(BitmapImage *src, float angle, float &duR, float &dvR)
{
	if(src==NULL)
	{
		cout<<"Input image can't be NULL"<<endl;
		return 0;
	}

	int u = 0,v = 0;

	int width,height,widthstep;
	int width_1, height_1;

	width=src->width;
	height=src->height;
	widthstep=src->widthStep;

	width_1 = width - 1;
	height_1 = height - 1;

	if( (width>9999) ||(height>9999) )
	{
		printf("图像宽度必须小于10000\n");
		return NULL;
	}

	float cos_a , sin_a;
	float hist[256] = {0};
	float max1=0;
	int maxi1=0;

	SfPoint corner[4];
	corner[0].x=0.0f;		corner[0].y=0.0f;
	corner[1].x=(float)width;  corner[1].y=0.0f;
	corner[2].x=0.0f;		corner[2].y=(float)height;
	corner[3].x=(float)width;  corner[3].y=(float)height;
	float xmax=-9999,xmin=9999,ymax=-9999,ymin=9999;
	cos_a=(float)cos(angle);
	sin_a=(float)sin(angle);

	float fCenterX0 = 0, fCenterY0 = 0, fCenterX1 = 0, fCenterY1 = 0;
	//原图的中心点
	fCenterX0 = (width - 1)/2.0f;
	fCenterY0 = (height - 1)/2.0f;

	//要求：围绕中心点旋转

	for(u=0;u<4;u++)
	{
		corner[u] = TimeRotationMatrix2D(corner[u], cos_a, sin_a);
		if (corner[u].x<xmin) 
		{
			xmin=(float)corner[u].x;
		}
		if (corner[u].x>xmax) 
		{
			xmax=(float)corner[u].x;
		}
		if (corner[u].y<ymin) 
		{
			ymin=(float)corner[u].y;
		}
		if (corner[u].y>ymax) 
		{
			ymax=(float)corner[u].y;// 找出X、Y的最大，最小值
		}
	}
	int newwidth, newheight;
	newwidth = int(xmax-xmin+0.5f);
	newheight = int(ymax-ymin+0.5f);

	//如果原图的宽度（高度）为偶数（奇数），则旋转后的图像的宽度（高度）也应为偶数（奇数），要保证旋转中心不变
	if(width%2==0)//如果为偶数
	{
		newwidth = (newwidth+1)/2*2;
	}
	else//如果是奇数
	{
		if( newwidth%2==0 )
		{
			newwidth += 1;
		}
	}

	if(height%2==0)//如果为偶数
	{
		newheight = (newheight+1)/2*2;
	}
	else//如果是奇数
	{
		if( newheight%2==0 )
		{
			newheight += 1;
		}
	}

	// 计算平移矩阵 (fOffsetX, fOffsetY)
	dvR = -fCenterX0*cos_a + fCenterY0*sin_a + (newwidth-1)/2.f ;
	duR = -fCenterX0*sin_a - fCenterY0*cos_a + (newheight-1)/2.f ;

	return 1;
}

bool Resize(BitmapImage *pSrcImage, BitmapImage *pDstImage)
{
	if( (pSrcImage==NULL) || (pDstImage==NULL) )
		return false;

	int wDst, hDst, wsDst;
	GetImageSize( pDstImage, wDst, hDst, wsDst );

	int wSrc, hSrc, wsSrc;
	GetImageSize( pSrcImage, wSrc, hSrc, wsSrc );

	int hSrc_1 = hSrc - 1;
	int wSrc_1 = wSrc - 1;

	if( (wSrc==wDst) && (hSrc==hDst) )
	{
		memcpy( pDstImage->imageData, pSrcImage->imageData, wsDst*hDst );
		return true;
	}

	float fSx, fSy;
	fSx = (float)wSrc/wDst;
	fSy = (float)hSrc/hDst;

	if(pSrcImage->nChannels==1)
	{
		for(int i=0; i<hDst; i++)
		{
			unsigned char* pRowDst = pDstImage->imageData + i*wsDst;
			float fSrcY = i*fSy;
			if( (fSrcY<0)||(fSrcY>=hSrc_1) )
				continue;
			for(int j=0; j<wDst; j++)
			{
				float fSrcX = j*fSx;
				if( (fSrcX<0)||(fSrcX>=wSrc_1) )
					continue;
				// 双线性插值
				int ySrc = int(fSrcY);
				int xSrc = int(fSrcX);

				float p = fSrcY - ySrc;
				float q = fSrcX - xSrc;

				unsigned char* pSrcTemp = pSrcImage->imageData + ySrc*wsSrc + xSrc;

				*(pRowDst + j) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
					*(pSrcTemp + 1)*(1-p)*q + 
					*(pSrcTemp + wsSrc)*p*(1-q) + 
					*(pSrcTemp + wsSrc + 1)*p*q );
			}
		}
	}
	else if(pSrcImage->nChannels==3) // 彩色图像
	{ 
		for(int i=0; i<hDst; i++)
		{
			unsigned char* pRowDst = pDstImage->imageData + i*wsDst;
			float fSrcY = i*fSy;
			if( (fSrcY<0)||(fSrcY>=hSrc_1) )
				continue;
			for(int j=0; j<wDst; j++)
			{
				float fSrcX = j*fSx;
				if( (fSrcX<0)||(fSrcX>=wSrc_1) )
					continue;
				// 双线性插值
				int ySrc = int(fSrcY);
				int xSrc = int(fSrcX);

				float p = fSrcY - ySrc;
				float q = fSrcX - xSrc;

				unsigned char* pSrcTemp = pSrcImage->imageData + ySrc*wsSrc + 3*xSrc;

				*(pRowDst + 3*j) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
					*(pSrcTemp + 3)*(1-p)*q + 
					*(pSrcTemp + wsSrc)*p*(1-q) + 
					*(pSrcTemp + wsSrc + 3)*p*q );

				pSrcTemp = pSrcImage->imageData + ySrc*wsSrc + 3*xSrc+1;
				*(pRowDst + 3*j+1) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
					*(pSrcTemp + 3)*(1-p)*q + 
					*(pSrcTemp + wsSrc)*p*(1-q) + 
					*(pSrcTemp + wsSrc + 3)*p*q );

				pSrcTemp = pSrcImage->imageData + ySrc*wsSrc + 3*xSrc+2;
				*(pRowDst + 3*j+2) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
					*(pSrcTemp + 3)*(1-p)*q + 
					*(pSrcTemp + wsSrc)*p*(1-q) + 
					*(pSrcTemp + wsSrc + 3)*p*q );
			}
		}
		// 处理边界
		for(int i=0; i<hDst; i++)
		{
			unsigned char* pRowDst = pDstImage->imageData + i*wsDst;
			float fSrcY = i*fSy;
			
			for(int j=0; j<wDst; j++)
			{
				float fSrcX = j*fSx;
				if( (fSrcX>=wSrc_1) ||(fSrcY>=hSrc_1) )
				{	
				}
				else
				{
					continue;
				}
				// 双线性插值
				int ySrc = int(fSrcY);
				int xSrc = int(fSrcX);

				float p = fSrcY - ySrc;
				float q = fSrcX - xSrc;

				unsigned char* pSrcTemp = NULL; 
				// blue
				pSrcTemp = pSrcImage->imageData + ySrc*wsSrc + 3*xSrc;
				*(pRowDst + 3*j) = *(pSrcTemp);
				// green
				pSrcTemp = pSrcImage->imageData + ySrc*wsSrc + 3*xSrc+1;
				*(pRowDst + 3*j+1) = *(pSrcTemp);
				// red
				pSrcTemp = pSrcImage->imageData + ySrc*wsSrc + 3*xSrc+2;
				*(pRowDst + 3*j+2) = *(pSrcTemp);
			}
		}
	}

	return true;
}

int SubImage128(BitmapImage* pSrc1, BitmapImage* pSrc2, BitmapImage* pDst)
{
	int w, h, ws;
	GetImageSize(pSrc1, w, h, ws);
	if(pSrc1->nChannels==1)
	{
		for(int y=0; y<h; y++)
		{
			unsigned char* pRowSrc1 = pSrc1->imageData + h*ws;
			unsigned char* pRowSrc2 = pSrc2->imageData + h*ws;
			unsigned char* pRowDst = pSrc1->imageData + h*ws;
			for(int x=0; x<w; x++)
			{
				int gray = *(pRowSrc1 + x) - *(pRowSrc2 + x) + 128;
				gray = Min(gray, 255);
				gray = Max(gray, 0);
				*(pRowDst + x) = gray;
			}
		}
	}
	else if(pSrc1->nChannels==3)
	{
		for(int y=0; y<h; y++)
		{
			unsigned char* pRowSrc1 = pSrc1->imageData + y*ws;
			unsigned char* pRowSrc2 = pSrc2->imageData + y*ws;
			unsigned char* pRowDst = pDst->imageData + y*ws;
			for(int x=0; x<w; x++)
			{
				*(pRowDst + 3*x) = Max( Min(*(pRowSrc1 + 3*x) - *(pRowSrc2 + 3*x) + 128, 255), 0);
				*(pRowDst + 3*x+1) = Max( Min(*(pRowSrc1 + 3*x+1) - *(pRowSrc2 + 3*x+1) + 128, 255), 0);
				*(pRowDst + 3*x+2) = Max( Min(*(pRowSrc1 + 3*x+2) - *(pRowSrc2 + 3*x+2) + 128, 255), 0);
			}
		}
	}

	return 0;
}

// 建立拉普拉斯金字塔
int BuildLaplacianPyramid(const pool::BitmapImage* pSrcImage, const int pyramidLevel, const float sigma, 
						  pool::BitmapImage* pLapPyramImgs[10])
{
	// 首先创建高斯(均值)金字塔
	pool::BitmapImage* pGaussPyramImgs[10] = {NULL};
	CreateImagesPyramid(pSrcImage, pyramidLevel, pGaussPyramImgs);
	
	pool::BitmapImage* pReconstructed = CloneBitmap8U( pGaussPyramImgs[pyramidLevel-1] );
	pLapPyramImgs[pyramidLevel-1] = CloneBitmap8U( pReconstructed );
	for(int n=pyramidLevel-2; n>=0; n--)
	{
		BitmapImage* pCurrentLevel = CreateBitmap8U( pGaussPyramImgs[n]->width, pGaussPyramImgs[n]->height, pGaussPyramImgs[n]->nChannels );
		Resize(pReconstructed, pCurrentLevel);
		ReleaseBitmap8U(pReconstructed);
		pReconstructed = pCurrentLevel;
		pLapPyramImgs[n] = CreateBitmap8U( pGaussPyramImgs[n]->width, pGaussPyramImgs[n]->height, pGaussPyramImgs[n]->nChannels );
		//SaveImageATL(pGaussPyramImgs[n], "c:\\gaussPyramid.bmp");
		//SaveImageATL(pReconstructed, "c:\\reconstructed.bmp");
		SubImage128(pGaussPyramImgs[n], pReconstructed, pLapPyramImgs[n]);
		
		//SaveImageATL(pLapPyramImgs[n], "c:\\lapPyramImgs.bmp");
	}

	ReleaseBitmap8U(pReconstructed);
	ReleasePyramid(pGaussPyramImgs, pyramidLevel);
	return 0;
}

//创建图像金字塔
int CreateImagesPyramid(const BitmapImage* pSrcImage, int dwPyramidLevel,
						BitmapImage* apPyramidImages[10])
{
	if(pSrcImage==NULL)
	{
		return 0;
	}

	if( (dwPyramidLevel<=0) || (dwPyramidLevel>10) )
	{
		return 0;
	}

	// 清空原来的内存
	for(int u=0; u<10; u++)
	{
		if(apPyramidImages[u] != NULL )
		{
			ReleaseBitmap8U( apPyramidImages[u] );
		}
	}

	int width, height;
	width = pSrcImage->width;
	height = pSrcImage->height;

	apPyramidImages[0] = CloneBitmap8U( pSrcImage );

	int wNew = width, hNew = height;
	for(int u=1; u<dwPyramidLevel; u++) 
	{
		wNew /= 2;
		hNew /= 2;

		apPyramidImages[u] = CreateBitmap8U(wNew, hNew, pSrcImage->nChannels);

		//Resize(apPyramidImages[u-1], apPyramidImages[u]);	//缩放源图像到目标图像
		DownSampleByScaleTwo( apPyramidImages[u-1], apPyramidImages[u] );
#ifdef _DEBUG
		//SaveImageATL( apPyramidImages[u], _T("c:\\pyramid.bmp") );
#endif
	}

	return 1;
}

//创建图像金字塔
int BuildGaussPyramid(const BitmapImage* pSrcImage, int dwPyramidLevel, float sigma, 
						BitmapImage* pGaussPyramid[10])
{
	if(pSrcImage==NULL)
	{
		return 0;
	}

	if( (dwPyramidLevel<=0) || (dwPyramidLevel>10) )
	{
		return 0;
	}

	// 清空原来的内存
	for(int u=0; u<10; u++)
	{
		if(pGaussPyramid[u] != NULL )
		{
			ReleaseBitmap8U( pGaussPyramid[u] );
		}
	}

	int width, height;
	width = pSrcImage->width;
	height = pSrcImage->height;

	pGaussPyramid[0] = CloneBitmap8U( pSrcImage );

	int wNew = width, hNew = height;
	for(int u=1; u<dwPyramidLevel; u++) 
	{
		wNew /= 2;
		hNew /= 2;

		pGaussPyramid[u] = CreateBitmap8U(wNew, hNew, pSrcImage->nChannels);

		BitmapImage* pSmooth = CreateBitmap8U(pGaussPyramid[u-1]->width, pGaussPyramid[u-1]->height, pGaussPyramid[u-1]->nChannels);
		GaussFilter_Int(pGaussPyramid[u-1], pSmooth, sigma, 5);
		DownSampleByScaleTwo( pSmooth, pGaussPyramid[u] );

#ifdef _DEBUG
		//SaveImageATL( pGaussPyramid[u], _T("c:\\pyramid.bmp") );
#endif
	}

	return 1;
}

// 释放图像金字塔
bool ReleasePyramid(pool::BitmapImage* apPyramidImages[], int nLevel)
{
	for(int n=0; n<nLevel; n++)
	{
		ReleaseBitmap8U( apPyramidImages[n] );
	}
	return true;
}

// 向下采样, 2倍尺度
bool DownSampleByScaleTwo(const pool::BitmapImage* pSrc, pool::BitmapImage*pDst)
{
	if( (pSrc==NULL) || (pDst==NULL) )
		return false;

	int w, h, ws;
	GetImageSize( pSrc, w, h, ws );
	int wDst, hDst, wsDst;
	GetImageSize( pDst, wDst, hDst, wsDst );

	if(pSrc->nChannels==1)
	{
		int yDst = 0;
		for(int y=0; y+1<h; y+=2)
		{
			int xDst = 0;
			for(int x=0; x+1<w; x+=2)
			{
				unsigned char g00, g01, g10, g11;
				g00 = *(pSrc->imageData + y * ws + x);
				g01 = *(pSrc->imageData + y * ws + x + 1);
				g10 = *(pSrc->imageData + (y+1) * ws + x);
				g11 = *(pSrc->imageData + (y+1) * ws + x + 1);
				unsigned char meanGray = (unsigned char)( (g00 + g01 + g10 + g11)*0.25f + 0.5f );

				*(pDst->imageData + yDst*wsDst + xDst) = meanGray;

				xDst++;
			}
			yDst++;
		}
	}
	else if(pSrc->nChannels==3) // 彩色图像
	{
		int yDst = 0;
		for(int y=0; y+1<h; y+=2)
		{
			int xDst = 0;
			for(int x=0; x+1<w; x+=2)
			{
				unsigned char g00, g01, g10, g11;

				g00 = *( pSrc->imageData + y * ws + 3*x );
				g01 = *( pSrc->imageData + y * ws + 3*(x+1) );
				g10 = *( pSrc->imageData + (y+1) * ws + 3*x );
				g11 = *( pSrc->imageData + (y+1) * ws + 3*(x+1) );
				unsigned char meanGray = (unsigned char)( (g00 + g01 + g10 + g11)*0.25f + 0.5f );
				*(pDst->imageData + yDst*wsDst + 3*xDst) = meanGray;

				g00 = *( pSrc->imageData + y * ws + 3*x + 1 );
				g01 = *( pSrc->imageData + y * ws + 3*(x+1) + 1 );
				g10 = *( pSrc->imageData + (y+1) * ws + 3*x + 1 );
				g11 = *( pSrc->imageData + (y+1) * ws + 3*(x+1) + 1 );
				meanGray = (unsigned char)( (g00 + g01 + g10 + g11)*0.25f + 0.5f );
				*(pDst->imageData + yDst*wsDst + 3*xDst+1) = meanGray;

				g00 = *( pSrc->imageData + y * ws + 3*x + 2);
				g01 = *( pSrc->imageData + y * ws + 3*(x+1) + 2 );
				g10 = *( pSrc->imageData + (y+1) * ws + 3*x + 2);
				g11 = *( pSrc->imageData + (y+1) * ws + 3*(x+1) + 2 );
				meanGray = (unsigned char)( (g00 + g01 + g10 + g11)*0.25f + 0.5f );
				*(pDst->imageData + yDst*wsDst + 3*xDst + 2) = meanGray;

				xDst++;
			}
			yDst++;
		}
	}

	return true;
}

// 灰度图转彩色图
bool GrayToColor(const BitmapImage* pSrcGray, BitmapImage* pDstColor)
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

// 函数功能：对图像进行缩放
// 变换前后，中心点不变
bool ScalingImage(BitmapImage* pSrc, BitmapImage* pDst, float scale)
{
	if( (pSrc==NULL) || (pDst==NULL) )
		return false;

	if(scale==1)
	{
		memcpy( pDst->imageData, pSrc->imageData, pSrc->widthStep * pSrc->height );
	}
	else
	{
		// 先置0
		memset(pDst->imageData, 0, pDst->widthStep * pDst->height);
		int wDst, hDst, wsDst;
		int wSrc, hSrc, wsSrc;
		GetImageSize( pSrc, wSrc, hSrc, wsSrc );
		GetImageSize( pDst, wDst, hDst, wsDst );
		float xc2 = (wDst - 1) * 0.5f;
		float yc2 = (hDst - 1) * 0.5f;
		float xc1 = (wSrc - 1) * 0.5f;
		float yc1 = (hSrc - 1) * 0.5f;
		int wSrc_1 = wSrc - 1;
		int hSrc_1 = hSrc - 1;

		for(int yDst = 0; yDst<hDst; yDst++)
		{
			unsigned char *pRowDst = pDst->imageData + yDst * wsDst;
			for(int xDst = 0; xDst<wDst; xDst++)
			{
				float x0 = (xDst - xc2)/scale + xc1;
				float y0 = (yDst - yc2)/scale + yc1;
				// 双线性插值
				if( (x0>=0) && (x0<wSrc_1) && (y0>=0) && (y0<hSrc_1) )
				{
					int i = int(y0);
					int j = int(x0);

					unsigned char* pTemp = pSrc->imageData + i*wsSrc + j;

					// 浮点运算
					float p = y0 - i;
					float q = x0 - j;

					unsigned char gray1 = (unsigned char)*(pTemp);
					unsigned char gray2 = (unsigned char)*(pTemp + 1);
					unsigned char gray3 = (unsigned char)*(pTemp + wsSrc);
					unsigned char gray4 = (unsigned char)*(pTemp + wsSrc + 1);

					pRowDst[xDst] = unsigned char( gray1*(1-p)*(1-q) + 
						gray2*(1-p)*q + 
						gray3*p*(1-q) + 
						gray4*p*q );
					// 版本2结束...
				}
			}
		}
	}
	return true;
}

// 函数功能：只在X方向做涨缩
bool ScalePolarOnlyX(pool::BitmapImage* pSrc, pool::BitmapImage* pDst, float scale)
{
	if( (pSrc==NULL) || (pDst==NULL) )
		return false;

	if(scale==1)
	{
		memcpy( pDst->imageData, pSrc->imageData, pSrc->widthStep * pSrc->height );
	}
	else
	{
		// 先置0
		memset(pDst->imageData, 0, pDst->widthStep * pDst->height);
		int wDst, hDst, wsDst;
		int wSrc, hSrc, wsSrc;
		GetImageSize( pSrc, wSrc, hSrc, wsSrc );
		GetImageSize( pDst, wDst, hDst, wsDst );

		int wSrc_1 = wSrc - 1;
		int hSrc_1 = hSrc - 1;

		for(int yDst = 0; yDst<hDst; yDst++)
		{
			unsigned char *pRowDst = pDst->imageData + yDst * wsDst;
			int i = yDst;
			for(int xDst = 0; xDst<wDst; xDst++)
			{
				float x0 = xDst / scale;
				
				// 双线性插值
				if( (x0>=0) && (x0<wSrc_1) )
				{
					int j = int(x0);
					unsigned char* pTemp = pSrc->imageData + i*wsSrc + j;

					// 浮点运算
					float q = x0 - j;

					unsigned char gray1 = (unsigned char)*(pTemp);
					unsigned char gray2 = (unsigned char)*(pTemp + 1);

					pRowDst[xDst] = unsigned char( gray1*(1-q) + gray2*q  );
					// 版本2结束...
				}
			}
		}
	}

	return true;
}

// 函数功能：给定原尺寸图像的polar-image, 生成尺度空间中的其他的polar-image
bool ScalingPolarImage(pool::BitmapImage* pSrcPolar, 
					   float minScale, float maxScale,
					   ScaledImage** &pScaledImages, int &nScaledImages)
{
	const float MAX_SCALE = 6.5f;
	if(pSrcPolar==NULL)
		return false;
	if( (minScale<1.0f/MAX_SCALE) || (maxScale>MAX_SCALE) )
		return false;

	int wM = pSrcPolar->width;
	float scaleStep = 2.0f / wM;
	int nMaxScaledImages = int( log(maxScale/minScale) / log(scaleStep + 1) ) + 2;
	pScaledImages = new ScaledImage*[nMaxScaledImages];
	// 先处理放大的
	float currentScale = 1;
	do
	{
		ScaledImage* pScaledImage = new ScaledImage();
		if(currentScale==1)
		{
			pScaledImage->pImage = CloneBitmap8U( pSrcPolar );
			pScaledImage->scale = currentScale;
			pScaledImage->validWidth = pScaledImage->pImage->width;
		}
		else
		{
			BitmapImage *pCurrentImage = CreateBitmap8U( wM, wM, 1 );
			ScalePolarOnlyX(pScaledImages[nScaledImages-1]->pImage, pCurrentImage, 1+scaleStep);
			pScaledImage->pImage = pCurrentImage;
			pScaledImage->scale = currentScale;
			pScaledImage->validWidth = int(wM * currentScale);
		}
		pScaledImages[nScaledImages] = pScaledImage;
		nScaledImages++;

		currentScale *= (1 + scaleStep);
	}
	while(currentScale<maxScale);

	// 处理缩小
	currentScale = 1/(1+scaleStep);
	do
	{
		BitmapImage *pCurrentImage = CreateBitmap8U( wM, wM, 1 );
		ScaledImage* pScaledImage = new ScaledImage();

		ScalePolarOnlyX(pScaledImages[0]->pImage, pCurrentImage, 1-scaleStep);
		pScaledImage->pImage = pCurrentImage;
		pScaledImage->scale = currentScale;
		pScaledImage->validWidth = int(wM * currentScale);

		currentScale /= (1 + scaleStep); // 更新尺度

		// 把所有的元素后移一位
		for(int n=nScaledImages-1; n>=0; n--)
		{
			pScaledImages[n+1] = pScaledImages[n];
		}

		pScaledImages[0] = pScaledImage;
		nScaledImages++;
	}
	while(currentScale>minScale);
	
	return true;
}

// 函数功能：对金字塔的最顶层的图像进行缩放
// pImagePyramid---图像的金字塔
// nScaledImages---尺度变换后的图像的个数
// nPyramid--------必须要大于3
bool ScalingImage(BitmapImage* pImagePyramid[], int nPyramid, 
				  float minScale, float maxScale,
				  ScaledImage** &pScaledImages, int &nScaledImages)
{
	if(pImagePyramid==NULL)
		return false;
	const float MAX_SCALE = 6.5f;
	if( (minScale<1.0f/MAX_SCALE) || (maxScale>MAX_SCALE) )
		return false;

	nScaledImages = 0;
	if(nPyramid<3)
		return false;

	int wM = pImagePyramid[nPyramid-1]->width;
	float scaleStep = 2.0f / wM;
	int nMaxScaledImages = int( log(maxScale/minScale) / log(scaleStep + 1) ) + 2;
	pScaledImages = new ScaledImage*[nMaxScaledImages];
	// 先处理放大的
	float currentScale = 1;
	do
	{
		ScaledImage* pScaledImage = new ScaledImage();
		if(currentScale==1)
		{
			pScaledImage->pImage = CloneBitmap8U( pImagePyramid[nPyramid-1] );
			pScaledImage->scale = currentScale;
			pScaledImage->validWidth = pScaledImage->pImage->width;
		}
		else
		{
			BitmapImage *pCurrentImage = CreateBitmap8U( wM, wM, 1 );
			for(int n=nPyramid-1; n>=1; n--)
			{
				//float currentLevelScale = (float)(1<<(nPyramid - 1 - n));
				float smallS = (float)(1<<(nPyramid - 1 - n));
				float bigS = (float)(1<<(nPyramid - 1 - n + 1));
				if( (currentScale >= smallS) && (currentScale<bigS) )
				{
					int nSelected = 0;
					if(abs(smallS-currentScale) < abs(bigS-currentScale))
						nSelected = n;
					else 
						nSelected = n - 1;
					// 从第n层做缩放得到目标图像
					float currentLevelScale = (float)(1<<(nPyramid - 1 - nSelected));
					float scale2 = currentScale / currentLevelScale;
					ScalingImage( pImagePyramid[nSelected], pCurrentImage, scale2);
					pScaledImage->pImage = pCurrentImage;
					pScaledImage->scale = currentScale;
					pScaledImage->validWidth = pScaledImage->pImage->width;
					break;
				}
			}
		}
		currentScale *= (1 + scaleStep);

#ifdef _DEBUG
		//SaveImageATL(pScaledImage->pImage, "c:\\scaled.bmp" );
#endif

		pScaledImages[nScaledImages] = pScaledImage;
		nScaledImages++;
	}
	while(currentScale<maxScale);

	currentScale = 1/(1+scaleStep);
	// 处理缩小的
	BitmapImage* pPyramidDown[10] = {NULL};
	CreateImagesPyramid( pImagePyramid[nPyramid-1], 3, pPyramidDown ); // 创建一个3层的金字塔
	do
	{
		BitmapImage *pCurrentImage = CreateBitmap8U( wM, wM, 1 );
		ScaledImage* pScaledImage = new ScaledImage();
		for(int n=0; n<3; n++)
		{
			//float currentLevelScale = 1/float(1<<n);
			float bigS = 1 / float(1<<n);
			float smallS = 1 / float( 1<<(n+1) );
			if( (currentScale >= smallS) && (currentScale<bigS) )
			{
				int nSelected = 0;
				if(abs(smallS-currentScale) < abs(bigS-currentScale))
					nSelected = n+1;
				else 
					nSelected = n;
				float currentLevelScale = 1 / float(1<<nSelected);
				// 从第n层做缩放得到目标图像
				float scale2 = currentScale / currentLevelScale;
				ScalingImage( pPyramidDown[nSelected], pCurrentImage, scale2);
				pScaledImage->pImage = pCurrentImage;
				pScaledImage->scale = currentScale;
				pScaledImage->validWidth = int(pImagePyramid[nPyramid-1]->width * currentScale);
				break;
			}
		}
#ifdef _DEBUG
		//SaveImageATL(pScaledImage->pImage, "c:\\scaled.bmp" );
#endif
		currentScale /= (1 + scaleStep); // 更新尺度
		// 把所有的元素后移一位
		for(int n=nScaledImages-1; n>=0; n--)
		{
			pScaledImages[n+1] = pScaledImages[n];
		}
		pScaledImages[0] = pScaledImage;
		nScaledImages++;
	}
	while(currentScale>minScale);

	for(int i=0; i<3; i++)
	{
		ReleaseBitmap8U( pPyramidDown[i] );
	}
	return true;
}

// 把图像向右旋转90度
int Rotate90Right(IplImage* pSrc, IplImage* pDst)
{
	if( (pSrc==NULL) || (pDst==NULL) )
	{
		return -1;
	}
	int w = pSrc->width;
	int h = pSrc->height;
	int ws = pSrc->widthStep;
	int wD = pDst->width;
	int hD = pDst->height;
	int wsD = pDst->widthStep;

	for(int y2 = 0; y2<hD; y2++)
	{
		for(int x2=0; x2<wD; x2++)
		{
			int x = y2;         ///
			int y = h-1-x2;   ///

			*(pDst->imageData + y2*wsD + x2) = *(pSrc->imageData + y*ws + x);
		}
	}

	return 0;
}

// 把图像向右旋转90度
int Rotate90Left(IplImage* pSrc, IplImage* pDst)
{
	if( (pSrc==NULL) || (pDst==NULL) )
	{
		return -1;
	}
	int w = pSrc->width;
	int h = pSrc->height;
	int ws = pSrc->widthStep;
	int wD = pDst->width;
	int hD = pDst->height;
	int wsD = pDst->widthStep;

	for(int y2 = 0; y2<hD; y2++)
	{
		for(int x2=0; x2<wD; x2++)
		{
			int x = w-1-y2;
			int y = x2;

			*(pDst->imageData + y2*wsD + x2) = *(pSrc->imageData + y*ws + x);
		}
	}

	return 0;
}

// 把图像旋转180度
BitmapImage* Rotate180(pool::BitmapImage* pSrc)
{
	if(pSrc==NULL)
		return NULL;

	BitmapImage *pDst = CreateBitmap8U( pSrc->width, pSrc->height, pSrc->nChannels );
	int w, h, ws;
	GetImageSize( pSrc, w, h, ws );

	for(int y = 0; y<h; y++)
	{
		unsigned char *pSrcRow = pSrc->imageData + y * ws;
		int yDst = -y + h - 1;
		unsigned char *pDstRow = pDst->imageData + yDst * ws;
		for(int x = 0; x<w; x++)
		{
			int xDst = -x + w - 1;
			pDstRow[xDst] = pSrcRow[x];
		}
	}

	return pDst;
}

// 仿射变换(以图像的中心为变换中心)
bool AffineTransform(const pool::BitmapImage* pSrc, pool::BitmapImage* pDst, 
					 float a, float b, float c, float d)
{
	if( (pSrc==NULL) || (pDst==NULL) )
		return NULL;

	int w, h, ws;
	GetImageSize( pSrc, w, h, ws );
	int wSrc_1 = w - 1;
	int hSrc_1 = h - 1;

	float centerX = (w-1) * 0.5f;
	float centerY = (h-1) * 0.5f;

	for(int yDst= 0; yDst<h; yDst++)
	{
		unsigned char *pRowDst = pDst->imageData + yDst * ws;
		for(int xDst=0; xDst<w; xDst++)
		{
			float x2 = xDst - centerX;
			float y2 = yDst - centerY;

			float x1 = (d*x2 - b*y2) / (a*d-b*c);
			float y1 = (c*x2 - a*y2) / (b*c-a*d);
			float x0 = x1 + centerX;
			float y0 = y1 + centerY;
			// 双线性插值
			if( (x0>=0) && (x0<wSrc_1) && (y0>=0) && (y0<hSrc_1) )
			{
				int i = int(y0);
				int j = int(x0);

				unsigned char* pTemp = pSrc->imageData + i*ws + j;

				// 浮点运算
				float p = y0 - i;
				float q = x0 - j;

				unsigned char gray1 = (unsigned char)*(pTemp);
				unsigned char gray2 = (unsigned char)*(pTemp + 1);
				unsigned char gray3 = (unsigned char)*(pTemp + ws);
				unsigned char gray4 = (unsigned char)*(pTemp + ws + 1);

				pRowDst[xDst] = unsigned char( gray1*(1-p)*(1-q) + 
					gray2*(1-p)*q + 
					gray3*p*(1-q) + 
					gray4*p*q );
				// 版本2结束...
			}
		}
	}

	return true;
}

// 函数功能：在图片中截取图片，给定两个 对角点
// 新分配了内存
IplImage* CutPatchImage(IplImage* src,
						pool::IntPoint startp, pool::IntPoint endp)
{
	if(src==NULL)
	{ 
		std::cout<<"Error:Input source image wrong in CutPatchImage()!"<<std::endl;
		return NULL;
	}

	int width = 0, height = 0, widthstep = 0;
	//GetImageSize( src, width, height, widthstep );
	width = src->width;
	height = src->height;
	widthstep = src->widthStep;

	// 判断，防止出界
	if( (startp.y<0) || (startp.y>height-1) )
		return false;
	if( (startp.x<0) || (startp.x>width-1) ) 
		return false;
	if( (endp.y>height-1) || (endp.y<0) ) 
		return false;
	if( (endp.x>width-1) || (endp.x<0) )
		return false;
	// 边界判断结束......

	int newHeight = 0, newWidth = 0;
	newHeight = endp.y-startp.y+1;
	newWidth = endp.x-startp.x+1;

	if( (newWidth<=0) || (newHeight<=0) ) 
		return false;

	int nChannels = src->nChannels;
	int depth = src->depth;

	IplImage* dst = NULL;
	dst = cvCreateImage( cvSize(newWidth, newHeight), depth, src->nChannels );

	if(dst->imageData==NULL)
		return false;

	int newwidthstep = dst->widthStep;

	int uStart,uEnd;
	int vStart,vEnd;
	uStart = startp.y;
	uEnd = endp.y;
	vStart = startp.x;
	vEnd = endp.x;

	if( (nChannels==1) && (depth==8) )
	{
		int i = 0;
		for(int u=uStart; u<=uEnd; u++) 
		{
			unsigned char* pDstRow = (unsigned char*)dst->imageData + i*newwidthstep;
			unsigned char* pSrcRow = (unsigned char*)src->imageData + u*widthstep;
			int j=0;
			for(int v=vStart; v<=vEnd; v++) 
			{
				*(pDstRow + j) = (unsigned char)*(pSrcRow + v);
				j++;			
			}
			i++;
		}
	}
	else if( (nChannels==3) && (depth==8) )
	{
		int i = 0;
		for(int u=uStart; u<=uEnd; u++) 
		{
			unsigned char* pDstRow = (unsigned char*)dst->imageData + i * newwidthstep;
			unsigned char* pSrcRow = (unsigned char*)src->imageData + u * widthstep;
			int j=0;
			for(int v=vStart; v<=vEnd; v++) 
			{
				*(pDstRow + 3*j) = (unsigned char)*(pSrcRow + 3*v);
				*(pDstRow + 3*j + 1) = (unsigned char)*(pSrcRow + 3*v + 1);
				*(pDstRow + 3*j + 2) = (unsigned char)*(pSrcRow + 3*v + 2);
				j++;			
			}
			i++;
		}
	}
	else if( (nChannels==1) && (depth==64) )
	{
		int wP = vEnd - vStart + 1;
		int i = 0;
		for(int u=uStart; u<=uEnd; u++) 
		{
			double* pSrcRow = (double*)src->imageData + u*width + vStart;
			double* pDstRow = (double*)dst->imageData + i*newWidth;
			
			memcpy(pDstRow, pSrcRow, wP*sizeof(double));

			i++;
		}
	}

	return dst;
}

// 由三角形的三条边计算三角的三个角度
void AngleFrom3Edges( double a, double b, double c, 
					   double &A, double &B, double &C )
{
	double cosA = (b*b + c*c - a*a) / (2*b*c);
	double cosB = (a*a + c*c - b*b) / (2*a*c);
	double cosC = (a*a + b*b - c*c) / (2*a*b);
	A = acos(cosA);
	B = acos(cosB);
	C = acos(cosB);
	return;
}

// 先绕Z转, 再Y, 最后X
void GetR_ZYX(double ax, double ay, double az, double R[9])
{
	double Rx[9]={1,	0,	0,
		0, cos(ax), -sin(ax),
		0, sin(ax), cos(ax)};

	double Ry[9]={cos(ay),	0,	-sin(ay),
		0,		1,		0,
		sin(ay), 0,	cos(ay)};

	double Rz[9]={  cos(az), -sin(az), 0,
		sin(az), cos(az),  0,
		0,	0,	 1};
	double temp[9];
	//R = Rx*Ry*Rz
	MulMatrix(Rx, 3, 3, Ry, 3, 3, temp);
	MulMatrix(temp, 3, 3, Rz, 3, 3, R);
}

// 旋转平面 
// 对平面进行旋转，使得与X0Y平面平行
// 平面为ax + by + cz + d = 0;
// AZ为绕Z轴旋转的角度
// AY为绕Y轴旋转的角度
// 先绕Z轴旋转, 再绕Y轴旋转
int RotatePlane( double a, double b, double c, double &AZ, double &AY )
{
	if(c<0)
	{
		a *= -1;
		b *= -1;
		c *= -1;
	}

	// 先绕Z轴旋转
	AngleofPoint360(a, b, AZ);

	AZ = -AZ;

	// 
	double newX = sqrt(a*a + b*b);

	double B = atan(c/newX);

	AY = (pool::PI_64F/2.0 - B);

	//AY = 0;

	return 0;
}

int Top2Down(IplImage* src)
{
	int w, h, ws;
	CvGetImageSize(src, w, h, ws);

	IplImage* temp = cvCloneImage(src);

	for(int r=0; r<h; r++)
	{
		memcpy(src->imageData+r*ws, temp->imageData + (h-1-r)*ws, ws);
	}

	cvReleaseImage(&temp);

	return 0;
}