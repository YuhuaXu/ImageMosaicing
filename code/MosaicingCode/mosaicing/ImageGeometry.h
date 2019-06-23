

// 图像几何
#pragma once
#include "Bitmap.h"
#include "usingCV24.h"

struct ScaledImage
{
	pool::BitmapImage* pImage; // 当前图像指针
	float scale; // 当前图像对应的尺度
	int validWidth;  // 当前图像的有效宽度
};
 
//功能：围绕中心旋转
pool::BitmapImage* RotateImageCenter2( pool::BitmapImage *pSrc, const float fAngle);

//功能：围绕中心旋转 (三次卷积) 
// Yuhua Xu, 2016-11-10
IplImage* RotateImageCenterCubic( IplImage *pSrc, const float fAngle);

IplImage* RotateImageCenterBilinear( IplImage *pSrc, const float fAngle);

// 三次卷积插值函数
float S3Fun(float x);

// 双线性插值
bool Resize(pool::BitmapImage *pSrcImage, pool::BitmapImage *pDstImage);

// 函数功能：旋转 图像带来的坐标偏移
int RotateImageOffsetCenter(pool::BitmapImage *src, float angle, float &duR, float &dvR);

// 创建图像金字塔
int CreateImagesPyramid(const pool::BitmapImage* pSrcImage, int dwPyramidLevel,
						pool::BitmapImage* apPyramidImages[10]);

// 创建高斯金字塔
int BuildGaussPyramid(const pool::BitmapImage* pSrcImage, int dwPyramidLevel, float sigma, 
					  pool::BitmapImage* pGaussPyramid[10]);

// 建立拉普拉斯金字塔
int BuildLaplacianPyramid(const pool::BitmapImage* pSrcImage, const int pyramidLevel, const float sigma, 
						  pool::BitmapImage* apPyramidImages[10]);

bool ReleasePyramid(pool::BitmapImage* apPyramidImages[], int nLevel);

// 图像平移2011-01-17
bool Translation( const pool::BitmapImage*  pSrcImage,  pool::BitmapImage* _OUT pDstImage, 
				  const float  deltaX,  const float  deltaY);

// 图像平移2011-01-17
bool Translation( const IplImage*  pSrcImage, IplImage* _OUT pDstImage, 
				 const float  deltaX,  const float  deltaY);

// 图像平移x
bool TranslationX( const IplImage*  pSrcImage, IplImage* _OUT pDstImage, 
				  const float  deltaX);

// 向下采样, 2倍尺度
bool DownSampleByScaleTwo(const pool::BitmapImage* pSrc, pool::BitmapImage*pDst);

// mirror, 0为X方向镜像, 1为Y方向镜像
bool ImageMirror( const pool::BitmapImage* pSrcImage, pool::BitmapImage* pDstImage, int mirrorType);

// mirror, 0为X方向镜像, 1为Y方向镜像
bool ImageMirror( pool::BitmapImage* pSrcImage, int mirrorType);

// 灰度图转彩色图
bool GrayToColor(const pool::BitmapImage* pSrcGray, pool::BitmapImage* pDstColor);

// 函数功能：对图像进行缩放, 图像的尺寸不变
// 变换前后，中心点不变
bool ScalingImage(pool::BitmapImage* pSrc, pool::BitmapImage* pDst, float scale);

// 函数功能：对金字塔的最顶层的图像进行缩放
// pImagePyramid---图像的金字塔
// nScaledImages---尺度变换后的图像的个数
// nPyramid--------必须要大于3
bool ScalingImage(pool::BitmapImage* pImagePyramid[], int nPyramid, 
				  float minScale, float maxScale,
				  ScaledImage** &pScaledImages, int &nScaledImages);

// 函数功能：给定原尺寸图像的polar-image, 生成尺度空间中的其他的polar-image
bool ScalingPolarImage(pool::BitmapImage* pSrcPolar, 
					   float minScale, float maxScale,
					   ScaledImage** &pScaledImages, int &nScaledImages);

// 把图像旋转180度
pool::BitmapImage* Rotate180(pool::BitmapImage* pSrc);

// 把图像向右旋转90度
// 
int Rotate90Right(IplImage* pSrc, IplImage* pDst);

// 把图像向左旋转90度
// y = x2
// x = w-1-y2
int Rotate90Left(IplImage* pSrc, IplImage* pDst);

// 仿射变换(以图像的中心为变换中心)
bool AffineTransform(const pool::BitmapImage* pSrc, pool::BitmapImage* dst, 
					 float a, float b, float c, float d);

// 函数功能：在图片中截取图片，给定两个 对角点
// 新分配了内存
IplImage* CutPatchImage(IplImage* src,
						pool::IntPoint startp, pool::IntPoint endp);

int WriteBackToImageRegion(IplImage* segPatch, IplImage* dst, int shiftX, int shiftY);

// 由三角形的三条边计算三角的三个角度
void AngleFrom3Edges(double a, double b, double c,
					   double &A, double &B, double &C);

// 旋转平面 
// 对平面进行旋转，使得与X0Y平面平行
// 平面为ax + by + cz + d = 0;
// AZ为绕Z轴旋转的角度
// AY为绕Y轴旋转的角度
// 先绕Z轴旋转, 再绕Y轴旋转
int RotatePlane(double a, double b, double c, 
				   double &AZ, double &AY);

// 先绕Z转, 再Y, 最后X
void GetR_ZYX(double ax, double ay, double az, double R[9]);

int Top2Down(IplImage* src);