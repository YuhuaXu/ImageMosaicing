#pragma once

#include "StdAfx.h"

#include "opencv2\opencv.hpp"
//#include "opencv2/stitching/stitcher.hpp"
//#include "opencv2/stitching/detail/seam_finders.hpp"
using namespace cv;

#if _MSC_VER == 1800 //Visual Studio 2013
	#ifdef _DEBUG
		//#pragma comment (lib, "opencv_features2d240d.lib")
		//#pragma comment (lib, "opencv_highgui240d.lib")
		//#pragma comment (lib, "opencv_imgproc240d.lib")
		//#pragma comment (lib, "opencv_ml240d.lib")
		//#pragma comment (lib, "opencv_core240d.lib")
		//#pragma comment (lib, "opencv_stitching240d.lib")
		//#pragma comment (lib, "opencv_nonfree240d.lib")
		//#pragma comment (lib, "opencv_flann240d.lib")
		//#pragma comment (lib, "opencv_calib3d240d.lib")
		//#pragma comment (lib, "opencv_legacy240d.lib")

		#pragma comment (lib, "opencv_features2d2410d.lib")
		#pragma comment (lib, "opencv_highgui2410d.lib")
		#pragma comment (lib, "opencv_imgproc2410d.lib")
		#pragma comment (lib, "opencv_ml2410d.lib")
		#pragma comment (lib, "opencv_core2410d.lib")
		#pragma comment (lib, "opencv_stitching2410d.lib")
		#pragma comment (lib, "opencv_nonfree2410d.lib")
		#pragma comment (lib, "opencv_flann2410d.lib")
		#pragma comment (lib, "opencv_calib3d2410d.lib")
		#pragma comment (lib, "opencv_legacy2410d.lib")
	#endif
	#ifndef _DEBUG
		//#pragma comment (lib, "opencv_features2d240.lib")
		//#pragma comment (lib, "opencv_highgui240.lib")
		//#pragma comment (lib, "opencv_imgproc240.lib")
		//#pragma comment (lib, "opencv_ml240.lib")
		//#pragma comment (lib, "opencv_core240.lib")
		//#pragma comment (lib, "opencv_stitching240.lib")
		//#pragma comment (lib, "opencv_nonfree240.lib")
		//#pragma comment (lib, "opencv_flann240.lib")
		//#pragma comment (lib, "opencv_calib3d240.lib")
		//#pragma comment (lib, "opencv_legacy240.lib")

		#pragma comment (lib, "opencv_features2d2410.lib")
		#pragma comment (lib, "opencv_highgui2410.lib")
		#pragma comment (lib, "opencv_imgproc2410.lib")
		#pragma comment (lib, "opencv_ml2410.lib")
		#pragma comment (lib, "opencv_core2410.lib")
		#pragma comment (lib, "opencv_stitching2410.lib")
		#pragma comment (lib, "opencv_nonfree2410.lib")
		#pragma comment (lib, "opencv_flann2410.lib")
		#pragma comment (lib, "opencv_calib3d2410.lib")
		#pragma comment (lib, "opencv_legacy2410.lib")
	#endif
#endif

#if _MSC_VER == 1500 //Visual Studio 2008
	#ifdef _DEBUG
		#pragma comment (lib, "opencv_features2d240d.lib")
		#pragma comment (lib, "opencv_highgui240d.lib")
		#pragma comment (lib, "opencv_imgproc240d.lib")
		//#pragma comment (lib, "opencv_ml240d.lib")
		#pragma comment (lib, "opencv_core240d.lib")
		#pragma comment (lib, "opencv_stitching240d.lib")
		#pragma comment (lib, "opencv_nonfree240d.lib")
		#pragma comment (lib, "opencv_flann240d.lib")
		#pragma comment (lib, "opencv_calib3d240d.lib")
	#endif

	#ifndef _DEBUG
		#pragma comment (lib, "opencv_features2d240.lib")
		#pragma comment (lib, "opencv_highgui240.lib")
		#pragma comment (lib, "opencv_imgproc240.lib")
		#pragma comment (lib, "opencv_ml240.lib")
		#pragma comment (lib, "opencv_core240.lib")
		#pragma comment (lib, "opencv_stitching240.lib")
		#pragma comment (lib, "opencv_nonfree240.lib")
		#pragma comment (lib, "opencv_flann240.lib")
		#pragma comment (lib, "opencv_calib3d240.lib")
		#pragma comment (lib, "opencv_legacy240.lib")
	#endif
#endif

void CvGetImageSize(const IplImage* pSrc, int &width, int &height, int &widthStep);

// 开始计时
void StartCalTime(float &t);

// 停止计时，输出计时结果
void StopCalTime(float &t, char* textout);

//显示图片
int ShowImageCv(IplImage* src,char win_name[]);

// 画直方图 彩色
// color  =  "red"   "green"   "blue" 
void PlotHistogram2(float hist[256], char aName[200], const string &color );