#pragma once

#include "Bitmap.h"
//#include "atlstr.h"
#include "usingCV24.h"

using namespace pool;

//显示图片
int ShowImage(pool::BitmapImage* pSrc, char win_name[]);

bool CreateFolder(std::string   strFolderPath);

// 读取raw8图像数据
int ReadRaw8Image(char* path, IplImage* pImage);

// 读取raw8图像数据
int ReadRaw16Image(char* path, unsigned short* pImage, int width, int height);

// 读取raw12图像数据
// pImage---16位图像
int ReadRaw16ImageV2(char* path, unsigned short* pImage16, int width, int height);
