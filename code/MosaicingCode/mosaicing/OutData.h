
// 输出数据到TXT

#pragma once

#include "Bitmap.h"
#include "ImageIO.h"
#include "Point.h"
#include <iostream>
#include <fstream>
#include "tchar.h"
#include "usingCV24.h"
//#include <cstring>

// 导出数据
class MV_MODULE_TYPE COutData
{

public:

	template <class T1>
	static void OutAllPoints2Txt(const T1* pPoints, const int pointsNum, const char* pFileName )
	{
		if(pPoints==NULL)
			return;
		if(pFileName==NULL)
			return;

		cout<<"输出2d数据到文本TXT"<<endl;
		ofstream outData(pFileName, ios::trunc);

		outData<<pointsNum<<" ";
		outData<<pointsNum<<endl;

		for(int u=0; u<pointsNum; u++)
		{
			outData<<pPoints[u].x<<" ";
			outData<<pPoints[u].y<<endl;
		}
		outData.close();
	}

	template <class T1>
	static void OutAll3dPoints2Txt(const T1* pPoints, const int w, const int h, const char* pFileName )
	{
		if(pPoints==NULL)
			return;
		if(pFileName==NULL)
			return;

		cout<<"输出3d数据到文本TXT"<<endl;
		ofstream outData(pFileName, ios::trunc);

		for(int r=0; r<h; r++)
		{
			for(int c=0; c<w; c++)
			{			
				outData<<pPoints[r*w+c].x<<" ";
			}
			outData<<endl;
		}
		for(int r=0; r<h; r++)
		{
			for(int c=0; c<w; c++)
			{			
				outData<<pPoints[r*w+c].y<<" ";
			}
			outData<<endl;
		}
		for(int r=0; r<h; r++)
		{
			for(int c=0; c<w; c++)
			{			
				outData<<pPoints[r*w+c].z<<" ";
			}
			outData<<endl;
		}

		outData.close();
	}

	template <class T1>
	static void OutData2Txt(T1 data, char* filename)
	{
		cout<<"输出2d数据到txt"<<endl;

		ofstream outm(filename, ios::app);

		outm<<data<<endl;

		outm.close();
	}

	template <class T1>
	static void OutData2Txt1(T1 *data, int N, char* filename)
	{
		cout<<"输出2d数据到txt"<<endl;

		ofstream outm(filename, ios::trunc);

		for(int n=0; n<N; n++)
		{
			//if(data[n]>=0)
			
			outm<<data[n]<<endl;
			
			//else
			//{
			//	outm<<0<<endl;
			//}
		}

		outm.close();
	}

	//static void OutData2Txt(int *pdata, int ndata, char* filename);

	template<class T1>
	static void OutData2Txt(T1 *pdata, int ndata, char* filename)
	{
		if(pdata==0)
		{
			return;
		}

		cout<<"输出2d数据到txt"<<endl;
		ofstream outm(filename, ios::trunc);

		for(int u=0; u<ndata; u++){
			outm<<pdata[u]<<" ";
		}
		outm.close();
	}

	template<class T1>
	static void OutData2Txt(T1 *pData2D, int w, int h, int ws, char* filename)
	{
		if(pData2D==0)
			return;

		cout<<"输出2d数据到txt"<<endl;
		ofstream outm(filename, ios::trunc);

		for(int u=0; u<h; u++)
		{
			T1* pDataRow = pData2D + u*ws;
			for(int v = 0; v<w; v++)
			{
				outm<<pDataRow[v]<<" ";
				//cout<<pDataRow[v]<<" ";
			}
			outm<<endl;
			//cout<<endl;
		}
		outm.close();
	}

	template<class PointType>
	static void Out3dPoints2Txt(PointType *ppoints, int npt, char* filename)
	{
		if(ppoints==0)
			return;

		cout<<"输出3d点云到Txt文件"<<endl;
		cout<<filename<<endl;
		ofstream outm(filename, ios::trunc);

		for(int v = 0; v<npt; v++)
		{
			outm<<ppoints[v].x<<" ";
			outm<<ppoints[v].y<<" ";
			outm<<ppoints[v].z<<endl;
		}
		
		outm.close();	
	}

	template<class PointType>
	static void WritePointCloud(PointType *ppoints, int npt, char* filename)
	{
		if(ppoints==0)
			return;

		cout<<"输出3d点云到Txt文件"<<endl;
		cout<<filename<<endl;
		FILE*fp = fopen(filename, "w");

		for(int v = 0; v<npt; v++)
		{
			fprintf(fp, "%f ", ppoints[v].x);
			fprintf(fp, "%f ", ppoints[v].y);
			fprintf(fp, "%f\n", ppoints[v].z);
		}

		fclose(fp);
	}

	// M[16]
	// *.tfm
	static void WriteMatrix4TFM(float *M, char* filename)
	{
		FILE*fp = fopen(filename, "w");

		for(int i=0; i<4; i++)
		{
			fprintf(fp, "%f %f %f %f\n", M[i*4+0], M[i*4+1], M[i*4+2], M[i*4+3]);
		}
		fprintf(fp, "%s\n", "Units: m");
		fclose(fp);
	}

	// 带法向
	template<class PointType>
	static void WriteXYZN(PointType *ppoints, int npt, PointType *normals, char* filename)
	{
		if(ppoints==NULL)
			return;

		cout<<"输出xyzn数据到 *.xyzn文件"<<endl;
		cout<<filename<<endl;
		FILE*fp = fopen(filename, "w");

		for(int v = 0; v<npt; v++)
		{
			if(_isnan(normals[v].x)==0)
			{
				fprintf(fp, "%f %f %f %f %f %f\n", ppoints[v].x, ppoints[v].y, ppoints[v].z, normals[v].x, normals[v].y, normals[v].z);
				//fprintf(fp, "%f ", ppoints[v].y);
				//fprintf(fp, "%f ", ppoints[v].z);
				//fprintf(fp, "%f ", normals[v].x);
				//fprintf(fp, "%f ", normals[v].y);
				//fprintf(fp, "%f\n", normals[v].z);
			}
		}
		fclose(fp);
	}

	// 带法向
	template<class PointType>
	static void WriteXYZN2(PointType *ppoints, int npt, PointType *normals, char* filename)
	{
		if(ppoints==NULL)
		{
			return;
		}

		cout<<"输出xyzn数据到 *.xyzn文件"<<endl;
		cout<<filename<<endl;
		FILE*fp = fopen(filename, "w");

		for(int v = 0; v<npt; v++)
		{
			if(_isnan(normals[v].x)==0)
			{
				fprintf(fp, "v %f ", ppoints[v].x);
				fprintf(fp, "%f ", ppoints[v].y);
				fprintf(fp, "%f\n", ppoints[v].z);
				fprintf(fp, "vn %f ", normals[v].x);
				fprintf(fp, "%f ", normals[v].y);
				fprintf(fp, "%f\n", normals[v].z);
			}
		}
		fclose(fp);
	}

	template<class DataType>
	static void Write2dData(DataType *pData, int w, int h, char* filename)
	{
		if(pData==0)
			return;

		cout<<"输出2d数据到Txt文件"<<endl;
		cout<<filename<<endl;

		FILE *fp = fopen(filename, "w");

		for(int r = 0; r<h; r++)
		{
			for(int c=0; c<w; c++)
			{
				fprintf(fp, "%f ", pData[r*w+c]);
			}
			fprintf(fp, "\n");
		}

		fclose(fp);
	}

	// 输出3D点云和法向 *.xyzn
	template<class PointType>
	static void Out3dPoints_Normal2Txt(PointType *ppoints, int npt, PointType *normals, char* filename)
	{
		if(ppoints==0)
			return;

		cout<<"输出3d点云到Txt文件"<<endl;
		cout<<filename<<endl;
		ofstream outm(filename, ios::trunc);

		for(int v = 0; v<npt; v++)
		{
			outm<<ppoints[v].x<<" "<<ppoints[v].y<<" "<<ppoints[v].z<<" ";
			outm<<normals[v].x<<" "<<normals[v].y<<" "<<normals[v].z<<endl;
		}

		outm.close();	
	}

	// 输出颜色点云到ply文件
	// PLY格式
	template<class PointType>
	static void Out3dColorPoints2Txt(PointType *ppoints, pool::PixelRGB *color, int npt, char* filename)
	{
		if(ppoints==0)
			return;

		cout<<"输出3d数据到txt"<<endl;
		ofstream outm(filename, ios::trunc);
		//  ply
		//	format ascii 1.0
		//	comment VCGLIB generated
		//	element vertex 93590
		//	property float x
		//	property float y
		//	property float z
		//	property uchar red
		//	property uchar green
		//	property uchar blue
		//	property uchar alpha
		//	element face 0
		//	property list uchar int vertex_indices
		//	end_header

		outm<<"ply"<<endl
			<<"format ascii 1.0"<<endl
			<<"comment VCGLIB generated"<<endl
			<<"element vertex"<<" "<<npt<<endl
			<<"property float x"<<endl
			<<"property float y"<<endl
			<<"property float z"<<endl
			<<"property uchar red"<<endl
			<<"property uchar green"<<endl
			<<"property uchar blue"<<endl
			<<"property uchar alpha"<<endl
			<<"element face 0"<<endl
			<<"property list uchar int vertex_indices"<<endl
			<<"end_header"<<endl;

		for(int v = 0; v<npt; v++)
		{
			outm<<ppoints[v].x<<" "<<ppoints[v].y<<" "<<ppoints[v].z<<" "
				<<color[v].r<<" "<<color[v].g<<" "<<color[v].b<<" "<<"255"<<endl;
		}

		outm.close();	
	}

	// CString转string
	//void CString2String(CString src, string &dst);

	// 输出颜色点云到ply文件
	// PLY格式
	template<class PointType>
	static void SaveXYZNRangeGrid2Ply(PointType *points, PointType *normals, int npt, 
									int *rangeGrid, int width, int height, 
									char* filename)
	{
		if(points==0)
			return;

		cout<<"输出xyzn-rangeGrid数据到ply"<<endl;
		ofstream outm(filename, ios::trunc);

		outm<<"ply"<<endl
			<<"format ascii 1.0"<<endl
			<<"obj_info num_cols "<<width<<endl
			<<"obj_info num_rows "<<height<<endl
			<<"element vertex"<<" "<<npt<<endl
			<<"property float x"<<endl
			<<"property float y"<<endl
			<<"property float z"<<endl
			<<"property float nx"<<endl
			<<"property float ny"<<endl
			<<"property float nz"<<endl
			<<"element range_grid "<<width*height<<endl
			<<"property list uchar int vertex_indices"<<endl
			<<"end_header"<<endl;

		for(int v = 0; v<npt; v++)
		{
			outm<<points[v].x<<" "<<points[v].y<<" "<<points[v].z<<" "
				<<normals[v].x<<" "<<normals[v].y<<" "<<normals[v].z<<endl;
		}

		int imageSize = width * height;
		for(int r=0; r<height; r++)
		{
			for(int c=0; c<width; c++)
			{
				if( rangeGrid[r*width+c]>=0 )
				{
					outm<<1<<" "<<rangeGrid[r*width+c]<<endl;
				}
				else
				{
					outm<<0<<endl;
				}
			}
		}

		outm.close();
	}

	static bool OutImage2Txt(pool::BitmapImage* pSrc, char* fileName);

	// 换行
	static void EndLine(char* pFileName);

	static void OutData2TxtApp(float *pdata, int ndata,char* filename);

	static void OutPoints2Txt(pool::fPoint *ppoints,int npt);

	static void OutPoints2Txt(pool::fPoint *ppoints,int npt,char* p_path_x,char* p_path_y);

	static void OutAllPoints2Txt(pool::fPoint *ppoints,int npt);

	static void OutAllPoints2Txt(pool::fPoint *ppoints,int npt,char* p_path_x,char* p_path_y);

	static int OutAllPoints2Txt(pool::sfPoint *p_points, int npt);

	static int OutAllPoints2Txt( vector<pool::sfPoint> p_points );

	static int OutAllPoints2Txt( const vector<float> &vecPoints, char* pFileName );

};

bool ShowPoints(pool::BitmapImage* pSrc,
				pool::Point *pPoint, int nPoints,
				unsigned char grayLevel,TCHAR* pName = _T("c:\\points.bmp" ));

void OutMat2Txt(CvMat* src, char* filename);


template <class T>
void OutData2Txt(vector<T> vecData, char* filename)
{
	cout<<"输出数据到txt"<<endl;

	ofstream outm(filename, ios::trunc);
	int nData = vecData.size();
	for(int u=0; u<nData; u++)
	{
		outm<<vecData[u]<<" ";
	}
	outm.close();
}

template<class TypePoint>
bool ShowPoints2(pool::BitmapImage* pSrc,
				TypePoint *pPoint, int nPoints,
				unsigned char grayLevel = 255,
				TCHAR* pName = _T("c:\\points.bmp"))
{
	if(pPoint==NULL)
		return false;
	if(pSrc==NULL)
		return false;

	int ws = pSrc->widthStep;

	for(int i=0; i<nPoints; i++)
	{
		*(pSrc->imageData + (int)(pPoint[i].y)*ws + (int)(pPoint[i].x)) = grayLevel;
	}

	SaveImageATL( pSrc, pName );

	return true;
}

template<class TypePoint>
bool ShowPoints3(pool::BitmapImage* pSrc,
				 TypePoint *pFlagPoint, int nPoints,
				 unsigned char grayLevel = 255,
				 TCHAR* pName = _T("c:\\points.bmp"))
{
	if(pFlagPoint==NULL)
		return false;
	if(pSrc==NULL)
		return false;

	int ws = pSrc->widthStep;

	for(int i=0; i<nPoints; i++)
	{
		if(pFlagPoint[i].seq>0)
			*(pSrc->imageData + (int)(pFlagPoint[i].y)*ws + (int)(pFlagPoint[i].x)) = grayLevel;
	}

	//SaveImageATL( pSrc, pName );

	return true;
}

template<class T1>
int ReadTxtData(char* pName, T1* pData, int nData)
{
	fstream readFile;

	// 读取ROI区域数据
	readFile.open(pName, ios::in);
	if(!readFile) 
	{ 
		return -1;	
	}

	for(int n=0; n<nData; n++)
	{
		readFile>>pData[n];
	}

	readFile.close();

	return 0;
}

// 获取当前EXE所在目录
string GetCurExeDir(void);

// 从txt文件中读取数据 [存储格式为矩阵]
void ReadTxtData64F(char* pPath, vector<double> &data, int &row, int &col);

void ReadTxtData32F(char* pPath, vector<float> &data32f, int &row, int &col);

//void ReadTxtData32F(char* pPath, vector<float> &data, int &row, int &col);

//****************************************
// 读取无法向点云数据
int ReadPointCloud(char pathSrc[256], vector<CvPoint3D32f> &point);

//****************************************
// 读取带法向的点云
//****************************************
int ReadPointCloudWithNormal(char pathSrc[256], vector<CvPoint3D32f> &points, vector<CvPoint3D32f> &normals);

//****************************************
// 读取带法向的点云
//****************************************
int ReadPointCloudWithNormalColor(char pathSrc[256], vector<CvPoint3D32f> &points, vector<CvPoint3D32f> &normals, vector<CvPoint3D32f> &color);

// 功能: 导入2D点 
// 第1行 x0 y0
// 第2行 x1 y1
// 数据为N行2列
int LoadPoints2D(vector<pool::SfPoint> &pt, char pPath[256]);

extern "C" __declspec(dllexport) 
void OutSingleDataToTxt(float data, char filename[200]);

extern "C" __declspec(dllexport) 
void OutDFDataToTxt(double data, char filename[200]);