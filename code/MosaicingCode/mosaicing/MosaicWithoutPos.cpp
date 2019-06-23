
//#include <afxwin.h>
#include "MosaicWithoutPos.h"
#include "Bitmap.h"
#include "Matrix.h"
#include "OutData.h"
#include "MosaicImage.h"
//include "harriscorner.h"
#include "opencv2\opencv.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "using_cholmod.h"
#include "ImageIO.h"
//#include "FitSurface.h"
//#include "using_cholmod.h"

using namespace cv;

// 先绕Z轴旋转, 再绕Y轴旋转
// 计算平面旋转角度 bundler
int CalRotateAngle( double R[9],
			  const vector<CvPoint3D64f> &points)
{
	//if(points.empty())
	//	return -1;

	//double AZ, AY;

	//double a, b, c;
	//
	//double ransacDist = 2.5;
	//double K[3];
	//// f(x,y) = k0*x + k1*y + k2
	//FitSurface1DRansac(&points[0], points.size(), K, ransacDist);

	//a = K[0];
	//b = K[1];
	//c = -1;

	//cout<<"a b c of plane "<<a<<" "<<b<<" "<<c<<endl<<endl;

	//// ax + by + cz + d = 0;
	//// 先绕Z轴旋转, 再绕Y轴旋转
	//RotatePlane(a, b, c, AZ, AY );

	//cout<<"AZ "<<AZ<<" AY "<<AY<<endl<<endl;
	//GetR_ZYX(0, AY, AZ, R);

	return 0;
}

// 读取list中的图像的名字
int ReadImageList(char* pPath, vector<string> &vecList)
{
	if(pPath==NULL)
		return -1;

	cout<<"ReadImageList..."<<endl;
	ifstream fin;
	fin.open(pPath);
	string line;
	int nLines = 0;
	//double camElem[15];

	while(getline(fin, line)) // 按行读取文件
	{
		//cout<<1<<endl;

		int comma_n = 0;
		vector<string> str_list;
		//vector<double> db_list;
		do
		{
			string temp_s ="";
			comma_n = line.find(" ");
			if(-1==comma_n)
			{
				temp_s = line.substr(0,line.length());
				str_list.push_back(temp_s);
				//db_list.push_back( atof(temp_s.c_str()) ); // string to double
				break;
			}
			temp_s = line.substr(0, comma_n);
			line.erase(0, comma_n+1);
			//str_list.push_back(temp_s);
			//db_list.push_back( atof(temp_s.c_str()) );
			str_list.push_back(temp_s);
		}
		while(comma_n>0);

		vecList.push_back(str_list[0]);
	}
	return 0;
}


// 使用bundler的输出结果
//We use a pinhole camera model; the parameters we estimate for each camera are a focal length (f), two radial distortion parameters (k1 and k2), a rotation (R), and translation (t), as described in the file specification above. The formula for projecting a 3D point X into a camera (R, t, f) is:
//
//P = R * X + t       (conversion from world to camera coordinates)
//p = -P / P.z        (perspective division)
//p' = f * r(p) * p   (conversion to pixel coordinates)
//where P.z is the third (z) coordinate of P. In the last equation, r(p) is a function that computes a scaling factor to undo the radial distortion:
//
//r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4.
//This gives a projection in pixels, where the origin of the image is the center of the image, the positive x-axis points right, and the positive y-axis points up (in addition, in the camera coordinate system, the positive z-axis points backwards, so the camera is looking down the negative z-axis, as in OpenGL).
//
//Finally, the equations above imply that the camera viewing direction is:
//
//R' * [0 0 -1]'  (i.e., the third row of -R or third column of -R')
//				 (where ' indicates the transpose of a matrix or vector).
//
//				 and the 3D position of a camera is
//
//				 -R' * t .
int ImportBundlerOut(char* pPath, vector<CameraPose64F> &camPose, vector<CvPoint3D64f> &points,
					 vector<MatchPointPairs> &vecMatchPoints)
{
	cout<<"ImportBundlerOut..."<<endl;
	ifstream fin;
	fin.open(pPath);
	string line;
	int nLines = 0;
	double camElem[15];
	int nCam = 0;
	int nCamElem = 0;
	int nPointsElem = 0;
	double realZ = 0;
	while(getline(fin, line)) // 按行读取文件
	{
		if(nLines==0)
		{
			nLines++;
			continue;
		}

		//cout<<1<<endl;

		int comma_n = 0;
		vector<string> str_list;
		vector<double> db_list;
		do
		{
			string temp_s ="";
			comma_n = line.find(" ");
			if(-1==comma_n)
			{
				temp_s = line.substr(0,line.length());
				//str_list.push_back(temp_s);
				db_list.push_back( atof(temp_s.c_str()) ); // string to double
				break;
			}
			temp_s = line.substr(0, comma_n);
			line.erase(0, comma_n+1);
			//str_list.push_back(temp_s);
			db_list.push_back( atof(temp_s.c_str()) );
			
		}
		while(comma_n>0);

		if(nLines==1)
		{
			nCam = (int)db_list[0];
		}
		if( (nLines>1) && (nLines<nCam*5+2) ) // 读取像机参数
		{
			camElem[nCamElem+0] = db_list[0];
			camElem[nCamElem+1] = db_list[1];
			camElem[nCamElem+2] = db_list[2];
			nCamElem += 3;
			if(nCamElem==15)
			{
				nCamElem = 0;
				CameraPose64F temCam;
				temCam.F = camElem[0]; // 焦距
				temCam.k[0] = camElem[1]; // 畸变1
				temCam.k[1] = camElem[2]; // 畸变1
				memcpy(temCam.R, &camElem[3], sizeof(double)*9); // 旋转矩阵
				temCam.R[3] *= -1; // 因为bundler输出的(R T), 其像机观察的方向是沿着Z_axis的负方向 
				temCam.R[4] *= -1; // 转换前: X向右为正, Y向上为正, Z向后为正
				temCam.R[5] *= -1; // 转换后: X向右为正, Y向下为正, Z向前为正

				temCam.R[6] *= -1; // 因为bundler输出的(R T), 其像机观察的方向是沿着Z_axis的负方向
				temCam.R[7] *= -1;
				temCam.R[8] *= -1;

				memcpy(temCam.T, &camElem[12], sizeof(double)*3); // 平移矩阵
				temCam.T[1] *= -1; // 因为bundler输出的(R T), 其像机观察的方向是沿着Z_axis的负方向
				temCam.T[2] *= -1; // 因为bundler输出的(R T), 其像机观察的方向是沿着Z_axis的负方向

				double TBar[3];
				double cp[3] = {0, 0, realZ};
				MulMatrix(temCam.R, 3, 3, cp, 3, 1, TBar);

				for(int p=0; p<3; p++)
				{
					temCam.T[p] += TBar[p];
				}
				
				camPose.push_back(temCam);
			}
		}
		if(nLines>=nCam*5+2) // 读3d点信息
		{
			if(nPointsElem==0) // 3d点坐标
			{
				CvPoint3D64f tempPoint3D;
				tempPoint3D.x = db_list[0];
				tempPoint3D.y = db_list[1];
				tempPoint3D.z = db_list[2];

				//tempPoint3D.x ;
				//tempPoint3D.y ;
				tempPoint3D.z += (-realZ); // 目的: 使得3d点处于XY平面上
				points.push_back(tempPoint3D);
			}
			if(nPointsElem==1) // 3d点的RGB值
			{
			}
			if(nPointsElem==2) // 3d点的图像特征点
			{
				int nPts = (int)db_list[0]; // 特征点个数
				for(int i=0; i<nPts; i++)
				{
					for(int j=i+1; j<nPts; j++)
					{
						// 第i个和第j个形成一对
						MatchPointPairs tempMat;
						tempMat.ptA_i = (int)db_list[i*4 + 1 + 0];
						tempMat.ptA.id = (int)db_list[i*4 + 1 + 1];
						tempMat.ptA.x = (float)db_list[i*4 + 1 + 2];
						tempMat.ptA.y = (float)db_list[i*4 + 1 + 3];
						tempMat.ptA_Fixed = 0;
						
						tempMat.ptB_i = (int)db_list[j*4 + 1 + 0];
						tempMat.ptB.id = (int)db_list[j*4 + 1 + 1];
						tempMat.ptB.x = (float)db_list[j*4 + 1 + 2];
						tempMat.ptB.y = (float)db_list[j*4 + 1 + 3];
						tempMat.ptB_Fixed = 0;
						vecMatchPoints.push_back(tempMat);
					}
				}
			}
			nPointsElem++;
			if(points.size()>100)
			{
				//break;
			}
			if(nPointsElem==3)
			{
				nPointsElem = 0;
			}
		}
		nLines++;
	}

	fin.close();
	cout<<"Done"<<endl;
	return 0;
}

int ImportVisualSFM_Match(char* pPath, vector<MatchPointPairs> &vecMatchPoints, float cx, float cy)
{
	cout<<"ImportVisualSFM_Match..."<<endl;
	ifstream fin;
	fin.open(pPath);
	string line;
	int nLines = 0;
	double camElem[15];
	int nCam = 0;
	int nPts = 0;
	int nCamElem = 0;
	int nPointsElem = 0;
	double realZ = 0;
	while(getline(fin, line)) // 按行读取文件
	{
		if(nLines<2)
		{
			nLines++;
			continue;
		}

		//cout<<1<<endl;

		int comma_n = 0;
		vector<string> str_list;
		vector<double> db_list;
		do
		{
			string temp_s ="";
			comma_n = line.find(" ");
			if(-1==comma_n)
			{
				temp_s = line.substr(0,line.length());
				//str_list.push_back(temp_s);
				db_list.push_back( atof(temp_s.c_str()) ); // string to double
				break;
			}
			temp_s = line.substr(0, comma_n);
			line.erase(0, comma_n+1);
			//str_list.push_back(temp_s);
			db_list.push_back( atof(temp_s.c_str()) );

		}
		while(comma_n>0);

		if(nLines==2)
		{
			nCam = (int)db_list[0];
		}

		if(nLines==nCam+4) // 读3d点信息
		{
			nPts = (int)db_list[0]; // 特征点个数
		}
		
		if(nLines>nCam+4) // 读3d点信息
		{
			
			int nMatch = db_list[6];
			for(int i=0; i<nMatch; i++)
			{
				// 第i个和第j个形成一对
				MatchPointPairs tempMat;
				tempMat.ptA_i = (int)db_list[i*4 + 7 + 0];
				tempMat.ptA.id = (int)db_list[i*4 + 7 + 1];
				tempMat.ptA.x = (float)db_list[i*4 + 7 + 2]+cx;
				tempMat.ptA.y = (float)db_list[i*4 + 7 + 3]+cy;
				tempMat.ptA_Fixed = 0;
				for(int j=i+1; j<nMatch; j++)
				{
					tempMat.ptB_i = (int)db_list[j*4 + 7 + 0];
					tempMat.ptB.id = (int)db_list[j*4 + 7 + 1];
					tempMat.ptB.x = (float)db_list[j*4 + 7 + 2]+cx;
					tempMat.ptB.y = (float)db_list[j*4 + 7 + 3]+cy;
					tempMat.ptB_Fixed = 0;
					vecMatchPoints.push_back(tempMat);
				}
			}
			nPointsElem++;
			if(nPointsElem>=nPts)
				break;
		}
		nLines++;
		
	}

	fin.close();
	cout<<"Done"<<endl;
	return 0;
}

int ImportVisualSFM_Match2(char* pPath, vector<MatchPointPairs> &vecMatchPoints, float cx, float cy)
{
	cout<<"ImportVisualSFM_Match2..."<<endl;
	ifstream fin;
	fin.open(pPath);
	string line;
	int nLines = 0;
	double camElem[15];
	int nCam = 0;
	int nPts = 0;
	int nCamElem = 0;
	int nPointsElem = 0;
	double realZ = 0;
	int nRound = 0;
	while(getline(fin, line)) // 按行读取文件
	{
		if(nLines<1)
		{
			nLines++;
			continue;
		}

		//cout<<1<<endl;

		int comma_n = 0;
		vector<string> str_list;
		vector<double> db_list;
		do
		{
			string temp_s ="";
			comma_n = line.find(" ");
			if(-1==comma_n)
			{
				temp_s = line.substr(0,line.length());
				//str_list.push_back(temp_s);
				db_list.push_back( atof(temp_s.c_str()) ); // string to double
				break;
			}
			temp_s = line.substr(0, comma_n);
			line.erase(0, comma_n+1);
			//str_list.push_back(temp_s);
			db_list.push_back( atof(temp_s.c_str()) );

		}
		while(comma_n>0);

		if(nLines==1) // row 1
		{
			nCam = (int)db_list[0]; // 相机个数
		}

		if(nLines==1) // 读3d点信息
		{
			nPts = (int)db_list[1]; // 特征点个数
		}

		
		if(nLines>=nCam*5+2) // 读3d点信息
		{
			nRound++;
			int nMatch = 0;
			if(nRound==3)
			{
				nMatch = db_list[0];
			}
			else
				continue;
			
			for(int i=0; i<nMatch; i++)
			{
				// 第i个和第j个形成一对
				MatchPointPairs tempMat;
				tempMat.ptA_i = (int)db_list[i*4 + 1 + 0];
				tempMat.ptA.id = (int)db_list[i*4 + 1 + 1];
				tempMat.ptA.x = (float)db_list[i*4 + 1 + 2]+cx;
				tempMat.ptA.y = (float)db_list[i*4 + 1 + 3]+cy;
				tempMat.ptA_Fixed = 0;
				for(int j=i+1; j<nMatch; j++)
				{
					tempMat.ptB_i = (int)db_list[j*4 + 1 + 0];
					tempMat.ptB.id = (int)db_list[j*4 + 1 + 1];
					tempMat.ptB.x = (float)db_list[j*4 + 1 + 2]+cx;
					tempMat.ptB.y = (float)db_list[j*4 + 1 + 3]+cy;
					tempMat.ptB_Fixed = 0;
					vecMatchPoints.push_back(tempMat);
				}
			}

			nPointsElem++;
			if(nPointsElem>=nPts)
				break;
			
			if(nRound>=3)
				nRound = 0;
		}
		nLines++;

	}

	fin.close();
	cout<<"Done"<<endl;
	return 0;
}

int ImportCMVSOut(char* pPath, vector<CameraPose64F> &camPose, vector<CvPoint3D64f> &points,
				  vector<MatchPointPairs> &vecMatchPoints)
{
	cout<<"ImportCMVSOut..."<<endl;
	ifstream fin;
	fin.open(pPath);
	string line;
	int nLines = 0;
	double camElem[15];
	int nCam = 0;
	int nCamElem = 0;
	int nPointsElem = 0;
	double realZ = 0;
	double RBar[9] = {	1, 0, 0,
						0, -1, 0,
						0, 0, -1};
	while(getline(fin, line)) // 按行读取文件
	{
		if(nLines==0)
		{
			nLines++;
			continue;
		}

		//cout<<1<<endl;

		int comma_n = 0;
		vector<string> str_list;
		vector<double> db_list;
		do
		{
			string temp_s ="";
			comma_n = line.find(" ");
			if(-1==comma_n)
			{
				temp_s = line.substr(0,line.length());
				//str_list.push_back(temp_s);
				db_list.push_back( atof(temp_s.c_str()) ); // string to double
				break;
			}
			temp_s = line.substr(0, comma_n);
			line.erase(0, comma_n+1);
			//str_list.push_back(temp_s);
			db_list.push_back( atof(temp_s.c_str()) );

		}
		while(comma_n>0);

		if(nLines==1)
		{
			nCam = (int)db_list[0];
		}
		if( (nLines>1) && (nLines<nCam*5+2) ) // 读取像机参数
		{
			camElem[nCamElem+0] = db_list[0];
			camElem[nCamElem+1] = db_list[1];
			camElem[nCamElem+2] = db_list[2];
			nCamElem += 3;
			if(nCamElem==15)
			{
				nCamElem = 0;
				CameraPose64F temCam;
				temCam.F = camElem[0]; // 焦距
				temCam.k[0] = camElem[1]; // 畸变1
				temCam.k[1] = camElem[2]; // 畸变1
				memcpy(temCam.R, &camElem[3], sizeof(double)*9); // 旋转矩阵

				double tempR[9];
				// 先对R进行变换 for cmvs wu-changchang
				MulMatrix(temCam.R, 3, 3, RBar, 3, 3, tempR);
				memcpy( temCam.R, tempR, sizeof(tempR) );

				temCam.R[3] *= -1; // 因为bundler输出的(R T), 其像机观察的方向是沿着Z_axis的负方向 
				temCam.R[4] *= -1; // 转换前: X向右为正, Y向上为正, Z向后为正
				temCam.R[5] *= -1; // 转换后: X向右为正, Y向下为正, Z向前为正

				temCam.R[6] *= -1; // 因为bundler输出的(R T), 其像机观察的方向是沿着Z_axis的负方向
				temCam.R[7] *= -1;
				temCam.R[8] *= -1;

				memcpy(temCam.T, &camElem[12], sizeof(double)*3); // 平移矩阵
				temCam.T[1] *= -1; // 因为bundler输出的(R T), 其像机观察的方向是沿着Z_axis的负方向
				temCam.T[2] *= -1; // 因为bundler输出的(R T), 其像机观察的方向是沿着Z_axis的负方向

				double TBar[3];
				double cp[3] = {0, 0, realZ};
				MulMatrix(temCam.R, 3, 3, cp, 3, 1, TBar);

				for(int p=0; p<3; p++)
				{
					temCam.T[p] += TBar[p];
				}

				camPose.push_back(temCam);
			}
		}
		if(nLines>=nCam*5+2) // 读3d点信息
		{
			if(nPointsElem==0)
			{
				CvPoint3D64f tempPoint3D;
				tempPoint3D.x = db_list[0];
				tempPoint3D.y = db_list[1];
				tempPoint3D.z = db_list[2];

				tempPoint3D.y *= -1;
				tempPoint3D.z *= -1;

				//tempPoint3D.x ;
				//tempPoint3D.y ;
				tempPoint3D.z += (-realZ); // 目的: 使得3d点处于XY平面上
				points.push_back(tempPoint3D);
			}
			if(nPointsElem==1) // 3d点的RGB值
			{
			}
			if(nPointsElem==2) // 3d点的图像特征点
			{
				int nPts = (int)db_list[0]; // 特征点个数
				for(int i=0; i<nPts; i++)
				{
					for(int j=i+1; j<nPts; j++)
					{
						// 第i个和第j个形成一对
						MatchPointPairs tempMat;
						tempMat.ptA_i = (int)db_list[i*4 + 1 + 0];
						tempMat.ptA.id = (int)db_list[i*4 + 1 + 1];
						tempMat.ptA.x = (float)db_list[i*4 + 1 + 2];
						tempMat.ptA.y = (float)db_list[i*4 + 1 + 3];
						tempMat.ptA_Fixed = 0;

						tempMat.ptB_i = (int)db_list[j*4 + 1 + 0];
						tempMat.ptB.id = (int)db_list[j*4 + 1 + 1];
						tempMat.ptB.x = (float)db_list[j*4 + 1 + 2];
						tempMat.ptB.y = (float)db_list[j*4 + 1 + 3];
						tempMat.ptB_Fixed = 0;
						vecMatchPoints.push_back(tempMat);
					}
				}
			}
			nPointsElem++;
			if(points.size()>100)
			{
				//break;
			}
			if(nPointsElem==3)
			{
				nPointsElem = 0;
			}
		}
		nLines++;
	}

	fin.close();
	cout<<"Done"<<endl;
	return 0;
}

// Bundler's out
// 由像机参数计算每幅图像的单应变换矩阵
// 适用于近似平面场景的图像拼接
int CamPose2Homo(const ImagePoseInfo *pImagePose, int nImages, const vector<CvPoint3D64f> &vecPoints, 
				 vector<ProjectMat64F> &homos)
{
	cout<<"CamPose2Homo"<<endl;
	if(NULL==pImagePose)
		return -1;

	//double AX_Norm = pool::PI_64F/2; // 绕X轴转90度 [尚洋P28]
	double AY_Norm = 0;
	double AZ_Norm = 0;

	double R_Norm[9] = {9.9940453624e-001, -3.0585486429e-002, -1.5971880631e-002,
						-3.3055823784e-002, -9.8141617175e-001, -1.8902277733e-001,
						-9.8937083538e-003, 1.8943818479e-001, -9.8184280243e-001};
	double H_Norm[9];
	//GetR(AX_Norm, AY_Norm, AZ_Norm, R_Norm);

	for(int n=0; n<nImages; n++)
	{
		// 焦距
		double fx = pImagePose[n].camPose.F;
		double fy =  pImagePose[n].camPose.F;

		//double cx =  pImagePose[n].pImg->width * 0.5;
		//double cy =  pImagePose[n].pImg->height * 0.5;
		double cx =  0;
		double cy =  0;

		double K[9] = {  fx, 0,  cx, 
						0,  fy, cy,
						0,  0,  1};

		double dx = pImagePose[n].pImg->width*0.5;
		double dy = pImagePose[n].pImg->height*0.5;

		//double TBar[3];
		//MulMatrix(pImagePose[n].camPose.T, 3, 1, )

		double H_Real[9];
		double exParam[9]; // 外参矩阵
		{
			exParam[0] = pImagePose[n].camPose.R[0];
			exParam[3] = pImagePose[n].camPose.R[3];
			exParam[6] = pImagePose[n].camPose.R[6];

			exParam[1] = pImagePose[n].camPose.R[1];
			exParam[4] = pImagePose[n].camPose.R[4];
			exParam[7] = pImagePose[n].camPose.R[7];

			exParam[2] = pImagePose[n].camPose.T[0];
			exParam[5] = pImagePose[n].camPose.T[1];
			exParam[8] = pImagePose[n].camPose.T[2];

			MulMatrix(K, 3, 3, exParam, 3, 3, H_Real );

			COutData::OutData2Txt(H_Real, 3, 3, 3, "C:\\hReal.txt");
		}

		if(n==0) // 计算H_Norm
		{                           
			for(int p=0; p<5; p++)
			{
				double PC[3];
				double PW[3];
				PW[0] = vecPoints[p].x;
				PW[1] = vecPoints[p].y;
				PW[2] = vecPoints[p].z;
				MulMatrix(exParam, 3, 3, PW, 3, 1, PC);

				double u, v;
				ApplyProject9(H_Real, PW[0], PW[1], u, v);
				u = u;
			}

			double RBar[9];
			double invR[9];
			InverseMatrix(pImagePose[n].camPose.R, 3, invR, 1e-12);
			MulMatrix(R_Norm, 3, 3, invR, 3, 3, RBar);

			double TBar[3];
			MulMatrix(RBar, 3, 3, 
				(double*)pImagePose[n].camPose.T, 3, 1, 
				TBar);

			double exParamNorm[9]; // 规范外参矩阵
			exParamNorm[0] = R_Norm[0];
			exParamNorm[3] = R_Norm[3];
			exParamNorm[6] = R_Norm[6];

			exParamNorm[1] = R_Norm[1];
			exParamNorm[4] = R_Norm[4];
			exParamNorm[7] = R_Norm[7];

			exParamNorm[2] = TBar[0];
			exParamNorm[5] = TBar[1];
			exParamNorm[8] = TBar[2];

			// 计算H_Norm
			MulMatrix(K, 3, 3, exParamNorm, 3, 3, H_Norm );
		}

		double inv_H_Real[9];
		InverseMatrix( H_Real, 3, inv_H_Real, 1e-12 );

		ProjectMat64F homoCurrent;
		double H0[9];
		MulMatrix(H_Norm, 3, 3, inv_H_Real, 3, 3, H0); // 当前图像的变换单应矩阵

		homoCurrent.m[0] = H0[0] + dx*H0[6];
		homoCurrent.m[1] = H0[1] + dx*H0[7];
		homoCurrent.m[2] = H0[2] + dx*H0[8];

		homoCurrent.m[3] = H0[3] + dy*H0[6];
		homoCurrent.m[4] = H0[4] + dy*H0[7];
		homoCurrent.m[5] = H0[5] + dy*H0[8];

		homoCurrent.m[6] = H0[6];
		homoCurrent.m[7] = H0[7];
		homoCurrent.m[8] = H0[8];
		//memcpy(homoCurrent.m, H0, sizeof(H0) );

		homos.push_back(homoCurrent);
	}

	return 0;
}

// Bundler's out
// 由像机参数计算每幅图像的单应变换矩阵
// 适用于近似平面场景的图像拼接
int CamPose2Homo2(const ImagePoseInfo *pImagePose, int nImages, const vector<CvPoint3D64f> &vecPoints, 
				 vector<ProjectMat64F> &homos)
{
	cout<<"CamPose2Homo"<<endl;
	if(NULL==pImagePose)
		return -1;

	double H_Norm[9];
	int nNorm = 16;
	{
		// 焦距
		double fx = pImagePose[nNorm].camPose.F;
		double fy =  pImagePose[nNorm].camPose.F;

		double cx =  0;
		double cy =  0;

		double K[9] = {  fx, 0,  0, 
						0,  fy, 0,
						0,  0,  1};

		//double H_Real[9];
		double exParam[9]; // 外参矩阵

		exParam[0] = pImagePose[nNorm].camPose.R[0];
		exParam[3] = pImagePose[nNorm].camPose.R[3];
		exParam[6] = pImagePose[nNorm].camPose.R[6];

		exParam[1] = pImagePose[nNorm].camPose.R[1];
		exParam[4] = pImagePose[nNorm].camPose.R[4];
		exParam[7] = pImagePose[nNorm].camPose.R[7];

		exParam[2] = pImagePose[nNorm].camPose.T[0];
		exParam[5] = pImagePose[nNorm].camPose.T[1];
		exParam[8] = pImagePose[nNorm].camPose.T[2];

		MulMatrix(K, 3, 3, exParam, 3, 3, H_Norm );
	}
	
	for(int n=0; n<nImages; n++)
	{
		// 焦距
		double fx = pImagePose[n].camPose.F;
		double fy =  pImagePose[n].camPose.F;

		double cx =  0;
		double cy =  0;

		double K[9] = {  fx, 0,  0, 
						0,  fy, 0,
						0,  0,  1};

		double dx = pImagePose[n].pImg->width*0.5;
		double dy = pImagePose[n].pImg->height*0.5;

		//double TBar[3];
		//MulMatrix(pImagePose[n].camPose.T, 3, 1, )

		double H_Real[9];
		double exParam[9]; // 外参矩阵
		{
			exParam[0] = pImagePose[n].camPose.R[0];
			exParam[3] = pImagePose[n].camPose.R[3];
			exParam[6] = pImagePose[n].camPose.R[6];

			exParam[1] = pImagePose[n].camPose.R[1];
			exParam[4] = pImagePose[n].camPose.R[4];
			exParam[7] = pImagePose[n].camPose.R[7];

			exParam[2] = pImagePose[n].camPose.T[0];
			exParam[5] = pImagePose[n].camPose.T[1];
			exParam[8] = pImagePose[n].camPose.T[2];

			MulMatrix(K, 3, 3, exParam, 3, 3, H_Real );

			//COutData::OutData2Txt(H_Real, 3, 3, 3, "C:\\hReal.txt");
		}

		//if(n==0) // 计算H_Norm
		{                           
			//for(int p=0; p<5; p++)
			{
				double PC[3];
				double PW[3] = {4.1717162072e+000, -2.6450844711e+000, -0.3593635967e+000};
				/*PW[0] = 3.0515419350e+000;   
				PW[1] = -3.3053599856e+000;
				PW[2] = -2.7298161149e+000;*/
				//MulMatrix(exParam, 3, 3, PW, 3, 1, PC);

				double u, v;
				ApplyProject9(H_Real, PW[0], PW[1], u, v); // 单应矩阵法
				u = u;

				// world to image
				MulMatrix((double*)pImagePose[n].camPose.R, 3, 3, PW, 3, 1, PC);
				PC[0] += pImagePose[n].camPose.T[0];
				PC[1] += pImagePose[n].camPose.T[1];
				PC[2] += pImagePose[n].camPose.T[2];
				//PC = R * X + T       //*%*/(conversion from world to camera coordinates)
				double p = fx * PC[0] / PC[2];        //(perspective division)
				double q = fy * PC[1] / PC[2];       //(perspective division)
				p = p;
			}
		}

		double inv_H_Real[9];
		InverseMatrix( H_Real, 3, inv_H_Real, 1e-12 );

		ProjectMat64F homoCurrent;
		double H0[9];
		MulMatrix(H_Norm, 3, 3, inv_H_Real, 3, 3, H0); // 当前图像的变换单应矩阵

		homoCurrent.m[0] = H0[0] + dx*H0[6];
		homoCurrent.m[1] = H0[1] + dx*H0[7];
		homoCurrent.m[2] = H0[2] + dx*H0[8];

		homoCurrent.m[3] = H0[3] + dy*H0[6];
		homoCurrent.m[4] = H0[4] + dy*H0[7];
		homoCurrent.m[5] = H0[5] + dy*H0[8];

		homoCurrent.m[6] = H0[6];
		homoCurrent.m[7] = H0[7];
		homoCurrent.m[8] = H0[8];
		//memcpy(homoCurrent.m, H0, sizeof(H0) );

		homos.push_back(homoCurrent);
	}

	return 0;
}

// Bundler's out
// 由像机参数计算每幅图像的单应变换矩阵
// 适用于近似平面场景的图像拼接
int CMosaicByPose::CamPose2Homo3(ImagePoseInfo *pImagePose, int nImages, vector<CvPoint3D64f> &vecPoints, 
				  vector<ProjectMat64F> &homos, vector<MatchPointPairs> vecMatchPairs)
{	
	if(NULL==pImagePose)
		return -1;

	COutData::Out3dPoints2Txt(&vecPoints[0], (int)vecPoints.size(), "c:\\point.txt");

	// 计算平面旋转量, 使得平面与XY平面平行 先绕Z轴旋转, 再绕Y轴旋转 
	double planeR[9]; // meanZ为3d点的z轴的平均高度
	CalRotateAngle(planeR, vecPoints);

	double inv_surfaceR[9];
	InverseMatrix(planeR, 3, inv_surfaceR, 1e-12);

	// 对3d点进行变换
	double meanZ = 0;
	for(int i=0; i<(int)vecPoints.size(); i++)
	{
		double pW[3], pW_bar[3];
		pW[0] = vecPoints[i].x;
		pW[1] = vecPoints[i].y;
		pW[2] = vecPoints[i].z;
		MulMatrix(planeR, 3, 3, pW, 3, 1, pW_bar);
		vecPoints[i].x = pW_bar[0];
		vecPoints[i].y = pW_bar[1];
		vecPoints[i].z = pW_bar[2];
		meanZ += vecPoints[i].z;
	}
	meanZ /= (int)vecPoints.size(); // 3d点的z轴的平均高度.
	for(int i=0; i<(int)vecPoints.size(); i++)
	{
		vecPoints[i].z -= meanZ;
	}

	double meanH = 0;
	vector<CvPoint3D64f> vecCamPos;
	// 对3d点和像机R进行更新
	for(int n=0; n<nImages; n++)
	{
		double R_RS[9];
		MulMatrix(pImagePose[n].camPose.R, 3, 3, inv_surfaceR, 3, 3, R_RS);
		memcpy( pImagePose[n].camPose.R, R_RS, sizeof(R_RS) ); // 新的像机旋转矩阵

		// 对T进行更新
		pImagePose[n].camPose.T[0] += R_RS[2] * meanZ;
		pImagePose[n].camPose.T[1] += R_RS[5] * meanZ;
		pImagePose[n].camPose.T[2] += R_RS[8] * meanZ;

		// 计算相机光心的坐标
		double invR[9];
		InverseMatrix(pImagePose[n].camPose.R, 3, invR, 1e-12);
		double optCt[3]; // 光心坐标 
		CvPoint3D64f opticCenter; // 光心坐标 
		MulMatrix(invR, 3, 3, pImagePose[n].camPose.T, 3, 1, optCt);
		opticCenter.x = optCt[0] * -1;
		opticCenter.y = optCt[1] * -1;
		opticCenter.z = optCt[2] * -1;
		pImagePose[n].camPose.pos = opticCenter;
		vecCamPos.push_back(opticCenter);
		meanH += opticCenter.z;
	}
	meanH /= nImages;

	COutData::Out3dPoints2Txt(&vecPoints[0], vecPoints.size(), "c:\\point2.txt");

	HeightNorm(meanH);
	Mosaic2(pImagePose, nImages, vecMatchPairs);

	return 0;
}

//void ApplyProject9(const float h[9], const float xSrc, const float ySrc, float &xDst, float &yDst)
//{
//	xDst = (h[0]*xSrc + h[1]*ySrc + h[2]) / (h[6]*xSrc + h[7]*ySrc + h[8]);
//	yDst = (h[3]*xSrc + h[4]*ySrc + h[5]) / (h[6]*xSrc + h[7]*ySrc + h[8]);
//}

double CalMatchError(ImageTransform* pX, int nImages, MatchPointPairs *pPairs, int nPair)
{
	double error = 0;
	for(int n=0; n<nPair; n++)
	{
		int ptA_i = pPairs[n].ptA_i;
		int ptB_i = pPairs[n].ptB_i;
		DfPoint ptA_Bar;
		double hA64F[9];
		for(int t=0; t<9; t++)
		{
			hA64F[t] = pX[ptA_i].h.m[t];
		}
		ApplyProject9(hA64F, pPairs[n].ptA.x, pPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);

		DfPoint ptB_Bar;
		double hB64F[9];
		for(int t=0; t<9; t++)
		{
			hB64F[t] = pX[ptB_i].h.m[t];
		}
		ApplyProject9(hB64F, pPairs[n].ptB.x, pPairs[n].ptB.y, ptB_Bar.x, ptB_Bar.y);
		double dist;
		DistanceOfTwoPoints(ptA_Bar.x, ptA_Bar.y, ptB_Bar.x, ptB_Bar.y, dist);
		error += dist*dist;
	}
	error = sqrt(error/nPair);
	return error;
}

double CalMatchError2(ImageTransform* pX, CvMat* pX2, int nUnfixed, MatchPointPairs *pPairs, int nPair,
					  vector<int> vecFreqMatch, vector<int> vecAccFixed, double &rotCost)
{
	double error = 0;
	for(int n=0; n<nPair; n++)
	{
		int ptA_i = pPairs[n].ptA_i;
		int ptB_i = pPairs[n].ptB_i;

		DfPoint ptA_Bar;
		double hA64F[9];
		for(int t=0; t<9; t++)
		{
			hA64F[t] = pX[ptA_i].h.m[t];
		}
		ApplyProject9(hA64F, pPairs[n].ptA.x, pPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);

		DfPoint ptB_Bar;

		double hB64F[9];
		for(int t=0; t<9; t++)
		{
			hB64F[t] = pX[ptB_i].h.m[t];
		}
		ApplyProject9(hB64F, pPairs[n].ptB.x, pPairs[n].ptB.y, ptB_Bar.x, ptB_Bar.y);
		double dist;
		DistanceOfTwoPoints(ptA_Bar.x, ptA_Bar.y, ptB_Bar.x, ptB_Bar.y, dist);
		error += dist*dist;
	}

	rotCost = 0;

	for(int n=0; n<nUnfixed; n++)
	{
		int nC = vecFreqMatch[n];

		double a = cvGetReal2D(pX2, 8*n+0, 0); // a
		double b = cvGetReal2D(pX2, 8*n+1, 0); // b
		double e = cvGetReal2D(pX2, 8*n+2, 0); // e
		double c = cvGetReal2D(pX2, 8*n+3, 0); // c
		double d = cvGetReal2D(pX2, 8*n+4, 0); // d
		double f = cvGetReal2D(pX2, 8*n+5, 0); // f
		double g = cvGetReal2D(pX2, 8*n+6, 0); // g
		double h = cvGetReal2D(pX2, 8*n+7, 0); // h

		double wRot = vecFreqMatch[n];
		rotCost += wRot*(a*b + c*d)*(a*b + c*d);
		rotCost += wRot*(a*a + c*c - 1)*(a*a + c*c - 1);
		rotCost += wRot*(b*b + d*d - 1)*(b*b + d*d - 1);
		rotCost += wRot*(g*g + h*h)*(g*g + h*h);
	}

	error += rotCost;

	//error = sqrt(error/nPair);

	return error;
}

double CalMatchError3(ImageTransform* pX, CvMat* pX2, int nUnfixed, MatchPointPairs *pPairs, int nPair,
					  vector<int> vecFreqMatch, vector<int> vecAccFixed, int refImgIdx0, int refImgIdx,float weight)
{
	double error = 0;
	for(int n=0; n<nPair; n++)
	{
		int ptA_i = pPairs[n].ptA_i;
		int ptB_i = pPairs[n].ptB_i;

		DfPoint ptA_Bar;
		double hA64F[9];
		for(int t=0; t<9; t++)
		{
			hA64F[t] = pX[ptA_i].h.m[t];
		}
		ApplyProject9(hA64F, pPairs[n].ptA.x, pPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);

		DfPoint ptB_Bar;

		double hB64F[9];
		for(int t=0; t<9; t++)
		{
			hB64F[t] = pX[ptB_i].h.m[t];
		}
		ApplyProject9(hB64F, pPairs[n].ptB.x, pPairs[n].ptB.y, ptB_Bar.x, ptB_Bar.y);
		double dist;
		DistanceOfTwoPoints(ptA_Bar.x, ptA_Bar.y, ptB_Bar.x, ptB_Bar.y, dist);
		error += dist*dist;
	}

	for(int n=0; n<nUnfixed; n++)
	{
		int nC = vecFreqMatch[n];

		double a = cvGetReal2D(pX2, 8*n+0, 0); // a
		double b = cvGetReal2D(pX2, 8*n+1, 0); // b
		double e = cvGetReal2D(pX2, 8*n+2, 0); // e
		double c = cvGetReal2D(pX2, 8*n+3, 0); // c
		double d = cvGetReal2D(pX2, 8*n+4, 0); // d
		double f = cvGetReal2D(pX2, 8*n+5, 0); // f
		double g = cvGetReal2D(pX2, 8*n+6, 0); // g
		double h = cvGetReal2D(pX2, 8*n+7, 0); // h

		double wRot = vecFreqMatch[n];
		error += wRot*(a*b + c*d)*(a*b + c*d);
		error += wRot*(a*a + c*c - 1)*(a*a + c*c - 1);
		error += wRot*(b*b + d*d - 1)*(b*b + d*d - 1);
		error += wRot*(g*g + h*h)*(g*g + h*h);
	}

	// ref cost--------------------------------------
	int ptA_i = refImgIdx;
	double hA[9];
	for(int i=0; i<8; i++)
	{
		hA[i] = cvGetReal2D(pX2, 8*ptA_i+i, 0);
	}
	hA[8] = 1;
	int n=0;
	for(int np=0; np<nPair; np++)
	{
		float xA, yA;
		if(pPairs[np].ptA_i==refImgIdx0)
		{
			xA = pPairs[np].ptA.x;
			yA = pPairs[np].ptA.y;
		}
		else if(pPairs[np].ptB_i==refImgIdx0)
		{
			xA = pPairs[np].ptB.x;
			yA = pPairs[np].ptB.y;
		}
		else
		{
			continue;
		}

		double h6x_h7y_1_a  = hA[6]*xA + hA[7]*yA + 1;
		double h0x_h1y_h2_a = hA[0]*xA + hA[1]*yA + hA[2];
		double h3x_h4y_h5_a = hA[3]*xA + hA[4]*yA + hA[5];

		double f1 = (h0x_h1y_h2_a - xA*h6x_h7y_1_a) * weight;
		double f2 = (h3x_h4y_h5_a - yA*h6x_h7y_1_a) * weight;

		error += (f1*f1 + f2*f2);

		n++;
	}

	return error;
}

double CalMatchErrorAffine(ImageTransform* pX, CvMat* pX2, int nUnfixed, MatchPointPairs *pPairs, int nPair,
						vector<int> vecFreqMatch, vector<int> vecAccFixed)
{
	double error = 0;
	for(int n=0; n<nPair; n++)
	{
		int ptA_i = pPairs[n].ptA_i;
		int ptB_i = pPairs[n].ptB_i;

		DfPoint ptA_Bar;
		double hA64F[9];
		for(int t=0; t<9; t++)
		{
			hA64F[t] = pX[ptA_i].h.m[t];
		}
		ApplyProject9(hA64F, pPairs[n].ptA.x, pPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);

		DfPoint ptB_Bar;

		double hB64F[9];
		for(int t=0; t<9; t++)
		{
			hB64F[t] = pX[ptB_i].h.m[t];
		}
		ApplyProject9(hB64F, pPairs[n].ptB.x, pPairs[n].ptB.y, ptB_Bar.x, ptB_Bar.y);
		double dist;
		DistanceOfTwoPoints(ptA_Bar.x, ptA_Bar.y, ptB_Bar.x, ptB_Bar.y, dist);
		error += dist*dist;
	}


	for(int n=0; n<nUnfixed; n++)
	{
		int nC = vecFreqMatch[n];

		double a = cvGetReal2D(pX2, 6*n+0, 0); // a
		double b = cvGetReal2D(pX2, 6*n+1, 0); // b
		double e = cvGetReal2D(pX2, 6*n+2, 0); // e
		double c = cvGetReal2D(pX2, 6*n+3, 0); // c
		double d = cvGetReal2D(pX2, 6*n+4, 0); // d
		double f = cvGetReal2D(pX2, 6*n+5, 0); // f

		double wRot = vecFreqMatch[n];
		error += wRot*(a*b + c*d)*(a*b + c*d);
		error += wRot*(a*a + c*c - 1)*(a*a + c*c - 1);
		error += wRot*(b*b + d*d - 1)*(b*b + d*d - 1);
	}
	//error = sqrt(error/nPair);
	return error;
}

int cvPrintMatrix(CvMat* pSrc, char* pName, int syme)
{
	ofstream outM(pName, ios::trunc);

	int row = pSrc->rows;
	int col = pSrc->cols;

	IplImage* pDense = cvCreateImage(cvSize(col, row), 8, 1);
	cvZero(pDense);

	int nz = 0;
	for(int r=0; r<row; r++)
	{
		for(int c=0; c<col; c++)
		{
			double val = cvGetReal2D(pSrc, r, c); // a
			outM<<val<<" ";
			if(val>0)
			{
				*(pDense->imageData + r * pDense->widthStep + c) = (unsigned char)255;
			}
			if(syme==1)
			{
				if(r>c)
				{
					continue;
				}
				if(val!=0)
				{
					nz++;
				}
			}	
			else
			{
				if(val!=0)
				{
					nz++;
				}
			}
		}
		outM<<endl;
	}

	static int i = 0;
	char pImgName[200];
	sprintf_s(pImgName, "c:\\dense-%d.bmp", i);
	i++;
	cvSaveImage(pImgName, pDense);

	outM.close();
	cvReleaseImage(&pDense);

	cout<<"nz "<<nz<<endl;

	return 0;
}

void GetR(double ax, double ay, double az, double R[9])
{
	double Rx[9]={1,	0,	0,
				0, cos(ax), -sin(ax),
				0, sin(ax), cos(ax)};

	double Ry[9]={cos(ay),	0,	-sin(ay),
				0,		1,		0,
				sin(ay), 0,	cos(ay)};

	double Rz[9]={cos(az), -sin(az), 0,
				sin(az), cos(az),  0,
				0,		0,		 1};
	double temp[9];
	//R = Rz*Rx*Ry
	MulMatrix(Rz, 3, 3, Rx, 3, 3, temp);
	MulMatrix(temp, 3, 3, Ry, 3, 3, R);
}

// 正下视校正
int CMosaicByPose::RectifyImage2(const ImagePoseInfo *pImagePose, ProjectMat64F &H, double &imgCxPix, double &imgCyPix)
{
	double homography[9];

	double fx = m_inParams.fx;
	double fy = m_inParams.fy;
	double cx = m_inParams.cx;
	double cy = m_inParams.cy;
	double K[9] = {  fx, 0,  cx, 
					0,  fy, cy,
					0,  0,  1};

	double AX_Norm = pool::PI_64F; // 绕X轴转180度
	double AY_Norm = 0;
	double AZ_Norm = -pool::PI_64F/2;

	double AX = pImagePose->camPose.AX + AX_Norm; 
	double AY = pImagePose->camPose.AY + AY_Norm;
	double AZ = pImagePose->camPose.AZ + AZ_Norm;

	double TZ = m_heightNorm; // m
	double R_Real[9];
	GetR(AX, AY, AZ, R_Real);

	double CW[9] = { 1, 0, 0,
					0, 1, 0,
					0, 0, -m_heightNorm}; // 摄像机中心在世界坐标系中的坐标

	// 规范的角度
	double TZ_Norm = TZ; // 假设相机高度没有变化
	AX = AX_Norm;
	AY = AY_Norm;
	AZ = AZ_Norm;
	double R_Norm[9];
	GetR(AX, AY, AZ, R_Norm);

	double KR_Real[9], KR_Norm[9], H_Norm[9], H_tilt[9];
	double inv_H_tilt[9];

	MulMatrix(K, 3, 3, R_Real, 3, 3, KR_Real);
	MulMatrix(KR_Real, 3, 3, CW, 3, 3, H_tilt);

	MulMatrix(K, 3, 3, R_Norm, 3, 3, KR_Norm);
	MulMatrix(KR_Norm, 3, 3, CW, 3, 3, H_Norm);
	
	//for(int i=0; i<9; i++)
	//{
	//	cout<<H_tilt[i]<<" ";
	//}
	cout<<endl;
	//InverseMatrix(H_tilt, 3, inv_H_tilt, 1e-12);
	CvMat* pSrc = cvCreateMat(3, 3, CV_64F);
	CvMat* pDst = cvCreateMat(3, 3, CV_64F);
	for(int r=0; r<3; r++)
	{
		for(int c=0; c<3; c++)
		{
			cvSetReal2D(pSrc, r, c, H_tilt[3*r+ c]);
		}
	}
	cvInvert(pSrc, pDst);
	for(int r=0; r<3; r++)
	{
		for(int c=0; c<3; c++)
		{
			inv_H_tilt[3*r+ c] = cvGetReal2D(pDst, r, c);
		}
	}
	cvReleaseMat(&pSrc);
	cvReleaseMat(&pDst);

	MulMatrix(H_Norm, 3, 3, inv_H_tilt, 3, 3, homography);

	// 求校正后的图像的主点坐标, H-Norm*[0 0 1]'
	imgCxPix = ( (H_tilt[0]*0 + H_tilt[1]*0 + H_tilt[2]) / (H_tilt[6]*0 + H_tilt[7]*0 + H_tilt[8]) );
	imgCyPix = ( (H_tilt[3]*0 + H_tilt[4]*0 + H_tilt[5]) / (H_tilt[6]*0 + H_tilt[7]*0 + H_tilt[8]) );

	for(int i=0; i<9; i++)
	{
		H.m[i] = homography[i];
		cout<<H.m[i]<<", ";
		if((i+1)%3==0)
		{
			cout<<endl;
		}
	}
	cout<<endl;

	return 0;
}

// 正下视校正
// 假设场景为一个平面
int CMosaicByPose::RectifyImage3(const ImagePoseInfo *pImagePose, ProjectMat64F &H, double &imgCxPix, double &imgCyPix)
{
	double homography[9];

	double fx = pImagePose->camPose.F;
	double fy = fx;
	if(fx<=0)
	{
		memset(H.m, 0, sizeof(double)*9);
		return 0;
	}

	double cx = pImagePose->pImg->width*0.5;
	double cy = pImagePose->pImg->height*0.5;

	double K[9] = {  fx, 0,  cx, 
					0,  fy, cy,
					0,  0,  1}; // 内参矩阵

	double TX = pImagePose->camPose.pos.x; // 像机中心在世界坐标系中的坐标
	double TY = pImagePose->camPose.pos.y; 
	double TZ = pImagePose->camPose.pos.z; // 像机中心在世界坐标系中的高度
	//TX = TY = 0;
	
	double R_Real[9];
	memcpy(R_Real, pImagePose->camPose.R, sizeof(R_Real));

	double CW_Norm[9] = { 1, 0, -TX,
						  0, 1, -TY,
						  0, 0, -m_heightNorm}; // 摄像机中心在世界坐标系中的坐标

	double CW_Real[9] = { 1, 0, -TX,
						  0, 1, -TY,
						  0, 0, -TZ}; // 摄像机中心在世界坐标系中的坐标

	double R_Norm[9] = {1, 0, 0,
						0, -1, 0,
						0, 0, -1};

	double KR_Real[9], KR_Norm[9], H_Norm[9], H_tilt[9];
	double inv_H_tilt[9];

	MulMatrix(K, 3, 3, R_Real, 3, 3, KR_Real);
	MulMatrix(KR_Real, 3, 3, CW_Real, 3, 3, H_tilt);

	MulMatrix(K, 3, 3, R_Norm, 3, 3, KR_Norm);
	MulMatrix(KR_Norm, 3, 3, CW_Norm, 3, 3, H_Norm); // 

	for(int i=0; i<9; i++)
	{
		//cout<<H_tilt[i]<<" ";
	}
	//cout<<endl;
	InverseMatrix(H_tilt, 3, inv_H_tilt, 1e-12);

	MulMatrix(H_Norm, 3, 3, inv_H_tilt, 3, 3, homography);

	// 求校正后的图像的主点坐标, H_Norm*[0 0 1]'
	imgCxPix = ( (H_tilt[0]*TX + H_tilt[1]*TY + H_tilt[2]) / (H_tilt[6]*TX + H_tilt[7]*TY + H_tilt[8]) );
	imgCyPix = ( (H_tilt[3]*TX + H_tilt[4]*TY + H_tilt[5]) / (H_tilt[6]*TX + H_tilt[7]*TY + H_tilt[8]) );
	//cout<<"imgCxPix "<<imgCxPix<<" imgCyPix "<<imgCyPix<<endl;
	for(int i=0; i<9; i++)
	{
		H.m[i] = homography[i];
		//cout<<H.m[i]<<", ";
	}
	//cout<<endl;

	return 0;
}

int CMosaicByPose::MosaicByBundler(ImagePoseInfo *pImgPoses, const int nImages,
								   vector<CvPoint3D64f> &vecPoints, vector<MatchPointPairs> vecMatchPairs)
{
	vector<ProjectMat64F> vecHomo64F;

	CamPose2Homo3(pImgPoses, nImages, vecPoints,
		vecHomo64F, vecMatchPairs);

	return 0;
}


int CMosaicByPose::MosaicByPose_Image(ImagePoseInfo *pImgPoses, const int nImages)
{
	vector<ImageTransform> vecImageTransformRefined;

	for(int i=0; i<nImages; i++)
	{
		// 图像正视校正, 计算出正视校正的变换矩阵
		ProjectMat64F tempH;
		double imgCxPix, imgCyPix;
		//RectifyImage(pImgPoses, tempH, imgCxPix, imgCyPix);
		RectifyImage2(&pImgPoses[i], tempH, imgCxPix, imgCyPix);

		// 计算出图像中心所在的位置
		double imgCxPys = pImgPoses[i].camPose.pos.x;
		double imgCyPys = pImgPoses[i].camPose.pos.y;
		RectifiedImage tempRectified;
		tempRectified.H = tempH;
		tempRectified.imgCxPys = imgCxPys;
		tempRectified.imgCyPys = imgCyPys;
		tempRectified.imgCxPix = imgCxPix;
		tempRectified.imgCyPix = imgCyPix;
		vecRectified.push_back(tempRectified);
	}	
	// 用GPS/IMU数据计算图像的初始变换参数
	vector<ProjectMat> vecImagesTransform0;
	CalImgTransformByPosIMU(pImgPoses, nImages, &vecRectified[0], vecImagesTransform0 );

	vector<ImageTransform> vecImageTransformInit;
	for(int i=0; i<(int)vecImagesTransform0.size(); i++)
	{
		ImageTransform temp;
		temp.fixed = pImgPoses[i].fixed;
		temp.h = vecImagesTransform0[i];
		vecImageTransformInit.push_back(temp);
	}

	// 对图像进行分段
	vector<Segment> vecSegment;
	SegmentSequenceImages(pImgPoses, nImages, &vecRectified[0], vecSegment);
	pImgPoses[3].fixed = 1;
	pImgPoses[8].fixed = 1;
	pImgPoses[15].fixed = 1;
	pImgPoses[16].fixed = 1;
	pImgPoses[21].fixed = 1;
	pImgPoses[30].fixed = 1;
	pImgPoses[31].fixed = 1;
	pImgPoses[32].fixed = 1;
	pImgPoses[33].fixed = 1;
	pImgPoses[0].fixed = 1;
	//pImgPoses[36].fixed = 1;
	//pImgPoses[35].fixed = 1;

	int usingSegment = 0;
	if(usingSegment) // 用分段的方法
	{
		for(int nSeg=0; nSeg<(int)vecSegment.size(); nSeg++)
		{
			int iBeg = vecSegment[nSeg].beg;
			int iEnd = vecSegment[nSeg].end;
			int nSegImages = iEnd - iBeg + 1;

			for(int i=iBeg; i<=iEnd; i++)
			{
				vecImageTransformInit[i].fixed = pImgPoses[i].fixed;
			}

			// 分段计算图像的变换参数
			vector<ImageTransform> vecTransformSeg;
			MosaicBySegment(&pImgPoses[iBeg], nSegImages, &vecImageTransformInit[iBeg],
							vecTransformSeg);
			if(nSeg==0)
			{
				for(int i=iBeg; i<=iEnd; i++)
				{
					vecImageTransformRefined.push_back(vecTransformSeg[i]);
				}
			}
			else
			{
				for(int i=iBeg+1; i<=iEnd; i++)
				{
					vecImageTransformRefined.push_back(vecTransformSeg[i-iBeg]);
				}
			}
		}
	}
	else // 全部图像同时求解
	{
		for(int n=0; n<nImages; n++)
		{
			//int iBeg = vecSegment[nSeg].beg;
			//int iEnd = vecSegment[nSeg].end;
			//int nSegImages = iEnd - iBeg + 1;

			vecImageTransformInit[n].fixed = pImgPoses[n].fixed;
		}
		MosaicBySegment(&pImgPoses[0], nImages, &vecImageTransformInit[0],
			vecImageTransformRefined);
	}

	cout<<"镶嵌图像"<<endl<<endl;

	// 镶嵌图像
	
	if(m_blending==2) // 拉普拉斯图像融合
	{
		MergeImagesRefined(pImgPoses, nImages, &vecImageTransformRefined[0]);
		//MergeImagesRefined(pImgPoses, nImages, &vecImageTransformInit[0]);
	}
	else
	{
		MosaicImagesRefined(pImgPoses, nImages, &vecImageTransformRefined[0]);
	}

	return 0;
}


// 用GPS信息和图像配准, 来做图像拼接
int CMosaicByPose::MosaicBySegment(ImagePoseInfo *pImgPoses, const int nImages, ImageTransform* pImageTransformInit,
								   vector<ImageTransform> &vecImageTransformRefined )
{
	if(NULL==pImgPoses)
		return -1;

	// 用图像配准精化变换参数
	AdjustmentByImage(pImgPoses, nImages, pImageTransformInit, vecImageTransformRefined);

	return 0;
}

// 
int CMosaicByPose::Mosaic( ImagePoseInfo *pImgPoses, int nImages )
{
	if(NULL==pImgPoses)
		return -1;

	vector<RectifiedImage> vecRectified;
	for(int i=0; i<nImages; i++)
	{
		// 图像正视校正, 计算出正视校正的变换矩阵
		ProjectMat64F tempH;
		double imgCxPix, imgCyPix;
		//RectifyImage(pImgPoses, tempH, imgCxPix, imgCyPix);
		RectifyImage2(&pImgPoses[i], tempH, imgCxPix, imgCyPix);
		
		// 计算出图像中心所在的位置
		double imgCxPys = pImgPoses[i].camPose.pos.x;
		double imgCyPys = pImgPoses[i].camPose.pos.y;
		RectifiedImage tempRectified;
		tempRectified.H = tempH;
		tempRectified.imgCxPys = imgCxPys;
		tempRectified.imgCyPys = imgCyPys;
		tempRectified.imgCxPix = imgCxPix;
		tempRectified.imgCyPix = imgCyPix;
		vecRectified.push_back(tempRectified);
	}
	
	// 图像镶嵌
	//int MosaicImagesByPoses(ImagePoseInfo *pImgPoses, int nImages, RectifiedImage* vecRectified)
	MosaicImagesByPoses(pImgPoses, nImages, &vecRectified[0]);
	
	return 0;
}

// 用特征点对再一次调整变换参数
int CMosaicByPose::AdjustByFeaturePairs(ImagePoseInfo *pImgPoses, 
						 vector<ImageTransform> &vecTrans, const vector<MatchPointPairs> &vecMatchedPoints, int nImages,
						 int adjust)
{
	int iFixed = 0;
	vector<MatchPointPairs> vecMatPtTrans;
	int nPairs = (int)vecMatchedPoints.size();
	double meanDist = 0;
	vector<double> vecErrorPerImg(nImages);
	vector<int> vecErrorNum(nImages);
	vector<MatchNeighbourError> vecNeighbourError(nImages);
	memset(&vecErrorPerImg[0], 0, sizeof(double)*nImages);
	memset(&vecErrorNum[0], 0, sizeof(int)*nImages);
	for(int n=0; n<nPairs; n++)
	{
		int ptA_i = vecMatchedPoints[n].ptA_i;
		int ptB_i = vecMatchedPoints[n].ptB_i;

		if( (ptA_i<nImages) && (ptB_i<nImages) )
		{
			if( (vecTrans[ptA_i].h.m[8]==0) || (vecTrans[ptB_i].h.m[8]==0) )
			{
				continue;
			}
			float wHalfA = pImgPoses[ptA_i].pImg->width*0.5f;
			float hHalfA = pImgPoses[ptA_i].pImg->height*0.5f;
			float wHalfB = pImgPoses[ptB_i].pImg->width*0.5f;
			float hHalfB = pImgPoses[ptB_i].pImg->height*0.5f;

			MatchPointPairs temp;
			temp.ptA_Fixed = temp.ptB_Fixed = 0;
			temp.ptA_i = vecMatchedPoints[n].ptA_i;
			temp.ptB_i = vecMatchedPoints[n].ptB_i;
			ApplyProject9(vecTrans[ptA_i].h.m, vecMatchedPoints[n].ptA.x+wHalfA, -vecMatchedPoints[n].ptA.y+hHalfA, temp.ptA.x, temp.ptA.y);
			ApplyProject9(vecTrans[ptB_i].h.m, vecMatchedPoints[n].ptB.x+wHalfB, -vecMatchedPoints[n].ptB.y+hHalfB, temp.ptB.x, temp.ptB.y);
			if(ptA_i==iFixed)
			{
				temp.ptA_Fixed = 1;
			}
			if(ptB_i==iFixed)
			{
				temp.ptB_Fixed = 1;
			}
			vecMatPtTrans.push_back(temp);
			float dist = 0;
			DistanceOfTwoPoints(temp.ptA.x, temp.ptA.y, temp.ptB.x, temp.ptB.y, dist);
			float distSquare = dist*dist;
			meanDist += (double)distSquare;
			vecErrorPerImg[ptA_i] += (double)distSquare;
			vecErrorPerImg[ptB_i] += (double)distSquare;
			vecErrorNum[ptA_i]++;
			vecErrorNum[ptB_i]++;
		}
	}
	meanDist = sqrt(meanDist/(int)vecMatPtTrans.size());
	cout<<"------meanDist "<<meanDist<<endl;
	for(int n=0; n<nImages; n++)
	{
		vecNeighbourError[n].nMatch = vecErrorNum[n];
		vecNeighbourError[n].iImage = n;
		if(vecErrorNum[n]>0)
		{
			vecErrorPerImg[n] = sqrt(vecErrorPerImg[n]/vecErrorNum[n]);
			//cout<<n<<" error "<<vecErrorPerImg[n]<<" vecErrorNum[n] "<<vecErrorNum[n]<<endl;
			vecNeighbourError[n].error = vecErrorPerImg[n];
		}
	}

	if(adjust==1) // 调整平移
	{
		int nFixed = 0;
		vector<Translation32F> vecTranslation0, vecTranslation;
		for(int n=0; n<nImages; n++)
		{
			Translation32F temp;
			temp.fixed = 0;
			temp.dx = 0; temp.dy = 0;
			if( (n==iFixed) || (vecTrans[n].h.m[8]==0) )
			{
				temp.fixed = 1;
				nFixed++;
			}
			vecTranslation0.push_back(temp);
		}

		SBA_Translation(&vecMatPtTrans[0], vecMatPtTrans.size(), &vecTranslation0[0], 
						nImages, nFixed, 
						vecTranslation);

		for(int n=0; n<nImages; n++)
		{
			HomographyPlusTranslation(vecTrans[n].h.m, vecTranslation[n].dx, vecTranslation[n].dy);
		}
	}
	else if(adjust==2) // 仿射调整
	{
		int nFixed = 0;
		sort( vecNeighbourError.begin(), vecNeighbourError.end() );

		float hEye[9] = {1, 0, 0, 
						 0, 1, 0, 
						 0, 0, 1};
		vector<ImageTransform> vecTrans0(nImages);
		for(int i=0; i<(int)vecTrans0.size(); i++)
		{
			vecTrans0[i].fixed = 0;
			memcpy(vecTrans0[i].h.m, hEye, sizeof(hEye)); // 全部置为单位阵
			bool shouldFixed = false;
			for(int j=0; j<nImages/3; j++) // 确定单应fixed的图像
			{
				if(i==vecNeighbourError[j].iImage)
				{
					shouldFixed = true;
					break;
				}
			}
			if(shouldFixed)
			{
				vecTrans0[i].fixed = 1;
				nFixed++;
			}
		}

		// 确定每个fixed的点
		for(int i=0; i<(int)vecMatPtTrans.size(); i++)
		{
			int ptA_i = vecMatPtTrans[i].ptA_i;
			int ptB_i = vecMatPtTrans[i].ptB_i;
			if((ptA_i==29) || (ptB_i==29))
			{
				i=i;
			}
			if((ptA_i==68) || (ptB_i==68))
			{
				i=i;
			}
			vecMatPtTrans[i].ptA_Fixed = 0;
			vecMatPtTrans[i].ptB_Fixed = 0;
			if(vecTrans0[ptA_i].fixed)
			{
				vecMatPtTrans[i].ptA_Fixed = 1;
			}
			if(vecTrans0[ptB_i].fixed)
			{
				vecMatPtTrans[i].ptB_Fixed = 1;
			}
		}
		
		vector<ImageTransform> vecTransRefined;
		//int nRetBA = BundleAdjustment(&vecMatPtTrans[0], vecMatPtTrans.size(), 
		//									&vecTrans0[0], nImages, nFixed, vecTransRefined);
		int nRetBA = BundleAdjustment(&vecMatPtTrans[0], vecMatPtTrans.size(), 
			&vecTrans0[0], nImages, nFixed, vecTransRefined);

		if(nRetBA==0)
		{
			// 更新单应矩阵
			for(int n=0; n<nImages; n++)
			{
				float tempDst[9];
				MulMatrix(vecTransRefined[n].h.m, 3, 3, vecTrans[n].h.m, 3, 3, tempDst);
				memcpy(vecTrans[n].h.m, tempDst, sizeof(tempDst));
			}
		}
	}

	return 0;
}

// 
int CMosaicByPose::Mosaic2( ImagePoseInfo *pImgPoses, int nImages, vector<MatchPointPairs> vecMatchedPoints )
{
	if(NULL==pImgPoses)
		return -1;

	vector<RectifiedImage> vecRectified;
	for(int i=0; i<nImages; i++)
	{
		// 图像正视校正, 计算出正视校正的变换矩阵
		ProjectMat64F tempH;
		double imgCxPix, imgCyPix;
		//RectifyImage(pImgPoses, tempH, imgCxPix, imgCyPix);
		RectifyImage3(&pImgPoses[i], tempH, imgCxPix, imgCyPix);

		// 计算出图像中心所在的物理位置
		double imgCxPys = pImgPoses[i].camPose.pos.x;
		double imgCyPys = pImgPoses[i].camPose.pos.y;
		RectifiedImage tempRectified;
		tempRectified.H = tempH;
		tempRectified.imgCxPys = imgCxPys;
		tempRectified.imgCyPys = imgCyPys;
		tempRectified.imgCxPix = imgCxPix;
		tempRectified.imgCyPix = imgCyPix;
		vecRectified.push_back(tempRectified);
	}

	// 图像镶嵌
	//int MosaicImagesByPoses(ImagePoseInfo *pImgPoses, int nImages, RectifiedImage* vecRectified)
	vector<ImageTransform> vecTrans;
	MosaicImagesByPoses(pImgPoses, nImages, &vecRectified[0], vecTrans);

	// 用特征点对再一次调整
	int adjust = 1; // 平移调整
	AdjustByFeaturePairs(pImgPoses, vecTrans, vecMatchedPoints, nImages, adjust);

	adjust = 2; // 仿射调整1
	AdjustByFeaturePairs(pImgPoses, vecTrans, vecMatchedPoints, nImages, adjust);

	//adjust = 2; // 仿射调整2
	//AdjustByFeaturePairs(pImgPoses, vecTrans, vecMatchedPoints, nImages, adjust);

	adjust = 0; // 不调调整, 看误差
	AdjustByFeaturePairs(pImgPoses, vecTrans, vecMatchedPoints, nImages, adjust);

	if(!vecTrans.empty())
	{
		if(m_blending==2) // 拉普拉斯图像融合
		{
			MergeImagesRefined(pImgPoses, nImages, &vecTrans[0]);
		}
		else
		{
			MosaicImagesRefined(pImgPoses, nImages, &vecTrans[0]);
		}
	}

	cvSaveImage("c:\\mosaic.bmp", GetMosaicResult());

	return 0;
}

// 根据GPS和惯导数据计算图像的变换参数
int CMosaicByPose::CalImgTransformByPosIMU(const ImagePoseInfo *pImgPoses, const int nImages, const RectifiedImage* pRectified,
										   vector<ProjectMat> &vecImagesTranform)
{
	if(NULL==pImgPoses)
		return -1;

	vector<DfPoint> vecPosInMosPix;
	float minX = 1<<29, minY = 1<<29, maxX = -1<<29, maxY = -1<<29;
	// 找出所有图像的最小外接矩形的坐标
	for(int n=0; n<nImages; n++)
	{
		//求纠正后的图像的主点在拼接图上的图像坐标
		DfPoint posInMosPix;
		posInMosPix.x = pRectified[n].imgCxPys / m_heightNorm * m_inParams.fx;
		posInMosPix.y = -pRectified[n].imgCyPys / m_heightNorm * m_inParams.fy;
		vecPosInMosPix.push_back(posInMosPix);

		// 对图像的4个顶点进行单应矩阵变换, 再进行平移
		// 0 1
		// 3 2
		SfPoint corner[4], cornerBar[4];
		corner[0].x = 0;									corner[0].y = 0;
		corner[1].x = (float)(pImgPoses[n].pImg->width-1);	corner[1].y = 0;
		corner[2].x = (float)(pImgPoses[n].pImg->width-1);	corner[2].y = (float)(pImgPoses[n].pImg->height-1);
		corner[3].x = 0;									corner[3].y = (float)(pImgPoses[n].pImg->height-1);
		float dx = 0, dy = 0; // 平移量
		dx = 0 - pRectified[n].imgCxPix + posInMosPix.x;
		dy = 0 - pRectified[n].imgCyPix + posInMosPix.y;
		//for(int i=0; i<4; i++)
		//{
		//	// 单应变换
		//	ApplyProject9(pRectified[n].H.m, corner[i].x, corner[i].y, cornerBar[i].x, cornerBar[i].y);
		//}

		// 单应变换后 + (dx,dy)
		ProjectMat temp;
		temp.m[0] = pRectified[n].H.m[0] + dx * pRectified[n].H.m[6];
		temp.m[1] = pRectified[n].H.m[1] + dx * pRectified[n].H.m[7];
		temp.m[2] = pRectified[n].H.m[2] + dx * pRectified[n].H.m[8];

		temp.m[3] = pRectified[n].H.m[3] + dy * pRectified[n].H.m[6];
		temp.m[4] = pRectified[n].H.m[4] + dy * pRectified[n].H.m[7];
		temp.m[5] = pRectified[n].H.m[5] + dy * pRectified[n].H.m[8];

		temp.m[6] = pRectified[n].H.m[6];
		temp.m[7] = pRectified[n].H.m[7];
		temp.m[8] = pRectified[n].H.m[8];
		vecImagesTranform.push_back(temp);
	}
	return 0;
}

// 功能: 镶嵌纠正后的图像
// 物理尺寸(X,Y)和与像素(x,y)之间的关系?
// dx = dX*Fx/Zc  *dx的单位为pixel, dX的单位为米 *
// dy = dY*Fy/Zc
int CMosaicByPose::MosaicImagesByPoses(const ImagePoseInfo *pImgPoses, const int nImages, const RectifiedImage* pRectified,
									   vector<ImageTransform> &vecTrans)
{
	if(NULL==pImgPoses)
		return -1;

	double minX = 1<<29, minY = 1<<29, maxX = -1<<29, maxY = -1<<29;

	for(int n=0; n<nImages; n++)
	{
		//求纠正后的图像的主点在拼接图上的图像坐标
		DfPoint posInMosPix;
		posInMosPix.x = posInMosPix.y = 0;
		if(pImgPoses[n].camPose.F>0)
		{
			posInMosPix.x = pRectified[n].imgCxPys / m_heightNorm * pImgPoses[n].camPose.F;
			posInMosPix.y = -pRectified[n].imgCyPys / m_heightNorm * pImgPoses[n].camPose.F; // 因为图像坐标的Y方向与世界坐标系的Y方向相反
		}

		double dx = 0, dy = 0; // 平移量
		double Hx = 0, Hy = 0; // 校正后的图像主点坐标
		if(pRectified[n].H.m[8]!=0)
		{
			ApplyProject9(pRectified[n].H.m, pRectified[n].imgCxPix, pRectified[n].imgCyPix, Hx, Hy);
		}
		cout<<"Hx "<<Hx<<" Hy "<<Hy<<endl;
		dx = posInMosPix.x - Hx;
		dy = posInMosPix.y - Hy;

		ImageTransform tempT;
		tempT.h.m[0] = pRectified[n].H.m[0] + dx * pRectified[n].H.m[6];
		tempT.h.m[1] = pRectified[n].H.m[1] + dx * pRectified[n].H.m[7];
		tempT.h.m[2] = pRectified[n].H.m[2] + dx * pRectified[n].H.m[8];

		tempT.h.m[3] = pRectified[n].H.m[3] + dy * pRectified[n].H.m[6];
		tempT.h.m[4] = pRectified[n].H.m[4] + dy * pRectified[n].H.m[7];
		tempT.h.m[5] = pRectified[n].H.m[5] + dy * pRectified[n].H.m[8];

		tempT.h.m[6] = pRectified[n].H.m[6];
		tempT.h.m[7] = pRectified[n].H.m[7];
		tempT.h.m[8] = pRectified[n].H.m[8];

		vecTrans.push_back(tempT);
	}
	return 0;
}

// 功能: 镶嵌纠正后的图像
// 物理尺寸(X,Y)和与像素(x,y)之间的关系?
// dx = dX*Fx/Zc  *dx的单位为pixel, dX的单位为米 *
// dy = dY*Fy/Zc
int CMosaicByPose::MosaicImagesByPoses(const ImagePoseInfo *pImgPoses, const int nImages, const RectifiedImage* pRectified)
{
	if(NULL==pImgPoses)
		return -1;
	
	vector<DfPoint> vecPosInMosPix;
	double minX = 1<<29, minY = 1<<29, maxX = -1<<29, maxY = -1<<29;
	// 找出所有图像的最小外接矩形的坐标
	for(int n=0; n<nImages; n++)
	{
		//求纠正后的图像的主点在拼接图上的图像坐标
		DfPoint posInMosPix;
		posInMosPix.x = pRectified[n].imgCxPys / m_heightNorm * pImgPoses[n].camPose.F;
		posInMosPix.y = -pRectified[n].imgCyPys / m_heightNorm * pImgPoses[n].camPose.F; // 因为图像坐标的Y方向与世界坐标系的Y方向相反
		vecPosInMosPix.push_back(posInMosPix);

		// 对图像的4个顶点进行单应矩阵变换, 再进行平移
		// 0 1
		// 3 2
		DfPoint corner[4], cornerBar[4];
		corner[0].x = 0;									corner[0].y = 0;
		corner[1].x = (double)(pImgPoses[n].pImg->width-1);	corner[1].y = 0;
		corner[2].x = (double)(pImgPoses[n].pImg->width-1);	corner[2].y = (double)(pImgPoses[n].pImg->height-1);
		corner[3].x = 0;									corner[3].y = (double)(pImgPoses[n].pImg->height-1);
		double dx = 0, dy = 0; // 平移量
		double Hx, Hy;
		ApplyProject9(pRectified[n].H.m, pRectified[n].imgCxPix, pRectified[n].imgCyPix, Hx, Hy);
		dx = posInMosPix.x - Hx;
		dy = posInMosPix.y - Hy;

		//ImageTransform tempT;
		//tempT.h.m[0] = pRectified[n].H.m[0] + dx*pRectified[n].H.m[6];
		//tempT.h.m[1] = pRectified[n].H.m[1] + dx*pRectified[n].H.m[7];
		//tempT.h.m[2] = pRectified[n].H.m[2] + dx*pRectified[n].H.m[8];

		//tempT.h.m[3] = pRectified[n].H.m[3] + dy*pRectified[n].H.m[6];
		//tempT.h.m[4] = pRectified[n].H.m[4] + dy*pRectified[n].H.m[7];
		//tempT.h.m[5] = pRectified[n].H.m[5] + dy*pRectified[n].H.m[8];

		//tempT.h.m[6] = pRectified[n].H.m[6];
		//tempT.h.m[7] = pRectified[n].H.m[7];
		//tempT.h.m[8] = pRectified[n].H.m[8];

		for(int i=0; i<4; i++)
		{
			// 单应变换
			ApplyProject9(pRectified[n].H.m, corner[i].x, corner[i].y, cornerBar[i].x, cornerBar[i].y);

			// 平移
			if(cornerBar[i].x + dx<minX)
			{
				minX = cornerBar[i].x + dx;
			}
			if(cornerBar[i].x + dx>maxX)
			{
				maxX = cornerBar[i].x + dx;
			}
			if(cornerBar[i].y + dy<minY)
			{
				minY = cornerBar[i].y + dy;
			}
			if(cornerBar[i].y + dy>maxY)
			{
				maxY = cornerBar[i].y + dy;
			}	
		}
	}
	unsigned int mosaicWidth = int(maxX - minX + 1.5f);
	unsigned int mosaicHeight = int(maxY - minY + 1.5f);
	cvReleaseImage(&m_pMosaicResult);
	m_pMosaicResult = cvCreateImage(cvSize(mosaicWidth,mosaicHeight), 8, 3);
	memset(m_pMosaicResult->imageData, 0, m_pMosaicResult->widthStep*m_pMosaicResult->height);

	int wsRes = m_pMosaicResult->widthStep;
	double dGX = -minX, dGY = -minY;
	
	// 注册图像
	for(int n=0; n<nImages; n++)
	{
		
		int wSrc, hSrc, wsSrc;
		wSrc = pImgPoses[n].pImg->width; 
		hSrc = pImgPoses[n].pImg->height; 
		wsSrc = pImgPoses[n].pImg->widthStep;
		double dx = 0, dy = 0; // 平移量
		//dx = 0 - pRectified[n].imgCxPix + vecPosInMosPix[n].x;
		//dy = 0 - pRectified[n].imgCyPix + vecPosInMosPix[n].y;
		double Hx, Hy;
		ApplyProject9(pRectified[n].H.m, pRectified[n].imgCxPix, pRectified[n].imgCyPix, Hx, Hy);
		dx = vecPosInMosPix[n].x - Hx;
		dy = vecPosInMosPix[n].y - Hy;

		// 双线性插值版本dst -> src
		int wSrc_1 = pImgPoses[n].pImg->width-1;
		int hSrc_1 = pImgPoses[n].pImg->height-1;
		double invH[9];
		InverseMatrix(pRectified[n].H.m, 3, invH, 1e-12);
		DfPoint corner[4], cornerBar[4];
		corner[0].x = 0;									corner[0].y = 0;
		corner[1].x = (double)(pImgPoses[n].pImg->width-1);	corner[1].y = 0;
		corner[2].x = (double)(pImgPoses[n].pImg->width-1);	corner[2].y = (double)(pImgPoses[n].pImg->height-1);
		corner[3].x = 0;									corner[3].y = (double)(pImgPoses[n].pImg->height-1);
		double minX = 1<<29, minY = 1<<29, maxX = -1<<29, maxY = -1<<29;
		for(int i=0; i<4; i++)
		{
			// 单应变换
			ApplyProject9(pRectified[n].H.m, corner[i].x, corner[i].y, cornerBar[i].x, cornerBar[i].y);
			cornerBar[i].x += (dx + dGX);
			cornerBar[i].y += (dy + dGY);
			if(cornerBar[i].x<minX)
			{
				minX = cornerBar[i].x;
			}
			if(cornerBar[i].x>maxX)
			{
				maxX = cornerBar[i].x;
			}
			if(cornerBar[i].y<minY)
			{
				minY = cornerBar[i].y;
			}
			if(cornerBar[i].y>maxY)
			{
				maxY = cornerBar[i].y;
			}
		}
		int begY = int(minY-0.5f), endY = int(maxY+0.5f);
		int begX = int(minX-0.5f), endX = int(maxX+0.5f);
		for(int yDst=begY; yDst<=endY; yDst++)
		{
			unsigned char* pRowDst = (unsigned char*)m_pMosaicResult->imageData + yDst*wsRes;

			for(int xDst=begX; xDst<=endX; xDst++)
			{
				double xMid = xDst - dx - dGX;
				double yMid = yDst - dy - dGY;
				double xSrc32F, ySrc32F;
				ApplyProject9(invH, xMid, yMid, xSrc32F, ySrc32F);
				// 双线性插值
				int xSrc = int(xSrc32F);
				int ySrc = int(ySrc32F);

				double p = ySrc32F - ySrc;
				double q = xSrc32F - xSrc;
				
				if( (ySrc32F<0)||(ySrc32F>=hSrc_1) )
					continue;
				if( (xSrc32F<0)||(xSrc32F>=wSrc_1) )
					continue;

				float eff = 0;
				//if(n==0)
				{
					eff = 1;
					unsigned char* pSrcTemp = (unsigned char*)pImgPoses[n].pImg->imageData + ySrc*wsSrc + 3*xSrc;

					*(pRowDst + 3*xDst) =  unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
						*(pSrcTemp + 3)*(1-p)*q + 
						*(pSrcTemp + wsSrc)*p*(1-q) + 
						*(pSrcTemp + wsSrc + 3)*p*q );

					pSrcTemp = (unsigned char*)pImgPoses[n].pImg->imageData + ySrc*wsSrc + 3*xSrc+1;
					*(pRowDst + 3*xDst+1) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
						*(pSrcTemp + 3)*(1-p)*q + 
						*(pSrcTemp + wsSrc)*p*(1-q) + 
						*(pSrcTemp + wsSrc + 3)*p*q );

					pSrcTemp = (unsigned char*)pImgPoses[n].pImg->imageData + ySrc*wsSrc + 3*xSrc+2;
					*(pRowDst + 3*xDst+2) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
						*(pSrcTemp + 3)*(1-p)*q + 
						*(pSrcTemp + wsSrc)*p*(1-q) + 
						*(pSrcTemp + wsSrc + 3)*p*q );
				}
				//else
				//{
				//	eff = 0.5;
				//	unsigned char* pSrcTemp = (unsigned char*)pImgPoses[n].pImg->imageData + ySrc*wsSrc + 3*xSrc;

				//	*(pRowDst + 3*xDst) += eff * unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
				//		*(pSrcTemp + 3)*(1-p)*q + 
				//		*(pSrcTemp + wsSrc)*p*(1-q) + 
				//		*(pSrcTemp + wsSrc + 3)*p*q );

				//	pSrcTemp = (unsigned char*)pImgPoses[n].pImg->imageData + ySrc*wsSrc + 3*xSrc+1;
				//	*(pRowDst + 3*xDst+1) += eff * unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
				//		*(pSrcTemp + 3)*(1-p)*q + 
				//		*(pSrcTemp + wsSrc)*p*(1-q) + 
				//		*(pSrcTemp + wsSrc + 3)*p*q );

				//	pSrcTemp = (unsigned char*)pImgPoses[n].pImg->imageData + ySrc*wsSrc + 3*xSrc+2;
				//	*(pRowDst + 3*xDst+2) += eff * unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
				//		*(pSrcTemp + 3)*(1-p)*q + 
				//		*(pSrcTemp + wsSrc)*p*(1-q) + 
				//		*(pSrcTemp + wsSrc + 3)*p*q );
				//}

			}
		}
		
		if(0)
		{
			cvSaveImage("c:\\mosaic.bmp", m_pMosaicResult);
			cvZero(m_pMosaicResult);
		}
	}
	
	return 0;
}

// 融合图像
int CMosaicByPose::MergeImagesRefined(ImagePoseInfo *pImgPoses, const int nImages, 
									  const ImageTransform* pRectified)
{
	if(nImages<=1)
	{
		return -2;
	}
	int blending = 1;
	int referenceType = 0;
	vector<IplImage*> vecImages;
	vector<ProjectMat> vecHomo;
	for(int i=0; i<nImages; i++)
	{
		vecImages.push_back(pImgPoses[i].pImg);
		vecHomo.push_back(pRectified[i].h);
	}
	//m_pMosaicResult = MergeMultiImages2((const IplImage**)&vecImages[0], nImages, &vecHomo[0], blending, referenceType);

	int band = 5;
	m_pMosaicResult = LaplacianPyramidBlending((IplImage**)&vecImages[0], nImages, &vecHomo[0], band, m_scale);

	for(int i=0; i<nImages; i++)
	{
		 pImgPoses[i].pImg = NULL; // 11-28
	}

	return 0;
}

// 功能: 镶嵌纠正后的图像
// 物理尺寸(X,Y)和与像素(x,y)之间的关系?
// dx = dX*Fx/Zc  *dx的单位为pixel, dX的单位为米 *
// dy = dY*Fy/Zc
int CMosaicByPose::MosaicImagesRefined(const ImagePoseInfo *pImgPoses, const int nImages, const ImageTransform* pRectified)
{
	if(NULL==pImgPoses)
		return -1;
	cout<<"without blending"<<endl;

	vector<SfPoint> vecPosInMosPix;
	float minX = 1<<29, minY = 1<<29, maxX = -1<<29, maxY = -1<<29;
	// 找出所有图像的最小外接矩形的坐标
	for(int n=0; n<nImages; n++)
	{
		if(pRectified[n].h.m[8]==0)
			continue;

		cout<<"镶嵌 "<<n<<" / "<<nImages<<endl;
		// 对图像的4个顶点进行单应矩阵变换, 再进行平移
		// 0 1
		// 3 2
		SfPoint corner[4], cornerBar[4];
		corner[0].x = 0;									corner[0].y = 0;
		corner[1].x = (float)(pImgPoses[n].pImg->width-1);	corner[1].y = 0;
		corner[2].x = (float)(pImgPoses[n].pImg->width-1);	corner[2].y = (float)(pImgPoses[n].pImg->height-1);
		corner[3].x = 0;									corner[3].y = (float)(pImgPoses[n].pImg->height-1);

		for(int i=0; i<4; i++)
		{
			// 单应变换
			ApplyProject9(pRectified[n].h.m, corner[i].x, corner[i].y, cornerBar[i].x, cornerBar[i].y);

			// 平移
			if(cornerBar[i].x<minX)
			{
				minX = cornerBar[i].x;
			}
			if(cornerBar[i].x>maxX)
			{
				maxX = cornerBar[i].x;
			}
			if(cornerBar[i].y<minY)
			{
				minY = cornerBar[i].y;
			}
			if(cornerBar[i].y>maxY)
			{
				maxY = cornerBar[i].y;
			}	
		}
	}
	unsigned int mosaicWidth = int(maxX - minX + 1.5f);
	unsigned int mosaicHeight = int(maxY - minY + 1.5f);
	cout<<"mosaicWidth "<<mosaicWidth<<"  "<<"mosaicHeight "<<mosaicHeight<<endl;

	cvReleaseImage(&m_pMosaicResult);
	m_pMosaicResult = cvCreateImage(cvSize(mosaicWidth,mosaicHeight), 8, 3);
	memset(m_pMosaicResult->imageData, 0, m_pMosaicResult->widthStep*m_pMosaicResult->height);

	int wsRes = m_pMosaicResult->widthStep;
	float dGX = -minX, dGY = -minY;

	// 注册图像
	for(int n=0; n<nImages; n++)
	{
		if(pRectified[n].h.m[8]==0)
			continue;

		if(n<26)
		{
			//continue;
		}
		//if(pRectified[n].fixed==0)
		//{
		//	continue;
		//}

		int wSrc, hSrc, wsSrc;
		wSrc = pImgPoses[n].pImg->width; hSrc = pImgPoses[n].pImg->height; wsSrc = pImgPoses[n].pImg->widthStep;

		// 双线性插值版本dst -> src
		int wSrc_1 = pImgPoses[n].pImg->width-1;
		int hSrc_1 = pImgPoses[n].pImg->height-1;
		float invH[9];
		InverseMatrix(pRectified[n].h.m, 3, invH, 1e-12f);
		SfPoint corner[4], cornerBar[4];
		corner[0].x = 0;									corner[0].y = 0;
		corner[1].x = (float)(pImgPoses[n].pImg->width-1);	corner[1].y = 0;
		corner[2].x = (float)(pImgPoses[n].pImg->width-1);	corner[2].y = (float)(pImgPoses[n].pImg->height-1);
		corner[3].x = 0;									corner[3].y = (float)(pImgPoses[n].pImg->height-1);
		float minX = 1<<29, minY = 1<<29, maxX = -1<<29, maxY = -1<<29;
		for(int i=0; i<4; i++)
		{
			// 单应变换
			ApplyProject9(pRectified[n].h.m, corner[i].x, corner[i].y, cornerBar[i].x, cornerBar[i].y);
			cornerBar[i].x += (0 + dGX);
			cornerBar[i].y += (0 + dGY);
			if(cornerBar[i].x<minX)
			{
				minX = cornerBar[i].x;
			}
			if(cornerBar[i].x>maxX)
			{
				maxX = cornerBar[i].x;
			}
			if(cornerBar[i].y<minY)
			{
				minY = cornerBar[i].y;
			}
			if(cornerBar[i].y>maxY)
			{
				maxY = cornerBar[i].y;
			}
		}
		int begY = int(minY-0.5f), endY = int(maxY+0.5f);
		int begX = int(minX-0.5f), endX = int(maxX+0.5f);
		for(int yDst=begY; yDst<=endY; yDst++)
		{
			unsigned char* pRowDst = (unsigned char*)m_pMosaicResult->imageData + yDst*wsRes;

			for(int xDst=begX; xDst<=endX; xDst++)
			{
				float xMid = xDst - 0 - dGX;
				float yMid = yDst - 0 - dGY;
				float xSrc32F, ySrc32F;
				ApplyProject9(invH, xMid, yMid, xSrc32F, ySrc32F);
				// 双线性插值
				int xSrc = int(xSrc32F);
				int ySrc = int(ySrc32F);

				float p = ySrc32F - ySrc;
				float q = xSrc32F - xSrc;

				if( (ySrc32F<0)||(ySrc32F>=hSrc_1) )
					continue;
				if( (xSrc32F<0)||(xSrc32F>=wSrc_1) )
					continue;

				unsigned char* pSrcTemp = (unsigned char*)pImgPoses[n].pImg->imageData + ySrc*wsSrc + 3*xSrc;

				*(pRowDst + 3*xDst) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
					*(pSrcTemp + 3)*(1-p)*q + 
					*(pSrcTemp + wsSrc)*p*(1-q) + 
					*(pSrcTemp + wsSrc + 3)*p*q );

				pSrcTemp = (unsigned char*)pImgPoses[n].pImg->imageData + ySrc*wsSrc + 3*xSrc+1;
				*(pRowDst + 3*xDst+1) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
					*(pSrcTemp + 3)*(1-p)*q + 
					*(pSrcTemp + wsSrc)*p*(1-q) + 
					*(pSrcTemp + wsSrc + 3)*p*q );

				pSrcTemp = (unsigned char*)pImgPoses[n].pImg->imageData + ySrc*wsSrc + 3*xSrc+2;
				*(pRowDst + 3*xDst+2) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
					*(pSrcTemp + 3)*(1-p)*q + 
					*(pSrcTemp + wsSrc)*p*(1-q) + 
					*(pSrcTemp + wsSrc + 3)*p*q );
			}
		}
	}

	return 0;
}

// 功能: 镶嵌纠正后的图像
// 物理尺寸(X,Y)和与像素(x,y)之间的关系?
// dx = dX*Fx/Zc  *dx的单位为pixel, dX的单位为米 *
// dy = dY*Fy/Zc
int CMosaicByPose::MosaicImagesRefined(const ImagePoseInfo *pImgPoses, const int nImages, const ImageTransform64F* pRectified)
{
	if(NULL==pImgPoses)
		return -1;

	vector<DfPoint> vecPosInMosPix;
	double minX = 1<<29, minY = 1<<29, maxX = -1<<29, maxY = -1<<29;
	// 找出所有图像的最小外接矩形的坐标
	for(int n=0; n<nImages; n++)
	{
		cout<<"镶嵌 "<<n<<" / "<<nImages<<endl;
		// 对图像的4个顶点进行单应矩阵变换, 再进行平移
		// 0 1
		// 3 2
		DfPoint corner[4], cornerBar[4];
		corner[0].x = 0;									corner[0].y = 0;
		corner[1].x = (double)(pImgPoses[n].pImg->width-1);	corner[1].y = 0;
		corner[2].x = (double)(pImgPoses[n].pImg->width-1);	corner[2].y = (double)(pImgPoses[n].pImg->height-1);
		corner[3].x = 0;									corner[3].y = (double)(pImgPoses[n].pImg->height-1);

		for(int i=0; i<4; i++)
		{
			// 单应变换
			ApplyProject9(pRectified[n].h.m, corner[i].x, corner[i].y, cornerBar[i].x, cornerBar[i].y);

			// 平移
			if(cornerBar[i].x<minX)
			{
				minX = cornerBar[i].x;
			}
			if(cornerBar[i].x>maxX)
			{
				maxX = cornerBar[i].x;
			}
			if(cornerBar[i].y<minY)
			{
				minY = cornerBar[i].y;
			}
			if(cornerBar[i].y>maxY)
			{
				maxY = cornerBar[i].y;
			}	
		}
	}
	unsigned int mosaicWidth = int(maxX - minX + 1.5f);
	unsigned int mosaicHeight = int(maxY - minY + 1.5f);
	cout<<"mosaicWidth "<<mosaicWidth<<"  "<<"mosaicHeight "<<mosaicHeight<<endl;

	cvReleaseImage(&m_pMosaicResult);
	m_pMosaicResult = cvCreateImage(cvSize(mosaicWidth,mosaicHeight), 8, 3);
	memset(m_pMosaicResult->imageData, 0, m_pMosaicResult->widthStep*m_pMosaicResult->height);

	int wsRes = m_pMosaicResult->widthStep;
	float dGX = -minX, dGY = -minY;

	// 注册图像
	for(int n=0; n<nImages; n++)
	{
		if(n<26)
		{
			//continue;
		}
		//if(pRectified[n].fixed==0)
		//{
		//	continue;
		//}

		int wSrc, hSrc, wsSrc;
		wSrc = pImgPoses[n].pImg->width; hSrc = pImgPoses[n].pImg->height; wsSrc = pImgPoses[n].pImg->widthStep;

		// 双线性插值版本dst -> src
		int wSrc_1 = pImgPoses[n].pImg->width-1;
		int hSrc_1 = pImgPoses[n].pImg->height-1;
		double invH[9];
		InverseMatrix(pRectified[n].h.m, 3, invH, 1e-12);
		DfPoint corner[4], cornerBar[4];
		corner[0].x = 0;									corner[0].y = 0;
		corner[1].x = (double)(pImgPoses[n].pImg->width-1);	corner[1].y = 0;
		corner[2].x = (double)(pImgPoses[n].pImg->width-1);	corner[2].y = (double)(pImgPoses[n].pImg->height-1);
		corner[3].x = 0;									corner[3].y = (double)(pImgPoses[n].pImg->height-1);
		double minX = 1<<29, minY = 1<<29, maxX = -1<<29, maxY = -1<<29;
		for(int i=0; i<4; i++)
		{
			// 单应变换
			ApplyProject9(pRectified[n].h.m, corner[i].x, corner[i].y, cornerBar[i].x, cornerBar[i].y);
			cornerBar[i].x += (0 + dGX);
			cornerBar[i].y += (0 + dGY);
			if(cornerBar[i].x<minX)
			{
				minX = cornerBar[i].x;
			}
			if(cornerBar[i].x>maxX)
			{
				maxX = cornerBar[i].x;
			}
			if(cornerBar[i].y<minY)
			{
				minY = cornerBar[i].y;
			}
			if(cornerBar[i].y>maxY)
			{
				maxY = cornerBar[i].y;
			}
		}
		int begY = int(minY-0.5f), endY = int(maxY+0.5f);
		int begX = int(minX-0.5f), endX = int(maxX+0.5f);
		for(int yDst=begY; yDst<=endY; yDst++)
		{
			unsigned char* pRowDst = (unsigned char*)m_pMosaicResult->imageData + yDst*wsRes;

			for(int xDst=begX; xDst<=endX; xDst++)
			{
				double xMid = xDst - 0 - dGX;
				double yMid = yDst - 0 - dGY;
				double xSrc32F, ySrc32F;
				ApplyProject9(invH, xMid, yMid, xSrc32F, ySrc32F);
				// 双线性插值
				int xSrc = int(xSrc32F);
				int ySrc = int(ySrc32F);

				float p = ySrc32F - ySrc;
				float q = xSrc32F - xSrc;

				if( (ySrc32F<0)||(ySrc32F>=hSrc_1) )
					continue;
				if( (xSrc32F<0)||(xSrc32F>=wSrc_1) )
					continue;

				unsigned char* pSrcTemp = (unsigned char*)pImgPoses[n].pImg->imageData + ySrc*wsSrc + 3*xSrc;

				*(pRowDst + 3*xDst) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
					*(pSrcTemp + 3)*(1-p)*q + 
					*(pSrcTemp + wsSrc)*p*(1-q) + 
					*(pSrcTemp + wsSrc + 3)*p*q );

				pSrcTemp = (unsigned char*)pImgPoses[n].pImg->imageData + ySrc*wsSrc + 3*xSrc+1;
				*(pRowDst + 3*xDst+1) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
					*(pSrcTemp + 3)*(1-p)*q + 
					*(pSrcTemp + wsSrc)*p*(1-q) + 
					*(pSrcTemp + wsSrc + 3)*p*q );

				pSrcTemp = (unsigned char*)pImgPoses[n].pImg->imageData + ySrc*wsSrc + 3*xSrc+2;
				*(pRowDst + 3*xDst+2) = unsigned char( *(pSrcTemp)*(1-p)*(1-q) + 
					*(pSrcTemp + 3)*(1-p)*q + 
					*(pSrcTemp + wsSrc)*p*(1-q) + 
					*(pSrcTemp + wsSrc + 3)*p*q );
			}
		}
	}

	return 0;
}

CMosaicByPose::CMosaicByPose()
{
	m_downViewConstraint = 1; // 正下视约束
	m_blending = 2;
	m_maxFeatureNum = 400;
	m_minHessian = 1000;
	m_matchDist = 0.5f;
	m_ransacDist = 2.0f;

	m_heightNorm = 0;
	m_pMosaicResult = NULL;

	MosaicParams mosaicParams;
	mosaicParams.blendingType = 0;
	mosaicParams.imagesNumEmergency = 2;
	mosaicParams.maxFeatruesNum = 400;
	mosaicParams.mergeImagesNum = 2;
	mosaicParams.minScore = 0.7f;
	mosaicParams.ransacDist = 2.0f;
	mosaicParams.referenceType = 0;
	mosaicParams.searchR = -1;
	mosaicParams.winR = 14;
	mosaic.SetParams(mosaicParams);

	m_pImgPoses = NULL;
	m_nImages = 0;

	m_nMatchThread = 1;

	m_scale = 1;
}

CMosaicByPose::~CMosaicByPose()
{
	cvReleaseImage(&m_pMosaicResult);
}

// 多航带图像特征点对应
int CMosaicByPose::GetMatchedPairsMultiStripSurf(const ImagePoseInfo *pImgPoses, const int nImages, 
							  vector<MatchPointPairs> &vecMatchPairs, int &nFixedImages)
{
	vector<IntPoint> vecMatched;
	const int MIN_INNER_POINTS = 11;
	float minScore = 0.7f;
	int maxFeatruesNum = 1000;
	int winR = 14;
	float ransacDist = 2;
	int searchR = -1;
	float distT = 0.5f;
	int minHessian = 800;
	SurfFeatureDetector detector( minHessian );
	SurfDescriptorExtractor extractor;
	for(int i=0; i<nImages; i++)
	{
		// 特征提取1
		std::vector<KeyPoint> keyPoints1;
		detector.detect( pImgPoses[i].pImg, keyPoints1 );
		// 特征描述1
		Mat descriptors1;
		extractor.compute(pImgPoses[i].pImg, keyPoints1, descriptors1);
		for(int j=i+1; j<nImages; j++)
		{
			if( (i==10)&&(j==13) )
			{
				i = i;
			}
			// 计算每幅图像的物理宽度
			double hCam = pImgPoses[i].camPose.pos.z;
			double wPhy = pImgPoses[i].pImg->width* hCam/ m_inParams.fx;
			double hPhy = pImgPoses[i].pImg->height* hCam/ m_inParams.fx;
			double imgWPhy = max(wPhy, hPhy); // 图像的物理宽度

			// 计算当前图像的主点到上一张图像的主点之间的距离
			double distOfTwoImages;
			DistanceOfTwoPoints(vecRectified[j].imgCxPys, vecRectified[j].imgCyPys, 
				vecRectified[i].imgCxPys, vecRectified[i].imgCyPys, distOfTwoImages);
			if(distOfTwoImages>1.5*imgWPhy)
			{
				continue;
			}

			// 特征提取2
			std::vector<KeyPoint> keyPoints2;
			Mat descriptors2;
			detector.detect( pImgPoses[j].pImg, keyPoints2 );
			// 特征描述2
			extractor.compute(pImgPoses[j].pImg, keyPoints2, descriptors2);
			
			// 特征匹配
			FlannBasedMatcher matcher;
			std::vector< DMatch > matches;
			matcher.match(descriptors1, descriptors2, matches);

			vector<SfPoint> vecMatch1, vecMatch2;
			for(int n=0; n<(int)matches.size(); n++)
			{
				if(matches[n].distance<distT)
				{
					SfPoint temp1;
					temp1.x = keyPoints1[matches[n].queryIdx].pt.x;
					temp1.y = keyPoints1[matches[n].queryIdx].pt.y;
					vecMatch1.push_back(temp1);
					SfPoint temp2;
					temp2.x = keyPoints2[matches[n].trainIdx].pt.x;
					temp2.y = keyPoints2[matches[n].trainIdx].pt.y;
					vecMatch2.push_back(temp2);
				}
			}

			BitmapImage* pSrc1 = new BitmapImage();
			BitmapImage* pSrc2 = new BitmapImage();
			CvImage2BitmapImage(pImgPoses[i].pImg, pSrc1);
			CvImage2BitmapImage(pImgPoses[j].pImg, pSrc2);

#ifdef _DEBUG
			ShowMatch(pSrc1, pSrc2, &vecMatch1[0], &vecMatch2[0], vecMatch1.size(), _T("c:\\matchSurf.bmp")); 
#endif
			cout<<"matched surf "<<vecMatch1.size()<<endl;
			
			float homo[9];
			CMosaicHarris mosaicHarris;
			mosaicHarris.Ransac(pSrc1, pSrc2, vecMatch1, vecMatch2, homo, ransacDist);
				
			delete pSrc1; pSrc1 = NULL;
			delete pSrc2; pSrc2 = NULL;

			//Mat img_matches;
			//drawMatches( pImgPoses[i].pImg, keyPoints1, pImgPoses[j].pImg, keyPoints2, matches, img_matches );
			////-- Show detected matches
			//imshow("Matches", img_matches );
			//waitKey(0);
			int nInnerPoints = (int)mosaicHarris.m_vecInnerPoints1.size();
			if(nInnerPoints>MIN_INNER_POINTS)
			{
				// 匹配成功
				for(int p=0; p<nInnerPoints; p++)
				{
					MatchPointPairs temp;
					temp.ptA = mosaicHarris.m_vecInnerPoints1[p];
					temp.ptA_Fixed = pImgPoses[i].fixed;
					temp.ptA_i = i;

					temp.ptB = mosaicHarris.m_vecInnerPoints2[p];
					temp.ptB_Fixed = pImgPoses[j].fixed;
					temp.ptB_i = j;
					vecMatchPairs.push_back(temp);
				}
				cout<<"i "<<i<<" j "<<j<<endl;
				IntPoint temp;
				temp.x = i;
				temp.y = j;
				vecMatched.push_back(temp);
			}
		}
	}
	for(int i=0; i<(int)vecMatched.size(); i++)
	{
		cout<<vecMatched[i].x<<" "<<vecMatched[i].y<<endl;
	}
	return 0;
}

int ClusterMatchNode(const vector<MatchEdge> &matchEdge, vector<int> &maxCluster, int nImages)
{
	int nMatch = matchEdge.size();

	// 初始化clusters
	vector<vector<int>> clusters(nMatch);
	for(int i=0; i<nMatch; i++)
	{
		clusters[i].push_back(matchEdge[i].idxImg1);
		clusters[i].push_back(matchEdge[i].idxImg2);
	}

	// 合并clusters
	bool clusterOver = true;
	do
	{
		clusterOver = true;
		for(int i=0; i<clusters.size(); i++)
		{
			for(int j=i+1; j<clusters.size();)
			{
				// 比较两个cluster,是否可以合并
				bool fOk= false;
				for(int p=0; p<clusters[i].size(); p++)
				{
					for(int q=0; q<clusters[j].size(); q++)
					{
						if(clusters[i][p]==clusters[j][q])
						{
							fOk = true;
							clusterOver = false;
							p = clusters[i].size();
							q = clusters[j].size();
						}
					}
				}
				if(fOk)
				{
					// 合并
					for(int q=0; q<clusters[j].size(); q++)
					{
						clusters[i].push_back(clusters[j][q]);
					}
					clusters[j] = clusters[clusters.size()-1];
					clusters.pop_back();
				}
				else
					j++;
			}
		}
	}
	while (!clusterOver);

	int iMax = 0, maxVal = 0;
	for(int i=0; i<clusters.size(); i++)
	{
		int label[10000] = {0};
		for(int j=0; j<clusters[i].size(); j++)
		{
			label[clusters[i][j]] = 1;
		}
		int nConnected = 0;
		for(int j=0; j<nImages; j++)
		{
			if(label[j]>0)
			{
				nConnected++;
			}
		}
		if(nConnected>maxVal)
		{
			maxVal = nConnected;
			iMax = i;
		}
	}

	maxCluster = clusters[iMax];

	return 0;
}

int Select_Connected_Matched_Images(const vector<MatchPointPairs> &vecMatchPairs, int nImages, int label[10000])
{	
	IplImage* pNode = cvCreateImage(cvSize(nImages, nImages), 8, 1);
	int ws8 = pNode->widthStep;
	cvZero(pNode);

	for(int i=0; i<vecMatchPairs.size(); i++)
	{
		int r = min(vecMatchPairs[i].ptA_i, vecMatchPairs[i].ptB_i);
		int c = max(vecMatchPairs[i].ptA_i, vecMatchPairs[i].ptB_i);
		*(pNode->imageData + r*ws8 + c) = 255;
	}

	cvSaveImage("c:/pNodeImage.png", pNode);

	vector<MatchEdge> vecMatchEdges;
	for(int r=0; r<nImages; r++)
	{
		label[r] = 0;
		for(int c=0; c<nImages; c++)
		{
			if( *((unsigned char*)pNode->imageData + r*ws8 + c)>0 )
			{
				MatchEdge temp;
				temp.idxImg1 = r;
				temp.idxImg2 = c;
				vecMatchEdges.push_back(temp);
			}
		}
	}

	vector<int> maxCluster;
	ClusterMatchNode(vecMatchEdges, maxCluster, nImages);

	for(int i=0; i<maxCluster.size(); i++)
	{
		label[maxCluster[i]] = 1;
	}
	
	cvReleaseImage(&pNode);
	
	return 0;
}

int OutTransform(const vector<ImageTransform> &transform, char* pName)
{
	int N = transform.size();

	ofstream outData(pName, ios::trunc);

	for(int i=1; i<N; i++)
	{
		for(int j=0; j<8; j++)
		{
			outData<<transform[i].h.m[j]<<" ";
		}
		outData<<transform[i].fixed;

		outData<<endl;
	}

	outData.close();

	return 0;
}

int ImportTransform(vector<ImageTransform> &transform, char* pName)
{
		fstream cinData;
		cinData.open(pName);
		int nData;
		cinData>>nData;
		for(int i=0; i<nData; i++)
		{
			ImageTransform temp;
			for(int j=0; j<9; j++)
			{
				cinData>>temp.h.m[j];
			}
			if(i==0)
				temp.fixed = 1;
			else
				temp.fixed = 0;
			transform.push_back(temp);
		}

	cinData.close();

	return 0;
}

// 确定每对特征点对应的世界坐标系中点的编号 (增量式)
// wpNum------------世界坐标稀疏点的个数
// newFeatureNum----新加入特征的个数
int DetermineWorldPointsV2(MatchPointPairs2* pMatchPairs, int nPairs,
						 int label[10000], int &wpNum,
						 int newFeatureNum)
{
	byte *flag = new byte[(nPairs+3)/4*4];
	memset(flag, 0, nPairs);

	//int wpIdx = 0;

	int numAdded = 0;

	bool over = false;
	while(over==false)
	{
		bool found = false;

		for(int j=nPairs-newFeatureNum; j<nPairs; j++)
		{				
			if(flag[j]>0)
				continue;
			int idx = j;
			for(int i=0; i<nPairs; i++)
			{
				if(i==idx)
					continue;
				if( ((pMatchPairs[idx].ptA.id==pMatchPairs[i].ptA.id) && (pMatchPairs[idx].ptA_i==pMatchPairs[i].ptA_i)) || 
					((pMatchPairs[idx].ptA.id==pMatchPairs[i].ptB.id) && (pMatchPairs[idx].ptA_i==pMatchPairs[i].ptB_i)) ||
					((pMatchPairs[idx].ptB.id==pMatchPairs[i].ptA.id) && (pMatchPairs[idx].ptB_i==pMatchPairs[i].ptA_i)) ||
					((pMatchPairs[idx].ptB.id==pMatchPairs[i].ptB.id) && (pMatchPairs[idx].ptB_i==pMatchPairs[i].ptB_i)) )
				{
					if(pMatchPairs[i].wpIdx>=0)
					{
						flag[j] = 1;
						pMatchPairs[j].wpIdx = pMatchPairs[i].wpIdx;
						numAdded++;
						
						i = nPairs;
						j = nPairs;

						found = true;
					}
				}
			}
		}

		if(found==false)
		{
			for(int j=nPairs-newFeatureNum; j<nPairs; j++)
			{
				if(flag[j]==0)
				{
					pMatchPairs[j].wpIdx = wpNum;
					flag[j] = 1;
					wpNum++;
					numAdded++;
					break;
				}
			}
		}

		if(numAdded>=newFeatureNum)
		{
			over = true;
		}
	}

	delete[] flag;

	return 0;
}

// 确定每对特征点对应的世界坐标系中点的编号
// wpNum------------世界坐标稀疏点的个数
// newFeatureNum----新加入特征的个数
int DetermineWorldPoints(MatchPointPairs2* pMatchPairs, int nPairs,
						 int label[10000], int &wpNum,
						 int newFeatureNum = 0)
{
	byte *flag = new byte[(nPairs+3)/4*4];
	memset(flag, 0, nPairs);

	//vector<vector<int>> g_cluster;

	int wpIdx = 0;

	for(int j=0; j<nPairs; j++)
	{
		vector<int> cluster;
		if(flag[j]==0)
		{
			cluster.push_back(j);
			flag[j] = 1; // 标记为已经确定
			pMatchPairs[j].wpIdx = wpIdx;
		}
		else
			continue;

		//bool over = false;
		for(int n=0; n<cluster.size(); n++)
		{
			int idx = cluster[n];
			
			for(int i=0; i<nPairs; i++)
			{
				if(flag[i]>0)
					continue;

				if( ((pMatchPairs[idx].ptA.id==pMatchPairs[i].ptA.id) && (pMatchPairs[idx].ptA_i==pMatchPairs[i].ptA_i)) || 
					((pMatchPairs[idx].ptA.id==pMatchPairs[i].ptB.id) && (pMatchPairs[idx].ptA_i==pMatchPairs[i].ptB_i)) ||
					((pMatchPairs[idx].ptB.id==pMatchPairs[i].ptA.id) && (pMatchPairs[idx].ptB_i==pMatchPairs[i].ptA_i)) ||
					((pMatchPairs[idx].ptB.id==pMatchPairs[i].ptB.id) && (pMatchPairs[idx].ptB_i==pMatchPairs[i].ptB_i)) )
				{
					cluster.push_back(i);
					flag[i] = 1;
					pMatchPairs[i].wpIdx = wpIdx;
				}
			}
		}

		wpIdx++;
	}

	wpNum = wpIdx;

	delete[] flag;

	return 0;
}

double CalCostBunder2D(const vector<ReconstructedPoint> &wPoints2, 
					   const vector<ReconstructedPoint> &wPointsFloat, 
					   const vector<ReconstructedPoint> &wPointsFixed, 
					   CvMat* pX,
					   int transformIdx[10000],
					   int refImIdx,
					   int nTransformation)
{
	int nFloatPoints = wPointsFloat.size();

	// 计算待优化的图像个数
	//int  = 0;
	//for(int i=0; i<wPoints.size(); i++)
	//{
	//	transformIdx[i] = nTransformation;
	//	if( (label[i]>0) && (i!=refImIdx) )
	//	{
	//		nTransformation++;
	//	}
	//}

	int offset = 8 * nTransformation;

	double costCurrent = 0;

	int iPair = 0;
	for(int n=0; n<wPoints2.size(); n++)
	{
		float xW, yW;
		if(n<nFloatPoints)
		{
			xW = cvGetReal2D(pX, offset + 2*n+0, 0);
			yW = cvGetReal2D(pX, offset + 2*n+1, 0);
		}
		else
		{
			xW = wPointsFixed[n-nFloatPoints].x;
			yW = wPointsFixed[n-nFloatPoints].y;
		}

		for(int i=0; i<wPoints2[n].imgPt.size(); i++)
		{
			int imIdx = wPoints2[n].imgPt[i].imIdx; // 如果等于refImIdx?
			float xi = wPoints2[n].imgPt[i].x;
			float yi = wPoints2[n].imgPt[i].y;

			int imIdxTran = transformIdx[imIdx];

			double hA[9] = {1, 0, 0, 
				0, 1, 0, 
				0, 0, 1};
			if(imIdx!=refImIdx)
			{
				for(int i=0; i<8; i++)
				{
					hA[i] = cvGetReal2D(pX, 8*imIdxTran+i, 0);
				}
				hA[8] = 1;
			}

			double h0x_h1y_h2 = hA[0]*xW + hA[1]*yW + hA[2];
			double h3x_h4y_h5 = hA[3]*xW + hA[4]*yW + hA[5];
			double h6x_h7y_1  = hA[6]*xW + hA[7]*yW + 1;
			double h6x_h7y_1_sq = h6x_h7y_1*h6x_h7y_1;

			//-----------------------------------------------------

			// 给B赋值
			float errX = h0x_h1y_h2/h6x_h7y_1 - xi;
			float errY = h3x_h4y_h5/h6x_h7y_1 - yi;

			costCurrent += errX*errX + errY*errY;

			iPair++;
		}
	}

	return costCurrent;
}

double CalCostBunder2D_2(const vector<ReconstructedPoint> &wPoints, 
					   CvMat* pX,
					   int transformIdx[10000],
					   int refImIdx,
					   int nTransformation)
{
	//int nFloatPoints = wPointsFloat.size();

	// 计算待优化的图像个数
	//int  = 0;
	//for(int i=0; i<wPoints.size(); i++)
	//{
	//	transformIdx[i] = nTransformation;
	//	if( (label[i]>0) && (i!=refImIdx) )
	//	{
	//		nTransformation++;
	//	}
	//}

	int offset = 8 * nTransformation;

	double costCurrent = 0;

	int iPair = 0;
	for(int n=0; n<wPoints.size(); n++)
	{
		float xW, yW;
		//if(n<nFloatPoints)
		{
			xW = cvGetReal2D(pX, offset + 2*n+0, 0);
			yW = cvGetReal2D(pX, offset + 2*n+1, 0);
		}
		//else
		//{
		//	xW = wPointsFixed[n-nFloatPoints].x;
		//	yW = wPointsFixed[n-nFloatPoints].y;
		//}

		for(int i=0; i<wPoints[n].imgPt.size(); i++)
		{
			int imIdx = wPoints[n].imgPt[i].imIdx; // 如果等于refImIdx?
			float xi = wPoints[n].imgPt[i].x;
			float yi = wPoints[n].imgPt[i].y;

			int imIdxTran = transformIdx[imIdx];

			double hA[9] = {1, 0, 0, 
				0, 1, 0, 
				0, 0, 1};
			if(imIdx!=refImIdx)
			{
				for(int i=0; i<8; i++)
				{
					hA[i] = cvGetReal2D(pX, 8*imIdxTran+i, 0);
				}
				hA[8] = 1;
			}

			double h0x_h1y_h2 = hA[0]*xW + hA[1]*yW + hA[2];
			double h3x_h4y_h5 = hA[3]*xW + hA[4]*yW + hA[5];
			double h6x_h7y_1  = hA[6]*xW + hA[7]*yW + 1;
			//double h6x_h7y_1_sq = h6x_h7y_1*h6x_h7y_1;

			//-----------------------------------------------------

			// 给B赋值
			//float errX = h0x_h1y_h2 - xi*(h6x_h7y_1);
			//float errY = h3x_h4y_h5 - yi*h6x_h7y_1;

			float errX = h0x_h1y_h2/h6x_h7y_1 - xi;
			float errY = h3x_h4y_h5/h6x_h7y_1 - yi;

			costCurrent += errX*errX + errY*errY;

			//float errX2 = h0x_h1y_h2/h6x_h7y_1 - xi;
			//float errY2 = h3x_h4y_h5/h6x_h7y_1 - yi;

			//costCurrent2 += errX2*errX2 + errY2*errY2;

			iPair++;
		}
	}

	costCurrent = sqrt(costCurrent/iPair);

	return costCurrent;
}

// 功能: 用2D捆绑调整做图像拼接
// 这里输入的变换参数Hi, worldPoints = Hi*imagePoints
// 但优化的Ho, imagePoints = Hi*worldPoints, So, Ho = inv(Hi)
// refImIdx---参考图像标号
int MosaicByBundler2D_V2(vector<ReconstructedPoint> &wPoints, 
					  vector<ImageTransform> &vecTransformRefined, 
					  int refImIdx,
					  int label[10000],
					  int nImages)
{
	cout<<"----MosaicByBundler2D_V2----"<<endl<<endl;

	//vector<vector<int>> graph(nImages); // 每个节点都与哪些节点直接相连
	//for(int n=0; n<nImages; n++)
	//{
	//	for(int i=0; i<wPoints.size(); i++)
	//	{
	//		for(int j=0; j<wPoints[i].imgPt.size(); j++)
	//		{
	//			
	//		}
	//	}
	//}

	// 求逆vecTransformRefined
	for(int n=0; n<nImages; n++)
	{
		if( (label[n]>0) && (n!=refImIdx) )
		{
			float hA[9];
			memcpy(hA, vecTransformRefined[n].h.m, sizeof(hA));

			float hInv[9];
			InverseMatrix(hA, 3, hInv, 1e-12f);

			for(int i=0; i<9; i++)
			{
				hInv[i] /= hInv[8];
			}

			memcpy(vecTransformRefined[n].h.m, hInv, sizeof(hInv));
		}
	}

	vector<ImageTransform> vecTransform0 = vecTransformRefined;
	//vector<ReconstructedPoint> wPointsFixed;

	//vector<ReconstructedPoint> wPointsFloat;

	//vector<ReconstructedPoint> wPoints2;

	int transformIdx[10000] = {0}; // 图像编号对应的变换参数编号

	// 计算待优化的世界点的个数
	int nPairs = 0;
	for(int i=0; i<wPoints.size(); i++)
	{
		//if(wPoints[i].fixed==1) // 固定的点
		//{
		//	wPointsFixed.push_back(wPoints[i]);
		//}
		//else
		//{
		//	wPointsFloat.push_back(wPoints[i]);
		//}
		for(int j=0; j<wPoints[i].imgPt.size(); j++)
		{
			nPairs++;
		}
		//nPairs += wPoints[i].imgPt.size();
	}

	//int nFloatPoints = wPointsFloat.size();
	//int nFixedPoints = wPointsFixed.size();

	//wPoints2 = wPointsFloat; // 浮动的点放在前面
	//wPoints2.insert(wPoints2.end(), wPointsFixed.begin(), wPointsFixed.end());

	// 计算待优化的图像个数
	int nTransformation = 0;
	for(int i=0; i<nImages; i++)
	{

		if( (label[i]>0) && (i!=refImIdx) )
		{
			transformIdx[i] = nTransformation;
			nTransformation++;
		}
		else
		{
		}
	}

	int nwPoints = wPoints.size();

	cout<<"nTransformation "<<nTransformation<<endl;

	cout<<"nPairs "<<nPairs<<endl;

	int rowA = 2*nPairs;
	int colA = 8 * nTransformation + 2*nwPoints; // 变量的个数
	int rowX = colA;
	int colX = 1;
	int rowB = rowA;
	int colB = 1;

	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pX2 = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pDX = cvCreateMat(rowX, colX, CV_64F);

	cvZero(pDX);

	// 初始化X
	int nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(vecTransform0[i].fixed!=1) // 未固定参数情况(包括0 和 2)
		{
			for(int j=0; j<8; j++)
			{
				cvSetReal2D(pX, 8*nUnFixed+j, 0, vecTransform0[i].h.m[j]);
			}
			nUnFixed++;
		}
	}

	cout<<"rowA "<<rowA<<" colA "<<colA<<endl;

	cout<<"nUnFixed "<<nUnFixed<<endl;

	int offset = 8 * nTransformation;

	// 初始化世界坐标系点
	for(int i=0; i<nwPoints; i++)
	{
		cvSetReal2D(pX, 2*i+0+offset, 0, wPoints[i].x);
		cvSetReal2D(pX, 2*i+1+offset, 0, wPoints[i].y);
	}

	pool::SparseMatrix eye;
	eye.mat.resize(colA);
	eye.col = colA;
	eye.row = colA;
	for(int i=0; i<colA; i++)
	{
		eye.mat[i].matCol.push_back(SparseMatElem(i, 1)); // 列 行
	}

	double beta = 10;
	double lambda64F = 0.00001;
	vector<double> vecCost, vecCost2, vecCostRef;
	int MAX_ITER = 50;
	// 迭代
	for(int iter = 0; iter<MAX_ITER; iter++)
	{
		//float *A2 = new float[rowA*colA];
		//memset(A2, 0, sizeof(float)*rowA*colA);

		//float *B2 = new float[rowB*colB];

		cout<<"iter "<<iter<<endl;
		double costCurrent = 0, costCurrent2 = 0;
		pool::SparseMatrix A;
		pool::SparseMatrix B;
		A.mat.resize(colA);
		B.mat.resize(colB);
		A.col = colA;
		A.row = rowA;
		B.col = colB;
		B.row = rowB;

		//double wCorr = ;

		int iPair = 0;
		for(int n=0; n<wPoints.size(); n++)
		{
			float xW, yW;

			xW = cvGetReal2D(pX, offset + 2*n+0, 0); // 物点
			yW = cvGetReal2D(pX, offset + 2*n+1, 0);

			for(int i=0; i<wPoints[n].imgPt.size(); i++)
			{
				int imIdx = wPoints[n].imgPt[i].imIdx; // 如果等于refImIdx?
				float xi = wPoints[n].imgPt[i].x; // 像点
				float yi = wPoints[n].imgPt[i].y;

				int imIdxTran = transformIdx[imIdx];

				double hA[9] = {1, 0, 0, 
								0, 1, 0, 
								0, 0, 1};
				if(imIdx!=refImIdx)
				{
					for(int j=0; j<8; j++)
					{
						hA[j] = cvGetReal2D(pX, 8*imIdxTran+j, 0);
					}
					hA[8] = 1;
				}
				else
				{
					iter = iter; // 如果是参考图像上的点
					//continue;
				}

				double h0x_h1y_h2 = hA[0]*xW + hA[1]*yW + hA[2];
				double h3x_h4y_h5 = hA[3]*xW + hA[4]*yW + hA[5];
				double h6x_h7y_1  = hA[6]*xW + hA[7]*yW + 1;
				double h6x_h7y_1_sq = h6x_h7y_1*h6x_h7y_1;

				if(imIdx!=refImIdx) // 不在参考图像, 用来约束图像参数
				{
					A.mat[8*imIdxTran + 0].matCol.push_back(SparseMatElem(2*iPair+0, xW/h6x_h7y_1)); // 列 行

					A.mat[8*imIdxTran + 1].matCol.push_back(SparseMatElem(2*iPair+0, yW/h6x_h7y_1)); // 列 行

					A.mat[8*imIdxTran + 2].matCol.push_back(SparseMatElem(2*iPair+0, 1/h6x_h7y_1)); // 列 行

					//A.mat[8*imIdxTran + 3].matCol.push_back(SparseMatElem(2*iPair+0, 0)); // 列 行

					//A.mat[8*imIdxTran + 4].matCol.push_back(SparseMatElem(2*iPair+0, 0)); // 列 行

					//A.mat[8*imIdxTran + 5].matCol.push_back(SparseMatElem(2*iPair+0, 0)); // 列 行

					A.mat[8*imIdxTran + 6].matCol.push_back(SparseMatElem(2*iPair+0, -h0x_h1y_h2*xW/h6x_h7y_1_sq)); // 列 行

					A.mat[8*imIdxTran + 7].matCol.push_back(SparseMatElem(2*iPair+0, -h0x_h1y_h2*yW/h6x_h7y_1_sq)); // 列 行


					A.mat[offset + 2*n + 0].matCol.push_back(SparseMatElem(2*iPair+0, (hA[0]*h6x_h7y_1-hA[6]*h0x_h1y_h2)/h6x_h7y_1_sq)); // 列 行

					A.mat[offset + 2*n + 1].matCol.push_back(SparseMatElem(2*iPair+0, (hA[1]*h6x_h7y_1-hA[7]*h0x_h1y_h2)/h6x_h7y_1_sq)); // 列 行
					
					// --------------------------------------------------
					//A.mat[8*imIdxTran + 0].matCol.push_back(SparseMatElem(2*iPair+1, 0)); // 列 行

					//A.mat[8*imIdxTran + 1].matCol.push_back(SparseMatElem(2*iPair+1, 0)); // 列 行

					//A.mat[8*imIdxTran + 2].matCol.push_back(SparseMatElem(2*iPair+1, 0)); // 列 行

					A.mat[8*imIdxTran + 3].matCol.push_back(SparseMatElem(2*iPair+1, xW/h6x_h7y_1)); // 列 行

					A.mat[8*imIdxTran + 4].matCol.push_back(SparseMatElem(2*iPair+1, yW/h6x_h7y_1)); // 列 行
	
					A.mat[8*imIdxTran + 5].matCol.push_back(SparseMatElem(2*iPair+1, 1/h6x_h7y_1)); // 列 行

					A.mat[8*imIdxTran + 6].matCol.push_back(SparseMatElem(2*iPair+1, -h3x_h4y_h5*xW/h6x_h7y_1_sq)); // 列 行

					A.mat[8*imIdxTran + 7].matCol.push_back(SparseMatElem(2*iPair+1, -h3x_h4y_h5*yW/h6x_h7y_1_sq)); // 列 行



					A.mat[offset + 2*n + 0].matCol.push_back(SparseMatElem(2*iPair+1, (hA[3]*h6x_h7y_1-hA[6]*h3x_h4y_h5) / h6x_h7y_1_sq)); // 列 行
						//A2[(offset + 2*n + 0) + (2*iPair+1)*colA] = (hA[3]*h6x_h7y_1-hA[6]*h3x_h4y_h5) / h6x_h7y_1_sq;
					A.mat[offset + 2*n + 1].matCol.push_back(SparseMatElem(2*iPair+1, (hA[4]*h6x_h7y_1-hA[7]*h3x_h4y_h5) / h6x_h7y_1_sq)); // 列 行
						//A2[(offset + 2*n + 1) + (2*iPair+1)*colA] = (hA[4]*h6x_h7y_1-hA[7]*h3x_h4y_h5) / h6x_h7y_1_sq;
				
					//-----------------------------------------------------

				}
				else // 在参考图上
				{
					A.mat[offset + 2*n + 0].matCol.push_back(SparseMatElem(2*iPair+0, 1)); // 列 行

					A.mat[offset + 2*n + 1].matCol.push_back(SparseMatElem(2*iPair+1, 1)); // 列 行
					
				}
				// --------------------------------------------------

				//-----------------------------------------------------

				// 给B赋值
				float errX = h0x_h1y_h2/h6x_h7y_1 - xi;
				float errY = h3x_h4y_h5/h6x_h7y_1 - yi;

				B.mat[0].matCol.push_back(SparseMatElem(2*iPair,   errX)); // 列 行
				//B2[2*iPair] = errX;
				B.mat[0].matCol.push_back(SparseMatElem(2*iPair+1, errY)); // 列 行
				//B2[2*iPair+1] = errY;

				costCurrent += errX*errX + errY*errY;

				//costCurrent2 += errX2*errX2 + errY2*errY2;

				iPair++;
			}
		}

		//COutData::OutData2Txt(A2, colA, rowA, colA, "c:/A.txt");
		//COutData::OutData2Txt(B2, colB, rowB, colB, "c:/B.txt");

		//delete[] A2;
		//delete[] B2;

		//cout<<"iPair "<<iPair<<endl;

		//cout<<"mse "<<sqrt(costCurrent/iPair)<<endl;

		costCurrent = sqrt(costCurrent/iPair);

		//cout<<"mse2 "<<sqrt(costCurrent2/iPair);

		for(int col=0; col<A.col; col++)
		{
			std::sort(A.mat[col].matCol.begin(), A.mat[col].matCol.end());
		}
		for(int col=0; col<eye.col; col++)
		{
			std::sort(eye.mat[col].matCol.begin(), eye.mat[col].matCol.end());
		}

		int nSingleIter = 0;
		int maxIterSingle = 20;
		bool convergence = true;
		do
		{
			if(SolveSparseSystemLM(A, B, eye, pDX, lambda64F)<0)
			//if(SolveSparseSystem(A, B, pDX)<0)
			{
				cout<<"SolveSparseSystemLM failed"<<endl;
				break;
			}
			nSingleIter++;

			//vector<float> Xm;
			//int col, row;
			//ReadTxtData32F("c://X.txt", Xm, row, col);
			//for(int i=0; i<Xm.size(); i++)
			//{
			//	cvSetReal2D(pDX,i, 0, Xm[i]);
			//}


			int nonzereos = 0;
			for(int i=0; i<rowX; i++)
			{
				float val = cvGetReal2D(pDX, i, 0);

				if(abs(val)>0)
				{
					nonzereos++;
					//cout<<val<<" ";
				}
			}
			cout<<endl;
			//cout<<"nonzereos "<<nonzereos<<endl;
			if(nonzereos==0)
			{
				break;
			}

			// 更新pX
			cvConvertScale(pDX, pDX, -1);
			cvAdd(pX, pDX, pX2);

			double costNew = CalCostBunder2D_2(wPoints, 
				pX2,
				transformIdx, refImIdx, nTransformation);

			//cout<<"costNew "<<costNew<<" totalCost "<<costCurrent<<" lambda64F "<<lambda64F<<endl;

			if(costNew<costCurrent)
			{
				cvCopy(pX2, pX);
				lambda64F /= beta;
				convergence = true;
				costCurrent = costNew;
				vecCost.push_back(costCurrent);
				//vecCost2[iter] = rotCost;
			}
			else
			{
				lambda64F *= beta;
				convergence = false;
			}
			if(nSingleIter>maxIterSingle)
			{
				break;
			}
		} while(convergence==false);

		if(convergence==false)
			break;


		//cout<<"iter "<<iter<<endl;
		if(vecCost.size()>1)
		{
			if(vecCost[vecCost.size()-1]>=vecCost[vecCost.size()-2])
			{
				break;
			}
			if( abs(vecCost[vecCost.size()-1]-vecCost[vecCost.size()-2])<1e-2f)
			{
				break;
			}
		}
	}

	cout<<"Total Cost"<<endl;
	for(int i=0; i<vecCost.size(); i++)
	{
		cout<<vecCost[i]<<" ";
	}
	cout<<endl;

	// 整理计算结果
	for(int n=0; n<nImages; n++)
	{
		if( (label[n]>0) && (n!=refImIdx) )
		{
			int imIdxTran = transformIdx[n];
			float hA[9];
			for(int i=0; i<8; i++)
			{
				hA[i] = cvGetReal2D(pX, 8*imIdxTran+i, 0);
			}
			hA[8] = 1;

			float hInv[9];
			InverseMatrix(hA, 3, hInv, 1e-12f);

			for(int i=0; i<9; i++)
			{
				hInv[i] /= hInv[8];
			}

			memcpy(vecTransformRefined[n].h.m, hInv, sizeof(hInv));
		}
	}

	return 0;
}



// 功能: 用2D捆绑调整做图像拼接
// 这里输入的变换参数Hi, worldPoints = Hi*imagePoints
// 但优化的Ho, imagePoints = Hi*worldPoints, So, Ho = inv(Hi)
// refImIdx---参考图像标号
int MosaicByBundler2D(vector<ReconstructedPoint> &wPoints, 
					   vector<ImageTransform> &vecTransformRefined, 
					   int refImIdx,
					   int label[10000],
					   int nImages)
{
	cout<<"MosaicByBundler2D"<<endl;

	//vector<vector<int>> graph(nImages); // 每个节点都与哪些节点直接相连
	//for(int n=0; n<nImages; n++)
	//{
	//	for(int i=0; i<wPoints.size(); i++)
	//	{
	//		for(int j=0; j<wPoints[i].imgPt.size(); j++)
	//		{
	//			
	//		}
	//	}
	//}
	
	// 求逆vecTransformRefined
	for(int n=0; n<nImages; n++)
	{
		if( (label[n]>0) && (n!=refImIdx) )
		{
			float hA[9];
			memcpy(hA, vecTransformRefined[n].h.m, sizeof(hA));

			float hInv[9];
			InverseMatrix(hA, 3, hInv, 1e-12f);

			for(int i=0; i<9; i++)
			{
				hInv[i] /= hInv[8];
			}

			memcpy(vecTransformRefined[n].h.m, hInv, sizeof(hInv));
		}
	}

	vector<ImageTransform> vecTransform0 = vecTransformRefined;
	vector<ReconstructedPoint> wPointsFixed;

	vector<ReconstructedPoint> wPointsFloat;

	vector<ReconstructedPoint> wPoints2;

	int transformIdx[10000] = {0}; // 图像编号对应的变换参数编号

	// 计算待优化的世界点的个数
	int nPairs = 0;
	for(int i=0; i<wPoints.size(); i++)
	{
		if(wPoints[i].fixed==1) // 固定的点
		{
			wPointsFixed.push_back(wPoints[i]);
		}
		else
		{
			wPointsFloat.push_back(wPoints[i]);
		}
		nPairs += wPoints[i].imgPt.size();
	}

	int nFloatPoints = wPointsFloat.size();
	int nFixedPoints = wPointsFixed.size();

	wPoints2 = wPointsFloat; // 浮动的点放在前面
	wPoints2.insert(wPoints2.end(), wPointsFixed.begin(), wPointsFixed.end());

	// 计算待优化的图像个数
	int nTransformation = 0;
	for(int i=0; i<nImages; i++)
	{
		
		if( (label[i]>0) && (i!=refImIdx) )
		{
			transformIdx[i] = nTransformation;
			nTransformation++;
		}
		else
		{
		}
	}

	cout<<"nTransformation "<<nTransformation<<endl;

	cout<<"nPairs "<<nPairs<<endl;

	int rowA = 2*nPairs;
	int colA = 8 * nTransformation + 2*nFloatPoints; // 变量的个数
	int rowX = colA;
	int colX = 1;
	int rowB = rowA;
	int colB = 1;

	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pX2 = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pDX = cvCreateMat(rowX, colX, CV_64F);

	cvZero(pDX);

	// 初始化X
	int nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(vecTransform0[i].fixed!=1) // 未固定参数情况(包括0 和 2)
		{
			for(int j=0; j<8; j++)
			{
				cvSetReal2D(pX, 8*nUnFixed+j, 0, vecTransform0[i].h.m[j]);
			}
			nUnFixed++;
		}
	}

	cout<<"rowA "<<rowA<<" rowB "<<rowB<<endl;

	cout<<"nUnFixed "<<nUnFixed<<endl;

	int offset = 8 * nTransformation;

	// 初始化世界坐标系点
	for(int i=0; i<nFloatPoints; i++)
	{
		cvSetReal2D(pX, 2*i+0+offset, 0, wPointsFloat[i].x);
		cvSetReal2D(pX, 2*i+1+offset, 0, wPointsFloat[i].y);
	}

	pool::SparseMatrix eye;
	eye.mat.resize(colA);
	eye.col = colA;
	eye.row = colA;
	for(int i=0; i<colA; i++)
	{
		eye.mat[i].matCol.push_back(SparseMatElem(i, 1)); // 列 行
	}

	double beta = 10;
	double lambda64F = 0.00001;
	vector<double> vecCost, vecCost2, vecCostRef;
	int MAX_ITER = 10;
	// 迭代
	for(int iter = 0; iter<MAX_ITER; iter++)
	{
		float *A2 = new float[rowA*colA];
		memset(A2, 0, sizeof(float)*rowA*colA);

		cout<<"iter "<<iter<<endl;
		double costCurrent = 0;
		pool::SparseMatrix A;
		pool::SparseMatrix B;
		A.mat.resize(colA);
		B.mat.resize(colB);
		A.col = colA;
		A.row = rowA;
		B.col = colB;
		B.row = rowB;

		//double wCorr = ;
		
		int iPair = 0;
		for(int n=0; n<wPoints2.size(); n++)
		{
			float xW, yW;
			if(n<nFloatPoints)
			{
				xW = cvGetReal2D(pX, offset + 2*n+0, 0);
				yW = cvGetReal2D(pX, offset + 2*n+1, 0);
			}
			else
			{
				xW = wPointsFixed[n-nFloatPoints].x;
				yW = wPointsFixed[n-nFloatPoints].y;
			}

			for(int i=0; i<wPoints2[n].imgPt.size(); i++)
			{
				int imIdx = wPoints2[n].imgPt[i].imIdx; // 如果等于refImIdx?
				float xi = wPoints2[n].imgPt[i].x;
				float yi = wPoints2[n].imgPt[i].y;

				int imIdxTran = transformIdx[imIdx];

				double hA[9] = {1, 0, 0, 
								0, 1, 0, 
								0, 0, 1};
				if(imIdx!=refImIdx)
				{
					for(int i=0; i<8; i++)
					{
						hA[i] = cvGetReal2D(pX, 8*imIdxTran+i, 0);
					}
					hA[8] = 1;
				}

				double h0x_h1y_h2 = hA[0]*xW + hA[1]*yW + hA[2];
				double h3x_h4y_h5 = hA[3]*xW + hA[4]*yW + hA[5];
				double h6x_h7y_1  = hA[6]*xW + hA[7]*yW + 1;
				double h6x_h7y_1_sq = h6x_h7y_1*h6x_h7y_1;

				A.mat[8*imIdxTran + 0].matCol.push_back(SparseMatElem(2*iPair+0, xW/h6x_h7y_1)); // 列 行
				A2[(8*imIdxTran + 0) + (2*iPair+0)*colA] = xW/h6x_h7y_1;

				A.mat[8*imIdxTran + 1].matCol.push_back(SparseMatElem(2*iPair+0, yW/h6x_h7y_1)); // 列 行
				A2[(8*imIdxTran + 1) + (2*iPair+0)*colA] = yW/h6x_h7y_1;

				A.mat[8*imIdxTran + 2].matCol.push_back(SparseMatElem(2*iPair+0, 1/h6x_h7y_1)); // 列 行
				A2[(8*imIdxTran + 2) + (2*iPair+0)*colA] = 1/h6x_h7y_1;
				if(1/h6x_h7y_1<-60000)
				{
					iter = iter;
				}
				A.mat[8*imIdxTran + 3].matCol.push_back(SparseMatElem(2*iPair+0, 0)); // 列 行
				A2[(8*imIdxTran + 3) + (2*iPair+0)*colA] = 0;
				A.mat[8*imIdxTran + 4].matCol.push_back(SparseMatElem(2*iPair+0, 0)); // 列 行
				A2[(8*imIdxTran + 4) + (2*iPair+0)*colA] = 0;
				A.mat[8*imIdxTran + 5].matCol.push_back(SparseMatElem(2*iPair+0, 0)); // 列 行
				A2[(8*imIdxTran + 5) + (2*iPair+0)*colA] = 0;
				A.mat[8*imIdxTran + 6].matCol.push_back(SparseMatElem(2*iPair+0, -h0x_h1y_h2*xW/h6x_h7y_1_sq)); // 列 行
				A2[(8*imIdxTran + 6) + (2*iPair+0)*colA] = -h0x_h1y_h2*xW/h6x_h7y_1_sq;
				if(A2[(8*imIdxTran + 6) + (2*iPair+0)*colA]<-600000)
				{
					iter = iter;
				}
				A.mat[8*imIdxTran + 7].matCol.push_back(SparseMatElem(2*iPair+0, -h0x_h1y_h2*yW/h6x_h7y_1_sq)); // 列 行
				A2[(8*imIdxTran + 7) + (2*iPair+0)*colA] = -h0x_h1y_h2*yW/h6x_h7y_1_sq;
				if(A2[(8*imIdxTran + 7) + (2*iPair+0)*colA]<-600000)
				{
					iter = iter;
				}

				if(n<nFloatPoints)
				{
					A.mat[offset + 2*n + 0].matCol.push_back(SparseMatElem(2*iPair+0, (hA[0]*h6x_h7y_1-hA[6]*h0x_h1y_h2)/h6x_h7y_1_sq)); // 列 行
					A2[(offset + 2*n + 0) + (2*iPair+0)*colA] = (hA[0]*h6x_h7y_1-hA[6]*h0x_h1y_h2)/h6x_h7y_1_sq;
					A.mat[offset + 2*n + 1].matCol.push_back(SparseMatElem(2*iPair+0, (hA[1]*h6x_h7y_1-hA[7]*h0x_h1y_h2)/h6x_h7y_1_sq)); // 列 行
					A2[(offset + 2*n + 1) + (2*iPair+0)*colA] = (hA[1]*h6x_h7y_1-hA[7]*h0x_h1y_h2)/h6x_h7y_1_sq;
				}
				// --------------------------------------------------
				A.mat[8*imIdxTran + 0].matCol.push_back(SparseMatElem(2*iPair+1, 0)); // 列 行
				A2[(8*imIdxTran + 0) + (2*iPair+1)*colA] = 0;
				A.mat[8*imIdxTran + 1].matCol.push_back(SparseMatElem(2*iPair+1, 0)); // 列 行
				A2[(8*imIdxTran + 1) + (2*iPair+1)*colA] = 0;
				A.mat[8*imIdxTran + 2].matCol.push_back(SparseMatElem(2*iPair+1, 0)); // 列 行
				A2[(8*imIdxTran + 2) + (2*iPair+1)*colA] = 0;
				A.mat[8*imIdxTran + 3].matCol.push_back(SparseMatElem(2*iPair+1, xW/h6x_h7y_1)); // 列 行
				A2[(8*imIdxTran + 3) + (2*iPair+1)*colA] = xW/h6x_h7y_1;
				A.mat[8*imIdxTran + 4].matCol.push_back(SparseMatElem(2*iPair+1, yW/h6x_h7y_1)); // 列 行
				A2[(8*imIdxTran + 4) + (2*iPair+1)*colA] = yW/h6x_h7y_1;
				A.mat[8*imIdxTran + 5].matCol.push_back(SparseMatElem(2*iPair+1, 1/h6x_h7y_1)); // 列 行
				A2[(8*imIdxTran + 5) + (2*iPair+1)*colA] = 1/h6x_h7y_1;
				A.mat[8*imIdxTran + 6].matCol.push_back(SparseMatElem(2*iPair+1, -h3x_h4y_h5*xW/h6x_h7y_1_sq)); // 列 行
				A2[(8*imIdxTran + 6) + (2*iPair+1)*colA] = -h3x_h4y_h5*xW/h6x_h7y_1_sq;
				A.mat[8*imIdxTran + 7].matCol.push_back(SparseMatElem(2*iPair+1, -h3x_h4y_h5*yW/h6x_h7y_1_sq)); // 列 行
				A2[(8*imIdxTran + 7) + (2*iPair+1)*colA] = -h3x_h4y_h5*yW/h6x_h7y_1_sq;

				if(n<nFloatPoints)
				{
					A.mat[offset + 2*n + 0].matCol.push_back(SparseMatElem(2*iPair+1, (hA[3]*h6x_h7y_1-hA[6]*h3x_h4y_h5) / h6x_h7y_1_sq)); // 列 行
					A2[(offset + 2*n + 0) + (2*iPair+1)*colA] = (hA[3]*h6x_h7y_1-hA[6]*h3x_h4y_h5) / h6x_h7y_1_sq;
					A.mat[offset + 2*n + 1].matCol.push_back(SparseMatElem(2*iPair+1, (hA[4]*h6x_h7y_1-hA[7]*h3x_h4y_h5) / h6x_h7y_1_sq)); // 列 行
					A2[(offset + 2*n + 1) + (2*iPair+1)*colA] = (hA[4]*h6x_h7y_1-hA[7]*h3x_h4y_h5) / h6x_h7y_1_sq;
				}
				//-----------------------------------------------------

				// 给B赋值
				float errX = h0x_h1y_h2/h6x_h7y_1 - xi;
				float errY = h3x_h4y_h5/h6x_h7y_1 - yi;
				B.mat[0].matCol.push_back(SparseMatElem(2*iPair,   errX)); // 列 行
				B.mat[0].matCol.push_back(SparseMatElem(2*iPair+1, errY)); // 列 行

				costCurrent += errX*errX + errY*errY;

				iPair++;
			}
		}

		COutData::OutData2Txt(A2, colA, rowA, colA, "c:/A.txt");

		delete[] A2;

		cout<<"iPair "<<iPair<<endl;

		cout<<"mse "<<sqrt(costCurrent/iPair);

		for(int col=0; col<A.col; col++)
		{
			std::sort(A.mat[col].matCol.begin(), A.mat[col].matCol.end());
		}
		for(int col=0; col<eye.col; col++)
		{
			std::sort(eye.mat[col].matCol.begin(), eye.mat[col].matCol.end());
		}

		int nSingleIter = 0;
		int maxIterSingle = 20;
		bool convergence = true;
		do
		{
			SolveSparseSystemLM(A, B, eye, pDX, lambda64F);
			nSingleIter++;

			int nonzereos = 0;
			for(int i=0; i<rowX; i++)
			{
				float val = cvGetReal2D(pDX, i, 0);
				
				if(abs(val)>0)
				{
					nonzereos++;
					cout<<val<<" ";
				}
			}
			cout<<endl;
			cout<<"nonzereos "<<nonzereos<<endl;
			if(nonzereos==0)
			{
				break;
			}

			// 更新pX
			cvConvertScale(pDX, pDX, -1);
			cvAdd(pX, pDX, pX2);
			
			double costNew = CalCostBunder2D(wPoints2, wPointsFloat, wPointsFixed,
											pX2,
											transformIdx, refImIdx, nTransformation);

			cout<<"costNew "<<costNew<<" totalCost "<<costCurrent<<" lambda64F "<<lambda64F<<endl;

			if(costNew<costCurrent)
			{
				cvCopy(pX2, pX);
				lambda64F /= beta;
				convergence = true;
				costCurrent = costNew;
				vecCost.push_back(costCurrent);
				//vecCost2[iter] = rotCost;
			}
			else
			{
				lambda64F *= beta;
				convergence = false;
			}
			if(nSingleIter>maxIterSingle)
			{
				break;
			}
		} while(convergence==false);


		cout<<"iter "<<iter<<endl;
		if(vecCost.size()>1)
		{
			if(vecCost[vecCost.size()-1]>=vecCost[vecCost.size()-2])
			{
				break;
			}
		}
	}

	cout<<"Total Cost"<<endl;
	for(int i=0; i<vecCost.size(); i++)
	{
		cout<<vecCost[i]<<" ";
	}

	// 整理计算结果
	for(int n=0; n<nImages; n++)
	{
		if( (label[n]>0) && (n!=refImIdx) )
		{
			int imIdxTran = transformIdx[n];
			float hA[9];
			for(int i=0; i<8; i++)
			{
				hA[i] = cvGetReal2D(pX, 8*imIdxTran+i, 0);
			}
			hA[8] = 1;

			float hInv[9];
			InverseMatrix(hA, 3, hInv, 1e-12f);

			memcpy(vecTransformRefined[n].h.m, hInv, sizeof(hInv));
		}
	}

	return 0;
}

// 用2D重建的思路拼接图像
// refImgIdx0-----参考图像的编号, 该幅图像作为拼接的参考平面
// wpNum----------世界坐标点个数
int MosaicByReconstruction2D(MatchPointPairs2* pMatchPairs2, int nPairs, 
							 int &wpNum,
							int refImgIdx,
							vector<ImageTransform> &vecTransformRefined,
							int label[10000],
							int nImages,
							int newFeatureNum = 0)
{
	cout<<"----MosaicByReconstruction2D----"<<endl<<endl;

	//cout<<"refImgIdx "<<refImgIdx<<endl;

	//vector<MatchPointPairs2> pMatchPairs2;//(nPairs);
	//for(int i=0; i<nPairs; i++)
	//{
	//	MatchPointPairs2 temp;

	//	temp.ptA = pMatchPairs[i].ptA;
	//
	//	temp.ptA_i = pMatchPairs[i].ptA_i;

	//	temp.ptA_Fixed = pMatchPairs[i].ptA_Fixed;


	//	temp.ptB = pMatchPairs[i].ptB;
	//
	//	temp.ptB_i = pMatchPairs[i].ptB_i;

	//	temp.ptB_Fixed = pMatchPairs[i].ptB_Fixed;

	//	if( (label[temp.ptA_i]>0) && (label[temp.ptB_i]>0) )
	//	{
	//		pMatchPairs2.push_back(temp);
	//	}
	//}

	cout<<"pMatchPairs.size() "<<nPairs<<endl;
	//cout<<"pMatchPairs2.size() "<<pMatchPairs2.size()<<endl;

	// 计算每个特征点所对应的世界坐标系点的编号
	if(0)
	{
		wpNum = 0; 
		DetermineWorldPoints( pMatchPairs2, nPairs,
			label, wpNum, newFeatureNum);
	}
	if(1)
	{
		DetermineWorldPointsV2( pMatchPairs2, nPairs,
			label, wpNum, newFeatureNum);
	}

	cout<<"wpNum "<<wpNum<<endl;

	vector<ReconstructedPoint> wPoints(wpNum);
	for(int i=0; i<nPairs; i++)
	{
		int wpIdx = pMatchPairs2[i].wpIdx;

		if(wPoints[wpIdx].idx<0)
		{
			// 初始化物点坐标
			float xW, yW;
			int ptA_i = pMatchPairs2[i].ptA_i;
			float xSrc = pMatchPairs2[i].ptA.x;
			float ySrc = pMatchPairs2[i].ptA.y;

			ApplyProject9(vecTransformRefined[ptA_i].h.m, xSrc, ySrc, xW, yW); 
			
			wPoints[wpIdx].x = xW;
			wPoints[wpIdx].y = yW;
			wPoints[wpIdx].z = 0;
			wPoints[wpIdx].idx = wpIdx;
			//wPoints[wpIdx].id = ptA_i;
			if(ptA_i==refImgIdx)
			{
				wPoints[wpIdx].fixed = 1;
			}
		}

		// 像点坐标
		bool added = false;
		for(int j=0; j<wPoints[wpIdx].imgPt.size(); j++)
		{
			if( (wPoints[wpIdx].imgPt[j].id==pMatchPairs2[i].ptA.id) && 
				(wPoints[wpIdx].imgPt[j].imIdx==pMatchPairs2[i].ptA_i) )
			{
				added = true;
				break;
			}
		}
		if(added==false)
		{
			ImagePoint temp;
			temp.x = pMatchPairs2[i].ptA.x;
			temp.y = pMatchPairs2[i].ptA.y;
			temp.imIdx = pMatchPairs2[i].ptA_i;
			temp.id = pMatchPairs2[i].ptA.id;

			{
				float xW, yW;
				int ptA_i = pMatchPairs2[i].ptA_i;
				float xSrc = pMatchPairs2[i].ptA.x;
				float ySrc = pMatchPairs2[i].ptA.y;

				ApplyProject9(vecTransformRefined[ptA_i].h.m, xSrc, ySrc, xW, yW);
				xW = xW;
			}

			wPoints[wpIdx].imgPt.push_back(temp);
		}

		// B
		for(int j=0; j<wPoints[wpIdx].imgPt.size(); j++)
		{
			if( (wPoints[wpIdx].imgPt[j].id==pMatchPairs2[i].ptB.id) && 
				(wPoints[wpIdx].imgPt[j].imIdx==pMatchPairs2[i].ptB_i) )
			{
				added = true;
				break;
			}
		}
		if(added==false)
		{
			ImagePoint temp;
			temp.x = pMatchPairs2[i].ptB.x;
			temp.y = pMatchPairs2[i].ptB.y;
			temp.imIdx = pMatchPairs2[i].ptB_i;
			temp.id = pMatchPairs2[i].ptB.id;
			{
				float xW, yW;
				int ptB_i = pMatchPairs2[i].ptB_i;
				float xSrc = pMatchPairs2[i].ptB.x;
				float ySrc = pMatchPairs2[i].ptB.y;

				ApplyProject9(vecTransformRefined[ptB_i].h.m, xSrc, ySrc, xW, yW);
				xW = xW;
			}
			wPoints[wpIdx].imgPt.push_back(temp);
		}
	}
	
	MosaicByBundler2D_V2(wPoints, vecTransformRefined, refImgIdx, label, nImages);

	return 0;
}

double CalAlignError(MatchPointPairs2* pMatchPairs, int nPairs,
				  vector<ImageTransform> &vecTransformRefined,
				  int label[10000],
				  int nImages,
				  int outData = 0)
{
	vector<fPoint> vA, vB;

	double err = 0;
	for(int n=0; n<nPairs; n++)
	{
		int ptAi = pMatchPairs[n].ptA_i;
		int ptBi = pMatchPairs[n].ptB_i;

		SfPoint ptA = pMatchPairs[n].ptA;
		SfPoint ptB = pMatchPairs[n].ptB;

		float xA, yA, xB, yB = 0;
		ApplyProject9(vecTransformRefined[ptAi].h.m, ptA.x, ptA.y, xA, yA);
		ApplyProject9(vecTransformRefined[ptBi].h.m, ptB.x, ptB.y, xB, yB);

		fPoint tempA, tempB;
		tempA.x = xA;
		tempA.y = yA;
		tempA.seq = 1;
		tempB.x = xB;
		tempB.y = yB;
		tempB.seq = 1;
		vA.push_back(tempA);
		vB.push_back(tempB);

		float err1 = (xA-xB)*(xA-xB) + (yA-yB)*(yA-yB);

		err += err1;

		if(sqrt(err1)>10)
		{
			//cout<<sqrt(err1)<<" ptAi "<<ptAi<<" ptBi "<<ptBi<<" ["<<ptA.x<<" "<<ptA.y<<"] - ["<<ptB.x<<" "<<ptB.y<<"]"<<endl;
		}
	}

	err = sqrt(err/nPairs);

	if(outData)
	{
		cout<<"vA.size() "<<vA.size()<<endl;
		//COutData::OutPoints2Txt(&vA[0], vA.size(), "c:/xa.txt", "c:/ya.txt");
		//COutData::OutPoints2Txt(&vB[0], vB.size(), "c:/xb.txt", "c:/yb.txt");
	}

	return err;
}

// 增量式拼接
int MosaicIncremental(MatchPointPairs* pMatchPairs0, int nPairs,
					  int refImgIdx,
					  vector<ImageTransform> &vecTransformRefined,
					  int label[10000],
					  int nImages)
{
	cout<<"----MosaicIncremental----"<<endl;
	cout<<"refImgIdx "<<refImgIdx<<endl;

	vector<MatchPointPairs2> pMatchPairs;//(nPairs);
	for(int i=0; i<nPairs; i++)
	{
		MatchPointPairs2 temp;

		temp.ptA = pMatchPairs0[i].ptA;
	
		temp.ptA_i = pMatchPairs0[i].ptA_i;

		temp.ptA_Fixed = pMatchPairs0[i].ptA_Fixed;


		temp.ptB = pMatchPairs0[i].ptB;
	
		temp.ptB_i = pMatchPairs0[i].ptB_i;

		temp.ptB_Fixed = pMatchPairs0[i].ptB_Fixed;

		pMatchPairs.push_back(temp);
	}

	byte imAdded[10000] = {0};
	float eye33[9] = {1,0,0,
					0,1,0,
					0,0,1};

	vector<vector<MatchPointPairs2>> matchPairsNode(nImages);
	for(int i=0; i<nPairs; i++)
	{
		int ptAi = pMatchPairs[i].ptA_i;
		int ptBi = pMatchPairs[i].ptB_i;

		matchPairsNode[ptAi].push_back(pMatchPairs[i]);
		matchPairsNode[ptBi].push_back(pMatchPairs[i]);
	}

	vector<byte> ftAdded(nPairs);
	memset(&ftAdded[0], 0, nPairs);

	vector<ImageTransform> vecTransformation(nImages);
	for(int i=0; i<vecTransformation.size(); i++)
	{
		vecTransformation[i].fixed = 1;
	}

	vector<int> vImNow; // 已经加入的图像

	vector<MatchPointPairs2> vMatchPairsNow; // 已经加入的图像中的特征对

	int wpNum = 0; // 世界点坐标个数

	vImNow.push_back(refImgIdx); // 首先加入参考图像
	imAdded[refImgIdx] = 1;
	vecTransformation[refImgIdx].fixed = 1;
	memcpy(vecTransformation[refImgIdx].h.m, eye33, sizeof(eye33)); // 变换参数

	//IplImage* pairMap = cvCreateImage(cvSize(nImages, nImages), 8, 1);
	
	bool allImAdded = false; // 所有的图像都已经加入的标记
	while(!allImAdded)
	{
		// 查找与vImNow连接的一幅图像, 并确定与哪幅图像相连, 一次加入一幅 (还不够!)
		// 找出与vImNow中的图像特征对数最多的一幅图像
		int idAdd = -1;
		int idPool = -1;
		int maxPairNum = 0;
		for(int n=0; n<nImages; n++)
		{
			if(label[n]<=0)
				continue;
			if(imAdded[n]==1)
				continue;

			for(int m1=0; m1<vImNow.size(); m1++)
			{
				int idPoolCan = vImNow[m1];

				if(idPoolCan==n)
					continue;
				for(int j=0; j<matchPairsNode[idPoolCan].size(); j++)
				{
					if( ((matchPairsNode[idPoolCan][j].ptA_i==idPoolCan)&&(matchPairsNode[idPoolCan][j].ptB_i==n)) ||
						((matchPairsNode[idPoolCan][j].ptA_i==n)&&(matchPairsNode[idPoolCan][j].ptB_i==idPoolCan)) )
					{
						int pairNum = 0;

						int idAddCan = n;
						//int idPoolCan = idPool;
						// 找到了一对连接 [n - idPool], 作为候选, 计算特征对数

						for(int k=0; k<matchPairsNode[idAddCan].size(); k++)
						{
							if((matchPairsNode[idAddCan][k].ptA_i==idAddCan)&&(matchPairsNode[idAddCan][k].ptB_i==idPoolCan))
							{
								pairNum++;
							}
							if((matchPairsNode[idAddCan][k].ptA_i==idPoolCan)&&(matchPairsNode[idAddCan][k].ptB_i==idAddCan))
							{
								pairNum++;
							}
						}

						if(pairNum>maxPairNum)
						{
							idAdd = idAddCan;
							idPool = idPoolCan;
							maxPairNum = pairNum;
						}
						
						//vImNow.push_back(n);
						//idAdd = n;
						
						//imAdded[idAdd] = 1;
						//vecTransformation[idAdd].fixed = 0;
						
						//m1 = vImNow.size();
						//n = nImages;
						break;
					}
				}
			}
		}

		if(idAdd==-1)
		{
			allImAdded = true;
			break;
		}
		else
		{
			vImNow.push_back(idAdd);

			imAdded[idAdd] = 1;
			vecTransformation[idAdd].fixed = 0;
		}

		// 查找idPool-idAdd的特征对
		vector<SfPoint> refPt, datPt;
		for(int j=0; j<matchPairsNode[idAdd].size(); j++)
		{
			if((matchPairsNode[idAdd][j].ptA_i==idAdd)&&(matchPairsNode[idAdd][j].ptB_i==idPool))
			{
				refPt.push_back(matchPairsNode[idAdd][j].ptB);
				datPt.push_back(matchPairsNode[idAdd][j].ptA);
			}
			if((matchPairsNode[idAdd][j].ptA_i==idPool)&&(matchPairsNode[idAdd][j].ptB_i==idAdd))
			{
				refPt.push_back(matchPairsNode[idAdd][j].ptA);
				datPt.push_back(matchPairsNode[idAdd][j].ptB);
			}
		}
		float Hm2n[9]; // m->n
		SolveHomographyMatrix(&refPt[0], &datPt[0], refPt.size(), Hm2n);
		cout<<"H error "<<Hm2n[8]<<endl;
		Hm2n[8] = 1;

		// 计算新加入图像的变换关系
		float Hm2g[9];
		MulMatrix(vecTransformation[idPool].h.m, 3, 3, Hm2n, 3, 3, Hm2g);
		for(int j=0; j<9; j++)
		{
			Hm2g[j] /= Hm2g[8];
		}
		memcpy(vecTransformation[idAdd].h.m, Hm2g, sizeof(Hm2g));

		//cvZero(pairMap);

		int newAddedNum = 0;
		// 查找已加入图像中的特征对
		for(int i=0; i<vImNow.size(); i++)
		{
			int id1 = vImNow[i];
			//for(int j=0; j<vImNow.size(); j++)
			int j = vImNow.size()-1;
			{
				if(i>=j)
					continue;
				int id2= vImNow[j];

				for(int p=0; p<matchPairsNode[id1].size(); p++)
				{
					if( (matchPairsNode[id1][p].ptA_i==id2) || (matchPairsNode[id1][p].ptB_i==id2) )
					{
						//if(ftAdded[])
						vMatchPairsNow.push_back(matchPairsNode[id1][p]);
						newAddedNum++;
					}
				}
			}
		}

		int label2[10000] = {0};
		for(int i=0; i<vImNow.size(); i++)
		{
			label2[vImNow[i]] = 1;
		}

		// 计算误差
		int outData = 0;
		if(vImNow.size()==96)
		{
			outData = 1;
		}
		double alignErr = CalAlignError( &vMatchPairsNow[0], vMatchPairsNow.size(),
			vecTransformation, label2, nImages, outData);

		cout<<endl<<"image num: "<<vImNow.size()<<" ";

		cout<<"alignErr "<<alignErr<<"-------------------------"<<endl;

		//if(idAdd==49)
		//{
		//	cout<<"\a";
		//	cout<<"idAdd "<<idAdd<<" idPool "<<idPool<<" feature pair num "<<refPt.size()<<endl;
		//}

		//if(vImNow.size()==96)
		//{
		//	cout<<"idAdd "<<idAdd<<" idPool "<<idPool<<" feature pair num "<<refPt.size()<<endl;
		//}

		{
			// 优化参数
			MosaicByReconstruction2D( &vMatchPairsNow[0], vMatchPairsNow.size(), 
				wpNum, 
				refImgIdx, 
				vecTransformation, label2, nImages, newAddedNum);
		}

		//if(vImNow.size()>=96)
		//	break;
	}
	cout<<endl;

	vecTransformRefined = vecTransformation;

	return 0;
}

// 没有位姿信息的图像匹配
int CMosaicByPose::MosaicWithoutPose(ImagePoseInfo *pImgPoses, const int nImages,
									 int &numMosaiced)
{
	m_pImgPoses = pImgPoses;
	m_nImages = nImages;

	int w = pImgPoses[0].pImg->width;
	int h = pImgPoses[0].pImg->height;
	float cx = (w-1)*0.5f;
	float cy = (h-1)*0.5f;
	int fLoadMatchPairs = m_fLoadMatchPairs;

	cout<<"Mosaic Without Pose"<<endl;

	float hEye[9] = {1, 0, 0, 
					 0, 1, 0, 
					 0, 0, 1};

	ImageTransform normT;
	normT.fixed = 0;
	memcpy(normT.h.m, hEye, sizeof(float)*9);

	//********************************************************
	// geometrical alignment
	int nSuccessImages = 1;
	vector<ImageTransform> vecTransformAffineRefined;
	{
		// feature matching
		vector<MatchPointPairs> vecMatchPairs;
		vecMatchPairs.clear();
		
		//GetMatchedPairsSingleStripSurf(pImgPoses, nImages, 
		//	vecMatchPairs, 
		//	nSuccessImages);

		if(fLoadMatchPairs)
		{
			cout<<"-----载入匹配对, 输入特征匹配文件路径 (*.match)"<<endl<<">> ";
			string matchFilePath;
			cin>>matchFilePath;
			
			LoadMatchPairs(vecMatchPairs, (char*)matchFilePath.c_str());
			//LoadMatchPairs(vecMatchPairs, "d:/feature_temp/matchPairs-UAV-620-SIFT.match");
			//LoadMatchPairs(vecMatchPairs, "d:/feature_temp/matchPairs-17-2.match");
			//LoadMatchPairs(vecMatchPairs, "c:/feature_temp/matchPairs-44-SURF.match");
			//ImportVisualSFM_Match((char*)matchFilePath.c_str(), vecMatchPairs, cx, cy);
			cout<<"Loaded vecMatchPairs size "<<vecMatchPairs.size()<<endl;
		}
		else
		{
			cout<<"compute feature matching"<<endl;
			//GetMatchedPairsOneToAllSurf(pImgPoses, nImages, 
			//	vecMatchPairs, 
			//	nSuccessImages);

			GetMatchedPairsOneToAllSIFT_MultiThread();
			vecMatchPairs = m_vecMatchPairs;

			//GetMatchedPairsSingleStripSurf(pImgPoses, nImages, 
			//	vecMatchPairs, 
			//	nSuccessImages);

			WriteMatchPairs(vecMatchPairs, "feature_temp/matchPairs.match");
		}
		//WriteMatchPairs(vecMatchPairs, "c:/feature_temp/matchPairs.match");

		//ShowMultiViewMatchPairs(pImgPoses, nImages, vecMatchPairs);

		//GetMatchedPairsMultiStripSurf(pImgPoses, nImages, 
		//	vecMatchPairs, iFixedImage);

		int label[10000]={0};

		Select_Connected_Matched_Images(vecMatchPairs, nImages, label);

		nSuccessImages = 0;
		for(int i=0; i<nImages; i++)
		{
			if(label[i]>0)
				nSuccessImages++;
		}

		for(int m=0; m<vecMatchPairs.size();)
		{
			if( (label[vecMatchPairs[m].ptA_i]==0) || (label[vecMatchPairs[m].ptB_i]==0) ) // 剔除未连接成一体的匹配对
			{
				vecMatchPairs[m] = vecMatchPairs[vecMatchPairs.size()-1];
				vecMatchPairs.pop_back();
			}
			else
			{
				 m++;
			}
		}

		int refNum[10] = {0, 1, 2, 4, 5, 6, 7, 8, 9};
		//                0  1    2    3    4    5    6     7    8   9         
		// select reference image index number
		int iRefImage = refNum[0]; 

		int refIndex = 0;
		//cout<<"input ref image index:"<<endl<<">>";
		//cin>>refIndex;
		iRefImage = refNum[refIndex];

		cout<<"iRefImage "<<iRefImage<<endl;

		cout<<"nSuccessImages "<<nSuccessImages<<endl;

		vector<ImageTransform> transformInit;
		for(int n=0; n<nImages; n++)
		{
			transformInit.push_back(normT);
		}
		int nMatchedPt = (int)vecMatchPairs.size();

		for(int nf=0; nf<nMatchedPt; nf++)
		{
			if(vecMatchPairs[nf].ptA_i==iRefImage)
			{
				vecMatchPairs[nf].ptA_Fixed = 1;
			}
			if(vecMatchPairs[nf].ptB_i==iRefImage)
			{
				vecMatchPairs[nf].ptB_Fixed = 1;
			}
		}

		// 确定参数不做优化的图像
		int nFixedImages = 0;
		for(int i=0; i<nImages; i++)
		{
			if(label[i]==0)
			{
				transformInit[i].fixed = 1; // 无效的图像(无需求解参数)
				pImgPoses[i].fixed = 1;	
				nFixedImages++;
			}
		}
		transformInit[iRefImage].fixed = 1; // 
		pImgPoses[iRefImage].fixed = 1; 
		nFixedImages++;

		cout<<"nSuccessImages "<<nSuccessImages<<endl;

		WriteMatchPairs_ASC2(vecMatchPairs, "feature_temp/matchPairs.txt");

		float t_optimization = 0;
		StartCalTime(t_optimization);

		float weight = 400;

		cout<<"global adjustment"<<endl;
		// bundle adjustment（affine model）
		if(!vecMatchPairs.empty())
		{
			if(m_downViewConstraint)
			{
				int methodType = 0; // 0----Rigid Constraint, 1----Bundler2D
				if(methodType==0)
				{
					BundleAdjustmentSparse(&vecMatchPairs[0], vecMatchPairs.size(), 
						&transformInit[0], nImages, nFixedImages, vecTransformAffineRefined);
				}
				else if(methodType==1) //
				{
					cout<<"2d bundler"<<endl;
					MosaicIncremental(&vecMatchPairs[0], vecMatchPairs.size(), iRefImage, 
						vecTransformAffineRefined, label,
						nImages);

					//
					//MosaicByReconstruction2D( &vecMatchPairs[0], vecMatchPairs.size(), iRefImage,
					//	vecTransformAffineRefined, label, nImages);
				}
				
				OutTransform(vecTransformAffineRefined, "tran0.txt");

				//vecTransformAffineRefined.clear(); ImportTransform(vecTransformAffineRefined, "c://refine.txt");

			}
			else
			{
				cout<<"BundleAdjustmentSparse"<<endl;
				BundleAdjustmentSparse(&vecMatchPairs[0], vecMatchPairs.size(), 
					&transformInit[0], nImages, nFixedImages, vecTransformAffineRefined);
			}
		}

		StopCalTime(t_optimization, "t_optimization");

		numMosaiced = nImages;

		//BundleAdjustment(&vecMatchPairs[0], vecMatchPairs.size(), 
		//				&transformInit[0], nImages, nFixedImages, vecTransformAffineRefined);

		vector<ImageTransform> vecTransformRefined;
		// 
		bool nonlinearAdjustment = false; // false
		if(nonlinearAdjustment)
		{
			if(!vecMatchPairs.empty())
			{
				BundleAdjustmentNonlinear(&vecMatchPairs[0], vecMatchPairs.size(), 
					&vecTransformAffineRefined[0], nSuccessImages, nFixedImages, 
					vecTransformRefined);
			}
		}
		else
		{
			for(int i=0; i<(int)vecTransformAffineRefined.size(); i++)
			{
				vecTransformRefined.push_back(vecTransformAffineRefined[i]);
			}
		}
	
		for(int i=0; i<nImages; i++)
		{
			if(label[i]==0)
			{
				vecTransformAffineRefined[i].h.m[8] = 0;
			}
		}
	}

	//*********************************************************
	cout<<"image blending"<<endl<<endl;

	if(m_blending==2)
	{ 
		// Laplacian blending
		if(!vecTransformAffineRefined.empty())
		{
			MergeImagesRefined(pImgPoses, nImages, &vecTransformAffineRefined[0]);
		}
	}
	else
	{
		// Linear blending
		if(!vecTransformAffineRefined.empty())
		{
			MosaicImagesRefined(pImgPoses, nImages, &vecTransformAffineRefined[0]);
		}
	}

	if(numMosaiced<2)
		return -2;
	
	return 0;
}

// 保存特征
int WriteSurfKeyPoints(const std::vector<KeyPoint> &keyPoints, const Mat &descriptors, 
					   char* pNameKeyPoints, char* pNameDiscriptor)
{
	// 写描述子
	//imwrite(pNameDiscriptor, descriptors);
	FileStorage fs(pNameDiscriptor, FileStorage::WRITE);
	fs<<"descriptor"<<descriptors;
	fs.release();

	// 写特征点
	if(!keyPoints.empty())
	{
		int nPoints = keyPoints.size();
		std::ofstream fout(pNameKeyPoints, std::ios::binary);
		fout.write((char*)&nPoints, sizeof(int));
		fout.write((char*)&keyPoints[0], keyPoints.size()*sizeof(KeyPoint));

		fout.close();
	}

	return 0;
}

// 载入特征
int LoadSurfKeyPoints(std::vector<KeyPoint> &keyPoints, Mat &descriptors, char* pNameKeyPoints, char* pNameDiscriptor)
{
	keyPoints.clear();

	// 读描述子
	//descriptors = imread(pNameDiscriptor);
	FileStorage fs(pNameDiscriptor, FileStorage::READ);
	fs["descriptor"]>>descriptors;

	// 读特征点
	std::ifstream fin(pNameKeyPoints, std::ios::binary);
	int nPoints = 0;
	if(!fin.read((char*)&nPoints, sizeof(int)))
		return -1;

	KeyPoint *pKeyPoints = new KeyPoint[nPoints];

	fin.read((char*)pKeyPoints, sizeof(KeyPoint) * nPoints);
	fin.close();

	for(int n=0; n<nPoints; n++)
	{
		keyPoints.push_back(pKeyPoints[n]);
	}

	delete[] pKeyPoints; pKeyPoints = NULL;

	return 0;
}

int WriteMatchPairs(const vector<MatchPointPairs> &vecMatchPairs, char* pName)
{	
	// 写匹配对
	if(!vecMatchPairs.empty())
	{
		int nPoints = vecMatchPairs.size();
		std::ofstream fout(pName, std::ios::binary);
		fout.write((char*)&nPoints, sizeof(int));
		fout.write((char*)&vecMatchPairs[0], vecMatchPairs.size()*sizeof(MatchPointPairs));

		fout.close();
	}
	return 0;
}

int WriteMatchPairs_ASC2(const vector<MatchPointPairs> &vecMatchPairs, char* pName)
{	
	int N = vecMatchPairs.size();

	ofstream outData(pName, ios::trunc);

	for(int i=0; i<N; i++)
	{
		outData<<vecMatchPairs[i].ptA_i<<" "
				<<vecMatchPairs[i].ptA.x<<" "
				<<vecMatchPairs[i].ptA.y<<" "
				<<vecMatchPairs[i].ptA_Fixed<<" "
				<<vecMatchPairs[i].ptB_i<<" "
				<<vecMatchPairs[i].ptB.x<<" "
				<<vecMatchPairs[i].ptB.y<<" "
				<<vecMatchPairs[i].ptB_Fixed<<endl;
	}

	outData.close();

	return 0;
}

int LoadMatchPairs(vector<MatchPointPairs> &vecMatchPairs, char* pName)
{
	vecMatchPairs.clear();

	// 读匹配对
	std::ifstream fin(pName, std::ios::binary);
	int nPoints = 0;
	if(!fin.read((char*)&nPoints, sizeof(int)))
		return -1;

	MatchPointPairs *pMatchPairs = new MatchPointPairs[nPoints];

	fin.read((char*)pMatchPairs, sizeof(MatchPointPairs) * nPoints);
	fin.close();

	for(int n=0; n<nPoints; n++)
	{
		vecMatchPairs.push_back(pMatchPairs[n]);
	}

	delete[] pMatchPairs; pMatchPairs = NULL;

	return 0;
}

void CheckFundamentalMatrix(const vector<SfPoint> &pt1, const vector<SfPoint> &pt2, Mat F)
{
	int N = pt1.size();
	double FM[9];
	FM[0] = F.at<double>(0,0);
	FM[1] = F.at<double>(0,1);
	FM[2] = F.at<double>(0,2);

	FM[3] = F.at<double>(1,0);
	FM[4] = F.at<double>(1,1);
	FM[5] = F.at<double>(1,2);

	FM[6] = F.at<double>(2,0);
	FM[7] = F.at<double>(2,1);
	FM[8] = F.at<double>(2,2);

	for(int n=0; n<N; n++)
	{
		float a = FM[0]*pt1[n].x + FM[1]*pt1[n].y + FM[2];
		float b = FM[3]*pt1[n].x + FM[4]*pt1[n].y + FM[5];
		float c = FM[6]*pt1[n].x + FM[7]*pt1[n].y + FM[8];

		float dist = 0;
		DistanceOfPointToABCLine(pt2[n].x, pt2[n].y, a, b, c, dist);
		cout<<dist<<endl;
	}
	cout<<endl;
}

// 多线程匹配
// 开自动处理线程
//CWinThread *pPrsThread = ::AfxBeginThread(OnProcessThread, this); // 开线程
//UINT OnProcessThread(LPVOID pThreadParam) // 线程函数
DWORD WINAPI SiftExtraction_Thread(LPVOID pThreadParam)
{
	CMosaicByPose* pParam = (CMosaicByPose*)pThreadParam;

	ImagePoseInfo *pImgPoses = pParam->m_pImgPoses;
	int nImages = pParam->m_nImages;
	int nThread = pParam->m_nMatchThread; // 匹配线程数
	int idxThread = pParam->m_idxMatchThread; // 当前线程编号

	vector<MatchPointPairs> vecMatchPairs;
	int nSuccess = 0;

	//cout<<"特征提取和匹配"<<endl;
	vector<IntPoint> vecMatched;
	const int MIN_INNER_POINTS = 18;
	int maxFeatruesNum = pParam->m_maxFeatureNum;
	float ransacDist = pParam->m_ransacDist;
	float distT = pParam->m_matchDist;
	int minHessian = pParam->m_minHessian;

	SiftFeatureDetector detector(2000, 3, 0.01, 20);
	//SurfFeatureDetector detector( minHessian );
	//SurfDescriptorExtractor extractor;
	SiftDescriptorExtractor extractor;

	bool failed = false;

	cout<<"feature extration"<<endl;
	// 特征提取
	for(int i=idxThread; i<nImages; i=i+nThread)
	{
		cout<<i<<" ";
		std::vector<KeyPoint> keyPoints;

		Mat descriptors;
		// 检测
		//detector.detect( pImgPoses[i].pImg, keyPoints );
		Mat img(pImgPoses[i].pImg);
		detector.detect(pImgPoses[i].pImg, keyPoints);
		// 描述
		extractor.compute(pImgPoses[i].pImg, keyPoints, descriptors);

		char nameKeyPoints[256], nameDiscriptor[256];
		sprintf_s(nameKeyPoints, "d:/feature_temp/keypoint_%d.key", i);
		sprintf_s(nameDiscriptor, "d:/feature_temp/discriptor_%d.xml", i);

		cout<<"feature number "<<keyPoints.size()<<endl;
		WriteSurfKeyPoints(keyPoints, descriptors, 
			nameKeyPoints, nameDiscriptor);
	}
	cout<<endl;

	pParam->MatchThreadOver_Plus();

	return 0;
}

// 计算单应矩阵
Mat CalHomoH_CV(vector<SfPoint> vecMatch1, vector<SfPoint> vecMatch2, 
				float ransacDist,
				vector<SfPoint> &inner1, vector<SfPoint> &inner2)
{
	vector<Point2f> pt1, pt2;
	for(int c=0; c<vecMatch1.size(); c++)
	{
		Point2f temp1, temp2;
		temp1.x = vecMatch1[c].x; temp1.y = vecMatch1[c].y;
		temp2.x = vecMatch2[c].x; temp2.y = vecMatch2[c].y;
		pt1.push_back(temp1);
		pt2.push_back(temp2);
	}			

	Mat H, mask;
	H = findHomography(pt1, pt2, CV_RANSAC, ransacDist, mask);

	int depth = mask.depth();
	int nInliers = 0;
	for(int c=0; c<vecMatch1.size(); c++)
	{
		if(mask.at<byte>(c,0)>0)
		{
			inner1.push_back(vecMatch1[c]);
			inner2.push_back(vecMatch2[c]);
		}
	}

	bool valid = true;
	for(int i=0; i<inner1.size(); i++)
	{
		for(int j=i+1; j<inner1.size();)
		{
			if( (inner1[i].x==inner1[j].x) && (inner1[i].y==inner1[j].y) )
			{
				swap(inner1[j], inner1[inner1.size()-1]);
				inner1.pop_back();
				swap(inner2[j], inner2[inner2.size()-1]);
				inner2.pop_back();
			}
			else
				j++;
		}
	}

	for(int i=0; i<inner2.size(); i++)
	{
		for(int j=i+1; j<inner2.size(); )
		{
			if( (inner2[i].x==inner2[j].x) && (inner2[i].y==inner2[j].y) )
			{
				swap(inner1[j], inner1[inner1.size()-1]);
				inner1.pop_back();
				swap(inner2[j], inner2[inner2.size()-1]);
				inner2.pop_back();
			}
			else
				j++;
		}
	}

	return H;	
}

int SelectMatchPairs(const std::vector<DMatch> &matches,
					 const std::vector<KeyPoint> &keyPoints1, const std::vector<KeyPoint> &keyPoints2, 
					 const int nMatch,
					 vector<SfPoint> &vecMatch1, vector<SfPoint> &vecMatch2)
{
	for(int n=0; n<nMatch; n++)
	{
		SfPoint temp1;
		temp1.x = keyPoints1[matches[n].queryIdx].pt.x;
		temp1.y = keyPoints1[matches[n].queryIdx].pt.y;
		temp1.id = matches[n].queryIdx;
		vecMatch1.push_back(temp1);
		SfPoint temp2;
		temp2.x = keyPoints2[matches[n].trainIdx].pt.x;
		temp2.y = keyPoints2[matches[n].trainIdx].pt.y;
		temp2.id = matches[n].trainIdx;
		vecMatch2.push_back(temp2);
	}

	return 0;
}

// 栅格化-均匀采样
int SelectMatchPairs(const std::vector<DMatch> &matches,
					 const std::vector<KeyPoint> &keyPoints1, const std::vector<KeyPoint> &keyPoints2, 
					 const int nMatch,
					 int width, int height, 
					 int gridX, int gridY,
					 vector<SfPoint> &vecMatch1, vector<SfPoint> &vecMatch2)
{
	vecMatch1.clear();
	vecMatch2.clear();

	int nGrids = gridX*gridY;
	//cout<<"nGrids "<<nGrids<<endl;
	int nMatchPerGrid = (float)nMatch/nGrids;
	//cout<<"nMatchPerGrid "<<nMatchPerGrid<<endl;
	int *label = new int[nGrids];
	memset(label, 0, sizeof(int)*nGrids);
	int stepX = width/gridX;
	int stepY = height/gridY;
	//cout<<"stepX "<<stepX<<endl;
	//cout<<"stepY "<<stepY<<endl;

	int nTotalMatch = matches.size();

	for(int n=0; n<nTotalMatch; n++)
	{
		SfPoint temp1;
		temp1.x = keyPoints1[matches[n].queryIdx].pt.x;
		temp1.y = keyPoints1[matches[n].queryIdx].pt.y;
		temp1.id = matches[n].queryIdx;

		int nX = temp1.x / stepX;
		int nY = temp1.y / stepY;

		if( label[gridX*nY+nX]>=nMatchPerGrid )
			continue;
		
		SfPoint temp2;
		temp2.x = keyPoints2[matches[n].trainIdx].pt.x;
		temp2.y = keyPoints2[matches[n].trainIdx].pt.y;
		temp2.id = matches[n].trainIdx;

		vecMatch1.push_back(temp1);
		vecMatch2.push_back(temp2);

		label[gridX*nY+nX]++;
	}

	//cout<<"vecMatch1.size() "<<vecMatch1.size()<<endl;

	delete[] label; label = NULL;
	return 0;
}

// 多线程匹配
DWORD WINAPI GetMatchedPairsOneToAllSIFTThread(LPVOID pThreadParam)
{
	CMosaicByPose* pParam = (CMosaicByPose*)pThreadParam;

	ImagePoseInfo *pImgPoses = pParam->m_pImgPoses;
	int nImages = pParam->m_nImages;
	int nThread = pParam->m_nMatchThread; // 匹配线程数
	int idxThread = pParam->m_idxMatchThread; // 当前线程编号
	int width = pParam->m_width;
	int height = pParam->m_height;
	int gridX = 3; 
	int gridY = 3;

	vector<MatchPointPairs> vecMatchPairs;
	int nSuccess = 0;

	//cout<<"特征提取和匹配"<<endl;
	vector<IntPoint> vecMatched;
	const int MIN_INNER_POINTS = 30;
	int maxFeatruesNum = pParam->m_maxFeatureNum;
	float ransacDist = pParam->m_ransacDist;
	float distT = pParam->m_matchDist;
	int minHessian = pParam->m_minHessian;

	SiftFeatureDetector detector(4000, 3, 0.01, 20);
	//SurfFeatureDetector detector( minHessian );
	//SurfDescriptorExtractor extractor;
	SiftDescriptorExtractor extractor;

	vector<SfPoint> vecMatch1, vecMatch2;
	vector<SfPoint> vecInnerPoints1, vecInnerPoints2;

	bool failed = false;
	nSuccess = 1;

	for(int i=idxThread; i<nImages; i=i+nThread)
	{
		cout<<i<<" "<<endl;
		// 载入特征点1
		std::vector<KeyPoint> keyPoints1;
		Mat descriptors1;

		char nameKeyPoints[256], nameDiscriptor[256];
		sprintf_s(nameKeyPoints, "d:/feature_temp/keypoint_%d.key", i);
		sprintf_s(nameDiscriptor, "d:/feature_temp/discriptor_%d.xml", i);
		LoadSurfKeyPoints(keyPoints1, descriptors1, nameKeyPoints, nameDiscriptor);

		//Mat src(pImgPoses[i].pImg), show(pImgPoses[i].pImg);
		//drawKeypoints(src, keyPoints1, show);
		//imshow("show", show);
		//cvWaitKey(-1);

		int jEnd = min(nImages, i+182); // uav-620
		for(int j=i+1; j<jEnd; j++)
		{

		//int ext = min(15, nImages/2-1);
		//for(int j0=i+1; j0<nImages+ext; j0++) // He-Hu
		//{
		//	if(j0-i>ext)
		//		continue;
		//	int j = j0;
		//	if(j0>=nImages)
		//		j = j0 - nImages;

			// 载入特征点2
			std::vector<KeyPoint> keyPoints2;
			Mat descriptors2;

			char nameKeyPoints[256], nameDiscriptor[256];
			sprintf_s(nameKeyPoints, "d:/feature_temp/keypoint_%d.key", j);
			sprintf_s(nameDiscriptor, "d:/feature_temp/discriptor_%d.xml", j);
			LoadSurfKeyPoints(keyPoints2, descriptors2, nameKeyPoints, nameDiscriptor);

			//cout<<"keypoints "<<keyPoints1.size()<<" "<<keyPoints2.size()<<endl;

			// 特征匹配
			FlannBasedMatcher matcher;
			std::vector< DMatch > matches;
			matcher.match(descriptors1, descriptors2, matches);
			std::sort(matches.begin(), matches.end());

			//Mat img_matches;
			//drawMatches( pImgPoses[i].pImg, keyPoints1, pImgPoses[j].pImg, keyPoints2, matches, img_matches );
			////-- Show detected matches
			//imshow("Matches", img_matches );
			//waitKey(-1);

			//distT = pParam->m_matchDist;
			//double dT = 0.05;
			//do
			//{
			//	vecMatch1.clear();
			//	vecMatch2.clear();
			//	for(int n=0; n<(int)matches.size(); n++)
			//	{
			//		if(matches[n].distance<distT)
			//		{
			//			SfPoint temp1;
			//			temp1.x = keyPoints1[matches[n].queryIdx].pt.x;
			//			temp1.y = keyPoints1[matches[n].queryIdx].pt.y;
			//			temp1.id = matches[n].queryIdx;
			//			vecMatch1.push_back(temp1);
			//			SfPoint temp2;
			//			temp2.x = keyPoints2[matches[n].trainIdx].pt.x;
			//			temp2.y = keyPoints2[matches[n].trainIdx].pt.y;
			//			temp2.id = matches[n].trainIdx;
			//			vecMatch2.push_back(temp2);
			//		}
			//	}
			//	distT -= dT;
			//}
			//while((int)vecMatch2.size()>maxFeatruesNum);

			// Match Sift
			int maxNum = 400;
			int nMatch = Min(maxNum, 0.3*matches.size());
			
			SelectMatchPairs(matches, keyPoints1, keyPoints2, 
								nMatch,
								width, height, 
								gridX, gridY, 
								vecMatch1, vecMatch2);

			//SelectMatchPairs(matches, keyPoints1, keyPoints2, 
			//	nMatch,
			//	vecMatch1, vecMatch2);

			//cout<<"match flann "<<vecMatch1.size()<<" "<<vecMatch2.size()<<endl;

			bool F_Matrix = false;
			int nInnerPoints = 0;
			//CMosaicHarris mosaicHarris;
			Mat F, mask;
			if(F_Matrix==false) //  用 单应矩阵
			{
				//Mat H = CalHomoH_CV(vecMatch1, vecMatch2, ransacDist, vecInnerPoints1, vecInnerPoints2);
				float homo[9];
				Ransac2D(vecMatch1, vecMatch2, vecInnerPoints1, vecInnerPoints2, homo, ransacDist);

				nInnerPoints = vecInnerPoints1.size();
			}
			else // 用基础矩阵
			{
				vector<Point2f> pt1, pt2;
				for(int c=0; c<vecMatch1.size(); c++)
				{
					Point2f temp1, temp2;
					temp1.x = vecMatch1[c].x; temp1.y = vecMatch1[c].y;
					temp2.x = vecMatch2[c].x; temp2.y = vecMatch2[c].y;
					pt1.push_back(temp1);
					pt2.push_back(temp2);
				}			

				F = findFundamentalMat(pt1, pt2, CV_FM_RANSAC, 1, 0.99, mask);

				int depth = mask.depth();
				int nInliers = 0;
				for(int c=0; c<vecMatch1.size(); c++)
				{
					if(mask.at<byte>(c,0)>0)
					{
						nInnerPoints++;
						vecInnerPoints1.push_back(vecMatch1[c]);
						vecInnerPoints2.push_back(vecMatch2[c]);
					}
				}
			}

			//cout<<"img "<<i<<" / "<<j<<" inner "<<vecInnerPoints1.size()<<" / "<<vecMatch1.size()<<" matched surf"<<endl;
			if(nInnerPoints>MIN_INNER_POINTS)
			{
				cout<<"img "<<i<<" / "<<j<<" inner "<<vecInnerPoints1.size()<<" / "<<vecMatch1.size()<<" matched surf"<<endl;

				//CheckFundamentalMatrix(mosaicHarris.m_vecInnerPoints1, mosaicHarris.m_vecInnerPoints2, F);

				// 匹配成功
				for(int p=0; p<nInnerPoints; p++)
				{
					MatchPointPairs temp;
					temp.ptA = vecInnerPoints1[p];
					temp.ptA_Fixed = pImgPoses[i].fixed;
					temp.ptA_i = i;

					temp.ptB = vecInnerPoints2[p];
					temp.ptB_Fixed = pImgPoses[j].fixed;
					temp.ptB_i = j;
					vecMatchPairs.push_back(temp);
				}
				nSuccess++;
			}
			else
			{
				//cout<<"img "<<i<<" / "<<j<<" inner "<<nInnerPoints<<" / "<<vecMatch1.size()<<" matched surf"<<endl;
				// 匹配失败
				failed = true;
			}
			vecInnerPoints1.clear();
			vecInnerPoints2.clear();
			vecMatch1.clear();
			vecMatch2.clear();
		}
	}
	cout<<endl;

	pParam->PushMatchPairs(vecMatchPairs);

	pParam->MatchThreadOver_Plus();

	return 0;
}

// 多线程匹配
int CMosaicByPose::GetMatchedPairsOneToAllSIFT_MultiThread()
{
	m_nMatchThread = max(GetCpuNum() - 1, 1);
	m_nMatchThread = min(8, m_nMatchThread);
	//m_nMatchThread = 1;

	//m_nMatchThread = 1;

	cout<<"cpu num "<<m_nMatchThread<<endl;

	m_nMatchThreadOver = 0; // 特征提取线程完成数

	// 多线程特征提取
	for(int i=0; i<m_nMatchThread; i++)
	{
		m_idxMatchThread = i;

		//CWinThread *pPrsThread = ::AfxBeginThread(SiftExtraction_Thread, this); // 开线程
		CreateThread(NULL, 0, SiftExtraction_Thread, this, 0, NULL);
		// 
		Sleep(20);
	}

	while(m_nMatchThreadOver<m_nMatchThread) // 等待特征提取完成
	{
		Sleep(20);
	}

	//printf("特征匹配线程完成数\n");

	m_nMatchThreadOver = 0; // 特征匹配线程完成数

	// 多线程匹配
	for(int i=0; i<m_nMatchThread; i++)
	{
		m_idxMatchThread = i;

		//if(i==1)
		{
			//CWinThread *pPrsThread = ::AfxBeginThread(GetMatchedPairsOneToAllSIFTThread, this); // 开线程
			CreateThread(NULL, 0, GetMatchedPairsOneToAllSIFTThread, this, 0, NULL);
			Sleep(20);
		}
	}

	while(m_nMatchThreadOver<m_nMatchThread) // 等待特征匹配完成
	{
		Sleep(20);
	}

	return 0;
}

// 特征匹配
// 没有位姿先验信息
// one-to-all
int CMosaicByPose::GetMatchedPairsOneToAllSurf(const ImagePoseInfo *pImgPoses, const int nImages, 
												vector<MatchPointPairs> &vecMatchPairs,
												int &nSuccess)
{
	//cout<<"特征提取和匹配"<<endl;
	vector<IntPoint> vecMatched;
	const int MIN_INNER_POINTS = 18;
	int maxFeatruesNum = m_maxFeatureNum;
	float ransacDist = m_ransacDist;
	float distT = m_matchDist;
	int minHessian = m_minHessian;

	//SiftFeatureDetector detector(2000, 3, 0.01, 20);
	SurfFeatureDetector detector( minHessian );
	SurfDescriptorExtractor extractor;
	//SiftDescriptorExtractor extractor;
	
	vector<SfPoint> vecMatch1, vecMatch2;

	bool failed = false;
	nSuccess = 1;

	cout<<"特征提取"<<endl;
	// 特征提取
	for(int i=0; i<nImages; i++)
	{
		cout<<i<<" ";
		std::vector<KeyPoint> keyPoints;

		Mat descriptors;
		// 检测
		//detector.detect( pImgPoses[i].pImg, keyPoints );
		Mat img(pImgPoses[i].pImg);
		detector.detect(pImgPoses[i].pImg, keyPoints);
		// 描述
		extractor.compute(pImgPoses[i].pImg, keyPoints, descriptors);

		char nameKeyPoints[256], nameDiscriptor[256];
		sprintf_s(nameKeyPoints, "d:/feature_temp/keypoint_%d.key", i);
		sprintf_s(nameDiscriptor, "d:/feature_temp/discriptor_%d.xml", i);

		cout<<"feature number "<<keyPoints.size()<<endl;
		WriteSurfKeyPoints(keyPoints, descriptors, 
						   nameKeyPoints, nameDiscriptor);
	}
	cout<<endl;

	//----------------------------------------------

	for(int i=0; i<nImages; i++)
	{
		cout<<i<<" "<<endl;
		// 载入特征点1
		std::vector<KeyPoint> keyPoints1;
		Mat descriptors1;

		char nameKeyPoints[256], nameDiscriptor[256];
		sprintf_s(nameKeyPoints, "d:/feature_temp/keypoint_%d.key", i);
		sprintf_s(nameDiscriptor, "d:/feature_temp/discriptor_%d.xml", i);
		LoadSurfKeyPoints(keyPoints1, descriptors1, nameKeyPoints, nameDiscriptor);
		
		//Mat src(pImgPoses[i].pImg), show(pImgPoses[i].pImg);
		//drawKeypoints(src, keyPoints1, show);
		//imshow("show", show);
		//cvWaitKey(-1);

		//int jEnd = min(nImages, i+100); // uav-620
		//for(int j=i+1; j<jEnd; j++)
		//{

		int ext = min(15, nImages/2-1);
		for(int j0=i+1; j0<nImages+ext; j0++) // He-Hu
		{
			if(j0-i>ext)
				continue;
			int j = j0;
			if(j0>=nImages)
				j = j0 - nImages;
				
			// 载入特征点2
			std::vector<KeyPoint> keyPoints2;
			Mat descriptors2;

			char nameKeyPoints[256], nameDiscriptor[256];
			sprintf_s(nameKeyPoints, "d:/feature_temp/keypoint_%d.key", j);
			sprintf_s(nameDiscriptor, "d:/feature_temp/discriptor_%d.xml", j);
			LoadSurfKeyPoints(keyPoints2, descriptors2, nameKeyPoints, nameDiscriptor);
			
			// 特征匹配
			FlannBasedMatcher matcher;
			std::vector< DMatch > matches;
			matcher.match(descriptors1, descriptors2, matches);
			std::sort(matches.begin(), matches.end());

			//Mat img_matches;
			//drawMatches( pImgPoses[i].pImg, keyPoints1, pImgPoses[j].pImg, keyPoints2, matches, img_matches );
			////-- Show detected matches
			//imshow("Matches", img_matches );
			//waitKey(-1);

			distT = m_matchDist;
			double dT = 0.05;
			do
			{
				vecMatch1.clear();
				vecMatch2.clear();
				for(int n=0; n<(int)matches.size(); n++)
				{
					if(matches[n].distance<distT)
					{
						SfPoint temp1;
						temp1.x = keyPoints1[matches[n].queryIdx].pt.x;
						temp1.y = keyPoints1[matches[n].queryIdx].pt.y;
						temp1.id = matches[n].queryIdx;
						vecMatch1.push_back(temp1);
						SfPoint temp2;
						temp2.x = keyPoints2[matches[n].trainIdx].pt.x;
						temp2.y = keyPoints2[matches[n].trainIdx].pt.y;
						temp2.id = matches[n].trainIdx;
						vecMatch2.push_back(temp2);
					}
				}
				distT -= dT;
			}
			while((int)vecMatch2.size()>maxFeatruesNum);

	//		int nMatch = min(200, 0.3*matches.size());
	//		for(int n=0; n<nMatch; n++)
	//		{
	//			SfPoint temp1;
	//			temp1.x = keyPoints1[matches[n].queryIdx].pt.x;
	//			temp1.y = keyPoints1[matches[n].queryIdx].pt.y;
	//			temp1.id = matches[n].queryIdx;
	//			vecMatch1.push_back(temp1);
	//			SfPoint temp2;
	//			temp2.x = keyPoints2[matches[n].trainIdx].pt.x;
	//			temp2.y = keyPoints2[matches[n].trainIdx].pt.y;
	//			temp2.id = matches[n].trainIdx;
	//			vecMatch2.push_back(temp2);
	//		}

			bool F_Matrix = false;
			int nInnerPoints = 0;
			CMosaicHarris mosaicHarris;
			Mat F, mask;
			if(F_Matrix==false) //  用 单应矩阵
			{
				BitmapImage* pSrc1 = new BitmapImage();
				BitmapImage* pSrc2 = new BitmapImage();
				CvImage2BitmapImage(pImgPoses[i].pImg, pSrc1);
				CvImage2BitmapImage(pImgPoses[j].pImg, pSrc2);

	#ifdef _DEBUG
				//ShowMatch(pSrc1, pSrc2, &vecMatch1[0], &vecMatch2[0], vecMatch1.size(), _T("c:\\matchSurf.bmp")); 
	#endif
				//cout<<"matched surf "<<vecMatch1.size()<<endl;

				float homo[9];
				
				mosaicHarris.Ransac(pSrc1, pSrc2, vecMatch1, vecMatch2, homo, ransacDist);
				nInnerPoints = (int)mosaicHarris.m_vecInnerPoints1.size();
				if(nInnerPoints<=MIN_INNER_POINTS) // 如果ransac失败, 重试一次
				{
					//mosaicHarris.Ransac(pSrc1, pSrc2, vecMatch1, vecMatch2, homo, ransacDist);
					//nInnerPoints = (int)mosaicHarris.m_vecInnerPoints1.size();
				}

				delete pSrc1; pSrc1 = NULL;
				delete pSrc2; pSrc2 = NULL;
			}
			else // 用基础矩阵
			{
				vector<Point2f> pt1, pt2;
				for(int c=0; c<vecMatch1.size(); c++)
				{
					Point2f temp1, temp2;
					temp1.x = vecMatch1[c].x; temp1.y = vecMatch1[c].y;
					temp2.x = vecMatch2[c].x; temp2.y = vecMatch2[c].y;
					pt1.push_back(temp1);
					pt2.push_back(temp2);
				}			
				
				F = findFundamentalMat(pt1, pt2, CV_FM_RANSAC, 1, 0.99, mask);
				
				int depth = mask.depth();
				int nInliers = 0;
				for(int c=0; c<vecMatch1.size(); c++)
				{
					if(mask.at<byte>(c,0)>0)
					{
						nInnerPoints++;
						mosaicHarris.m_vecInnerPoints1.push_back(vecMatch1[c]);
						mosaicHarris.m_vecInnerPoints2.push_back(vecMatch2[c]);
					}
				}
			}

			if(nInnerPoints>MIN_INNER_POINTS)
			{
				cout<<"img "<<i<<" / "<<j<<" inner "<<mosaicHarris.m_vecInnerPoints1.size()<<" / "<<vecMatch1.size()<<" matched surf"<<endl;

				//CheckFundamentalMatrix(mosaicHarris.m_vecInnerPoints1, mosaicHarris.m_vecInnerPoints2, F);

				// 匹配成功
				for(int p=0; p<nInnerPoints; p++)
				{
					MatchPointPairs temp;
					temp.ptA = mosaicHarris.m_vecInnerPoints1[p];
					temp.ptA_Fixed = pImgPoses[i].fixed;
					temp.ptA_i = i;

					temp.ptB = mosaicHarris.m_vecInnerPoints2[p];
					temp.ptB_Fixed = pImgPoses[j].fixed;
					temp.ptB_i = j;
					vecMatchPairs.push_back(temp);
				}
				nSuccess++;
			}
			else
			{
				//cout<<"img "<<i<<" / "<<j<<" inner "<<nInnerPoints<<" / "<<vecMatch1.size()<<" matched surf"<<endl;
				// 匹配失败
				failed = true;
			}
			mosaicHarris.m_vecInnerPoints1.clear();
			mosaicHarris.m_vecInnerPoints2.clear();
			vecMatch1.clear();
			vecMatch2.clear();
		}
	}
	cout<<endl;

	return 0;
}



// 单航带SURF图像特征点对应
// 帧间匹配
int CMosaicByPose::GetMatchedPairsSingleStripSurf(const ImagePoseInfo *pImgPoses, const int nImages, 
												 vector<MatchPointPairs> &vecMatchPairs,
												 int &nSuccess)
{
	cout<<"特征提取和匹配"<<endl;
	vector<IntPoint> vecMatched;
	const int MIN_INNER_POINTS = 15;
	int maxFeatruesNum = m_maxFeatureNum;
	float ransacDist = m_ransacDist;
	float distT = m_matchDist;
	int minHessian = m_minHessian;

	SurfFeatureDetector detector( minHessian );
	SurfDescriptorExtractor extractor;

	// 特征提取1
	std::vector<KeyPoint> keyPoints1;
	// 特征提取2
	std::vector<KeyPoint> keyPoints2;
	vector<SfPoint> vecMatch1, vecMatch2;
	Mat descriptors1;
	Mat descriptors2;
	bool failed = false;
	nSuccess = 1;
	for(int i=0; i<nImages; i++)
	{
		//cout<<"img "<<i<<endl;
		if(i==0)
		{
			detector.detect( pImgPoses[0].pImg, keyPoints1 );
			// 特征描述1
			extractor.compute(pImgPoses[0].pImg, keyPoints1, descriptors1);
		}
		if(i>0)
		{
			detector.detect( pImgPoses[i].pImg, keyPoints2 );
			// 特征描述2
			extractor.compute(pImgPoses[i].pImg, keyPoints2, descriptors2);

			if( (keyPoints1.size()<=0) || (keyPoints2.size()<=0) )
			{
				failed = 0;
				break;
			}

			// 特征匹配
			FlannBasedMatcher matcher;
			std::vector< DMatch > matches;
			matcher.match(descriptors1, descriptors2, matches);
			//matcher.knnMatch
			distT = m_matchDist;
			double dT = 0.05;
			do
			{
				vecMatch1.clear();
				vecMatch2.clear();
				for(int n=0; n<(int)matches.size(); n++)
				{
					if(matches[n].distance<distT)
					{
						SfPoint temp1;
						temp1.x = keyPoints1[matches[n].queryIdx].pt.x;
						temp1.y = keyPoints1[matches[n].queryIdx].pt.y;
						temp1.id = matches[n].queryIdx;
						vecMatch1.push_back(temp1);
						SfPoint temp2;
						temp2.x = keyPoints2[matches[n].trainIdx].pt.x;
						temp2.y = keyPoints2[matches[n].trainIdx].pt.y;
						temp2.id = matches[n].trainIdx;
						vecMatch2.push_back(temp2);
					}
				}
				distT -= dT;
				//cout<<vecMatch2.size()<<endl;
			}
			while((int)vecMatch2.size()>maxFeatruesNum);

			BitmapImage* pSrc1 = new BitmapImage();
			BitmapImage* pSrc2 = new BitmapImage();
			CvImage2BitmapImage(pImgPoses[i-1].pImg, pSrc1);
			CvImage2BitmapImage(pImgPoses[i].pImg, pSrc2);

#ifdef _DEBUG
			//ShowMatch(pSrc1, pSrc2, &vecMatch1[0], &vecMatch2[0], vecMatch1.size(), _T("c:\\matchSurf.bmp")); 
#endif
			//cout<<"matched surf "<<vecMatch1.size()<<endl;

			float homo[9];
			CMosaicHarris mosaicHarris;
			int nInnerPoints = 0;
			mosaicHarris.Ransac(pSrc1, pSrc2, vecMatch1, vecMatch2, homo, ransacDist);
			nInnerPoints = (int)mosaicHarris.m_vecInnerPoints1.size();
			if(nInnerPoints<=MIN_INNER_POINTS) // 如果ransac失败, 重试一次
			{
				mosaicHarris.Ransac(pSrc1, pSrc2, vecMatch1, vecMatch2, homo, ransacDist);
				nInnerPoints = (int)mosaicHarris.m_vecInnerPoints1.size();
			}

			delete pSrc1; pSrc1 = NULL;
			delete pSrc2; pSrc2 = NULL;

			//Mat img_matches;
			//drawMatches( pImgPoses[i].pImg, keyPoints1, pImgPoses[j].pImg, keyPoints2, matches, img_matches );
			////-- Show detected matches
			//imshow("Matches", img_matches );
			//waitKey(0);

			cout<<"img "<<i<<" / "<<nImages<<" inner "<<nInnerPoints<<" / "<<vecMatch1.size()<<" matched surf"<<endl;

			if(nInnerPoints>MIN_INNER_POINTS)
			{
				// 匹配成功
				for(int p=0; p<nInnerPoints; p++)
				{
					MatchPointPairs temp;
					temp.ptA = mosaicHarris.m_vecInnerPoints1[p];
					temp.ptA_Fixed = pImgPoses[i-1].fixed;
					temp.ptA_i = i-1;

					temp.ptB = mosaicHarris.m_vecInnerPoints2[p];
					temp.ptB_Fixed = pImgPoses[i].fixed;
					temp.ptB_i = i;
					vecMatchPairs.push_back(temp);
				}
				//cout<<"i "<<i<<" j "<<j<<endl;
				//IntPoint temp;
				//temp.x = i;
				//temp.y = j;
				//vecMatched.push_back(temp);
				nSuccess++;
			}
			else
			{
				// 匹配失败
				failed = true;
			}
			
			// feature2 -> feature1
			keyPoints1.clear();
			keyPoints1.insert(keyPoints1.begin(), keyPoints2.begin(), keyPoints2.end());
			keyPoints2.clear();
			descriptors1.release();
			//descriptors1.
			descriptors2.copyTo(descriptors1);
			descriptors2.release();
			if(failed)
			{
				break;
			}
		}
	}

	// 多帧跟踪
	TrackMultiFrames(vecMatchPairs, nSuccess);
	
	if(failed)
		return -2; // 匹配失败

	return 0;
}

// 多帧特征跟踪
int TrackMultiFrames(vector<MatchPointPairs> &matched, int nImages)
{
	int nImages_1 = nImages - 1;
	int nMatched = matched.size();
	if(nMatched<=1)
	{
		return 0;
	}
	vector<MatchPointPairs> tracked;
	vector<unsigned long> featuresNumFrame(nImages);
	vector<unsigned long> accNum(nImages);
	memset(&featuresNumFrame[0], 0, sizeof(int)*(nImages));
	for(int n=0; n<nMatched; n++)
	{
		int index = matched[n].ptB_i;
		featuresNumFrame[index]++;
	}
	accNum[0] = 0;
	for(int n=1; n<nImages; n++)
	{
		accNum[n] = accNum[n-1] + featuresNumFrame[n];
	}

	for(int n=1; n<nImages; n++)
	{
		int nTracked = 0;
		
		
		for(unsigned int p=accNum[n-1]; p<accNum[n]; p++)
		{
			bool newTrack = true;
			bool continueTrack = true;
			int idTracked = matched[p].ptB.id; // 初始tracking-id

			int i = 0;
			while( (continueTrack || newTrack) && (i<nImages) )
			{
				newTrack = false;
				for(i=n+1; i<nImages; i++)
				{
					continueTrack = false;
					for(unsigned int q=accNum[i-1]; q<accNum[i]; q++)
					{
						if(idTracked==matched[q].ptA.id)
						{
							MatchPointPairs temp;
							temp.ptA		= matched[p].ptA;
							temp.ptA_Fixed	= matched[p].ptA_Fixed;
							temp.ptA_i		= matched[p].ptA_i;

							temp.ptB		= matched[q].ptB;
							temp.ptB_Fixed	= matched[q].ptB_Fixed;
							temp.ptB_i		= matched[q].ptB_i;
							tracked.push_back(temp);
							//nTracked++;

							continueTrack = true;
							idTracked = temp.ptB.id;
							break;
						}
					}
					if(continueTrack==false)
					{
						break;
					}
				}
			}

			//if(0==nTracked)
			//{
			//	break;
			//}
			//break;
		}		
	}

	int nTrackedTotal = (int)tracked.size();
	for(int i=0; i<nTrackedTotal; i++)
	{
		matched.push_back(tracked[i]);
	}
	cout<<"nTrackedTotal "<<nTrackedTotal<<endl;
	
	return 0;
}

// 多航带图像特征点对应
int CMosaicByPose::GetMatchedPairsMultiStrip(const ImagePoseInfo *pImgPoses, const int nImages, 
											 vector<MatchPointPairs> &vecMatchPairs, int &nFixedImages)
{
	//vector<IntPoint> vecMatched;
	//CMosaicHarris mosaicHarris;
	//float minScore = 0.6f;
	//int maxFeatruesNum = 400;
	//int winR = 13;
	//float ransacDist = 2;
	//int searchR = -1;
	//mosaicHarris.RansacDist(ransacDist);
	//mosaicHarris.m_minScore = minScore;
	//mosaicHarris.m_winRadius = winR;

	//const int MIN_INNER_POINTS = 10;

	//pool::SfPoint* pCorner1 = NULL, * pCorner2 = NULL;
	//CornerDescriptor* pCornerDescriptor1 = NULL, *pCornerDescriptor2 = NULL;
	//int nCorner1 = 0, nCorner2 = 0;		
	//pCorner1 = new SfPoint[maxFeatruesNum];
	//pCorner2 = new SfPoint[maxFeatruesNum];

	//pCornerDescriptor1 = new CornerDescriptor[maxFeatruesNum];
	//pCornerDescriptor2 = new CornerDescriptor[maxFeatruesNum];

	//int minGradient = 10;
	//int byBlock = 1;

	//BitmapImage* pSrc1 = new BitmapImage();
	//BitmapImage* pSrc2 = new BitmapImage();
	//cout<<"nImages "<<nImages<<endl;
	//for(int i=0; i<nImages; i++)
	//{
	//	BitmapImage* pGray1 = NULL;
	//	CvImage2BitmapImage(pImgPoses[i].pImg, pSrc1);
	//	// 特征检测
	//	float fMinResponse = 1e2f;
	//	// 先变成灰度图
	//	int w1 = pSrc1->width, h1 = pSrc1->height;
	//	pGray1 = CreateBitmap8U( w1, h1, 1 );
	//	ColorToGray1(pSrc1, pGray1);
	//	// 特征提取1
	//	HarrisCorner( pGray1, 
	//		pCorner1, nCorner1, maxFeatruesNum, fMinResponse, byBlock, minGradient );
	//	// 特征描述1
	//	DescriptorForHarris(pGray1, winR, pCorner1, nCorner1, pCornerDescriptor1);

	//	for(int j=i+1; j<nImages; j++)
	//	{
	//		if( (i==10)&&(j==13) )
	//		{
	//			i = i;
	//		}
	//		//cout<<"i "<<i<<" j "<<j<<endl;
	//		// 计算每幅图像的物理宽度
	//		double hCam = pImgPoses[i].camPose.pos.z;
	//		double wPhy = pImgPoses[i].pImg->width* hCam/ m_inParams.fx;
	//		double hPhy = pImgPoses[i].pImg->height* hCam/ m_inParams.fx;
	//		double imgWPhy = max(wPhy, hPhy); // 图像的物理宽度

	//		// 计算当前图像的主点到上一张图像的主点之间的距离
	//		double distOfTwoImages;
	//		DistanceOfTwoPoints(vecRectified[j].imgCxPys, vecRectified[j].imgCyPys, 
	//			vecRectified[i].imgCxPys, vecRectified[i].imgCyPys, distOfTwoImages);
	//		if(distOfTwoImages>1.0*imgWPhy)
	//		{
	//			continue;
	//		}

	//		CvImage2BitmapImage(pImgPoses[j].pImg, pSrc2);
	//		// 先变成灰度图
	//		int w2 = pSrc2->width, h2 = pSrc2->height;
	//		BitmapImage* pGray2 = CreateBitmap8U( w2, h2, 1 );
	//		ColorToGray1(pSrc2, pGray2);

	//		// 特征提取2
	//		HarrisCorner( pGray2, 
	//			pCorner2, nCorner2, maxFeatruesNum, fMinResponse, byBlock, minGradient );
	//		// 特征描述2
	//		DescriptorForHarris(pGray2, winR, pCorner2, nCorner2, pCornerDescriptor2);

	//		// 特征匹配
	//		float aProjectMat[9];
	//		int mosaicTwoState = -1;
	//		int nInnerPoints = 0;
	//		mosaicTwoState = mosaicHarris.MosaicTwoImages(pGray1, pGray2, aProjectMat,
	//			maxFeatruesNum, winR, 
	//			minScore, 
	//			ransacDist, 
	//			searchR, pCornerDescriptor1, pCornerDescriptor2, nCorner1, nCorner2 );
	//		nInnerPoints = (int)mosaicHarris.m_vecInnerPoints1.size();
	//		if(nInnerPoints<=MIN_INNER_POINTS)
	//		{
	//			//mosaicTwoState = mosaicHarris.MosaicTwoImages(pGray1, pGray2, aProjectMat,
	//			//	maxFeatruesNum, winR, 
	//			//	minScore, 
	//			//	ransacDist, 
	//			//	searchR, pCornerDescriptor1, pCornerDescriptor2, nCorner1, nCorner2 );
	//			//nInnerPoints = (int)mosaicHarris.m_vecInnerPoints1.size();
	//		}
	//		if(nInnerPoints>MIN_INNER_POINTS)
	//		{
	//			// 匹配成功
	//			for(int p=0; p<nInnerPoints; p++)
	//			{
	//				MatchPointPairs temp;
	//				temp.ptA = mosaicHarris.m_vecInnerPoints1[p];
	//				temp.ptA_Fixed = pImgPoses[i].fixed;
	//				temp.ptA_i = i;

	//				temp.ptB = mosaicHarris.m_vecInnerPoints2[p];
	//				temp.ptB_Fixed = pImgPoses[j].fixed;
	//				temp.ptB_i = j;
	//				vecMatchPairs.push_back(temp);
	//			}
	//			cout<<"i "<<i<<" j "<<j<<endl;
	//			IntPoint temp;
	//			temp.x = i;
	//			temp.y = j;
	//			vecMatched.push_back(temp);
	//		}
	//		ReleaseBitmap8U(pGray2);
	//	}
	//	ReleaseBitmap8U(pGray1);
	//}

	//for(int i=0; i<(int)vecMatched.size(); i++)
	//{
	//	cout<<vecMatched[i].x<<" "<<vecMatched[i].y<<endl;
	//}

	//delete pSrc1; pSrc1 = NULL;
	//delete pSrc2; pSrc2 = NULL;

	//delete[] pCorner1; pCorner1 = NULL;
	//delete[] pCorner2; pCorner2 = NULL;

	//delete[] pCornerDescriptor1; pCornerDescriptor1 = NULL;
	//delete[] pCornerDescriptor2; pCornerDescriptor2 = NULL;

	return 0;
}

// 单航带图像特征点对应
int CMosaicByPose::GetMatchedPairsSingleStrip(const ImagePoseInfo *pImgPoses, const int nImages, 
											  vector<MatchPointPairs> &vecMatchPairs, int &nFixedImages)
{
	
	// 特征提取和匹配, 得到匹配的特征对
	for(int n=0; n<nImages; n++)
	{
		//cout<<"pImgPoses[n].fixed "<<n<<" "<<pImgPoses[n].fixed<<endl;
		if(pImgPoses[n].fixed)
		{
			nFixedImages++;
		}

		IplImage* pMosaicResult = NULL;
		mosaic.Mosaic(pImgPoses[n].pImg, pMosaicResult, false);
		if(n>0)
		{
			int nInner = 0;
			SfPoint* pInner1 = NULL, *pInner2 = NULL; // pInner2 is new
			GetInnerPoints(pInner1, pInner2, nInner);
			for(int i=0; i<nInner; i++)
			{
				MatchPointPairs temp;
				temp.ptA = pInner1[i];
				temp.ptA_Fixed = pImgPoses[n-1].fixed;
				temp.ptA_i = n-1;

				temp.ptB = pInner2[i];
				temp.ptB_Fixed = pImgPoses[n].fixed;
				temp.ptB_i = n;
				vecMatchPairs.push_back(temp);
			}			
		}
	}
	return 0;
}

int CMosaicByPose::AdjustmentByImage(const ImagePoseInfo *pImgPoses, const int nImages, ImageTransform* pTransformInit,
									 vector<ImageTransform> &vecTransformRefined)
{
	int nFixedImages = 0;
	for(int n=0; n<nImages; n++)
	{
		if(pImgPoses[n].fixed==1)
		{
			nFixedImages++;
		}
	}

	cout<<"Feature Start"<<endl;

	vector<MatchPointPairs> vecMatchPairs;
	//GetMatchedPairsSingleStrip(pImgPoses, nImages, vecMatchPairs, nFixedImages);
	
	GetMatchedPairsMultiStrip(pImgPoses, nImages, vecMatchPairs, nFixedImages);
	//GetMatchedPairsMultiStripSurf(pImgPoses, nImages, vecMatchPairs, nFixedImages);

	cout<<"nFixedImages "<<nFixedImages<<endl<<endl;

	// 捆集调整（仿射模型）
	vector<ImageTransform> vecTransformAffineRefined;
	//BundleAdjustment(&vecMatchPairs[0], vecMatchPairs.size(), 
	//				pTransformInit, nImages, nFixedImages, vecTransformAffineRefined);

	vecTransformAffineRefined.clear();
	BundleAdjustmentSparse(&vecMatchPairs[0], vecMatchPairs.size(), 
		pTransformInit, nImages, nFixedImages, vecTransformAffineRefined); // BundleAdjustmentSparse

	//QuadraticProgram(pImgPoses, &vecMatchPairs[0], vecMatchPairs.size(), 
	//	pTransformInit, nImages, nFixedImages, vecTransformAffineRefined);

	//BundleAdjustmentLagrange(pImgPoses, &vecMatchPairs[0], vecMatchPairs.size(), 
	//						pTransformInit, nImages, nFixedImages, vecTransformAffineRefined);

	//BundleAdjustmentLS(pImgPoses, &vecMatchPairs[0], vecMatchPairs.size(), 
	//				pTransformInit, nImages, nFixedImages, vecTransformAffineRefined);

	bool nonlinearAdjustment = false; // false
	if(nonlinearAdjustment)
	{
		// 非线性迭代（透视模型）
		BundleAdjustmentNonlinear(&vecMatchPairs[0], vecMatchPairs.size(), 
								&vecTransformAffineRefined[0], nImages, nFixedImages, 
								vecTransformRefined);
	}
	else
	{
		for(int i=0; i<(int)vecTransformAffineRefined.size(); i++)
		{
			vecTransformRefined.push_back(vecTransformAffineRefined[i]);
		}
	}
	
	return 0;
}

// 带约束的线性捆集调整
// AX = B, 用仿射变换模型
// x2 = a.x1 + b.y1 + e
// y2 = c.x1 + d.y1 + f
int CMosaicByPose::BundleAdjustmentSparseConstraint(MatchPointPairs* pMatchPairs, int nPairs,
										  const ImageTransform* pImagesTransform0, int nImages, 
										  int nFixedImages,
										  vector<ImageTransform> &vecTransformRefined)
{	
	cout<<"Bundle Adjustment Start 0000"<<endl<<endl;

	if(nImages<=1)
	{
		return -2;
	}

	vector<int> freqMatch(nImages);
	memset(&freqMatch[0], 0, sizeof(int)*nImages);

	for(int n=0; n<nPairs; n++)
	{
		if(pMatchPairs[n].ptA_Fixed==0)
		{
			freqMatch[pMatchPairs[n].ptA_i]++;
		}
		if(pMatchPairs[n].ptB_Fixed==0)
		{
			freqMatch[pMatchPairs[n].ptB_i]++;
		}
	}
	int nConstraint = 0;
	for(int n=0; n<nImages; n++)
	{
		freqMatch[n] /= 2;
		nConstraint += freqMatch[n];
	}

	vector<int> vecAccFixed(nImages);
	memset(&vecAccFixed[0], 0, sizeof(int)*nImages);

	// 处理固定的图像
	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<i; j++)
		{
			if(pImagesTransform0[j].fixed==1)
			{
				vecAccFixed[i] += 1;
			}
		}
	}

	int rowA = 2 * nPairs + nConstraint * 2;
	int colA = 6 * (nImages-nFixedImages);
	int rowX = 6 * (nImages-nFixedImages);
	int colX = 1;
	int rowB = rowA;
	int colB = 1;

	cout<<"rowA "<<rowA<<"colA "<<colA<<endl;
	cout<<"nImages "<<nImages<<" nPairs "<<nPairs<<endl;

	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);

	pool::SparseMatrix A;
	pool::SparseMatrix B;
	A.mat.resize(colA);
	B.mat.resize(colB);
	A.col = colA;
	A.row = rowA;
	B.col = colB;
	B.row = rowB;

	cout<<"赋值PA, PB"<<endl;

	for(int n=0; n<nPairs; n++)
	{
		//cout<<n<<" / "<<nPairs<<endl;
		if(n==47824)
		{
			n=n;
		}
		if( (pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==0) ) // 如果A点和B点都不是固定点
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号
			// 2*n行, 6*i列
			// ptA

			//cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
			A.mat[6*ptA_i + 0].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptA.x));
			//cvSetReal2D(pA, 2*n, 6*ptA_i + 1, pMatchPairs[n].ptA.y);
			A.mat[6*ptA_i + 1].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptA.y));
			//cvSetReal2D(pA, 2*n, 6*ptA_i + 4, 1);
			A.mat[6*ptA_i + 4].matCol.push_back(SparseMatElem(2*n, 1));
			//A.mat[].matCol.push_back(SparseMatElem(, ));

			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 2, pMatchPairs[n].ptA.x);
			A.mat[6*ptA_i + 2].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptA.x));
			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 3, pMatchPairs[n].ptA.y);
			A.mat[6*ptA_i + 3].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptA.y));
			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 5, 1);
			A.mat[6*ptA_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1));

			//// ptB
			//cvSetReal2D(pA, 2*n, 6*ptB_i + 0, -pMatchPairs[n].ptB.x);
			A.mat[6*ptB_i + 0].matCol.push_back(SparseMatElem(2*n, -pMatchPairs[n].ptB.x));
			//cvSetReal2D(pA, 2*n, 6*ptB_i + 1, -pMatchPairs[n].ptB.y);
			A.mat[6*ptB_i + 1].matCol.push_back(SparseMatElem(2*n, -pMatchPairs[n].ptB.y));
			//cvSetReal2D(pA, 2*n, 6*ptB_i + 4, -1);
			A.mat[6*ptB_i + 4].matCol.push_back(SparseMatElem(2*n, -1));

			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 2, -pMatchPairs[n].ptB.x);
			A.mat[6*ptB_i + 2].matCol.push_back(SparseMatElem(2*n+1, -pMatchPairs[n].ptB.x));
			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 3, -pMatchPairs[n].ptB.y);
			A.mat[6*ptB_i + 3].matCol.push_back(SparseMatElem(2*n+1, -pMatchPairs[n].ptB.y));
			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 5, -1);
			A.mat[6*ptB_i + 5].matCol.push_back(SparseMatElem(2*n+1, -1));

			//// 给B赋值
			//cvSetReal2D(pB, 2*n, 0, 0);
			//B.mat[0].matCol.push_back(SparseMatElem(2*n, 0));
			//cvSetReal2D(pB, 2*n+1, 0, 0);
			//B.mat[0].matCol.push_back(SparseMatElem(2*n+1, 0));
		}
		else if((pMatchPairs[n].ptA_Fixed==1) && (pMatchPairs[n].ptB_Fixed==0)) // 如果A点固定
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

			//// ptB
			//cvSetReal2D(pA, 2*n, 6*ptB_i + 0, pMatchPairs[n].ptB.x);
			A.mat[6*ptB_i + 0].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptB.x));
			//cvSetReal2D(pA, 2*n, 6*ptB_i + 1, pMatchPairs[n].ptB.y);
			A.mat[6*ptB_i + 1].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptB.y));
			//cvSetReal2D(pA, 2*n, 6*ptB_i + 4, 1);
			A.mat[6*ptB_i + 4].matCol.push_back(SparseMatElem(2*n, 1));

			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 2, pMatchPairs[n].ptB.x);
			A.mat[6*ptB_i + 2].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptB.x));
			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 3, pMatchPairs[n].ptB.y);
			A.mat[6*ptB_i + 3].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptB.y));
			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 5, 1);
			A.mat[6*ptB_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1));

			// 用固定的参数对A点进行变换
			DfPoint ptA_Bar;
			double h64F[9];
			for(int t=0; t<9; t++)
			{
				h64F[t] = pImagesTransform0[pMatchPairs[n].ptA_i].h.m[t];
			}
			ApplyProject9(h64F, pMatchPairs[n].ptA.x, pMatchPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);

			//// 给B赋值
			//cvSetReal2D(pB, 2*n, 0, ptA_Bar.x);
			B.mat[0].matCol.push_back(SparseMatElem(2*n, ptA_Bar.x));
			//cvSetReal2D(pB, 2*n+1, 0, ptA_Bar.y);
			B.mat[0].matCol.push_back(SparseMatElem(2*n+1, ptA_Bar.y));
		}
		else if((pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==1)) // 如果B点固定
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

			// 2*n行, 6*i列
			// ptA
			//cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
			A.mat[6*ptA_i + 0].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptA.x));
			//cvSetReal2D(pA, 2*n, 6*ptA_i + 1, pMatchPairs[n].ptA.y);
			A.mat[6*ptA_i + 1].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptA.y));
			//cvSetReal2D(pA, 2*n, 6*ptA_i + 4, 1);
			A.mat[6*ptA_i + 4].matCol.push_back(SparseMatElem(2*n, 1));

			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 2, pMatchPairs[n].ptA.x);
			A.mat[6*ptA_i + 2].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptA.x));
			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 3, pMatchPairs[n].ptA.y);
			A.mat[6*ptA_i + 3].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptA.y));
			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 5, 1);
			A.mat[6*ptA_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1));

			// 用固定的参数对B点进行变换
			DfPoint ptB_Bar;
			double hB64F[9];
			for(int t=0; t<9; t++)
			{
				hB64F[t] = pImagesTransform0[pMatchPairs[n].ptB_i].h.m[t];
			}
			ApplyProject9(hB64F, pMatchPairs[n].ptB.x, pMatchPairs[n].ptB.y, ptB_Bar.x, ptB_Bar.y);
			// 给B赋值
			//cvSetReal2D(pB, 2*n, 0, ptB_Bar.x);
			B.mat[0].matCol.push_back(SparseMatElem(2*n, ptB_Bar.x));
			//cvSetReal2D(pB, 2*n+1, 0, ptB_Bar.y);
			B.mat[0].matCol.push_back(SparseMatElem(2*n+1, ptB_Bar.y));
		}
	}
	// 向A中加入约束条件
	// a = d, b = -c
	for(int n=0; n<nImages; n++)
	{
		int cst_i = n - vecAccFixed[n]; 
		int nC = freqMatch[n]; // 当前图像的约束的个数
		for(int i=0; i<nC; i++)
		{
			//cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
			A.mat[6*cst_i + 0].matCol.push_back(SparseMatElem(2*(n+nPairs), 1));
			A.mat[6*cst_i + 3].matCol.push_back(SparseMatElem(2*(n+nPairs), -1));

			A.mat[6*cst_i + 1].matCol.push_back(SparseMatElem(2*(n+nPairs)+1, 1));
			A.mat[6*cst_i + 2].matCol.push_back(SparseMatElem(2*(n+nPairs)+1, 1));
		}
	}

	//CvMat* pATA = cvCreateMat(colA, colA, CV_64F);
	//CvMat* pAT = cvCreateMat(colA, rowA, CV_64F);
	//cvTranspose(pA, pAT);
	//cvGEMM(pAT, pA, 1, NULL, 0, pATA);
	//cvPrintMatrix(pATA, "c:\\ATA.txt");
	//cvReleaseMat(&pATA);
	//cvReleaseMat(&pAT);

	// CV解线性方程组
	cout<<"解线性方程组"<<endl<<endl;
	//cvSolve(pA, pB, pX, CV_SVD);
	//cvPrintMatrix(pX, "c:\\x-cv.txt");

	//SolveSparseSystem(pA, pB, pX);
	//
	for(int col=0; col<A.col; col++)
	{
		std::sort(A.mat[col].matCol.begin(), A.mat[col].matCol.end());
	}

	SolveSparseSystem2(A, B, pX);

	//cvPrintMatrix(pX, "c:\\x-cholmod.txt");

	int nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(pImagesTransform0[i].fixed==0) // 未固定参数情况
		{
			ImageTransform temp;
			temp.fixed = 0;
			temp.h.m[0] = (float)cvGetReal2D(pX, 6*nUnFixed+0, 0); // a
			temp.h.m[1] = (float)cvGetReal2D(pX, 6*nUnFixed+1, 0); // b
			temp.h.m[3] = (float)cvGetReal2D(pX, 6*nUnFixed+2, 0); // c
			temp.h.m[4] = (float)cvGetReal2D(pX, 6*nUnFixed+3, 0); // d
			temp.h.m[2] = (float)cvGetReal2D(pX, 6*nUnFixed+4, 0); // e
			temp.h.m[5] = (float)cvGetReal2D(pX, 6*nUnFixed+5, 0); // f

			nUnFixed++;

			temp.h.m[6] = 0;
			temp.h.m[7] = 0;
			temp.h.m[8] = 1;
			vecTransformRefined.push_back(temp);
		}
		else
		{
			vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
		}
	}

	cvReleaseMat(&pX);

	return 0;
}

// 带约束的线性捆集调整
// AX = B, 用仿射变换模型
// x2 = a.x1 + b.y1 + e
// y2 = c.x1 + d.y1 + f
// rot约束
int CMosaicByPose::SparseAffineRotConstraint(MatchPointPairs* pMatchPairs, int nPairs,
													const ImageTransform* pImagesTransform0, int nImages, 
													int nFixedImages, float weight, 
													vector<ImageTransform> &vecTransformRefined)
{	
	cout<<"Bundle Adjustment Start 0000"<<endl<<endl;

	if(nImages<=1)
	{
		return -2;
	}

	vector<int> freqMatch(nImages); // 某幅图像中匹配对出现的频率
	memset(&freqMatch[0], 0, sizeof(int)*nImages);

	for(int n=0; n<nPairs; n++)
	{
		if(pMatchPairs[n].ptA_Fixed==0)
		{
			freqMatch[pMatchPairs[n].ptA_i]++;
		}
		if(pMatchPairs[n].ptB_Fixed==0)
		{
			freqMatch[pMatchPairs[n].ptB_i]++;
		}
	}
	//int nConstraint = 0;
	for(int n=0; n<nImages; n++)
	{
		freqMatch[n] *= weight;
		//nConstraint += freqMatch[n];
	}

	vector<int> vecAccFixed(nImages);
	memset(&vecAccFixed[0], 0, sizeof(int)*nImages);

	// 处理固定的图像
	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<i; j++)
		{
			if(pImagesTransform0[j].fixed==1)
			{
				vecAccFixed[i] += 1;
			}
		}
	}

	int rowA = 2 * nPairs + 3 * (nImages-nFixedImages);
	int colA = 6 * (nImages-nFixedImages);
	int rowX = 6 * (nImages-nFixedImages);
	int colX = 1;
	int rowB = rowA;
	int colB = 1;

	pool::SparseMatrix eye;
	eye.mat.resize(colA);
	eye.col = colA;
	eye.row = colA;
	for(int i=0; i<colA; i++)
	{
		eye.mat[i].matCol.push_back(SparseMatElem(i, 1)); // 列 行
	}

	cout<<"rowA "<<rowA<<"colA "<<colA<<endl;
	cout<<"nImages "<<nImages<<" nPairs "<<nPairs<<endl;

	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pX2 = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pDX = cvCreateMat(rowX, colX, CV_64F);

	// 获取pX初值
	BundleAdjustmentSparseConstraint(pMatchPairs, nPairs,
		pImagesTransform0, nImages, 
		nFixedImages,
		vecTransformRefined);

	int nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(pImagesTransform0[i].fixed==0) // 未固定参数情况
		{
			cvSetReal2D(pX, 6*nUnFixed+0, 0, vecTransformRefined[i].h.m[0]); // a
			cvSetReal2D(pX, 6*nUnFixed+1, 0, vecTransformRefined[i].h.m[1]); // b
			cvSetReal2D(pX, 6*nUnFixed+2, 0, vecTransformRefined[i].h.m[3]); // c
			cvSetReal2D(pX, 6*nUnFixed+3, 0, vecTransformRefined[i].h.m[4]); // d
			cvSetReal2D(pX, 6*nUnFixed+4, 0, vecTransformRefined[i].h.m[2]); // e
			cvSetReal2D(pX, 6*nUnFixed+5, 0, vecTransformRefined[i].h.m[5]); // f

			nUnFixed++;
		}
	}

	vector<double> vecCostVal, vecCostVal2;

	double lambda64F = 0.01;
	double beta = 10;
	const int MAX_ITER = 10;
	int iter = 0;
	while(iter<MAX_ITER)
	{
		double cost = 0;

		pool::SparseMatrix A;
		pool::SparseMatrix B;
		A.mat.resize(colA);
		B.mat.resize(colB);
		A.col = colA;
		A.row = rowA;
		B.col = colB;
		B.row = rowB;

		cout<<"赋值PA, PB"<<endl;

		for(int n=0; n<nPairs; n++)
		{
			//cout<<n<<" / "<<nPairs<<endl;
			if(n==10000)
			{
				n=n;
			}
			if( (pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==0) ) // 如果A点和B点都不是固定点
			{
				int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
				int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号
				// 2*n行, 6*i列
				// ptA

				A.mat[6*ptA_i + 0].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptA.x));
				A.mat[6*ptA_i + 1].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptA.y));
				A.mat[6*ptA_i + 4].matCol.push_back(SparseMatElem(2*n, 1));

				A.mat[6*ptA_i + 2].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptA.x));
				A.mat[6*ptA_i + 3].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptA.y));
				A.mat[6*ptA_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1));

				//// ptB
				//cvSetReal2D(pA, 2*n, 6*ptB_i + 0, -pMatchPairs[n].ptB.x);
				A.mat[6*ptB_i + 0].matCol.push_back(SparseMatElem(2*n, -pMatchPairs[n].ptB.x));
				//cvSetReal2D(pA, 2*n, 6*ptB_i + 1, -pMatchPairs[n].ptB.y);
				A.mat[6*ptB_i + 1].matCol.push_back(SparseMatElem(2*n, -pMatchPairs[n].ptB.y));
				//cvSetReal2D(pA, 2*n, 6*ptB_i + 4, -1);
				A.mat[6*ptB_i + 4].matCol.push_back(SparseMatElem(2*n, -1));

				//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 2, -pMatchPairs[n].ptB.x);
				A.mat[6*ptB_i + 2].matCol.push_back(SparseMatElem(2*n+1, -pMatchPairs[n].ptB.x));
				//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 3, -pMatchPairs[n].ptB.y);
				A.mat[6*ptB_i + 3].matCol.push_back(SparseMatElem(2*n+1, -pMatchPairs[n].ptB.y));
				//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 5, -1);
				A.mat[6*ptB_i + 5].matCol.push_back(SparseMatElem(2*n+1, -1));

				//// 给B赋值
				//cvSetReal2D(pB, 2*n, 0, 0);
				double aA = cvGetReal2D(pX, 6*ptA_i+0, 0); // a
				double bA = cvGetReal2D(pX, 6*ptA_i+1, 0); // b
				double cA = cvGetReal2D(pX, 6*ptA_i+2, 0); // c
				double dA = cvGetReal2D(pX, 6*ptA_i+3, 0); // d
				double eA = cvGetReal2D(pX, 6*ptA_i+4, 0); // e
				double fA = cvGetReal2D(pX, 6*ptA_i+5, 0); // f

				double aB = cvGetReal2D(pX, 6*ptB_i+0, 0); // a
				double bB = cvGetReal2D(pX, 6*ptB_i+1, 0); // b
				double cB = cvGetReal2D(pX, 6*ptB_i+2, 0); // c
				double dB = cvGetReal2D(pX, 6*ptB_i+3, 0); // d
				double eB = cvGetReal2D(pX, 6*ptB_i+4, 0); // e
				double fB = cvGetReal2D(pX, 6*ptB_i+5, 0); // f

				double xA = pMatchPairs[n].ptA.x;
				double yA = pMatchPairs[n].ptA.y;

				double xB = pMatchPairs[n].ptB.x;
				double yB = pMatchPairs[n].ptB.y;

				double f1 = aA*xA + bA*yA + eA - (aB*xB + bB*yB + eB);
				double f2 = cA*xA + dA*yA + fA - (cB*xB + dB*yB + fB);

				B.mat[0].matCol.push_back(SparseMatElem(2*n, f1));
				B.mat[0].matCol.push_back(SparseMatElem(2*n+1, f2));

				cost += f1*f1;
				cost += f2*f2;
			}
			else if((pMatchPairs[n].ptA_Fixed==1) && (pMatchPairs[n].ptB_Fixed==0)) // 如果A点固定
			{
				//int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
				int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

				//// ptB
				//cvSetReal2D(pA, 2*n, 6*ptB_i + 0, pMatchPairs[n].ptB.x);
				A.mat[6*ptB_i + 0].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptB.x));
				//cvSetReal2D(pA, 2*n, 6*ptB_i + 1, pMatchPairs[n].ptB.y);
				A.mat[6*ptB_i + 1].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptB.y));
				//cvSetReal2D(pA, 2*n, 6*ptB_i + 4, 1);
				A.mat[6*ptB_i + 4].matCol.push_back(SparseMatElem(2*n, 1));

				//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 2, pMatchPairs[n].ptB.x);
				A.mat[6*ptB_i + 2].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptB.x));
				//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 3, pMatchPairs[n].ptB.y);
				A.mat[6*ptB_i + 3].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptB.y));
				//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 5, 1);
				A.mat[6*ptB_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1));

				// 用固定的参数对A点进行变换
				DfPoint ptA_Bar;
				double h64F[9];
				for(int t=0; t<9; t++)
				{
					h64F[t] = pImagesTransform0[pMatchPairs[n].ptA_i].h.m[t];
				}
				ApplyProject9(h64F, pMatchPairs[n].ptA.x, pMatchPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);

				// 给B赋值
				double aB = cvGetReal2D(pX, 6*ptB_i+0, 0); // a
				double bB = cvGetReal2D(pX, 6*ptB_i+1, 0); // b
				double cB = cvGetReal2D(pX, 6*ptB_i+2, 0); // c
				double dB = cvGetReal2D(pX, 6*ptB_i+3, 0); // d
				double eB = cvGetReal2D(pX, 6*ptB_i+4, 0); // e
				double fB = cvGetReal2D(pX, 6*ptB_i+5, 0); // f

				double xB = pMatchPairs[n].ptB.x;
				double yB = pMatchPairs[n].ptB.y;

				double f1 = aB*xB + bB*yB + eB - ptA_Bar.x;
				double f2 = cB*xB + dB*yB + fB - ptA_Bar.y;
				
				B.mat[0].matCol.push_back(SparseMatElem(2*n, f1));
				B.mat[0].matCol.push_back(SparseMatElem(2*n+1, f2));

				cost += f1*f1;
				cost += f2*f2;
			}
			else if((pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==1)) // 如果B点固定
			{
				int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
				//int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

				// 2*n行, 6*i列
				// ptA
				//cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
				A.mat[6*ptA_i + 0].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptA.x));
				//cvSetReal2D(pA, 2*n, 6*ptA_i + 1, pMatchPairs[n].ptA.y);
				A.mat[6*ptA_i + 1].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptA.y));
				//cvSetReal2D(pA, 2*n, 6*ptA_i + 4, 1);
				A.mat[6*ptA_i + 4].matCol.push_back(SparseMatElem(2*n, 1));

				//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 2, pMatchPairs[n].ptA.x);
				A.mat[6*ptA_i + 2].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptA.x));
				//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 3, pMatchPairs[n].ptA.y);
				A.mat[6*ptA_i + 3].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptA.y));
				//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 5, 1);
				A.mat[6*ptA_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1));

				// 用固定的参数对B点进行变换
				DfPoint ptB_Bar;
				double hB64F[9];
				for(int t=0; t<9; t++)
				{
					hB64F[t] = pImagesTransform0[pMatchPairs[n].ptB_i].h.m[t];
				}
				ApplyProject9(hB64F, pMatchPairs[n].ptB.x, pMatchPairs[n].ptB.y, ptB_Bar.x, ptB_Bar.y);

				// 给B赋值
				double aA = cvGetReal2D(pX, 6*ptA_i+0, 0); // a
				double bA = cvGetReal2D(pX, 6*ptA_i+1, 0); // b
				double cA = cvGetReal2D(pX, 6*ptA_i+2, 0); // c
				double dA = cvGetReal2D(pX, 6*ptA_i+3, 0); // d
				double eA = cvGetReal2D(pX, 6*ptA_i+4, 0); // e
				double fA = cvGetReal2D(pX, 6*ptA_i+5, 0); // f

				double xA = pMatchPairs[n].ptA.x;
				double yA = pMatchPairs[n].ptA.y;

				double f1 = aA*xA + bA*yA + eA - ptB_Bar.x;
				double f2 = cA*xA + dA*yA + fA - ptB_Bar.y;
				
				B.mat[0].matCol.push_back(SparseMatElem(2*n, f1));
				B.mat[0].matCol.push_back(SparseMatElem(2*n+1, f2));

				cost += f1*f1;
				cost += f2*f2;
			}
		}
		//// 向A中加入约束条件
		//// a = d, b = -c
		//for(int n=0; n<nImages; n++)
		//{
		//	int cst_i = n - vecAccFixed[n]; 
		//	int nC = freqMatch[n]; // 当前图像的约束的个数（当前图像中匹配对出现的次数）
		//	for(int i=0; i<nC; i++)
		//	{
		//		//cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
		//		A.mat[6*cst_i + 0].matCol.push_back(SparseMatElem(2*(n+nPairs), 1));
		//		A.mat[6*cst_i + 3].matCol.push_back(SparseMatElem(2*(n+nPairs), -1));

		//		A.mat[6*cst_i + 1].matCol.push_back(SparseMatElem(2*(n+nPairs)+1, 1));
		//		A.mat[6*cst_i + 2].matCol.push_back(SparseMatElem(2*(n+nPairs)+1, 1));
		//	}
		//}
		double cost1 = cost;

		cout<<"mse "<<sqrt(cost1/nPairs);

		// 向A中加入rot约束条件
		int nUnfixed = 0;
		vector<int> vecFreqMatch;
		for(int n0=0; n0<nImages; n0++)
		{
			if(pImagesTransform0[n0].fixed==1)
				continue;

			int nC = freqMatch[n0];
			vecFreqMatch.push_back(nC);

			int n = nUnfixed;
			double a = cvGetReal2D(pX, 6*n+0, 0); // a
			double b = cvGetReal2D(pX, 6*n+1, 0); // b
			double c = cvGetReal2D(pX, 6*n+2, 0); // c
			double d = cvGetReal2D(pX, 6*n+3, 0); // d
			double e = cvGetReal2D(pX, 6*n+4, 0); // e
			double f = cvGetReal2D(pX, 6*n+5, 0); // f

			double wRot = nC;

			double sqt_wRot = sqrt(wRot);
			A.mat[6*n + 0].matCol.push_back(SparseMatElem(3*n+2*nPairs+0, wRot*b));
			A.mat[6*n + 1].matCol.push_back(SparseMatElem(3*n+2*nPairs+0, wRot*a));
			A.mat[6*n + 2].matCol.push_back(SparseMatElem(3*n+2*nPairs+0, wRot*d));
			A.mat[6*n + 3].matCol.push_back(SparseMatElem(3*n+2*nPairs+0, wRot*c));

			A.mat[6*n + 0].matCol.push_back(SparseMatElem(3*n+2*nPairs+1, wRot*2*a));
			A.mat[6*n + 2].matCol.push_back(SparseMatElem(3*n+2*nPairs+1, wRot*2*c));

			A.mat[6*n + 1].matCol.push_back(SparseMatElem(3*n+2*nPairs+2, wRot*2*b));
			A.mat[6*n + 3].matCol.push_back(SparseMatElem(3*n+2*nPairs+2, wRot*2*d));

			// 向B赋值
			B.mat[0].matCol.push_back(SparseMatElem(3*n+2*nPairs+0, wRot*(a*b + c*d)));
			B.mat[0].matCol.push_back(SparseMatElem(3*n+2*nPairs+1, wRot*(a*a + c*c - 1)));
			B.mat[0].matCol.push_back(SparseMatElem(3*n+2*nPairs+2, wRot*(b*b + d*d - 1)));

			cost += wRot*(a*b + c*d)*(a*b + c*d);
			cost += wRot*(a*a + c*c - 1)*(a*a + c*c - 1);
			cost += wRot*(b*b + d*d - 1)*(b*b + d*d - 1);

			nUnfixed++;
		}
		cout<<"cost1 "<<cost1<<" rot cost "<<cost-cost1<<endl;

		//CvMat* pATA = cvCreateMat(colA, colA, CV_64F);
		//CvMat* pAT = cvCreateMat(colA, rowA, CV_64F);
		//cvTranspose(pA, pAT);
		//cvGEMM(pAT, pA, 1, NULL, 0, pATA);
		//cvPrintMatrix(pATA, "c:\\ATA.txt");
		//cvReleaseMat(&pATA);
		//cvReleaseMat(&pAT);

		// CV解线性方程组
		//cout<<"解线性方程组"<<endl<<endl;
		//cvSolve(pA, pB, pX, CV_SVD);
		//cvPrintMatrix(pX, "c:\\x-cv.txt");

		for(int col=0; col<A.col; col++)
		{
			std::sort(A.mat[col].matCol.begin(), A.mat[col].matCol.end());
		}

		SolveSparseSystem2(A, B, pDX);

		// 更新pX
		cvConvertScale(pDX, pDX, -1);
		cvAdd(pX, pDX, pX);

		//int nSingleIter = 0;
		//int maxIterSingle = 10;
		//bool convergence = true;
		//do
		//{
		//	SolveSparseSystemLM(A, B, eye, pDX, lambda64F);
		//	nSingleIter++;

		//	//cout<<"pX"<<endl;
		//	//for(int p=0; p<nImages-nFixedImages; p++)
		//	//{
		//	//	for(int q=0; q<8; q++)
		//	//	{
		//	//		printf("%.4f ", cvGetReal2D(pX, p*8 + q, 0));
		//	//	}
		//	//	cout<<endl;
		//	//}

		//	cout<<"pDX"<<endl;
		//	for(int p=0; p<1; p++)
		//	{
		//		for(int q=0; q<6; q++)
		//		{
		//			printf("%.4f ", cvGetReal2D(pDX, p*6 + q, 0));
		//		}
		//		cout<<endl;
		//	}

		//	// 更新pX
		//	cvConvertScale(pDX, pDX, -1);
		//	cvAdd(pX, pDX, pX2);

		//	// 整理计算结果
		//	
		//	vecTransformRefined.clear();
		//	nUnFixed = 0;
		//	for(int i=0; i<nImages; i++)
		//	{
		//		if(pImagesTransform0[i].fixed==0) // 未固定参数情况
		//		{
		//			ImageTransform temp;
		//			temp.fixed = 0;
		//			for(int j=0; j<6; j++)
		//			{
		//				temp.h.m[j] = (float)cvGetReal2D(pX2, 6*nUnFixed+j, 0); // a
		//			}
		//			temp.h.m[6] = 0;
		//			temp.h.m[7] = 0;
		//			temp.h.m[8] = 1;
		//			nUnFixed++;
		//			vecTransformRefined.push_back(temp);
		//		}
		//		else
		//		{
		//			vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
		//		}
		//	}

		//	double costNew = CalMatchErrorAffine(&vecTransformRefined[0], pX2, nUnfixed, pMatchPairs, nPairs, vecFreqMatch, vecAccFixed);
		//	cout<<"costNew "<<costNew<<" totalCost "<<cost<<" lambda64F "<<lambda64F<<endl;
		//	if(costNew<cost)
		//	{
		//		cvCopy(pX2, pX);
		//		lambda64F /= beta;
		//		convergence = true;
		//		cost = costNew;
		//	}
		//	else
		//	{
		//		lambda64F *= beta;
		//		convergence = false;
		//	}
		//	if(nSingleIter>maxIterSingle)
		//	{
		//		break;
		//	}
		//} while(convergence==false);

		//cout<<"cost "<<cost<<endl;
		vecCostVal.push_back(cost);
		
		double cost2 = cost - cost1;
		vecCostVal2.push_back(cost2);

		iter++;
	}

	cout<<"cost "<<endl;
	for(int i=0; i<vecCostVal.size(); i++)
	{
		cout<<vecCostVal[i]<<" ";
	}
	cout<<endl<<"cost rot "<<endl;
	for(int i=0; i<vecCostVal2.size(); i++)
	{
		cout<<vecCostVal2[i]<<" ";
	}
	cout<<endl;

	//cvPrintMatrix(pX, "c:\\x-cholmod.txt");

	nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(pImagesTransform0[i].fixed==0) // 未固定参数情况
		{
			ImageTransform temp;
			temp.fixed = 0;
			temp.h.m[0] = (float)cvGetReal2D(pX, 6*nUnFixed+0, 0); // a
			temp.h.m[1] = (float)cvGetReal2D(pX, 6*nUnFixed+1, 0); // b
			temp.h.m[3] = (float)cvGetReal2D(pX, 6*nUnFixed+2, 0); // c
			temp.h.m[4] = (float)cvGetReal2D(pX, 6*nUnFixed+3, 0); // d
			temp.h.m[2] = (float)cvGetReal2D(pX, 6*nUnFixed+4, 0); // e
			temp.h.m[5] = (float)cvGetReal2D(pX, 6*nUnFixed+5, 0); // f

			nUnFixed++;

			temp.h.m[6] = 0;
			temp.h.m[7] = 0;
			temp.h.m[8] = 1;
			vecTransformRefined[i] = temp;
		}
		else
		{
			vecTransformRefined[i] = pImagesTransform0[i]; // 固定的变换参数
		}
	}

	cvReleaseMat(&pX);
	cvReleaseMat(&pX2);
	cvReleaseMat(&pDX);

	return 0;
}

// 捆集调整---平移模型
int CMosaicByPose::SBA_Translation(MatchPointPairs* pMatchPairs, int nPairs,
									  const Translation32F* pImagesTransform0, int nImages, 
									  int nFixedImages,
									  vector<Translation32F> &vecTransformRefined)
{	
	cout<<"SBA_Translation Start..."<<endl<<endl;
	// AX = B, 用平移变换模型
	// x2 = x1 + e
	// y2 = y1 + f
	if(nImages<=1)
	{
		return -2;
	}

	vector<int> vecAccFixed(nImages);
	memset(&vecAccFixed[0], 0, sizeof(int)*nImages);

	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<i; j++)
		{
			if(pImagesTransform0[j].fixed==1)
			{
				vecAccFixed[i] += 1;
			}
		}
	}

	int DIM = 2;
	int rowA = 2 * nPairs;
	int colA = 2 * (nImages-nFixedImages);
	int rowX = 2 * (nImages-nFixedImages);
	int colX = 1;
	int rowB = 2*nPairs;
	int colB = 1;

	//cout<<"rowA "<<rowA<<" rowB "<<rowB<<endl;

	cout<<"创建PA,PX,PB"<<endl;
	cout<<"rowA "<<rowA<<"colA "<<colA<<endl;
	cout<<"nImages "<<nImages<<"nPairs "<<nPairs<<endl;
	//CvMat* pA = cvCreateMat(rowA, colA, CV_64F); // 2*n * 6*m , n为特征点数, m为图像数
	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);
	//CvMat* pB = cvCreateMat(rowB, colB, CV_64F);
	//CvMat *pA, *pB;
	pool::SparseMatrix A;
	pool::SparseMatrix B;
	A.mat.resize(colA);
	B.mat.resize(colB);
	A.col = colA;
	A.row = rowA;
	B.col = colB;
	B.row = rowB;

	cout<<"赋值PA,PB"<<endl;

	for(int n=0; n<nPairs; n++)
	{
		//cout<<n<<" / "<<nPairs<<endl;
		if(n==10000)
		{
			n=n;
		}
		if( (pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==0) ) // 如果A点和B点都不是固定点
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号
			// 2*n行, 6*i列
			// ptA
			A.mat[DIM*ptA_i + 0].matCol.push_back(SparseMatElem(2*n, 1));

			A.mat[DIM*ptA_i + 1].matCol.push_back(SparseMatElem(2*n+1, 1));

			//// ptB
			A.mat[DIM*ptB_i + 0].matCol.push_back(SparseMatElem(2*n, -1));

			A.mat[DIM*ptB_i + 1].matCol.push_back(SparseMatElem(2*n+1, -1));

			// 给B赋值
			B.mat[0].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptB.x-pMatchPairs[n].ptA.x));
			B.mat[0].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptB.y-pMatchPairs[n].ptA.y));
		}
		else if((pMatchPairs[n].ptA_Fixed==1) && (pMatchPairs[n].ptB_Fixed==0)) // 如果A点固定
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

			//// ptB
			A.mat[DIM*ptB_i + 0].matCol.push_back(SparseMatElem(2*n, 1));

			A.mat[DIM*ptB_i + 1].matCol.push_back(SparseMatElem(2*n+1, 1));

			// 给B赋值
			B.mat[0].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptA.x-pMatchPairs[n].ptB.x));
			B.mat[0].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptA.y-pMatchPairs[n].ptB.y));
		}
		else if((pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==1)) // 如果B点固定
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

			// 2*n行, 6*i列
			// ptA
			A.mat[DIM*ptA_i + 0].matCol.push_back(SparseMatElem(2*n, 1));

			A.mat[DIM*ptA_i + 1].matCol.push_back(SparseMatElem(2*n+1, 1));

			// 给B赋值
			B.mat[0].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptB.x-pMatchPairs[n].ptA.x));
			B.mat[0].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptB.y-pMatchPairs[n].ptA.y));
		}
	}

	//CvMat* pATA = cvCreateMat(colA, colA, CV_64F);
	//CvMat* pAT = cvCreateMat(colA, rowA, CV_64F);
	//cvTranspose(pA, pAT);
	//cvGEMM(pAT, pA, 1, NULL, 0, pATA);
	//cvPrintMatrix(pATA, "c:\\ATA.txt");
	//cvReleaseMat(&pATA);
	//cvReleaseMat(&pAT);

	// CV解线性方程组
	cout<<"解线性方程组"<<endl<<endl;
	//cvSolve(pA, pB, pX, CV_SVD);
	//cvPrintMatrix(pX, "c:\\x-cv.txt");

	//SolveSparseSystem(pA, pB, pX);
	//
	for(int col=0; col<A.col; col++)
	{
		std::sort(A.mat[col].matCol.begin(), A.mat[col].matCol.end());
	}

	SolveSparseSystem2(A, B, pX);

	//cvPrintMatrix(pX, "c:\\x-cholmod.txt");

	int nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(pImagesTransform0[i].fixed==0) // 未固定参数情况
		{
			Translation32F temp;
			temp.dx = (float)cvGetReal2D(pX, 2*nUnFixed+0, 0); // a
			temp.dy = (float)cvGetReal2D(pX, 2*nUnFixed+1, 0); // b

			nUnFixed++;
			vecTransformRefined.push_back(temp);
		}
		else
		{
			vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
		}
	}

	cvReleaseMat(&pX);

	return 0;
}

// 线性捆集调整
int CMosaicByPose::BundleAdjustmentSparse(MatchPointPairs* pMatchPairs, int nPairs,
									const ImageTransform* pImagesTransform0, int nImages, 
									int nFixedImages,
									vector<ImageTransform> &vecTransformRefined)
{	
	cout<<"----BundleAdjustmentSparse----"<<endl<<endl;
	// AX = B, 用仿射变换模型
	// x2 = a.x1 + b.y1 + e
	// y2 = c.x1 + d.y1 + f
	if(nImages<=1)
	{
		return -2;
	}

	vector<int> vecAccFixed(nImages);
	memset(&vecAccFixed[0], 0, sizeof(int)*nImages);

	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<i; j++)
		{
			if(pImagesTransform0[j].fixed==1)
			{
				vecAccFixed[i] += 1;
			}
		}
	}

	int rowA = 2 * nPairs;
	int colA = 6 * (nImages-nFixedImages);
	int rowX = 6 * (nImages-nFixedImages);
	int colX = 1;
	int rowB = 2*nPairs;
	int colB = 1;

	//cout<<"rowA "<<rowA<<" rowB "<<rowB<<endl;

	cout<<"创建PA,PX,PB"<<endl;
	cout<<"rowA "<<rowA<<"colA "<<colA<<endl;
	cout<<"nImages "<<nImages<<"nPairs "<<nPairs<<endl;
	//CvMat* pA = cvCreateMat(rowA, colA, CV_64F); // 2*n * 6*m , n为特征点数, m为图像数
	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);
	//CvMat* pB = cvCreateMat(rowB, colB, CV_64F);
	//CvMat *pA, *pB;
	pool::SparseMatrix A;
	pool::SparseMatrix B;
	A.mat.resize(colA);
	B.mat.resize(colB);
	A.col = colA;
	A.row = rowA;
	B.col = colB;
	B.row = rowB;

	cout<<"赋值PA,PB"<<endl;

	for(int n=0; n<nPairs; n++)
	{
		//cout<<n<<" / "<<nPairs<<endl;
		if(n==10000)
		{
			n=n;
		}
		if( (pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==0) ) // 如果A点和B点都不是固定点
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号
			// 2*n行, 6*i列
			// ptA
			
			//cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
			A.mat[6*ptA_i + 0].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptA.x));
			//cvSetReal2D(pA, 2*n, 6*ptA_i + 1, pMatchPairs[n].ptA.y);
			A.mat[6*ptA_i + 1].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptA.y));
			//cvSetReal2D(pA, 2*n, 6*ptA_i + 4, 1);
			A.mat[6*ptA_i + 4].matCol.push_back(SparseMatElem(2*n, 1));
			//A.mat[].matCol.push_back(SparseMatElem(, ));

			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 2, pMatchPairs[n].ptA.x);
			A.mat[6*ptA_i + 2].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptA.x));
			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 3, pMatchPairs[n].ptA.y);
			A.mat[6*ptA_i + 3].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptA.y));
			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 5, 1);
			A.mat[6*ptA_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1));

			//// ptB
			//cvSetReal2D(pA, 2*n, 6*ptB_i + 0, -pMatchPairs[n].ptB.x);
			A.mat[6*ptB_i + 0].matCol.push_back(SparseMatElem(2*n, -pMatchPairs[n].ptB.x));
			//cvSetReal2D(pA, 2*n, 6*ptB_i + 1, -pMatchPairs[n].ptB.y);
			A.mat[6*ptB_i + 1].matCol.push_back(SparseMatElem(2*n, -pMatchPairs[n].ptB.y));
			//cvSetReal2D(pA, 2*n, 6*ptB_i + 4, -1);
			A.mat[6*ptB_i + 4].matCol.push_back(SparseMatElem(2*n, -1));

			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 2, -pMatchPairs[n].ptB.x);
			A.mat[6*ptB_i + 2].matCol.push_back(SparseMatElem(2*n+1, -pMatchPairs[n].ptB.x));
			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 3, -pMatchPairs[n].ptB.y);
			A.mat[6*ptB_i + 3].matCol.push_back(SparseMatElem(2*n+1, -pMatchPairs[n].ptB.y));
			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 5, -1);
			A.mat[6*ptB_i + 5].matCol.push_back(SparseMatElem(2*n+1, -1));

			//// 给B赋值
			//cvSetReal2D(pB, 2*n, 0, 0);
			//B.mat[0].matCol.push_back(SparseMatElem(2*n, 0));
			//cvSetReal2D(pB, 2*n+1, 0, 0);
			//B.mat[0].matCol.push_back(SparseMatElem(2*n+1, 0));
		}
		else if((pMatchPairs[n].ptA_Fixed==1) && (pMatchPairs[n].ptB_Fixed==0)) // 如果A点固定
		{
			//int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

			//// ptB
			//cvSetReal2D(pA, 2*n, 6*ptB_i + 0, pMatchPairs[n].ptB.x);
			A.mat[6*ptB_i + 0].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptB.x));
			//cvSetReal2D(pA, 2*n, 6*ptB_i + 1, pMatchPairs[n].ptB.y);
			A.mat[6*ptB_i + 1].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptB.y));
			//cvSetReal2D(pA, 2*n, 6*ptB_i + 4, 1);
			A.mat[6*ptB_i + 4].matCol.push_back(SparseMatElem(2*n, 1));

			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 2, pMatchPairs[n].ptB.x);
			A.mat[6*ptB_i + 2].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptB.x));
			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 3, pMatchPairs[n].ptB.y);
			A.mat[6*ptB_i + 3].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptB.y));
			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 5, 1);
			A.mat[6*ptB_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1));

			// 用固定的参数对A点进行变换
			DfPoint ptA_Bar;
			double h64F[9];
			for(int t=0; t<9; t++)
			{
				h64F[t] = pImagesTransform0[pMatchPairs[n].ptA_i].h.m[t];
			}
			ApplyProject9(h64F, pMatchPairs[n].ptA.x, pMatchPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);

			//// 给B赋值
			//cvSetReal2D(pB, 2*n, 0, ptA_Bar.x);
			B.mat[0].matCol.push_back(SparseMatElem(2*n, ptA_Bar.x));
			//cvSetReal2D(pB, 2*n+1, 0, ptA_Bar.y);
			B.mat[0].matCol.push_back(SparseMatElem(2*n+1, ptA_Bar.y));
		}
		else if((pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==1)) // 如果B点固定
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			//int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

			// 2*n行, 6*i列
			// ptA
			//cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
			A.mat[6*ptA_i + 0].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptA.x));
			//cvSetReal2D(pA, 2*n, 6*ptA_i + 1, pMatchPairs[n].ptA.y);
			A.mat[6*ptA_i + 1].matCol.push_back(SparseMatElem(2*n, pMatchPairs[n].ptA.y));
			//cvSetReal2D(pA, 2*n, 6*ptA_i + 4, 1);
			A.mat[6*ptA_i + 4].matCol.push_back(SparseMatElem(2*n, 1));

			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 2, pMatchPairs[n].ptA.x);
			A.mat[6*ptA_i + 2].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptA.x));
			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 3, pMatchPairs[n].ptA.y);
			A.mat[6*ptA_i + 3].matCol.push_back(SparseMatElem(2*n+1, pMatchPairs[n].ptA.y));
			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 5, 1);
			A.mat[6*ptA_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1));

			// 用固定的参数对B点进行变换
			DfPoint ptB_Bar;
			double hB64F[9];
			for(int t=0; t<9; t++)
			{
				hB64F[t] = pImagesTransform0[pMatchPairs[n].ptB_i].h.m[t];
			}
			ApplyProject9(hB64F, pMatchPairs[n].ptB.x, pMatchPairs[n].ptB.y, ptB_Bar.x, ptB_Bar.y);
			// 给B赋值
			//cvSetReal2D(pB, 2*n, 0, ptB_Bar.x);
			B.mat[0].matCol.push_back(SparseMatElem(2*n, ptB_Bar.x));
			//cvSetReal2D(pB, 2*n+1, 0, ptB_Bar.y);
			B.mat[0].matCol.push_back(SparseMatElem(2*n+1, ptB_Bar.y));
		}
	}

	//CvMat* pATA = cvCreateMat(colA, colA, CV_64F);
	//CvMat* pAT = cvCreateMat(colA, rowA, CV_64F);
	//cvTranspose(pA, pAT);
	//cvGEMM(pAT, pA, 1, NULL, 0, pATA);
	//cvPrintMatrix(pATA, "c:\\ATA.txt");
	//cvReleaseMat(&pATA);
	//cvReleaseMat(&pAT);

	// CV解线性方程组
	cout<<"解线性方程组"<<endl<<endl;
	//cvSolve(pA, pB, pX, CV_SVD);
	//cvPrintMatrix(pX, "c:\\x-cv.txt");

	//SolveSparseSystem(pA, pB, pX);
	//
	for(int col=0; col<A.col; col++)
	{
		std::sort(A.mat[col].matCol.begin(), A.mat[col].matCol.end());
	}

	SolveSparseSystem2(A, B, pX);
	
	//cvPrintMatrix(pX, "c:\\x-cholmod.txt");

	int nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(pImagesTransform0[i].fixed==0) // 未固定参数情况
		{
			ImageTransform temp;
			temp.fixed = 0;
			temp.h.m[0] = (float)cvGetReal2D(pX, 6*nUnFixed+0, 0); // a
			temp.h.m[1] = (float)cvGetReal2D(pX, 6*nUnFixed+1, 0); // b
			temp.h.m[3] = (float)cvGetReal2D(pX, 6*nUnFixed+2, 0); // c
			temp.h.m[4] = (float)cvGetReal2D(pX, 6*nUnFixed+3, 0); // d
			temp.h.m[2] = (float)cvGetReal2D(pX, 6*nUnFixed+4, 0); // e
			temp.h.m[5] = (float)cvGetReal2D(pX, 6*nUnFixed+5, 0); // f

			nUnFixed++;

			temp.h.m[6] = 0;
			temp.h.m[7] = 0;
			temp.h.m[8] = 1;
			vecTransformRefined.push_back(temp);
		}
		else
		{
			vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
		}
	}

	cvReleaseMat(&pX);

	return 0;
}

// 线性捆集调整
// 要求: 固定的图像的放在图像序列的后面?
int CMosaicByPose::BundleAdjustment(MatchPointPairs* pMatchPairs, int nPairs,
									const ImageTransform* pImagesTransform0, int nImages, 
									int nFixedImages,
									vector<ImageTransform> &vecTransformRefined)
{	
	cout<<"Bundle Adjustment Start 0000"<<endl<<endl;
	// AX = B, 用仿射变换模型
	// x2 = a.x1 + b.y1 + e
	// y2 = c.x1 + d.y1 + f

	vector<int> vecAccFixed(nImages);
	memset(&vecAccFixed[0], 0, sizeof(int)*nImages);

	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<i; j++)
		{
			if(pImagesTransform0[j].fixed==1)
			{
				vecAccFixed[i] += 1;
			}
		}
	}
	
	int rowA = 2 * nPairs;
	int colA = 6 * (nImages-nFixedImages);
	int rowX = 6 * (nImages-nFixedImages);
	int colX = 1;
	int rowB = 2*nPairs;
	int colB = 1;

	//cout<<"rowA "<<rowA<<" rowB "<<rowB<<endl;

	cout<<"创建PA,PX,PB"<<endl;
	cout<<"rowA "<<rowA<<"colA "<<colA<<endl;
	cout<<"sizeA (byte)"<<rowA*colA*sizeof(double)<<endl;
	cout<<"nImages "<<nImages<<"nPairs "<<nPairs<<endl;
	CvMat* pA = cvCreateMat(rowA, colA, CV_64F); // 2*n * 6*m , n为特征点数, m为图像数
	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pB = cvCreateMat(rowB, colB, CV_64F);
	cvZero(pA);
	cvZero(pB);
	cvZero(pX);

	cout<<"赋值PA,PB"<<endl;

	for(int n=0; n<nPairs; n++)
	{
		//cout<<n<<" / "<<nPairs<<endl;
		if(n==2020)
		{
			n=n;
		}
		if( (pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==0) ) // 如果A点和B点都不是固定点
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号
			// 2*n行, 6*i列
			// ptA
			
			cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
			cvSetReal2D(pA, 2*n, 6*ptA_i + 1, pMatchPairs[n].ptA.y);
			cvSetReal2D(pA, 2*n, 6*ptA_i + 4, 1);

			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 2, pMatchPairs[n].ptA.x);
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 3, pMatchPairs[n].ptA.y);
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 5, 1);

			// ptB
			cvSetReal2D(pA, 2*n, 6*ptB_i + 0, -pMatchPairs[n].ptB.x);
			cvSetReal2D(pA, 2*n, 6*ptB_i + 1, -pMatchPairs[n].ptB.y);
			cvSetReal2D(pA, 2*n, 6*ptB_i + 4, -1);

			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 2, -pMatchPairs[n].ptB.x);
			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 3, -pMatchPairs[n].ptB.y);
			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 5, -1);
		
			// 给B赋值
			cvSetReal2D(pB, 2*n, 0, 0);
			cvSetReal2D(pB, 2*n+1, 0, 0);
		}
		else if((pMatchPairs[n].ptA_Fixed==1) && (pMatchPairs[n].ptB_Fixed==0)) // 如果A点固定
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

			// 2*n行, 6*i列
			// ptA
			//pA[2*n*colA + 6*ptA_i + 0] = pMatchPairs[n].ptA.x;
			//pA[2*n*colA + 6*ptA_i + 1] = pMatchPairs[n].ptA.y;
			//pA[2*n*colA + 6*ptA_i + 4] = 1;

			//pA[(2*n+1)*colA + 6*ptA_i + 2] = pMatchPairs[n].ptA.x;
			//pA[(2*n+1)*colA + 6*ptA_i + 3] = pMatchPairs[n].ptA.y;
			//pA[(2*n+1)*colA + 6*ptA_i + 5] = 1;

			// ptB
			cvSetReal2D(pA, 2*n, 6*ptB_i + 0, pMatchPairs[n].ptB.x);
			cvSetReal2D(pA, 2*n, 6*ptB_i + 1, pMatchPairs[n].ptB.y);
			cvSetReal2D(pA, 2*n, 6*ptB_i + 4, 1);

			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 2, pMatchPairs[n].ptB.x);
			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 3, pMatchPairs[n].ptB.y);
			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 5, 1);

			// 用固定的参数对A点进行变换
			DfPoint ptA_Bar;
			double h64F[9];
			for(int t=0; t<9; t++)
			{
				h64F[t] = pImagesTransform0[pMatchPairs[n].ptA_i].h.m[t];
			}
			ApplyProject9(h64F, pMatchPairs[n].ptA.x, pMatchPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);

			// 给B赋值
			cvSetReal2D(pB, 2*n, 0, ptA_Bar.x);
			cvSetReal2D(pB, 2*n+1, 0, ptA_Bar.y);
		}
		else if((pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==1)) // 如果B点固定
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号
			
			// 2*n行, 6*i列
			// ptA
			cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
			cvSetReal2D(pA, 2*n, 6*ptA_i + 1, pMatchPairs[n].ptA.y);
			cvSetReal2D(pA, 2*n, 6*ptA_i + 4, 1);
			
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 2, pMatchPairs[n].ptA.x);
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 3, pMatchPairs[n].ptA.y);
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 5, 1);
			
			//// ptB
			//pA[2*n*colA + 6*ptB_i + 0] = pMatchPairs[n].ptB.x;
			//pA[2*n*colA + 6*ptB_i + 1] = pMatchPairs[n].ptB.y;
			//pA[2*n*colA + 6*ptB_i + 4] = 1;
			
			//pA[(2*n+1)*colA + 6*ptB_i + 2] = pMatchPairs[n].ptB.x;
			//pA[(2*n+1)*colA + 6*ptB_i + 3] = pMatchPairs[n].ptB.y;
			//pA[(2*n+1)*colA + 6*ptB_i + 5] = 1;
			
			// 用固定的参数对B点进行变换
			DfPoint ptB_Bar;
			double hB64F[9];
			for(int t=0; t<9; t++)
			{
				hB64F[t] = pImagesTransform0[pMatchPairs[n].ptB_i].h.m[t];
			}
			ApplyProject9(hB64F, pMatchPairs[n].ptB.x, pMatchPairs[n].ptB.y, ptB_Bar.x, ptB_Bar.y);
			// 给B赋值
			cvSetReal2D(pB, 2*n, 0, ptB_Bar.x);
			cvSetReal2D(pB, 2*n+1, 0, ptB_Bar.y);
		}
	}

	//CvMat* pATA = cvCreateMat(colA, colA, CV_64F);
	//CvMat* pAT = cvCreateMat(colA, rowA, CV_64F);
	//cvTranspose(pA, pAT);
	//cvGEMM(pAT, pA, 1, NULL, 0, pATA);
	//cvPrintMatrix(pATA, "c:\\ATA.txt");
	//cvReleaseMat(&pATA);
	//cvReleaseMat(&pAT);

	// CV解线性方程组
	cout<<"解线性方程组"<<endl<<endl;
	cvSolve(pA, pB, pX, CV_SVD);
	//cvPrintMatrix(pX, "c:\\x-cv.txt");
	
	//cout<<"pA"<<endl;
	//cvPrintMatrix(pA, "c:\\pA.txt");
	//cout<<"pB"<<endl;
	//cvPrintMatrix(pB, "c:\\pB.txt");

	//SolveSparseSystem(pA, pB, pX);
	//cvPrintMatrix(pX, "c:\\x-cholmod.txt");

	int nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(pImagesTransform0[i].fixed==0) // 未固定参数情况
		{
			ImageTransform temp;
			temp.fixed = 0;
			temp.h.m[0] = (float)cvGetReal2D(pX, 6*nUnFixed+0, 0); // a
			temp.h.m[1] = (float)cvGetReal2D(pX, 6*nUnFixed+1, 0); // b
			temp.h.m[3] = (float)cvGetReal2D(pX, 6*nUnFixed+2, 0); // c
			temp.h.m[4] = (float)cvGetReal2D(pX, 6*nUnFixed+3, 0); // d
			temp.h.m[2] = (float)cvGetReal2D(pX, 6*nUnFixed+4, 0); // e
			temp.h.m[5] = (float)cvGetReal2D(pX, 6*nUnFixed+5, 0); // f

			nUnFixed++;

			temp.h.m[6] = 0;
			temp.h.m[7] = 0;
			temp.h.m[8] = 1;
			vecTransformRefined.push_back(temp);
		}
		else
		{
			vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
		}
	}

	cvReleaseMat(&pA);
	cvReleaseMat(&pB);
	cvReleaseMat(&pX);
	
	return 0;
}

int PrintMatchPoints(MatchPointPairs* pMatched, int nPairs)
{
	//P1.txt
	//P2.txt
	ofstream outData1("c:\\P1.txt", ios::trunc);
	ofstream outData2("c:\\P2.txt", ios::trunc);

	//outData1<<nPairs<<endl;
	//outData2<<nPairs<<endl;

	for(int i=0; i<nPairs; i++)
	{
		outData1<<pMatched[i].ptA.x<<" "<<pMatched[i].ptA.y<<" "<<pMatched[i].ptA_i<<endl;
		outData2<<pMatched[i].ptB.x<<" "<<pMatched[i].ptB.y<<" "<<pMatched[i].ptB_i<<endl;
	}

	outData1.close();
	outData2.close();

	return 0;
}

int PrintX0(const ImageTransform* pImagesTransform0, int nImages)
{
	// X0.txt
	cout<<"输出数据到文本TXT"<<endl;
	ofstream outData("c:\\X0.txt", ios::trunc);

	//outData<<nImages<<endl;

	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<9; j++)
		{
			outData<<pImagesTransform0[i].h.m[j]/pImagesTransform0[i].h.m[8]<<endl;
		}
	}

	outData.close();

	return 0;
}


//// 非线性规划方法
//int CMosaicByPose::NonlinearProgram(const ImagePoseInfo *pImgPoses,
//									MatchPointPairs* pMatchPairs, int nPairs,
//									const ImageTransform* pImagesTransform0, int nImages, 
//									int nFixedImages,
//									vector<ImageTransform> &vecTransformRefined)
//{	
//	cout<<"Quadratic Program Start"<<endl<<endl;
//	// AX = B, 用仿射变换模型
//	// x2 = a.x1 + b.y1 + e
//	// y2 = c.x1 + d.y1 + f
//	if(nFixedImages<2)
//	{
//		//return -1;
//	}
//
//	int rowA = 2 * nPairs;
//	int colA = 8 * nImages; 
//	int rowX = colA;
//	int colX = 1;	
//	int rowB = 2*nPairs;
//	int colB = 1;
//
//	int rowAN = 8 * nPairs;
//	int colAN = 8 * nImages; 
//
//	int rowBN = rowAN;
//	int colBN = 1;
//
//	cout<<"rowA "<<rowA<<" rowB "<<rowB<<endl;
//
//	CvMat* pA = cvCreateMat(rowA, colA, CV_64F); // 2*n * 6*m , n为特征点数, m为图像数
//	CvMat* pH = cvCreateMat(colA, colA, CV_64F);
//	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);
//	CvMat* pB = cvCreateMat(rowB, colB, CV_64F);
//
//	CvMat* pAN = cvCreateMat(rowAN, colAN, CV_64F);
//	CvMat* pBN = cvCreateMat(rowBN, colBN, CV_64F);
//
//	cvZero(pAN);
//	cvZero(pBN);
//
//	cvZero(pA);
//	cvZero(pB);
//	cvZero(pH);
//
//	double dx = 50, dy = 50; // pixel
//
//	int nFixedProcess = 0;
//	for(int n=0; n<nPairs; n++)
//	{
//		cout<<n<<" / "<<nPairs<<endl;
//
//		if(n==6241)
//		{
//			n=n;
//		}
//
//		{
//			//int ptA_i = pMatchPairs[n].ptA_i; // - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
//			//int ptB_i = pMatchPairs[n].ptB_i; // - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号
//
//			//// 2*n行, 6*i列
//			//// ptA
//			//cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
//			//cvSetReal2D(pA, 2*n, 6*ptA_i + 1, pMatchPairs[n].ptA.y);
//			//cvSetReal2D(pA, 2*n, 6*ptA_i + 4, 1);
//
//			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 2, pMatchPairs[n].ptA.x);
//			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 3, pMatchPairs[n].ptA.y);
//			//cvSetReal2D(pA, (2*n+1), 6*ptA_i + 5, 1);
//
//			//// ptB
//			//cvSetReal2D(pA, 2*n, 6*ptB_i + 0, -pMatchPairs[n].ptB.x);
//			//cvSetReal2D(pA, 2*n, 6*ptB_i + 1, -pMatchPairs[n].ptB.y);
//			//cvSetReal2D(pA, 2*n, 6*ptB_i + 4, -1);
//
//			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 2, -pMatchPairs[n].ptB.x);
//			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 3, -pMatchPairs[n].ptB.y);
//			//cvSetReal2D(pA, (2*n+1), 6*ptB_i + 5, -1);
//
//			//// 给B赋值
//			//cvSetReal2D(pB, 2*n, 0, 0);
//			//cvSetReal2D(pB, 2*n+1, 0, 0);
//		}
//		{
//			int ptA_i = pMatchPairs[n].ptA_i; // - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
//
//			//==================================================================
//			// 用初始参数对A点进行变换
//			DfPoint ptA_Bar;
//			double hA64F[9];
//			for(int t=0; t<9; t++)
//			{
//				hA64F[t] = pImagesTransform0[ptA_i].h.m[t];
//			}
//			ApplyProject9(hA64F, pMatchPairs[n].ptA.x, pMatchPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);
//			cvSetReal2D(pAN, 8*n,	6*ptA_i + 0,	pMatchPairs[n].ptA.x);
//			cvSetReal2D(pAN, 8*n,	6*ptA_i + 1,	pMatchPairs[n].ptA.y);
//			cvSetReal2D(pAN, 8*n,	6*ptA_i + 4,	1);
//			cvSetReal2D(pBN, 8*n,	0,				dx + ptA_Bar.x);
//
//			cvSetReal2D(pAN, 8*n+1,	6*ptA_i + 0,	-pMatchPairs[n].ptA.x);
//			cvSetReal2D(pAN, 8*n+1,	6*ptA_i + 1,	-pMatchPairs[n].ptA.y);
//			cvSetReal2D(pAN, 8*n+1,	6*ptA_i + 4,	-1);
//			cvSetReal2D(pBN, 8*n+1,	0,				dx - ptA_Bar.x);
//
//			cvSetReal2D(pAN, 8*n+2,	6*ptA_i + 2,	pMatchPairs[n].ptA.x);
//			cvSetReal2D(pAN, 8*n+2,	6*ptA_i + 3,	pMatchPairs[n].ptA.y);
//			cvSetReal2D(pAN, 8*n+2,	6*ptA_i + 5,	1);
//			cvSetReal2D(pBN, 8*n+2,	0,				dy + ptA_Bar.y);
//
//			cvSetReal2D(pAN, 8*n+3,	6*ptA_i + 2,	-pMatchPairs[n].ptA.x);
//			cvSetReal2D(pAN, 8*n+3,	6*ptA_i + 3,	-pMatchPairs[n].ptA.y);
//			cvSetReal2D(pAN, 8*n+3,	6*ptA_i + 5,	-1);
//			cvSetReal2D(pBN, 8*n+3,	0,				dy - ptA_Bar.y);
//		}
//		{
//			//---------------------------------------------------------------------------------------------
//			int ptB_i = pMatchPairs[n].ptB_i; // - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号
//			DfPoint ptB_Bar;
//			double hB64F[9];
//			for(int t=0; t<9; t++)
//			{
//				hB64F[t] = pImagesTransform0[ptB_i].h.m[t];
//			}
//			ApplyProject9(hB64F, pMatchPairs[n].ptA.x, pMatchPairs[n].ptA.y, ptB_Bar.x, ptB_Bar.y);
//			cvSetReal2D(pAN, 8*n+4,	6*ptB_i + 0,	pMatchPairs[n].ptA.x);
//			cvSetReal2D(pAN, 8*n+4,	6*ptB_i + 1,	pMatchPairs[n].ptA.y);
//			cvSetReal2D(pAN, 8*n+4,	6*ptB_i + 4,	1);
//			cvSetReal2D(pBN, 8*n+4,	0,				dx + ptB_Bar.x);
//
//			cvSetReal2D(pAN, 8*n+5,	6*ptB_i + 0,	-pMatchPairs[n].ptA.x);
//			cvSetReal2D(pAN, 8*n+5,	6*ptB_i + 1,	-pMatchPairs[n].ptA.y);
//			cvSetReal2D(pAN, 8*n+5,	6*ptB_i + 4,	-1);
//			cvSetReal2D(pBN, 8*n+5,	0,				dx - ptB_Bar.x);
//
//			cvSetReal2D(pAN, 8*n+6,	6*ptB_i + 2,	pMatchPairs[n].ptA.x);
//			cvSetReal2D(pAN, 8*n+6,	6*ptB_i + 3,	pMatchPairs[n].ptA.y);
//			cvSetReal2D(pAN, 8*n+6,	6*ptB_i + 5,	1);
//			cvSetReal2D(pBN, 8*n+6,	0,				dy + ptB_Bar.y);
//
//			cvSetReal2D(pAN, 8*n+7,	6*ptB_i + 2,	-pMatchPairs[n].ptA.x);
//			cvSetReal2D(pAN, 8*n+7,	6*ptB_i + 3,	-pMatchPairs[n].ptA.y);
//			cvSetReal2D(pAN, 8*n+7,	6*ptB_i + 5,	-1);
//			cvSetReal2D(pBN, 8*n+7,	0,				dy - ptB_Bar.y);
//		}
//	}	
//
//	//cvPrintMatrix(pA, "c:\\A.txt");
//	//cvPrintMatrix(pB, "c:\\B.txt");
//	cvPrintMatrix(pAN, "c:\\AN.txt");
//	cvPrintMatrix(pBN, "c:\\BN.txt");
//
//	PrintX0(pImagesTransform0, nImages);
//	PrintMatchPoints(pMatchPairs, nPairs);
//
//	fstream cinData;
//	cinData.open("c:\\X.txt");
//	int nData;
//	cinData>>nData;
//	for(int i=0; i<nData; i++)
//	{
//		ImageTransform temp;
//		cinData>>temp.h.m[0];
//		cinData>>temp.h.m[1];
//		cinData>>temp.h.m[3];
//		cinData>>temp.h.m[4];
//		cinData>>temp.h.m[2];
//		cinData>>temp.h.m[5];
//		for(int j=0; j<6; j++)
//		{
//			temp.h.m[j] *= 1000;
//		}
//		temp.h.m[6] = 0;
//		temp.h.m[7] = 0;
//		temp.h.m[8] = 1;
//		vecTransformRefined.push_back(temp);
//	}
//	cinData.close();
//
//	cvReleaseMat(&pA);
//	cvReleaseMat(&pA);
//	cvReleaseMat(&pH);
//	cvReleaseMat(&pX);
//	cvReleaseMat(&pB);
//
//	cvReleaseMat(&pAN);
//	cvReleaseMat(&pBN);
//
//	return 0;
//}

// QuadraticProgram方法
int CMosaicByPose::QuadraticProgram(const ImagePoseInfo *pImgPoses,
									MatchPointPairs* pMatchPairs, int nPairs,
									const ImageTransform* pImagesTransform0, int nImages, 
									int nFixedImages,
									vector<ImageTransform> &vecTransformRefined)
{	
	cout<<"Quadratic Program Start"<<endl<<endl;
	// AX = B, 用仿射变换模型
	// x2 = a.x1 + b.y1 + e
	// y2 = c.x1 + d.y1 + f
	if(nFixedImages<2)
	{
		//return -1;
	}

	int rowA = 2 * nPairs;
	int colA = 6 * nImages; // 第一幅图像变换参数固定
	int rowX = colA;
	int colX = 1;	
	int rowB = 2*nPairs;
	int colB = 1;

	int rowAN = 8 * nPairs;
	int colAN = 6 * nImages; // 第一幅图像变换参数固定
	
	int rowBN = rowAN;
	int colBN = 1;

	cout<<"rowA "<<rowA<<" rowB "<<rowB<<endl;

	CvMat* pA = cvCreateMat(rowA, colA, CV_64F); // 2*n * 6*m , n为特征点数, m为图像数
	CvMat* pH = cvCreateMat(colA, colA, CV_64F);
	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pB = cvCreateMat(rowB, colB, CV_64F);

	CvMat* pAN = cvCreateMat(rowAN, colAN, CV_64F);
	CvMat* pBN = cvCreateMat(rowBN, colBN, CV_64F);

	cvZero(pAN);
	cvZero(pBN);

	cvZero(pA);
	cvZero(pB);
	cvZero(pH);

	double dx = 50; // pixel
	double dy = dx;

	int nFixedProcess = 0;
	for(int n=0; n<nPairs; n++)
	{
		//cout<<n<<" / "<<nPairs<<endl;

		if(n==6241)
		{
			n=n;
		}

		{
			int ptA_i = pMatchPairs[n].ptA_i; // - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i; // - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

			// 2*n行, 6*i列
			// ptA
			cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
			cvSetReal2D(pA, 2*n, 6*ptA_i + 1, pMatchPairs[n].ptA.y);
			cvSetReal2D(pA, 2*n, 6*ptA_i + 4, 1);

			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 2, pMatchPairs[n].ptA.x);
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 3, pMatchPairs[n].ptA.y);
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 5, 1);

			// ptB
			cvSetReal2D(pA, 2*n, 6*ptB_i + 0, -pMatchPairs[n].ptB.x);
			cvSetReal2D(pA, 2*n, 6*ptB_i + 1, -pMatchPairs[n].ptB.y);
			cvSetReal2D(pA, 2*n, 6*ptB_i + 4, -1);

			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 2, -pMatchPairs[n].ptB.x);
			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 3, -pMatchPairs[n].ptB.y);
			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 5, -1);

			// 给B赋值
			cvSetReal2D(pB, 2*n, 0, 0);
			cvSetReal2D(pB, 2*n+1, 0, 0);
		}
		{
			int ptA_i = pMatchPairs[n].ptA_i; // - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			
			//==================================================================
			// 用初始参数对A点进行变换
			DfPoint ptA_Bar;
			double hA64F[9];
			for(int t=0; t<9; t++)
			{
				hA64F[t] = pImagesTransform0[ptA_i].h.m[t];
			}
			ApplyProject9(hA64F, pMatchPairs[n].ptA.x, pMatchPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);
			cvSetReal2D(pAN, 8*n,	6*ptA_i + 0,	pMatchPairs[n].ptA.x);
			cvSetReal2D(pAN, 8*n,	6*ptA_i + 1,	pMatchPairs[n].ptA.y);
			cvSetReal2D(pAN, 8*n,	6*ptA_i + 4,	1);
			cvSetReal2D(pBN, 8*n,	0,				dx + ptA_Bar.x);

			cvSetReal2D(pAN, 8*n+1,	6*ptA_i + 0,	-pMatchPairs[n].ptA.x);
			cvSetReal2D(pAN, 8*n+1,	6*ptA_i + 1,	-pMatchPairs[n].ptA.y);
			cvSetReal2D(pAN, 8*n+1,	6*ptA_i + 4,	-1);
			cvSetReal2D(pBN, 8*n+1,	0,				dx - ptA_Bar.x);

			cvSetReal2D(pAN, 8*n+2,	6*ptA_i + 2,	pMatchPairs[n].ptA.x);
			cvSetReal2D(pAN, 8*n+2,	6*ptA_i + 3,	pMatchPairs[n].ptA.y);
			cvSetReal2D(pAN, 8*n+2,	6*ptA_i + 5,	1);
			cvSetReal2D(pBN, 8*n+2,	0,				dy + ptA_Bar.y);

			cvSetReal2D(pAN, 8*n+3,	6*ptA_i + 2,	-pMatchPairs[n].ptA.x);
			cvSetReal2D(pAN, 8*n+3,	6*ptA_i + 3,	-pMatchPairs[n].ptA.y);
			cvSetReal2D(pAN, 8*n+3,	6*ptA_i + 5,	-1);
			cvSetReal2D(pBN, 8*n+3,	0,				dy - ptA_Bar.y);
		}
		{
			//---------------------------------------------------------------------------------------------
			int ptB_i = pMatchPairs[n].ptB_i; // - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号
			DfPoint ptB_Bar;
			double hB64F[9];
			for(int t=0; t<9; t++)
			{
				hB64F[t] = pImagesTransform0[ptB_i].h.m[t];
			}
			ApplyProject9(hB64F, pMatchPairs[n].ptA.x, pMatchPairs[n].ptA.y, ptB_Bar.x, ptB_Bar.y);
			cvSetReal2D(pAN, 8*n+4,	6*ptB_i + 0,	pMatchPairs[n].ptA.x);
			cvSetReal2D(pAN, 8*n+4,	6*ptB_i + 1,	pMatchPairs[n].ptA.y);
			cvSetReal2D(pAN, 8*n+4,	6*ptB_i + 4,	1);
			cvSetReal2D(pBN, 8*n+4,	0,				dx + ptB_Bar.x);

			cvSetReal2D(pAN, 8*n+5,	6*ptB_i + 0,	-pMatchPairs[n].ptA.x);
			cvSetReal2D(pAN, 8*n+5,	6*ptB_i + 1,	-pMatchPairs[n].ptA.y);
			cvSetReal2D(pAN, 8*n+5,	6*ptB_i + 4,	-1);
			cvSetReal2D(pBN, 8*n+5,	0,				dx - ptB_Bar.x);

			cvSetReal2D(pAN, 8*n+6,	6*ptB_i + 2,	pMatchPairs[n].ptA.x);
			cvSetReal2D(pAN, 8*n+6,	6*ptB_i + 3,	pMatchPairs[n].ptA.y);
			cvSetReal2D(pAN, 8*n+6,	6*ptB_i + 5,	1);
			cvSetReal2D(pBN, 8*n+6,	0,				dy + ptB_Bar.y);

			cvSetReal2D(pAN, 8*n+7,	6*ptB_i + 2,	-pMatchPairs[n].ptA.x);
			cvSetReal2D(pAN, 8*n+7,	6*ptB_i + 3,	-pMatchPairs[n].ptA.y);
			cvSetReal2D(pAN, 8*n+7,	6*ptB_i + 5,	-1);
			cvSetReal2D(pBN, 8*n+7,	0,				dy - ptB_Bar.y);
		}
	}	
	
	cvPrintMatrix(pA, "c:\\A.txt");
	cvPrintMatrix(pB, "c:\\B.txt");
	cvPrintMatrix(pAN, "c:\\AN.txt");
	cvPrintMatrix(pBN, "c:\\BN.txt");

	PrintX0(pImagesTransform0, nImages);
	PrintMatchPoints(pMatchPairs, nPairs);

	fstream cinData;
	cinData.open("c:\\X.txt");
	int nData;
	cinData>>nData;
	for(int i=0; i<nData; i++)
	{
		ImageTransform temp;
		cinData>>temp.h.m[0];
		cinData>>temp.h.m[1];
		cinData>>temp.h.m[3];
		cinData>>temp.h.m[4];
		cinData>>temp.h.m[2];
		cinData>>temp.h.m[5];
		for(int j=0; j<6; j++)
		{
			temp.h.m[j] *= 1000;
		}
		temp.h.m[6] = 0;
		temp.h.m[7] = 0;
		temp.h.m[8] = 1;
		vecTransformRefined.push_back(temp);
	}
	cinData.close();

	cvReleaseMat(&pA);
	cvReleaseMat(&pA);
	cvReleaseMat(&pH);
	cvReleaseMat(&pX);
	cvReleaseMat(&pB);

	cvReleaseMat(&pAN);
	cvReleaseMat(&pBN);

	return 0;
}


// Lagrange方法
int CMosaicByPose::BundleAdjustmentLagrange(const ImagePoseInfo *pImgPoses,
										MatchPointPairs* pMatchPairs, int nPairs,
										const ImageTransform* pImagesTransform0, int nImages, 
										int nFixedImages,
										vector<ImageTransform> &vecTransformRefined)
{	
	cout<<"Bundle Adjustment Start"<<endl<<endl;
	// AX = B, 用仿射变换模型
	// x2 = a.x1 + b.y1 + e
	// y2 = c.x1 + d.y1 + f
	if(nFixedImages<2)
	{
		return -1;
	}
	
	vector<int> vecAccFixed(nImages);
	memset(&vecAccFixed[0], 0, sizeof(int)*nImages);

	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<i; j++)
		{
			if(pImagesTransform0[j].fixed==1)
			{
				if(j==0)
				{
					vecAccFixed[i] += 1;
				}
			}
		}
	}

	int rowA = 2 * nPairs;
	int colA = 6 * (nImages-1); // 第一幅图像变换参数固定
	int rowX = colA;
	int colX = 1;	
	int rowB = 2*nPairs;
	int colB = 1;

	int rowAS = 2*(nFixedImages-1);
	int colAS = colA;

	cout<<"rowA "<<rowA<<" rowB "<<rowB<<endl;

	CvMat* pA = cvCreateMat(rowA, colA, CV_64F); // 2*n * 6*m , n为特征点数, m为图像数
	CvMat* pAT = cvCreateMat(colA, rowA, CV_64F);
	CvMat* pH = cvCreateMat(colA, colA, CV_64F);
	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pQC = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pB = cvCreateMat(rowB, colB, CV_64F);
	
	CvMat* pInvH = cvCreateMat(colA, colA, CV_64F);
	CvMat* pAS = cvCreateMat(rowAS, colAS, CV_64F);
	CvMat* pAST = cvCreateMat(colAS, rowAS, CV_64F);
	CvMat* pBS = cvCreateMat(rowAS, colB, CV_64F);
	CvMat* pAS_InvH = cvCreateMat(rowAS, colA, CV_64F);
	CvMat* pAS_InvH_AST = cvCreateMat(rowAS, rowAS, CV_64F);
	CvMat* pInv_AS_InvH_AST = cvCreateMat(rowAS, rowAS, CV_64F);
	CvMat* pR = cvCreateMat(rowAS, colA, CV_64F);
	CvMat* pRT = cvCreateMat(colA, rowAS, CV_64F);

	CvMat* pC = cvCreateMat(colAS, 1, CV_64F);
	CvMat* pQ = cvCreateMat(colA, colA, CV_64F);

	CvMat* pInvH_AST = cvCreateMat(colA, rowAS, CV_64F);

	cvZero(pAS);
	cvZero(pBS);

	cvZero(pA);
	cvZero(pB);
	cvZero(pX);

	int nFixedProcess = 0;
	for(int n=0; n<nPairs; n++)
	{
		cout<<n<<" / "<<nPairs<<endl;

		if(n==6241)
		{
			n=n;
		}

		if((pMatchPairs[n].ptA_Fixed==1) && (pMatchPairs[n].ptB_Fixed==0) && (pMatchPairs[n].ptA_i==0) ) // 如果A点固定
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

			// ptB
			cvSetReal2D(pA, 2*n, 6*ptB_i + 0, pMatchPairs[n].ptB.x);
			cvSetReal2D(pA, 2*n, 6*ptB_i + 1, pMatchPairs[n].ptB.y);
			cvSetReal2D(pA, 2*n, 6*ptB_i + 4, 1);

			cvSetReal2D(pA, 2*n+1, 6*ptB_i + 2, pMatchPairs[n].ptB.x);
			cvSetReal2D(pA, 2*n+1, 6*ptB_i + 3, pMatchPairs[n].ptB.y);
			cvSetReal2D(pA, 2*n+1, 6*ptB_i + 5, 1);

			// 用固定的参数对A点进行变换
			DfPoint ptA_Bar;
			double h64F[9];
			for(int t=0; t<9; t++)
			{
				h64F[t] = pImagesTransform0[ptA_i].h.m[t];
			}
			ApplyProject9(h64F, pMatchPairs[n].ptA.x, pMatchPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);

			// 给B赋值
			cvSetReal2D(pB, 2*n, 0, ptA_Bar.x);
			cvSetReal2D(pB, 2*n+1, 0, ptA_Bar.y);
		}
		else if((pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==1) && (pMatchPairs[n].ptB_i==0) ) // 如果B点固定
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

			// 2*n行, 6*i列
			// ptA
			cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
			cvSetReal2D(pA, 2*n, 6*ptA_i + 1, pMatchPairs[n].ptA.y);
			cvSetReal2D(pA, 2*n, 6*ptA_i + 4, 1);

			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 2, pMatchPairs[n].ptA.x);
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 3, pMatchPairs[n].ptA.y);
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 5, 1);

			// 用固定的参数对B点进行变换
			DfPoint ptB_Bar;
			double hB64F[9];
			for(int t=0; t<9; t++)
			{
				hB64F[t] = pImagesTransform0[ptB_i].h.m[t];
			}
			ApplyProject9(hB64F, pMatchPairs[n].ptB.x, pMatchPairs[n].ptB.y, ptB_Bar.x, ptB_Bar.y);
			// 给B赋值
			cvSetReal2D(pB, 2*n, 0, ptB_Bar.x);
			cvSetReal2D(pB, 2*n+1, 0, ptB_Bar.y);
		}
		else
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

			// 2*n行, 6*i列
			// ptA
			cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
			cvSetReal2D(pA, 2*n, 6*ptA_i + 1, pMatchPairs[n].ptA.y);
			cvSetReal2D(pA, 2*n, 6*ptA_i + 4, 1);

			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 2, pMatchPairs[n].ptA.x);
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 3, pMatchPairs[n].ptA.y);
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 5, 1);

			// ptB
			cvSetReal2D(pA, 2*n, 6*ptB_i + 0, -pMatchPairs[n].ptB.x);
			cvSetReal2D(pA, 2*n, 6*ptB_i + 1, -pMatchPairs[n].ptB.y);
			cvSetReal2D(pA, 2*n, 6*ptB_i + 4, -1);

			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 2, -pMatchPairs[n].ptB.x);
			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 3, -pMatchPairs[n].ptB.y);
			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 5, -1);

			// 给B赋值
			cvSetReal2D(pB, 2*n, 0, 0);
			cvSetReal2D(pB, 2*n+1, 0, 0);
		}
	}
	
	// 置AS, BS
	nFixedProcess = 0;
	for(int n=1; n<nImages; n++)
	{
		if(pImgPoses[n].fixed==1)
		{
			double cx = (pImgPoses[n].pImg->width-1)*0.5;
			double cy = (pImgPoses[n].pImg->height-1)*0.5;

			// ptB
			cvSetReal2D(pAS, 2*nFixedProcess, 6*(n-1) + 0, cx);
			cvSetReal2D(pAS, 2*nFixedProcess, 6*(n-1) + 1, cy);
			cvSetReal2D(pAS, 2*nFixedProcess, 6*(n-1) + 4, 1);

			cvSetReal2D(pAS, (2*nFixedProcess+1), 6*(n-1) + 2, cx);
			cvSetReal2D(pAS, (2*nFixedProcess+1), 6*(n-1) + 3, cy);
			cvSetReal2D(pAS, (2*nFixedProcess+1), 6*(n-1) + 5, 1);

			// 用固定的参数对B点进行变换
			DfPoint pt_Bar;
			double hB64F[9];
			for(int t=0; t<9; t++)
			{
				hB64F[t] = pImagesTransform0[n].h.m[t];
			}
			ApplyProject9(hB64F, cx, cy, pt_Bar.x, pt_Bar.y);

			// 给BS赋值
			cvSetReal2D(pBS, 2*nFixedProcess, 0, pt_Bar.x);
			cvSetReal2D(pBS, 2*nFixedProcess+1, 0, pt_Bar.y);
			nFixedProcess++;
		}
	}

	cvTranspose(pA, pAT);
	
	cvTranspose(pAS, pAST);

	cvGEMM(pAST, pBS, -2, NULL, 0, pC);
	
	cvGEMM(pAT, pA, 1, NULL, 0, pH); // 矩阵乘法

	cvPrintMatrix(pH, "c:\\H.txt");
	cvPrintMatrix(pAS, "c:\\A.txt");
	cvPrintMatrix(pBS, "c:\\B.txt");
	cvPrintMatrix(pC, "c:\\C.txt");

	double detH = cvDet(pH);

	cvInvert(pH, pInvH);

	cvGEMM(pInvH, pAST, 1, NULL, 0, pInvH_AST);

	cvGEMM(pAS, pInvH, 1, NULL, 0, pAS_InvH);

	cvGEMM(pAS_InvH, pAST, 1, NULL, 0, pAS_InvH_AST);

	cvInvert(pAS_InvH_AST, pInv_AS_InvH_AST);

	cvGEMM(pInv_AS_InvH_AST, pAS_InvH, 1, NULL, 0, pR);

	cvGEMM(pInvH_AST, pR, 1, NULL, 0, pQ);

	cvSub(pInvH, pQ, pQ);

	cvTranspose(pR, pRT);

	cvGEMM(pRT, pBS, 1, NULL, 0, pX);

	cvGEMM(pQ, pC, -1, NULL, 0, pQC); // pQC = -pQC
	
	cvAdd(pQC, pX, pX);

	//// 解线性方程组
	//cout<<"解线性方程组"<<endl<<endl;
	//cvSolve(pA, pB, pX, CV_SVD);

	int nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		//if(pImagesTransform0[i].fixed==0) // 未固定参数情况
		if(i==0)
		{
			vecTransformRefined.push_back(pImagesTransform0[0]);
		}
		else
		{
			ImageTransform temp;
			temp.fixed = 0;
			temp.h.m[0] = (float)cvGetReal2D(pX, 6*nUnFixed+0, 0); // a
			temp.h.m[1] = (float)cvGetReal2D(pX, 6*nUnFixed+1, 0); // b
			temp.h.m[3] = (float)cvGetReal2D(pX, 6*nUnFixed+2, 0); // c
			temp.h.m[4] = (float)cvGetReal2D(pX, 6*nUnFixed+3, 0); // d
			temp.h.m[2] = (float)cvGetReal2D(pX, 6*nUnFixed+4, 0); // e
			temp.h.m[5] = (float)cvGetReal2D(pX, 6*nUnFixed+5, 0); // f

			nUnFixed++;

			temp.h.m[6] = 0;
			temp.h.m[7] = 0;
			temp.h.m[8] = 1;
			vecTransformRefined.push_back(temp);
		}
		//else
		//{
		//	vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
		//}
	}

	cvReleaseMat(&pA);
	cvReleaseMat(&pB);
	cvReleaseMat(&pX);
	cvReleaseMat(&pA);
	cvReleaseMat(&pAT);
	cvReleaseMat(&pX);
	cvReleaseMat(&pB);
	cvReleaseMat(&pH);
	cvReleaseMat(&pInvH);
	cvReleaseMat(&pAS);
	cvReleaseMat(&pAST);
	cvReleaseMat(&pBS);
	cvReleaseMat(&pAS_InvH);
	cvReleaseMat(&pAS_InvH_AST);
	cvReleaseMat(&pInv_AS_InvH_AST);
	cvReleaseMat(&pR);
	cvReleaseMat(&pRT);

	return 0;
}

// 第一张图像固定
// 其他的固定的图像的中心点不变, 其余4个自由度可变, 固定的中心点, 直接加入矩阵A
int CMosaicByPose::BundleAdjustmentLS(const ImagePoseInfo *pImgPoses,
									MatchPointPairs* pMatchPairs, int nPairs,
									const ImageTransform* pImagesTransform0, int nImages, 
									int nFixedImages,
									vector<ImageTransform> &vecTransformRefined)
{	
	cout<<"Bundle Adjustment Start"<<endl<<endl;
	// AX = B, 用仿射变换模型
	// x2 = a.x1 + b.y1 + e
	// y2 = c.x1 + d.y1 + f

	vector<int> vecAccFixed(nImages);
	memset(&vecAccFixed[0], 0, sizeof(int)*nImages);

	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<i; j++)
		{
			if(pImagesTransform0[j].fixed==1)
			{
				if(j==0)
				{
					vecAccFixed[i] += 1;
				}
			}
		}
	}

	int rowA = 2 * nPairs + nFixedImages*2;
	int colA = 6 * (nImages-1);
	int rowX = colA;
	int colX = 1;
	int rowB = rowA;
	int colB = 1;

	cout<<"rowA "<<rowA<<" rowB "<<rowB<<endl;

	CvMat* pA = cvCreateMat(rowA, colA, CV_64F); // 2*n * 6*m , n为特征点数, m为图像数
	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pB = cvCreateMat(rowB, colB, CV_64F);

	cvZero(pA);
	cvZero(pB);
	cvZero(pX);

	int nFixedProcess = 0;
	for(int n=0; n<nPairs; n++)
	{
		cout<<n<<" / "<<nPairs<<endl;

		if(n==6241)
		{
			n=n;
		}

		if((pMatchPairs[n].ptA_Fixed==1) && (pMatchPairs[n].ptB_Fixed==0) && (pMatchPairs[n].ptA_i==0) ) // 如果A点固定
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

			// ptB
			cvSetReal2D(pA, 2*n, 6*ptB_i + 0, pMatchPairs[n].ptB.x);
			cvSetReal2D(pA, 2*n, 6*ptB_i + 1, pMatchPairs[n].ptB.y);
			cvSetReal2D(pA, 2*n, 6*ptB_i + 4, 1);

			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 2, pMatchPairs[n].ptB.x);
			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 3, pMatchPairs[n].ptB.y);
			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 5, 1);

			// 用固定的参数对A点进行变换
			DfPoint ptA_Bar;
			double h64F[9];
			for(int t=0; t<9; t++)
			{
				h64F[t] = pImagesTransform0[ptA_i].h.m[t];
			}
			ApplyProject9(h64F, pMatchPairs[n].ptA.x, pMatchPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);

			// 给B赋值
			cvSetReal2D(pB, 2*n, 0, ptA_Bar.x);
			cvSetReal2D(pB, 2*n+1, 0, ptA_Bar.y);
		}
		else if((pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==1) && (pMatchPairs[n].ptB_i==0) ) // 如果B点固定
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

			// 2*n行, 6*i列
			// ptA
			cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
			cvSetReal2D(pA, 2*n, 6*ptA_i + 1, pMatchPairs[n].ptA.y);
			cvSetReal2D(pA, 2*n, 6*ptA_i + 4, 1);

			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 2, pMatchPairs[n].ptA.x);
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 3, pMatchPairs[n].ptA.y);
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 5, 1);

			// 用固定的参数对B点进行变换
			DfPoint ptB_Bar;
			double hB64F[9];
			for(int t=0; t<9; t++)
			{
				hB64F[t] = pImagesTransform0[ptB_i].h.m[t];
			}
			ApplyProject9(hB64F, pMatchPairs[n].ptB.x, pMatchPairs[n].ptB.y, ptB_Bar.x, ptB_Bar.y);
			// 给B赋值
			cvSetReal2D(pB, 2*n, 0, ptB_Bar.x);
			cvSetReal2D(pB, 2*n+1, 0, ptB_Bar.y);
		}
		else
		{
			int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
			int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

			// 2*n行, 6*i列
			// ptA
			cvSetReal2D(pA, 2*n, 6*ptA_i + 0, pMatchPairs[n].ptA.x);
			cvSetReal2D(pA, 2*n, 6*ptA_i + 1, pMatchPairs[n].ptA.y);
			cvSetReal2D(pA, 2*n, 6*ptA_i + 4, 1);

			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 2, pMatchPairs[n].ptA.x);
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 3, pMatchPairs[n].ptA.y);
			cvSetReal2D(pA, (2*n+1), 6*ptA_i + 5, 1);

			// ptB
			cvSetReal2D(pA, 2*n, 6*ptB_i + 0, -pMatchPairs[n].ptB.x);
			cvSetReal2D(pA, 2*n, 6*ptB_i + 1, -pMatchPairs[n].ptB.y);
			cvSetReal2D(pA, 2*n, 6*ptB_i + 4, -1);

			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 2, -pMatchPairs[n].ptB.x);
			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 3, -pMatchPairs[n].ptB.y);
			cvSetReal2D(pA, (2*n+1), 6*ptB_i + 5, -1);

			// 给B赋值
			cvSetReal2D(pB, 2*n, 0, 0);
			cvSetReal2D(pB, 2*n+1, 0, 0);
		}
	}

	nFixedProcess = 0;
	for(int n=1; n<nImages; n++)
	{
		if( (pImgPoses[n].fixed==1) && (nFixedProcess>0) ) // 第一张图片固定
		{
			double cx = (pImgPoses[n].pImg->width-1)*0.5;
			double cy = (pImgPoses[n].pImg->height-1)*0.5;

			// ptB
			cvSetReal2D(pA, 2*nPairs + 2*nFixedProcess, 6*n + 0, cx);
			cvSetReal2D(pA, 2*nPairs + 2*nFixedProcess, 6*n + 1, cy);
			cvSetReal2D(pA, 2*nPairs + 2*nFixedProcess, 6*n + 4, 1);

			cvSetReal2D(pA, 2*nPairs + 2*nFixedProcess+1, 6*n + 2, cx);
			cvSetReal2D(pA, 2*nPairs + 2*nFixedProcess+1, 6*n + 3, cy);
			cvSetReal2D(pA, 2*nPairs + 2*nFixedProcess+1, 6*n + 5, 1);

			// 用固定的参数对B点进行变换
			DfPoint pt_Bar;
			double hB64F[9];
			for(int t=0; t<9; t++)
			{
				hB64F[t] = pImagesTransform0[n].h.m[t];
			}
			ApplyProject9(hB64F, cx, cy, pt_Bar.x, pt_Bar.y);

			// 给BS赋值
			cvSetReal2D(pB, 2*nPairs + 2*nFixedProcess, 0, pt_Bar.x);
			cvSetReal2D(pB, 2*nPairs + 2*nFixedProcess+1, 0, pt_Bar.y);

			nFixedProcess++;
		}
	}

	//// 解线性方程组
	//cout<<"解线性方程组"<<endl<<endl;
	cvSolve(pA, pB, pX, CV_SVD);

	int nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(i==0)
		{
			vecTransformRefined.push_back(pImagesTransform0[0]);
		}
		else
		{
			ImageTransform temp;
			temp.fixed = 0;
			temp.h.m[0] = (float)cvGetReal2D(pX, 6*nUnFixed+0, 0); // a
			temp.h.m[1] = (float)cvGetReal2D(pX, 6*nUnFixed+1, 0); // b
			temp.h.m[3] = (float)cvGetReal2D(pX, 6*nUnFixed+2, 0); // c
			temp.h.m[4] = (float)cvGetReal2D(pX, 6*nUnFixed+3, 0); // d
			temp.h.m[2] = (float)cvGetReal2D(pX, 6*nUnFixed+4, 0); // e
			temp.h.m[5] = (float)cvGetReal2D(pX, 6*nUnFixed+5, 0); // f

			nUnFixed++;

			temp.h.m[6] = 0;
			temp.h.m[7] = 0;
			temp.h.m[8] = 1;
			vecTransformRefined.push_back(temp);
		}
	}

	cvReleaseMat(&pA);
	cvReleaseMat(&pB);
	cvReleaseMat(&pX);

	return 0;
}

// 非线性稀疏捆集调整
int CMosaicByPose::BANonlinearSparse(MatchPointPairs* pMatchPairs, int nPairs,
											 ImageTransform* pImagesTransform0, int nImages, 
											 int nFixedImages,
											 vector<ImageTransform> &vecTransformRefined)
{	
	cout<<"Bundle Adjustment Nonlinear Start"<<endl<<endl;

	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<9; j++)
		{
			if(pImagesTransform0[i].h.m[8]!=0)
				pImagesTransform0[i].h.m[j] /= pImagesTransform0[i].h.m[8];
		}
	}

	int rowA = 2 * nPairs;
	int colA = 8 * (nImages-nFixedImages);
	int rowX = 8 * (nImages-nFixedImages);
	int colX = 1;
	int rowB = 2*nPairs;
	int colB = 1;

	//cout<<"rowA "<<rowA<<" rowB "<<rowB<<endl;

	CvMat* pA = cvCreateMat(rowA, colA, CV_64F); // 2*n * 6*m , n为特征点数, m为图像数
	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pDX = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pB = cvCreateMat(rowB, colB, CV_64F);
	cvZero(pA);
	cvZero(pB);
	cvZero(pDX);

	// 初始化X
	int nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		//pImagesTransform0[i].fixed = 0;//9999999999999999999999999999999999999999999999999999

		if(pImagesTransform0[i].fixed==0) // 未固定参数情况
		{
			for(int j=0; j<8; j++)
			{
				cvSetReal2D(pX, 8*nUnFixed+j, 0, pImagesTransform0[i].h.m[j]);
			}
			nUnFixed++;
		}
	}

	// 处理固定的参数对pX存储的影响
	vector<int> vecAccFixed(nImages);
	memset(&vecAccFixed[0], 0, sizeof(int)*nImages);

	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<i; j++)
		{
			if(pImagesTransform0[j].fixed==1)
			{
				vecAccFixed[i] += 1;
			}
		}
	}

	const int MAX_ITER = 1;
	// 迭代
	for(int iter = 0; iter<MAX_ITER; iter++)
	{
		for(int n=0; n<nPairs; n++)
		{
			//pMatchPairs[n].ptA_Fixed = 0;
			//pMatchPairs[n].ptB_Fixed = 0;
			//cout<<n<<" / "<<nPairs<<endl;
			if(n==782)
			{
				n=n;
			}
			if( (pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==0) ) // 如果A点和B点都不是固定点
			{
				// 2*n行, 8*i列
				// ptA
				int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号

				double hA[9];
				for(int i=0; i<8; i++)
				{
					hA[i] = cvGetReal2D(pX, 8*ptA_i+i, 0);
				}
				hA[8] = 1;
				double xA = pMatchPairs[n].ptA.x, yA = pMatchPairs[n].ptA.y;
				double h6x_h7y_1_a  = hA[6]*xA + hA[7]*yA + 1;
				double h0x_h1y_h2_a = hA[0]*xA + hA[1]*yA + hA[2];
				cvSetReal2D(pA, 2*n, 8*ptA_i + 0, xA / h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n, 8*ptA_i + 1, yA / h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n, 8*ptA_i + 2, 1 / h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n, 8*ptA_i + 6, -h0x_h1y_h2_a*xA / (h6x_h7y_1_a*h6x_h7y_1_a));
				cvSetReal2D(pA, 2*n, 8*ptA_i + 7, -h0x_h1y_h2_a*yA / (h6x_h7y_1_a*h6x_h7y_1_a));

				double h3x_h4y_h5_a = hA[3]*xA + hA[4]*yA + hA[5];
				cvSetReal2D(pA, 2*n+1, 8*ptA_i + 3, xA / h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n+1, 8*ptA_i + 4, yA / h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n+1, 8*ptA_i + 5, 1 / h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n+1, 8*ptA_i + 6, -h3x_h4y_h5_a*xA / (h6x_h7y_1_a*h6x_h7y_1_a));
				cvSetReal2D(pA, 2*n+1, 8*ptA_i + 7, -h3x_h4y_h5_a*yA / (h6x_h7y_1_a*h6x_h7y_1_a));

				int ptB_i0 = pMatchPairs[n].ptB_i;
				int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号
				// ptB
				double hB[9];
				for(int i=0; i<8; i++)
				{
					hB[i] = cvGetReal2D(pX, 8*ptB_i+i, 0);
				}
				hB[8] = 1;
				double xB = pMatchPairs[n].ptB.x, yB = pMatchPairs[n].ptB.y;
				double h6x_h7y_1_b  = hB[6]*xB + hB[7]*yB + 1;
				double h0x_h1y_h2_b = hB[0]*xB + hB[1]*yB + hB[2];
				cvSetReal2D(pA, 2*n, 8*ptB_i + 0, -xB / h6x_h7y_1_b);
				cvSetReal2D(pA, 2*n, 8*ptB_i + 1, -yB / h6x_h7y_1_b);
				cvSetReal2D(pA, 2*n, 8*ptB_i + 2, -1 / h6x_h7y_1_b);
				cvSetReal2D(pA, 2*n, 8*ptB_i + 6, h0x_h1y_h2_b*xB / (h6x_h7y_1_b*h6x_h7y_1_b));
				cvSetReal2D(pA, 2*n, 8*ptB_i + 7, h0x_h1y_h2_b*yB / (h6x_h7y_1_b*h6x_h7y_1_b));

				double h3x_h4y_h5_b = hB[3]*xB + hB[4]*yB + hB[5];
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 3, -xB / h6x_h7y_1_b);
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 4, -yB / h6x_h7y_1_b);
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 5, -1 / h6x_h7y_1_b);
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 6, h3x_h4y_h5_b*xB / (h6x_h7y_1_b*h6x_h7y_1_b));
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 7, h3x_h4y_h5_b*yB / (h6x_h7y_1_b*h6x_h7y_1_b));

				// 给B赋值
				cvSetReal2D(pB, 2*n, 0, h0x_h1y_h2_b/h6x_h7y_1_b - h0x_h1y_h2_a/h6x_h7y_1_a);
				cvSetReal2D(pB, 2*n+1, 0, h3x_h4y_h5_b/h6x_h7y_1_b - h3x_h4y_h5_a/h6x_h7y_1_a);
			}
			else if((pMatchPairs[n].ptA_Fixed==1) && (pMatchPairs[n].ptB_Fixed==0)) // 如果A点固定
			{
				int ptA_i0 = pMatchPairs[n].ptA_i;
				int ptB_i0 = pMatchPairs[n].ptB_i;
				int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号

				// ptB
				int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号
				// ptB
				double hB[9];
				for(int i=0; i<8; i++)
				{
					hB[i] = cvGetReal2D(pX, 8*ptB_i+i, 0);
				}
				hB[8] = 1;
				double xB = pMatchPairs[n].ptB.x, yB = pMatchPairs[n].ptB.y;
				double h6x_h7y_1  = hB[6]*xB + hB[7]*yB + 1;
				double h0x_h1y_h2 = hB[0]*xB + hB[1]*yB + hB[2];
				cvSetReal2D(pA, 2*n, 8*ptB_i + 0, xB/h6x_h7y_1);
				cvSetReal2D(pA, 2*n, 8*ptB_i + 1, yB/h6x_h7y_1);
				cvSetReal2D(pA, 2*n, 8*ptB_i + 2, 1/h6x_h7y_1);
				cvSetReal2D(pA, 2*n, 8*ptB_i + 6, -h0x_h1y_h2*xB / (h6x_h7y_1*h6x_h7y_1));
				cvSetReal2D(pA, 2*n, 8*ptB_i + 7, -h0x_h1y_h2*yB / (h6x_h7y_1*h6x_h7y_1));

				double h3x_h4y_h5 = hB[3]*xB + hB[4]*yB + hB[5];
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 3, xB/h6x_h7y_1);
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 4, yB/h6x_h7y_1);
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 5, 1/h6x_h7y_1);
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 6, -h3x_h4y_h5*xB / (h6x_h7y_1*h6x_h7y_1));
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 7, -h3x_h4y_h5*yB / (h6x_h7y_1*h6x_h7y_1));

				// 用固定的参数对A点进行变换
				DfPoint ptA_Bar;
				double h64F[9];
				for(int t=0; t<9; t++)
				{
					h64F[t] = pImagesTransform0[ptA_i0].h.m[t];
				}
				ApplyProject9(h64F, pMatchPairs[n].ptA.x, pMatchPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);

				// 给B赋值
				cvSetReal2D(pB, 2*n, 0, ptA_Bar.x - h0x_h1y_h2 / h6x_h7y_1);
				cvSetReal2D(pB, 2*n+1, 0, ptA_Bar.y - h3x_h4y_h5 / h6x_h7y_1);
			}
			else if((pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==1)) // 如果B点固定
			{
				int ptA_i0 = pMatchPairs[n].ptA_i;
				int ptB_i0 = pMatchPairs[n].ptB_i;
				int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

				// 2*n行, 8*i列
				// ptA
				int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号

				double hA[9];
				for(int i=0; i<8; i++)
				{
					hA[i] = cvGetReal2D(pX, 8*ptA_i+i, 0);
				}
				hA[8] = 1;
				double xA = pMatchPairs[n].ptA.x, yA = pMatchPairs[n].ptA.y;
				double h6x_h7y_1_a  = hA[6]*xA + hA[7]*yA + 1;
				double h0x_h1y_h2_a = hA[0]*xA + hA[1]*yA + hA[2];
				cvSetReal2D(pA, 2*n, 8*ptA_i + 0, xA/h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n, 8*ptA_i + 1, yA/h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n, 8*ptA_i + 2, 1/h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n, 8*ptA_i + 6, -h0x_h1y_h2_a*xA / (h6x_h7y_1_a*h6x_h7y_1_a));
				cvSetReal2D(pA, 2*n, 8*ptA_i + 7, -h0x_h1y_h2_a*yA / (h6x_h7y_1_a*h6x_h7y_1_a));

				double h3x_h4y_h5_a = hA[3]*xA + hA[4]*yA + hA[5];
				cvSetReal2D(pA, (2*n+1), 8*ptA_i + 3, xA/h6x_h7y_1_a);
				cvSetReal2D(pA, (2*n+1), 8*ptA_i + 4, yA/h6x_h7y_1_a);
				cvSetReal2D(pA, (2*n+1), 8*ptA_i + 5, 1/h6x_h7y_1_a);
				cvSetReal2D(pA, (2*n+1), 8*ptA_i + 6, -h3x_h4y_h5_a*xA/(h6x_h7y_1_a*h6x_h7y_1_a));
				cvSetReal2D(pA, (2*n+1), 8*ptA_i + 7, -h3x_h4y_h5_a*yA/(h6x_h7y_1_a*h6x_h7y_1_a));

				// 用固定的参数对B点进行变换
				DfPoint ptB_Bar;
				double hB64F[9];
				for(int t=0; t<9; t++)
				{
					hB64F[t] = pImagesTransform0[ptB_i0].h.m[t];
				}
				ApplyProject9(hB64F, pMatchPairs[n].ptB.x, pMatchPairs[n].ptB.y, ptB_Bar.x, ptB_Bar.y);
				// 给B赋值
				cvSetReal2D(pB, 2*n, 0, ptB_Bar.x - h0x_h1y_h2_a/h6x_h7y_1_a);
				cvSetReal2D(pB, 2*n+1, 0, ptB_Bar.y - h3x_h4y_h5_a/h6x_h7y_1_a);
			}
		}

		// 整理计算结果
		vecTransformRefined.clear();
		nUnFixed = 0;
		for(int i=0; i<nImages; i++)
		{
			if(pImagesTransform0[i].fixed==0) // 未固定参数情况
			{
				ImageTransform temp;
				temp.fixed = 0;
				for(int j=0; j<8; j++)
				{
					temp.h.m[j] = (float)cvGetReal2D(pX, 8*nUnFixed+j, 0); // a
				}
				temp.h.m[8] = 1;
				nUnFixed++;
				vecTransformRefined.push_back(temp);
			}
			else
			{
				vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
			}
		}

		double errorOld = CalMatchError(&vecTransformRefined[0], nImages, pMatchPairs, nPairs);

		// CV解线性方程组
		cout<<"解线性方程组"<<endl<<endl;
		//int ret = cvSolve(pA, pB, pDX, CV_SVD);

		// cholmod解稀疏线性方程组
		SolveSparseSystem(pA, pB, pDX);

		// 更新
		for(int j=0; j<rowX; j++)
		{
			double x= cvGetReal2D(pX, j, 0);
			double dx = cvGetReal2D(pDX, j, 0);
			x += dx;
			cvSetReal2D(pX, j, 0, x);
		}

		// 整理计算结果
		vecTransformRefined.clear();
		nUnFixed = 0;
		for(int i=0; i<nImages; i++)
		{
			if(pImagesTransform0[i].fixed==0) // 未固定参数情况
			{
				ImageTransform temp;
				temp.fixed = 0;
				for(int j=0; j<8; j++)
				{
					temp.h.m[j] = (float)cvGetReal2D(pX, 8*nUnFixed+j, 0); // a
				}
				temp.h.m[8] = 1;
				nUnFixed++;
				vecTransformRefined.push_back(temp);
			}
			else
			{
				vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
			}
		}

		double errorNew = CalMatchError(&vecTransformRefined[0], nImages, pMatchPairs, nPairs);
		cout<<"errorOld "<<errorOld<<" errorNew "<<errorNew<<endl;
	}

	// 整理计算结果
	vecTransformRefined.clear();
	nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(pImagesTransform0[i].fixed==0) // 未固定参数情况
		{
			ImageTransform temp;
			temp.fixed = 0;
			for(int j=0; j<8; j++)
			{
				temp.h.m[j] = (float)cvGetReal2D(pX, 8*nUnFixed+j, 0); // a
			}
			temp.h.m[8] = 1;
			nUnFixed++;
			vecTransformRefined.push_back(temp);
		}
		else
		{
			vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
		}
	}

	cvReleaseMat(&pA);
	cvReleaseMat(&pB);
	cvReleaseMat(&pX);
	cvReleaseMat(&pDX);

	return 0;
}

// 非线性稀疏捆集调整 + rot约束 - v2
int CMosaicByPose::SparseNonlinearRotConstraint_V2(MatchPointPairs* pMatchPairs, int nPairs,
												ImageTransform* pImagesTransform0, int nImages, 
												int nFixedImages, int refImgIdx0,
												int weight,
												vector<ImageTransform> &vecTransformRefined,
												int label[10000])
{	
	cout<<"SparseNonlinearRotConstraint_V2 Start"<<endl<<endl;

	vector<int> freqMatch(nImages); // 某幅图像中匹配对出现的频率
	memset(&freqMatch[0], 0, sizeof(int)*nImages);

	for(int n=0; n<nPairs; n++)
	{
		//if(pMatchPairs[n].ptA_Fixed==0)
		{
			freqMatch[pMatchPairs[n].ptA_i]++;
		}
		//if(pMatchPairs[n].ptB_Fixed==0)
		{
			freqMatch[pMatchPairs[n].ptB_i]++;
		}
	}

	double totalFreq = 0;
	for(int n=0; n<nImages; n++)
	{
		totalFreq += freqMatch[n];
	}

	for(int n=0; n<nImages; n++)
	{
		freqMatch[n] = weight*freqMatch[n];
		//freqMatch[n] = weight;
	}

	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<9; j++)
		{
			if(pImagesTransform0[i].h.m[8]>0)
				pImagesTransform0[i].h.m[j] /= pImagesTransform0[i].h.m[8];
		}
	}

	int rowA = 2 * nPairs + 4 * (nImages-nFixedImages);
	int colA = 8 * (nImages-nFixedImages);
	int rowX = colA;
	int colX = 1;
	int rowB = rowA;
	int colB = 1;

	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pX2 = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pDX = cvCreateMat(rowX, colX, CV_64F);

	cvZero(pDX);

	// 初始化X
	int nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(pImagesTransform0[i].fixed!=1) // 未固定参数情况(包括0 和 2)
		{
			for(int j=0; j<8; j++)
			{
				cvSetReal2D(pX, 8*nUnFixed+j, 0, pImagesTransform0[i].h.m[j]);
			}
			nUnFixed++;
		}
	}

	// 处理固定的参数对pX存储的影响
	vector<int> vecAccFixed(nImages);
	memset(&vecAccFixed[0], 0, sizeof(int)*nImages);

	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<i; j++)
		{
			if(pImagesTransform0[j].fixed==1)
			{
				vecAccFixed[i] += 1;
			}
		}
	}

	pool::SparseMatrix eye;
	eye.mat.resize(colA);
	eye.col = colA;
	eye.row = colA;
	for(int i=0; i<colA; i++)
	{
		eye.mat[i].matCol.push_back(SparseMatElem(i, 1)); // 列 行
	}

	double beta = 10;
	double lambda64F = 0.00001;
	vector<double> vecCost, vecCost2, vecCostRef;
	const int MAX_ITER = 10;
	// 迭代
	for(int iter = 0; iter<MAX_ITER; iter++)
	{
		double cost1 = 0;
		pool::SparseMatrix A;
		pool::SparseMatrix B;
		A.mat.resize(colA);
		B.mat.resize(colB);
		A.col = colA;
		A.row = rowA;
		B.col = colB;
		B.row = rowB;

		//double wCorr = ;

		for(int n=0; n<nPairs; n++)
		{
			if( (pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==0) ) // 如果A点和B点都不是固定点
			{
				// 2*n行, 8*i列
				// ptA
				int ptA_i0 = pMatchPairs[n].ptA_i;
				int ptA_i = ptA_i0 - vecAccFixed[ptA_i0]; // ptA所在的图像的序号

				double hA[9];
				for(int i=0; i<8; i++)
				{
					hA[i] = cvGetReal2D(pX, 8*ptA_i+i, 0);
				}
				hA[8] = 1;

				double xA = pMatchPairs[n].ptA.x;
				double yA = pMatchPairs[n].ptA.y;

				double h0x_h1y_h2_a = hA[0]*xA + hA[1]*yA + hA[2];
				double h3x_h4y_h5_a = hA[3]*xA + hA[4]*yA + hA[5];
				double h6x_h7y_1_a  = hA[6]*xA + hA[7]*yA + 1;

				A.mat[8*ptA_i + 0].matCol.push_back(SparseMatElem(2*n, xA / h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 1].matCol.push_back(SparseMatElem(2*n, yA / h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 2].matCol.push_back(SparseMatElem(2*n, 1 / h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 6].matCol.push_back(SparseMatElem(2*n, -h0x_h1y_h2_a*xA / (h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行
				A.mat[8*ptA_i + 7].matCol.push_back(SparseMatElem(2*n, -h0x_h1y_h2_a*yA / (h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行

				A.mat[8*ptA_i + 3].matCol.push_back(SparseMatElem(2*n+1, xA / h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 4].matCol.push_back(SparseMatElem(2*n+1, yA / h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1 / h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 6].matCol.push_back(SparseMatElem(2*n+1, -h3x_h4y_h5_a*xA / (h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行
				A.mat[8*ptA_i + 7].matCol.push_back(SparseMatElem(2*n+1, -h3x_h4y_h5_a*yA / (h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行

				int ptB_i0 = pMatchPairs[n].ptB_i;
				int ptB_i = ptB_i0 - vecAccFixed[ptB_i0]; // ptB所在的图像的序号

				// ptB
				double hB[9];
				for(int i=0; i<8; i++)
				{
					hB[i] = cvGetReal2D(pX, 8*ptB_i+i, 0);
				}
				hB[8] = 1;
				double xB = pMatchPairs[n].ptB.x;
				double yB = pMatchPairs[n].ptB.y;

				double h0x_h1y_h2_b = hB[0]*xB + hB[1]*yB + hB[2];
				double h3x_h4y_h5_b = hB[3]*xB + hB[4]*yB + hB[5];
				double h6x_h7y_1_b  = hB[6]*xB + hB[7]*yB + 1;

				A.mat[8*ptB_i + 0].matCol.push_back(SparseMatElem(2*n, -xB / h6x_h7y_1_b)); // 列 行
				A.mat[8*ptB_i + 1].matCol.push_back(SparseMatElem(2*n, -yB / h6x_h7y_1_b)); // 列 行
				A.mat[8*ptB_i + 2].matCol.push_back(SparseMatElem(2*n, -1 / h6x_h7y_1_b)); // 列 行
				A.mat[8*ptB_i + 6].matCol.push_back(SparseMatElem(2*n, h0x_h1y_h2_b*xB / (h6x_h7y_1_b*h6x_h7y_1_b))); // 列 行
				A.mat[8*ptB_i + 7].matCol.push_back(SparseMatElem(2*n, h0x_h1y_h2_b*yB / (h6x_h7y_1_b*h6x_h7y_1_b))); // 列 行

				A.mat[8*ptB_i + 3].matCol.push_back(SparseMatElem(2*n+1, -xB / h6x_h7y_1_b)); // 列 行
				A.mat[8*ptB_i + 4].matCol.push_back(SparseMatElem(2*n+1, -yB / h6x_h7y_1_b)); // 列 行
				A.mat[8*ptB_i + 5].matCol.push_back(SparseMatElem(2*n+1, -1 / h6x_h7y_1_b)); // 列 行
				A.mat[8*ptB_i + 6].matCol.push_back(SparseMatElem(2*n+1, h3x_h4y_h5_b*xB / (h6x_h7y_1_b*h6x_h7y_1_b))); // 列 行
				A.mat[8*ptB_i + 7].matCol.push_back(SparseMatElem(2*n+1, h3x_h4y_h5_b*yB / (h6x_h7y_1_b*h6x_h7y_1_b))); // 列 行

				// 给B赋值
				B.mat[0].matCol.push_back(SparseMatElem(2*n,   h0x_h1y_h2_a/h6x_h7y_1_a - h0x_h1y_h2_b/h6x_h7y_1_b)); // 列 行
				B.mat[0].matCol.push_back(SparseMatElem(2*n+1, h3x_h4y_h5_a/h6x_h7y_1_a - h3x_h4y_h5_b/h6x_h7y_1_b)); // 列 行

				double f1 = h0x_h1y_h2_a/h6x_h7y_1_a - h0x_h1y_h2_b/h6x_h7y_1_b;
				double f2 = h3x_h4y_h5_a/h6x_h7y_1_a - h3x_h4y_h5_b/h6x_h7y_1_b;
				cost1 += (f1*f1 + f2*f2);

			}
			else if((pMatchPairs[n].ptA_Fixed==1) && (pMatchPairs[n].ptB_Fixed==0)) // 如果A点固定
			{
				int ptA_i0 = pMatchPairs[n].ptA_i;
				int ptB_i0 = pMatchPairs[n].ptB_i;

				// ptB
				int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[ptB_i0]; // ptB所在的图像的序号

				double hB[9];
				for(int i=0; i<8; i++)
				{
					hB[i] = cvGetReal2D(pX, 8*ptB_i+i, 0);
				}
				hB[8] = 1;
				double xB = pMatchPairs[n].ptB.x;
				double yB = pMatchPairs[n].ptB.y;

				double h0x_h1y_h2 = hB[0]*xB + hB[1]*yB + hB[2];
				double h3x_h4y_h5 = hB[3]*xB + hB[4]*yB + hB[5];
				double h6x_h7y_1  = hB[6]*xB + hB[7]*yB + 1;

				A.mat[8*ptB_i + 0].matCol.push_back(SparseMatElem(2*n, xB/h6x_h7y_1)); // 列 行
				A.mat[8*ptB_i + 1].matCol.push_back(SparseMatElem(2*n, yB/h6x_h7y_1)); // 列 行
				A.mat[8*ptB_i + 2].matCol.push_back(SparseMatElem(2*n, 1/h6x_h7y_1)); // 列 行
				A.mat[8*ptB_i + 6].matCol.push_back(SparseMatElem(2*n, -h0x_h1y_h2*xB / (h6x_h7y_1*h6x_h7y_1))); // 列 行
				A.mat[8*ptB_i + 7].matCol.push_back(SparseMatElem(2*n, -h0x_h1y_h2*yB / (h6x_h7y_1*h6x_h7y_1))); // 列 行

				A.mat[8*ptB_i + 3].matCol.push_back(SparseMatElem(2*n+1, xB/h6x_h7y_1)); // 列 行
				A.mat[8*ptB_i + 4].matCol.push_back(SparseMatElem(2*n+1, yB/h6x_h7y_1)); // 列 行
				A.mat[8*ptB_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1/h6x_h7y_1)); // 列 行
				A.mat[8*ptB_i + 6].matCol.push_back(SparseMatElem(2*n+1, -h3x_h4y_h5*xB / (h6x_h7y_1*h6x_h7y_1))); // 列 行
				A.mat[8*ptB_i + 7].matCol.push_back(SparseMatElem(2*n+1, -h3x_h4y_h5*yB / (h6x_h7y_1*h6x_h7y_1))); // 列 行

				// 用固定的参数对A点进行变换
				DfPoint ptA_Bar;
				double h64F[9];
				for(int t=0; t<9; t++)
				{
					h64F[t] = pImagesTransform0[ptA_i0].h.m[t];
				}
				ApplyProject9(h64F, pMatchPairs[n].ptA.x, pMatchPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);

				// 给B赋值
				B.mat[0].matCol.push_back(SparseMatElem(2*n,   h0x_h1y_h2 / h6x_h7y_1 - ptA_Bar.x)); // 列 行
				B.mat[0].matCol.push_back(SparseMatElem(2*n+1, h3x_h4y_h5 / h6x_h7y_1 - ptA_Bar.y)); // 列 行

				double f1 = h0x_h1y_h2 / h6x_h7y_1 - ptA_Bar.x;
				double f2 = h3x_h4y_h5 / h6x_h7y_1 - ptA_Bar.y;
				cost1 += (f1*f1 + f2*f2);
			}
			else if((pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==1)) // 如果B点固定
			{
				int ptA_i0 = pMatchPairs[n].ptA_i;
				int ptB_i0 = pMatchPairs[n].ptB_i;

				// 2*n行, 8*i列
				// ptA
				int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[ptA_i0]; // ptA所在的图像的序号

				double hA[9];
				for(int i=0; i<8; i++)
				{
					hA[i] = cvGetReal2D(pX, 8*ptA_i+i, 0);
				}
				hA[8] = 1;
				double xA = pMatchPairs[n].ptA.x, yA = pMatchPairs[n].ptA.y;
				double h6x_h7y_1_a  = hA[6]*xA + hA[7]*yA + 1;
				double h0x_h1y_h2_a = hA[0]*xA + hA[1]*yA + hA[2];
				A.mat[8*ptA_i + 0].matCol.push_back(SparseMatElem(2*n, xA/h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 1].matCol.push_back(SparseMatElem(2*n, yA/h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 2].matCol.push_back(SparseMatElem(2*n, 1/h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 6].matCol.push_back(SparseMatElem(2*n, -h0x_h1y_h2_a*xA / (h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行
				A.mat[8*ptA_i + 7].matCol.push_back(SparseMatElem(2*n, -h0x_h1y_h2_a*yA / (h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行

				double h3x_h4y_h5_a = hA[3]*xA + hA[4]*yA + hA[5];
				A.mat[8*ptA_i + 3].matCol.push_back(SparseMatElem(2*n+1, xA/h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 4].matCol.push_back(SparseMatElem(2*n+1, yA/h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1/h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 6].matCol.push_back(SparseMatElem(2*n+1, -h3x_h4y_h5_a*xA/(h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行
				A.mat[8*ptA_i + 7].matCol.push_back(SparseMatElem(2*n+1, -h3x_h4y_h5_a*yA/(h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行

				// 用固定的参数对B点进行变换
				DfPoint ptB_Bar;
				double hB64F[9];
				for(int t=0; t<9; t++)
				{
					hB64F[t] = pImagesTransform0[ptB_i0].h.m[t];
				}
				ApplyProject9(hB64F, pMatchPairs[n].ptB.x, pMatchPairs[n].ptB.y, ptB_Bar.x, ptB_Bar.y);
				// 给B赋值
				B.mat[0].matCol.push_back(SparseMatElem(2*n,   h0x_h1y_h2_a/h6x_h7y_1_a - ptB_Bar.x)); // 列 行
				B.mat[0].matCol.push_back(SparseMatElem(2*n+1, h3x_h4y_h5_a/h6x_h7y_1_a - ptB_Bar.y)); // 列 行

				double f1 = h0x_h1y_h2_a/h6x_h7y_1_a - ptB_Bar.x;
				double f2 = h3x_h4y_h5_a/h6x_h7y_1_a - ptB_Bar.y;

				cost1 += (f1*f1 + f2*f2);
			}
		}

		
		double cost2 = 0;
		// 向A中加入rot约束条件
		vector<int> vecFreqMatch;
		int nUnfixed = 0;
		for(int n0=0; n0<nImages; n0++)
		{
			if(pImagesTransform0[n0].fixed==1)
				continue;

			int nC = freqMatch[n0];
			vecFreqMatch.push_back(nC);

			int n = nUnfixed;
			double a = cvGetReal2D(pX, 8*n+0, 0); // a
			double b = cvGetReal2D(pX, 8*n+1, 0); // b
			double e = cvGetReal2D(pX, 8*n+2, 0); // e
			double c = cvGetReal2D(pX, 8*n+3, 0); // c
			double d = cvGetReal2D(pX, 8*n+4, 0); // d
			double f = cvGetReal2D(pX, 8*n+5, 0); // f
			double g = cvGetReal2D(pX, 8*n+6, 0); // g
			double h = cvGetReal2D(pX, 8*n+7, 0); // h

			double wRot = nC;

			double sqt_wRot = sqrt(wRot);
			A.mat[8*n + 0].matCol.push_back(SparseMatElem(4*n+0+2*nPairs, sqt_wRot*b)); // 列 行
			A.mat[8*n + 1].matCol.push_back(SparseMatElem(4*n+0+2*nPairs, sqt_wRot*a)); // 列 行
			A.mat[8*n + 2].matCol.push_back(SparseMatElem(4*n+0+2*nPairs, sqt_wRot*d)); // 列 行
			A.mat[8*n + 3].matCol.push_back(SparseMatElem(4*n+0+2*nPairs, sqt_wRot*c));

			A.mat[8*n + 0].matCol.push_back(SparseMatElem(4*n+1+2*nPairs, sqt_wRot*2*a)); // 列 行
			A.mat[8*n + 2].matCol.push_back(SparseMatElem(4*n+1+2*nPairs, sqt_wRot*2*c)); // 列 行

			A.mat[8*n + 1].matCol.push_back(SparseMatElem(4*n+2+2*nPairs, sqt_wRot*2*b)); // 列 行
			A.mat[8*n + 3].matCol.push_back(SparseMatElem(4*n+2+2*nPairs, sqt_wRot*2*d)); // 列 行

			A.mat[8*n + 6].matCol.push_back(SparseMatElem(4*n+3+2*nPairs, sqt_wRot*2*g)); // 列 行
			A.mat[8*n + 7].matCol.push_back(SparseMatElem(4*n+3+2*nPairs, sqt_wRot*2*h)); // 列 行

			// 向B赋值
			B.mat[0].matCol.push_back(SparseMatElem(4*n+0+2*nPairs, sqt_wRot*(a*b + c*d))); // 列 行
			B.mat[0].matCol.push_back(SparseMatElem(4*n+1+2*nPairs, sqt_wRot*(a*a + c*c - 1))); // 列 行
			B.mat[0].matCol.push_back(SparseMatElem(4*n+2+2*nPairs, sqt_wRot*(b*b + d*d - 1))); // 列 行
			B.mat[0].matCol.push_back(SparseMatElem(4*n+3+2*nPairs, sqt_wRot*(g*g + h*h))); // 列 行

			cost2 += wRot*(a*b + c*d)*(a*b + c*d);
			cost2 += wRot*(a*a + c*c - 1)*(a*a + c*c - 1);
			cost2 += wRot*(b*b + d*d - 1)*(b*b + d*d - 1);
			cost2 += wRot*(g*g + h*h)*(g*g + h*h);

			nUnfixed++;
		}

		//// 参考图像约束
		//double weight = 30.0/nRefImgPairs;
		//weight = 0;
		//cout<<"weight "<<weight<<endl;

		//double costRef = 0;
		//int nStep = 2 * nPairs + 4 * (nImages-nFixedImages);

		//int ptA_i = refImgIdx;
		//double hA[9];
		//for(int i=0; i<8; i++)
		//{
		//	hA[i] = cvGetReal2D(pX, 8*ptA_i+i, 0);
		//}
		//hA[8] = 1;
		//int n=0;
		//for(int np=0; np<nPairs; np++)
		//{
		//	float xA, yA;
		//	if(pMatchPairs[np].ptA_i==refImgIdx0)
		//	{
		//		xA = pMatchPairs[np].ptA.x;
		//		yA = pMatchPairs[np].ptA.y;
		//	}
		//	else if(pMatchPairs[np].ptB_i==refImgIdx0)
		//	{
		//		xA = pMatchPairs[np].ptB.x;
		//		yA = pMatchPairs[np].ptB.y;
		//	}
		//	else
		//	{
		//		continue;
		//	}

		//	double h6x_h7y_1_a  = hA[6]*xA + hA[7]*yA + 1;
		//	double h0x_h1y_h2_a = hA[0]*xA + hA[1]*yA + hA[2];
		//	double h3x_h4y_h5_a = hA[3]*xA + hA[4]*yA + hA[5];

		//	A.mat[8*ptA_i + 0].matCol.push_back(SparseMatElem(2*n+nStep, xA*weight)); // 列 行
		//	A.mat[8*ptA_i + 1].matCol.push_back(SparseMatElem(2*n+nStep, yA*weight)); // 列 行
		//	A.mat[8*ptA_i + 2].matCol.push_back(SparseMatElem(2*n+nStep, 1*weight)); // 列 行
		//	A.mat[8*ptA_i + 6].matCol.push_back(SparseMatElem(2*n+nStep, -xA*xA*weight)); // 列 行
		//	A.mat[8*ptA_i + 7].matCol.push_back(SparseMatElem(2*n+nStep, -xA*yA*weight)); // 列 行

		//	A.mat[8*ptA_i + 3].matCol.push_back(SparseMatElem(2*n+1+nStep, xA*weight)); // 列 行
		//	A.mat[8*ptA_i + 4].matCol.push_back(SparseMatElem(2*n+1+nStep, yA*weight)); // 列 行
		//	A.mat[8*ptA_i + 5].matCol.push_back(SparseMatElem(2*n+1+nStep, 1*weight)); // 列 行
		//	A.mat[8*ptA_i + 6].matCol.push_back(SparseMatElem(2*n+1+nStep, -yA*xA*weight)); // 列 行
		//	A.mat[8*ptA_i + 7].matCol.push_back(SparseMatElem(2*n+1+nStep, -yA*yA*weight)); // 列 行

		//	double f1 = (h0x_h1y_h2_a - xA*h6x_h7y_1_a)*weight;
		//	double f2 = (h3x_h4y_h5_a - yA*h6x_h7y_1_a)*weight;

		//	B.mat[0].matCol.push_back(SparseMatElem(2*n+nStep,   f1)); // 列 行
		//	B.mat[0].matCol.push_back(SparseMatElem(2*n+1+nStep, f2)); // 列 行
		//	
		//	costRef += (f1*f1 + f2*f2);

		//	n++;
		//}
		//cout<<"fixed pair num "<<n<<endl;

		double totalCost = cost1 + cost2;// + costRef;
		vecCost.push_back(totalCost);
		vecCost2.push_back(cost2);
		//vecCostRef.push_back(costRef);

		// CV解线性方程组
		//cout<<"解线性方程组"<<endl<<endl;

		for(int col=0; col<A.col; col++)
		{
			std::sort(A.mat[col].matCol.begin(), A.mat[col].matCol.end());
		}
		for(int col=0; col<eye.col; col++)
		{
			std::sort(eye.mat[col].matCol.begin(), eye.mat[col].matCol.end());
		}

		//SolveSparseSystem2(A, B, pDX);

		//// 更新pX
		//cvConvertScale(pDX, pDX, -1);
		//cvAdd(pX, pDX, pX);

		cout<<"pDX"<<endl;
		for(int p=0; p<1; p++)
		{
			for(int q=0; q<8; q++)
			{
				printf("%.4f ", cvGetReal2D(pDX, p*8 + q, 0));
			}
			cout<<endl;
		}

		int nSingleIter = 0;
		int maxIterSingle = 20;
		bool convergence = true;
		do
		{
			SolveSparseSystemLM(A, B, eye, pDX, lambda64F);
			nSingleIter++;

			//cout<<"pX"<<endl;
			//for(int p=0; p<nImages-nFixedImages; p++)
			//{
			//	for(int q=0; q<8; q++)
			//	{
			//		printf("%.4f ", cvGetReal2D(pX, p*8 + q, 0));
			//	}
			//	cout<<endl;
			//}

			//cout<<"pDX"<<endl;
			//for(int p=0; p<1; p++)
			//{
			//	for(int q=0; q<8; q++)
			//	{
			//		printf("%.4f ", cvGetReal2D(pDX, p*8 + q, 0));
			//	}
			//	cout<<endl;
			//}

			// 更新pX
			cvConvertScale(pDX, pDX, -1);
			cvAdd(pX, pDX, pX2);

			// 整理计算结果
			vecTransformRefined.clear();
			nUnFixed = 0;
			for(int i=0; i<nImages; i++)
			{
				if(pImagesTransform0[i].fixed!=1) // 未固定参数情况
				{
					ImageTransform temp;
					temp.fixed = 0;
					for(int j=0; j<8; j++)
					{
						temp.h.m[j] = (float)cvGetReal2D(pX2, 8*nUnFixed+j, 0); // a
					}
					temp.h.m[8] = 1;
					nUnFixed++;
					vecTransformRefined.push_back(temp);
				}
				else
				{
					vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
				}
			}

			//double costNew = CalMatchError3(&vecTransformRefined[0], pX2, nUnfixed, pMatchPairs, nPairs, 
			//	vecFreqMatch, vecAccFixed, refImgIdx0, refImgIdx, weight);
			double rotCost = 0;
			double costNew = CalMatchError2(&vecTransformRefined[0], pX2, nUnfixed, pMatchPairs, nPairs, 
					vecFreqMatch, vecAccFixed, rotCost);

			cout<<"costNew "<<costNew<<" totalCost "<<totalCost<<" lambda64F "<<lambda64F<<endl;

			if(costNew<totalCost)
			{
				cvCopy(pX2, pX);
				lambda64F /= beta;
				convergence = true;
				totalCost = costNew;
				vecCost[iter] = totalCost;
				vecCost2[iter] = rotCost;
			}
			else
			{
				lambda64F *= beta;
				convergence = false;
			}
			if(nSingleIter>maxIterSingle)
			{
				break;
			}
		} 
		while(convergence==false);

		cout<<"iter "<<iter<<endl;
		if(vecCost.size()>1)
		{
			if(vecCost[vecCost.size()-1]>=vecCost[vecCost.size()-2])
			{
				break;
			}
		}
	}

	cout<<"Total Cost"<<endl;
	for(int i=0; i<vecCost.size(); i++)
	{
		cout<<vecCost[i]<<" ";
	}
	cout<<endl<<"Rot Cost"<<endl;
	for(int i=0; i<vecCost2.size(); i++)
	{
		cout<<vecCost2[i]<<" ";
	}
	cout<<endl;

	// 整理计算结果
	vecTransformRefined.clear();
	nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(pImagesTransform0[i].fixed!=1) // 未固定参数情况
		{
			ImageTransform temp;
			temp.fixed = 0;
			for(int j=0; j<8; j++)
			{
				temp.h.m[j] = (float)cvGetReal2D(pX, 8*nUnFixed+j, 0); // a
			}
			temp.h.m[8] = 1;
			nUnFixed++;
			vecTransformRefined.push_back(temp);
		}
		else
		{
			vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
		}
	}

	//cvReleaseMat(&pA);
	//cvReleaseMat(&pB);
	cvReleaseMat(&pX);
	cvReleaseMat(&pX2);
	cvReleaseMat(&pDX);

	return 0;
}


// 非线性稀疏捆集调整 + rot约束
int CMosaicByPose::SparseNonlinearRotConstraint(MatchPointPairs* pMatchPairs, int nPairs,
									 ImageTransform* pImagesTransform0, int nImages, 
									 int nFixedImages, float weight,
									 vector<ImageTransform> &vecTransformRefined,
									 int label[10000])
{	
	cout<<"----Bundle Adjustment Nonlinear-----"<<endl<<endl;

	cout<<"nPairs "<<nPairs<<endl;
	
	vector<int> freqMatch(nImages); // 某幅图像中匹配对出现的频率
	memset(&freqMatch[0], 0, sizeof(int)*nImages);

	for(int n=0; n<nPairs; n++)
	{
		if(pMatchPairs[n].ptA_Fixed==0)
		{
			freqMatch[pMatchPairs[n].ptA_i]++;
		}
		if(pMatchPairs[n].ptB_Fixed==0)
		{
			freqMatch[pMatchPairs[n].ptB_i]++;
		}
	}

	for(int n=0; n<nImages; n++)
	{
		freqMatch[n] *= weight;
	}

	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<9; j++)
		{
			if(pImagesTransform0[i].h.m[8]!=0)
				pImagesTransform0[i].h.m[j] /= pImagesTransform0[i].h.m[8];
		}
	}

	int rowA = 2 * nPairs + 4 * (nImages-nFixedImages);
	//int rowA = 2 * nPairs;
	int colA = 8 * (nImages-nFixedImages);
	int rowX = 8 * (nImages-nFixedImages);
	int colX = 1;
	int rowB = rowA;
	int colB = 1;

	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pX2 = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pDX = cvCreateMat(rowX, colX, CV_64F);
	
	cvZero(pDX);

	// 初始化X
	int nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(pImagesTransform0[i].fixed==0) // 未固定参数情况
		{
			for(int j=0; j<8; j++)
			{
				cvSetReal2D(pX, 8*nUnFixed+j, 0, pImagesTransform0[i].h.m[j]);
			}
			nUnFixed++;
		}
	}

	// 处理固定的参数对pX存储的影响
	vector<int> vecAccFixed(nImages);
	memset(&vecAccFixed[0], 0, sizeof(int)*nImages);

	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<i; j++)
		{
			if(pImagesTransform0[j].fixed==1)
			{
				vecAccFixed[i] += 1;
			}
		}
	}

	pool::SparseMatrix eye;
	eye.mat.resize(colA);
	eye.col = colA;
	eye.row = colA;
	for(int i=0; i<colA; i++)
	{
		eye.mat[i].matCol.push_back(SparseMatElem(i, 1)); // 列 行
	}

	double beta = 2;
	double lambda64F = 0.01;
	vector<double> vecCost, vecCost2;
	const int MAX_ITER = 10;
	// 迭代
	for(int iter = 0; iter<MAX_ITER; iter++)
	{
		double cost1 = 0;
		pool::SparseMatrix A;
		pool::SparseMatrix B;
		A.mat.resize(colA);
		B.mat.resize(colB);
		A.col = colA;
		A.row = rowA;
		B.col = colB;
		B.row = rowB;

		for(int n=0; n<nPairs; n++)
		{
			if( (pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==0) ) // 如果A点和B点都不是固定点
			{
				// 2*n行, 8*i列
				// ptA
				int ptA_i0 = pMatchPairs[n].ptA_i;
				int ptA_i = ptA_i0 - vecAccFixed[ptA_i0]; // ptA所在的图像的序号

				double hA[9];
				for(int i=0; i<8; i++)
				{
					hA[i] = cvGetReal2D(pX, 8*ptA_i+i, 0);
				}
				hA[8] = 1;

				double xA = pMatchPairs[n].ptA.x;
				double yA = pMatchPairs[n].ptA.y;

				double h0x_h1y_h2_a = hA[0]*xA + hA[1]*yA + hA[2];
				double h3x_h4y_h5_a = hA[3]*xA + hA[4]*yA + hA[5];
				double h6x_h7y_1_a  = hA[6]*xA + hA[7]*yA + 1;

				A.mat[8*ptA_i + 0].matCol.push_back(SparseMatElem(2*n, xA / h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 1].matCol.push_back(SparseMatElem(2*n, yA / h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 2].matCol.push_back(SparseMatElem(2*n, 1 / h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 6].matCol.push_back(SparseMatElem(2*n, -h0x_h1y_h2_a*xA / (h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行
				A.mat[8*ptA_i + 7].matCol.push_back(SparseMatElem(2*n, -h0x_h1y_h2_a*yA / (h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行
				
				A.mat[8*ptA_i + 3].matCol.push_back(SparseMatElem(2*n+1, xA / h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 4].matCol.push_back(SparseMatElem(2*n+1, yA / h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1 / h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 6].matCol.push_back(SparseMatElem(2*n+1, -h3x_h4y_h5_a*xA / (h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行
				A.mat[8*ptA_i + 7].matCol.push_back(SparseMatElem(2*n+1, -h3x_h4y_h5_a*yA / (h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行

				int ptB_i0 = pMatchPairs[n].ptB_i;
				int ptB_i = ptB_i0 - vecAccFixed[ptB_i0]; // ptB所在的图像的序号

				// ptB
				double hB[9];
				for(int i=0; i<8; i++)
				{
					hB[i] = cvGetReal2D(pX, 8*ptB_i+i, 0);
				}
				hB[8] = 1;
				double xB = pMatchPairs[n].ptB.x;
				double yB = pMatchPairs[n].ptB.y;
				
				double h0x_h1y_h2_b = hB[0]*xB + hB[1]*yB + hB[2];
				double h3x_h4y_h5_b = hB[3]*xB + hB[4]*yB + hB[5];
				double h6x_h7y_1_b  = hB[6]*xB + hB[7]*yB + 1;
				
				A.mat[8*ptB_i + 0].matCol.push_back(SparseMatElem(2*n, -xB / h6x_h7y_1_b)); // 列 行
				A.mat[8*ptB_i + 1].matCol.push_back(SparseMatElem(2*n, -yB / h6x_h7y_1_b)); // 列 行
				A.mat[8*ptB_i + 2].matCol.push_back(SparseMatElem(2*n, -1 / h6x_h7y_1_b)); // 列 行
				A.mat[8*ptB_i + 6].matCol.push_back(SparseMatElem(2*n, h0x_h1y_h2_b*xB / (h6x_h7y_1_b*h6x_h7y_1_b))); // 列 行
				A.mat[8*ptB_i + 7].matCol.push_back(SparseMatElem(2*n, h0x_h1y_h2_b*yB / (h6x_h7y_1_b*h6x_h7y_1_b))); // 列 行
				
				A.mat[8*ptB_i + 3].matCol.push_back(SparseMatElem(2*n+1, -xB / h6x_h7y_1_b)); // 列 行
				A.mat[8*ptB_i + 4].matCol.push_back(SparseMatElem(2*n+1, -yB / h6x_h7y_1_b)); // 列 行
				A.mat[8*ptB_i + 5].matCol.push_back(SparseMatElem(2*n+1, -1 / h6x_h7y_1_b)); // 列 行
				A.mat[8*ptB_i + 6].matCol.push_back(SparseMatElem(2*n+1, h3x_h4y_h5_b*xB / (h6x_h7y_1_b*h6x_h7y_1_b))); // 列 行
				A.mat[8*ptB_i + 7].matCol.push_back(SparseMatElem(2*n+1, h3x_h4y_h5_b*yB / (h6x_h7y_1_b*h6x_h7y_1_b))); // 列 行

				// 给B赋值
				B.mat[0].matCol.push_back(SparseMatElem(2*n,   h0x_h1y_h2_a/h6x_h7y_1_a - h0x_h1y_h2_b/h6x_h7y_1_b)); // 列 行
				B.mat[0].matCol.push_back(SparseMatElem(2*n+1, h3x_h4y_h5_a/h6x_h7y_1_a - h3x_h4y_h5_b/h6x_h7y_1_b)); // 列 行

				double f1 = h0x_h1y_h2_a/h6x_h7y_1_a - h0x_h1y_h2_b/h6x_h7y_1_b;
				double f2 = h3x_h4y_h5_a/h6x_h7y_1_a - h3x_h4y_h5_b/h6x_h7y_1_b;
				cost1 += (f1*f1 + f2*f2);
				
			}
			else if((pMatchPairs[n].ptA_Fixed==1) && (pMatchPairs[n].ptB_Fixed==0)) // 如果A点固定
			{
				int ptA_i0 = pMatchPairs[n].ptA_i;
				int ptB_i0 = pMatchPairs[n].ptB_i;

				// ptB
				int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[ptB_i0]; // ptB所在的图像的序号
		
				double hB[9];
				for(int i=0; i<8; i++)
				{
					hB[i] = cvGetReal2D(pX, 8*ptB_i+i, 0);
				}
				hB[8] = 1;
				double xB = pMatchPairs[n].ptB.x;
				double yB = pMatchPairs[n].ptB.y;
				
				double h0x_h1y_h2 = hB[0]*xB + hB[1]*yB + hB[2];
				double h3x_h4y_h5 = hB[3]*xB + hB[4]*yB + hB[5];
				double h6x_h7y_1  = hB[6]*xB + hB[7]*yB + 1;

				A.mat[8*ptB_i + 0].matCol.push_back(SparseMatElem(2*n, xB/h6x_h7y_1)); // 列 行
				A.mat[8*ptB_i + 1].matCol.push_back(SparseMatElem(2*n, yB/h6x_h7y_1)); // 列 行
				A.mat[8*ptB_i + 2].matCol.push_back(SparseMatElem(2*n, 1/h6x_h7y_1)); // 列 行
				A.mat[8*ptB_i + 6].matCol.push_back(SparseMatElem(2*n, -h0x_h1y_h2*xB / (h6x_h7y_1*h6x_h7y_1))); // 列 行
				A.mat[8*ptB_i + 7].matCol.push_back(SparseMatElem(2*n, -h0x_h1y_h2*yB / (h6x_h7y_1*h6x_h7y_1))); // 列 行
				
				A.mat[8*ptB_i + 3].matCol.push_back(SparseMatElem(2*n+1, xB/h6x_h7y_1)); // 列 行
				A.mat[8*ptB_i + 4].matCol.push_back(SparseMatElem(2*n+1, yB/h6x_h7y_1)); // 列 行
				A.mat[8*ptB_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1/h6x_h7y_1)); // 列 行
				A.mat[8*ptB_i + 6].matCol.push_back(SparseMatElem(2*n+1, -h3x_h4y_h5*xB / (h6x_h7y_1*h6x_h7y_1))); // 列 行
				A.mat[8*ptB_i + 7].matCol.push_back(SparseMatElem(2*n+1, -h3x_h4y_h5*yB / (h6x_h7y_1*h6x_h7y_1))); // 列 行

				// 用固定的参数对A点进行变换
				DfPoint ptA_Bar;
				double h64F[9];
				for(int t=0; t<9; t++)
				{
					h64F[t] = pImagesTransform0[ptA_i0].h.m[t];
				}
				ApplyProject9(h64F, pMatchPairs[n].ptA.x, pMatchPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);

				// 给B赋值
				B.mat[0].matCol.push_back(SparseMatElem(2*n,   h0x_h1y_h2 / h6x_h7y_1 - ptA_Bar.x)); // 列 行
				B.mat[0].matCol.push_back(SparseMatElem(2*n+1, h3x_h4y_h5 / h6x_h7y_1 - ptA_Bar.y)); // 列 行

				double f1 = h0x_h1y_h2 / h6x_h7y_1 - ptA_Bar.x;
				double f2 = h3x_h4y_h5 / h6x_h7y_1 - ptA_Bar.y;
				cost1 += (f1*f1 + f2*f2);
			}
			else if((pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==1)) // 如果B点固定
			{
				int ptA_i0 = pMatchPairs[n].ptA_i;
				int ptB_i0 = pMatchPairs[n].ptB_i;

				// 2*n行, 8*i列
				// ptA
				int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[ptA_i0]; // ptA所在的图像的序号

				double hA[9];
				for(int i=0; i<8; i++)
				{
					hA[i] = cvGetReal2D(pX, 8*ptA_i+i, 0);
				}
				hA[8] = 1;
				double xA = pMatchPairs[n].ptA.x, yA = pMatchPairs[n].ptA.y;
				double h6x_h7y_1_a  = hA[6]*xA + hA[7]*yA + 1;
				double h0x_h1y_h2_a = hA[0]*xA + hA[1]*yA + hA[2];
				A.mat[8*ptA_i + 0].matCol.push_back(SparseMatElem(2*n, xA/h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 1].matCol.push_back(SparseMatElem(2*n, yA/h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 2].matCol.push_back(SparseMatElem(2*n, 1/h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 6].matCol.push_back(SparseMatElem(2*n, -h0x_h1y_h2_a*xA / (h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行
				A.mat[8*ptA_i + 7].matCol.push_back(SparseMatElem(2*n, -h0x_h1y_h2_a*yA / (h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行

				double h3x_h4y_h5_a = hA[3]*xA + hA[4]*yA + hA[5];
				A.mat[8*ptA_i + 3].matCol.push_back(SparseMatElem(2*n+1, xA/h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 4].matCol.push_back(SparseMatElem(2*n+1, yA/h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 5].matCol.push_back(SparseMatElem(2*n+1, 1/h6x_h7y_1_a)); // 列 行
				A.mat[8*ptA_i + 6].matCol.push_back(SparseMatElem(2*n+1, -h3x_h4y_h5_a*xA/(h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行
				A.mat[8*ptA_i + 7].matCol.push_back(SparseMatElem(2*n+1, -h3x_h4y_h5_a*yA/(h6x_h7y_1_a*h6x_h7y_1_a))); // 列 行

				// 用固定的参数对B点进行变换
				DfPoint ptB_Bar;
				double hB64F[9];
				for(int t=0; t<9; t++)
				{
					hB64F[t] = pImagesTransform0[ptB_i0].h.m[t];
				}
				ApplyProject9(hB64F, pMatchPairs[n].ptB.x, pMatchPairs[n].ptB.y, ptB_Bar.x, ptB_Bar.y);
				// 给B赋值
				B.mat[0].matCol.push_back(SparseMatElem(2*n,   h0x_h1y_h2_a/h6x_h7y_1_a - ptB_Bar.x)); // 列 行
				B.mat[0].matCol.push_back(SparseMatElem(2*n+1, h3x_h4y_h5_a/h6x_h7y_1_a - ptB_Bar.y)); // 列 行

				double f1 = h0x_h1y_h2_a/h6x_h7y_1_a - ptB_Bar.x;
				double f2 = h3x_h4y_h5_a/h6x_h7y_1_a - ptB_Bar.y;

				cost1 += (f1*f1 + f2*f2);
			}
		}

		cout<<"mse "<<sqrt(cost1/nPairs)<<endl;

		double cost2 = 0;
		// 向A中加入rot约束条件
		vector<int> vecFreqMatch;
		int nUnfixed = 0;
		for(int n0=0; n0<nImages; n0++)
		{
			if(pImagesTransform0[n0].fixed==1)
				continue;

			int nC = freqMatch[n0];
			vecFreqMatch.push_back(nC);

			int n = nUnfixed;
			double a = cvGetReal2D(pX, 8*n+0, 0); // a
			double b = cvGetReal2D(pX, 8*n+1, 0); // b
			double e = cvGetReal2D(pX, 8*n+2, 0); // e
			double c = cvGetReal2D(pX, 8*n+3, 0); // c
			double d = cvGetReal2D(pX, 8*n+4, 0); // d
			double f = cvGetReal2D(pX, 8*n+5, 0); // f
			double g = cvGetReal2D(pX, 8*n+6, 0); // g
			double h = cvGetReal2D(pX, 8*n+7, 0); // h

			double wRot = nC;

			double sqt_wRot = sqrt(wRot);
			A.mat[8*n + 0].matCol.push_back(SparseMatElem(4*n+0+2*nPairs, sqt_wRot*b)); // 列 行
			A.mat[8*n + 1].matCol.push_back(SparseMatElem(4*n+0+2*nPairs, sqt_wRot*a)); // 列 行
			A.mat[8*n + 2].matCol.push_back(SparseMatElem(4*n+0+2*nPairs, sqt_wRot*d)); // 列 行
			A.mat[8*n + 3].matCol.push_back(SparseMatElem(4*n+0+2*nPairs, sqt_wRot*c));
			
			A.mat[8*n + 0].matCol.push_back(SparseMatElem(4*n+1+2*nPairs, sqt_wRot*2*a)); // 列 行
			A.mat[8*n + 2].matCol.push_back(SparseMatElem(4*n+1+2*nPairs, sqt_wRot*2*c)); // 列 行
			
			A.mat[8*n + 1].matCol.push_back(SparseMatElem(4*n+2+2*nPairs, sqt_wRot*2*b)); // 列 行
			A.mat[8*n + 3].matCol.push_back(SparseMatElem(4*n+2+2*nPairs, sqt_wRot*2*d)); // 列 行

			A.mat[8*n + 6].matCol.push_back(SparseMatElem(4*n+3+2*nPairs, sqt_wRot*2*g)); // 列 行
			A.mat[8*n + 7].matCol.push_back(SparseMatElem(4*n+3+2*nPairs, sqt_wRot*2*h)); // 列 行

			// 向B赋值
			B.mat[0].matCol.push_back(SparseMatElem(4*n+0+2*nPairs, sqt_wRot*(a*b + c*d))); // 列 行
			B.mat[0].matCol.push_back(SparseMatElem(4*n+1+2*nPairs, sqt_wRot*(a*a + c*c - 1))); // 列 行
			B.mat[0].matCol.push_back(SparseMatElem(4*n+2+2*nPairs, sqt_wRot*(b*b + d*d - 1))); // 列 行
			B.mat[0].matCol.push_back(SparseMatElem(4*n+3+2*nPairs, sqt_wRot*(g*g + h*h))); // 列 行

			cost2 += wRot*(a*b + c*d)*(a*b + c*d);
			cost2 += wRot*(a*a + c*c - 1)*(a*a + c*c - 1);
			cost2 += wRot*(b*b + d*d - 1)*(b*b + d*d - 1);
			cost2 += wRot*(g*g + h*h)*(g*g + h*h);

			nUnfixed++;
		}
		
		double totalCost = cost1 + cost2;
		vecCost.push_back(totalCost);
		vecCost2.push_back(cost2);

		// CV解线性方程组
		//cout<<"解线性方程组"<<endl<<endl;

		for(int col=0; col<A.col; col++)
		{
			std::sort(A.mat[col].matCol.begin(), A.mat[col].matCol.end());
		}
		for(int col=0; col<eye.col; col++)
		{
			std::sort(eye.mat[col].matCol.begin(), eye.mat[col].matCol.end());
		}

		//SolveSparseSystem2(A, B, pDX);
		//double lambda64F = 0.1;
		int nSingleIter = 0;
		int maxIterSingle = 20;
		bool convergence = true;
		do
		{
			SolveSparseSystemLM(A, B, eye, pDX, lambda64F);
			//SolveSparseSystem2(A, B, pDX);
			nSingleIter++;

			//cout<<"pX"<<endl;
			//for(int p=0; p<nImages-nFixedImages; p++)
			//{
			//	for(int q=0; q<8; q++)
			//	{
			//		printf("%.4f ", cvGetReal2D(pX, p*8 + q, 0));
			//	}
			//	cout<<endl;
			//}

			//cout<<"pDX"<<endl;
			//for(int p=0; p<1; p++)
			//{
			//	for(int q=0; q<8; q++)
			//	{
			//		printf("%.4f ", cvGetReal2D(pDX, p*8 + q, 0));
			//	}
			//	cout<<endl;
			//}

			// 更新pX
			cvConvertScale(pDX, pDX, -1);
			cvAdd(pX, pDX, pX2);

			// 整理计算结果
			vecTransformRefined.clear();
			nUnFixed = 0;
			for(int i=0; i<nImages; i++)
			{
				if(pImagesTransform0[i].fixed==0) // 未固定参数情况
				{
					ImageTransform temp;
					temp.fixed = 0;
					for(int j=0; j<8; j++)
					{
						temp.h.m[j] = (float)cvGetReal2D(pX2, 8*nUnFixed+j, 0); // a
					}
					temp.h.m[8] = 1;
					nUnFixed++;
					vecTransformRefined.push_back(temp);
				}
				else
				{
					vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
				}
			}

			double rotCost = 0;
			double costNew = CalMatchError2(&vecTransformRefined[0], pX2, nUnfixed, pMatchPairs, nPairs, 
				vecFreqMatch, vecAccFixed, rotCost);
			cout<<"costNew "<<costNew<<" totalCost "<<totalCost<<" lambda64F "<<lambda64F<<endl;
			if(costNew<totalCost)
			{
				cvCopy(pX2, pX);
				lambda64F /= beta;
				convergence = true;
				totalCost = costNew;
				vecCost[iter] = totalCost;
				vecCost2[iter] = rotCost;
			}
			else
			{
				lambda64F *= beta;
				convergence = false;
			}
			if(nSingleIter>maxIterSingle)
			{
				break;
			}
		} while(convergence==false);
		cout<<"iter "<<iter<<endl;
		if(vecCost.size()>1)
		{
			if(vecCost[vecCost.size()-1]>=vecCost[vecCost.size()-2])
			{
				break;
			}
		}
	}

	cout<<"Total Cost"<<endl;
	for(int i=0; i<vecCost.size(); i++)
	{
		cout<<vecCost[i]<<" ";
	}
	cout<<endl<<"Rot Cost"<<endl;
	for(int i=0; i<vecCost2.size(); i++)
	{
		cout<<vecCost2[i]<<" ";
	}
	cout<<endl;

	// 整理计算结果
	vecTransformRefined.clear();
	nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(pImagesTransform0[i].fixed==0) // 未固定参数情况
		{
			ImageTransform temp;
			temp.fixed = 0;
			for(int j=0; j<8; j++)
			{
				temp.h.m[j] = (float)cvGetReal2D(pX, 8*nUnFixed+j, 0); // a
			}
			temp.h.m[8] = 1;
			nUnFixed++;
			vecTransformRefined.push_back(temp);
		}
		else
		{
			vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
		}
	}

	//cvReleaseMat(&pA);
	//cvReleaseMat(&pB);
	cvReleaseMat(&pX);
	cvReleaseMat(&pX2);
	cvReleaseMat(&pDX);

	return 0;
}

// 非线性捆集调整
int CMosaicByPose::BundleAdjustmentNonlinear(MatchPointPairs* pMatchPairs, int nPairs,
											ImageTransform* pImagesTransform0, int nImages, 
											int nFixedImages,
											vector<ImageTransform> &vecTransformRefined)
{	
	if(nImages<=1)
	{
		return -2;
	}
	if(nPairs<=4)
	{
		return -3;
	}
	//nFixedImages = 0; // 999999999999999999999999999999999999999999999999999999999999999999999

	cout<<"Bundle Adjustment Nonlinear Start"<<endl<<endl;
	// AX = B, 用仿射变换模型

	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<9; j++)
		{
			pImagesTransform0[i].h.m[j] /= pImagesTransform0[i].h.m[8];
		}
	}

	int rowA = 2 * nPairs;
	int colA = 8 * (nImages-nFixedImages);
	int rowX = 8 * (nImages-nFixedImages);
	int colX = 1;
	int rowB = 2*nPairs;
	int colB = 1;

	//cout<<"rowA "<<rowA<<" rowB "<<rowB<<endl;

	CvMat* pA = cvCreateMat(rowA, colA, CV_64F); // 2*n * 6*m , n为特征点数, m为图像数
	CvMat* pX = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pDX = cvCreateMat(rowX, colX, CV_64F);
	CvMat* pB = cvCreateMat(rowB, colB, CV_64F);
	cvZero(pA);
	cvZero(pB);
	cvZero(pDX);

	// 初始化X
	int nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		//pImagesTransform0[i].fixed = 0;//9999999999999999999999999999999999999999999999999999

		if(pImagesTransform0[i].fixed==0) // 未固定参数情况
		{
			for(int j=0; j<8; j++)
			{
				cvSetReal2D(pX, 8*nUnFixed+j, 0, pImagesTransform0[i].h.m[j]);
			}
			nUnFixed++;
		}
	}

	// 处理固定的参数对pX存储的影响
	vector<int> vecAccFixed(nImages);
	memset(&vecAccFixed[0], 0, sizeof(int)*nImages);

	for(int i=0; i<nImages; i++)
	{
		for(int j=0; j<i; j++)
		{
			if(pImagesTransform0[j].fixed==1)
			{
				vecAccFixed[i] += 1;
			}
		}
	}

	const int MAX_ITER = 1;
	// 迭代
	for(int iter = 0; iter<MAX_ITER; iter++)
	{
		for(int n=0; n<nPairs; n++)
		{
			//pMatchPairs[n].ptA_Fixed = 0;
			//pMatchPairs[n].ptB_Fixed = 0;
			//cout<<n<<" / "<<nPairs<<endl;
			if(n==782)
			{
				n=n;
			}
			if( (pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==0) ) // 如果A点和B点都不是固定点
			{
				// 2*n行, 8*i列
				// ptA
				int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号

				double hA[9];
				for(int i=0; i<8; i++)
				{
					hA[i] = cvGetReal2D(pX, 8*ptA_i+i, 0);
				}
				hA[8] = 1;
				double xA = pMatchPairs[n].ptA.x, yA = pMatchPairs[n].ptA.y;
				double h6x_h7y_1_a  = hA[6]*xA + hA[7]*yA + 1;
				double h0x_h1y_h2_a = hA[0]*xA + hA[1]*yA + hA[2];
				cvSetReal2D(pA, 2*n, 8*ptA_i + 0, xA / h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n, 8*ptA_i + 1, yA / h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n, 8*ptA_i + 2, 1 / h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n, 8*ptA_i + 6, -h0x_h1y_h2_a*xA / (h6x_h7y_1_a*h6x_h7y_1_a));
				cvSetReal2D(pA, 2*n, 8*ptA_i + 7, -h0x_h1y_h2_a*yA / (h6x_h7y_1_a*h6x_h7y_1_a));

				double h3x_h4y_h5_a = hA[3]*xA + hA[4]*yA + hA[5];
				cvSetReal2D(pA, 2*n+1, 8*ptA_i + 3, xA / h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n+1, 8*ptA_i + 4, yA / h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n+1, 8*ptA_i + 5, 1 / h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n+1, 8*ptA_i + 6, -h3x_h4y_h5_a*xA / (h6x_h7y_1_a*h6x_h7y_1_a));
				cvSetReal2D(pA, 2*n+1, 8*ptA_i + 7, -h3x_h4y_h5_a*yA / (h6x_h7y_1_a*h6x_h7y_1_a));

				int ptB_i0 = pMatchPairs[n].ptB_i;
				int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号
				// ptB
				double hB[9];
				for(int i=0; i<8; i++)
				{
					hB[i] = cvGetReal2D(pX, 8*ptB_i+i, 0);
				}
				hB[8] = 1;
				double xB = pMatchPairs[n].ptB.x, yB = pMatchPairs[n].ptB.y;
				double h6x_h7y_1_b  = hB[6]*xB + hB[7]*yB + 1;
				double h0x_h1y_h2_b = hB[0]*xB + hB[1]*yB + hB[2];
				cvSetReal2D(pA, 2*n, 8*ptB_i + 0, -xB / h6x_h7y_1_b);
				cvSetReal2D(pA, 2*n, 8*ptB_i + 1, -yB / h6x_h7y_1_b);
				cvSetReal2D(pA, 2*n, 8*ptB_i + 2, -1 / h6x_h7y_1_b);
				cvSetReal2D(pA, 2*n, 8*ptB_i + 6, h0x_h1y_h2_b*xB / (h6x_h7y_1_b*h6x_h7y_1_b));
				cvSetReal2D(pA, 2*n, 8*ptB_i + 7, h0x_h1y_h2_b*yB / (h6x_h7y_1_b*h6x_h7y_1_b));

				double h3x_h4y_h5_b = hB[3]*xB + hB[4]*yB + hB[5];
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 3, -xB / h6x_h7y_1_b);
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 4, -yB / h6x_h7y_1_b);
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 5, -1 / h6x_h7y_1_b);
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 6, h3x_h4y_h5_b*xB / (h6x_h7y_1_b*h6x_h7y_1_b));
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 7, h3x_h4y_h5_b*yB / (h6x_h7y_1_b*h6x_h7y_1_b));
		
				// 给B赋值
				cvSetReal2D(pB, 2*n, 0, h0x_h1y_h2_b/h6x_h7y_1_b - h0x_h1y_h2_a/h6x_h7y_1_a);
				cvSetReal2D(pB, 2*n+1, 0, h3x_h4y_h5_b/h6x_h7y_1_b - h3x_h4y_h5_a/h6x_h7y_1_a);
			}
			else if((pMatchPairs[n].ptA_Fixed==1) && (pMatchPairs[n].ptB_Fixed==0)) // 如果A点固定
			{
				int ptA_i0 = pMatchPairs[n].ptA_i;
				int ptB_i0 = pMatchPairs[n].ptB_i;
				int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号

				// ptB
				int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号
				// ptB
				double hB[9];
				for(int i=0; i<8; i++)
				{
					hB[i] = cvGetReal2D(pX, 8*ptB_i+i, 0);
				}
				hB[8] = 1;
				double xB = pMatchPairs[n].ptB.x, yB = pMatchPairs[n].ptB.y;
				double h6x_h7y_1  = hB[6]*xB + hB[7]*yB + 1;
				double h0x_h1y_h2 = hB[0]*xB + hB[1]*yB + hB[2];
				cvSetReal2D(pA, 2*n, 8*ptB_i + 0, xB/h6x_h7y_1);
				cvSetReal2D(pA, 2*n, 8*ptB_i + 1, yB/h6x_h7y_1);
				cvSetReal2D(pA, 2*n, 8*ptB_i + 2, 1/h6x_h7y_1);
				cvSetReal2D(pA, 2*n, 8*ptB_i + 6, -h0x_h1y_h2*xB / (h6x_h7y_1*h6x_h7y_1));
				cvSetReal2D(pA, 2*n, 8*ptB_i + 7, -h0x_h1y_h2*yB / (h6x_h7y_1*h6x_h7y_1));

				double h3x_h4y_h5 = hB[3]*xB + hB[4]*yB + hB[5];
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 3, xB/h6x_h7y_1);
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 4, yB/h6x_h7y_1);
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 5, 1/h6x_h7y_1);
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 6, -h3x_h4y_h5*xB / (h6x_h7y_1*h6x_h7y_1));
				cvSetReal2D(pA, (2*n+1), 8*ptB_i + 7, -h3x_h4y_h5*yB / (h6x_h7y_1*h6x_h7y_1));

				// 用固定的参数对A点进行变换
				DfPoint ptA_Bar;
				double h64F[9];
				for(int t=0; t<9; t++)
				{
					h64F[t] = pImagesTransform0[ptA_i0].h.m[t];
				}
				ApplyProject9(h64F, pMatchPairs[n].ptA.x, pMatchPairs[n].ptA.y, ptA_Bar.x, ptA_Bar.y);

				// 给B赋值
				cvSetReal2D(pB, 2*n, 0, ptA_Bar.x - h0x_h1y_h2 / h6x_h7y_1);
				cvSetReal2D(pB, 2*n+1, 0, ptA_Bar.y - h3x_h4y_h5 / h6x_h7y_1);
			}
			else if((pMatchPairs[n].ptA_Fixed==0) && (pMatchPairs[n].ptB_Fixed==1)) // 如果B点固定
			{
				int ptA_i0 = pMatchPairs[n].ptA_i;
				int ptB_i0 = pMatchPairs[n].ptB_i;
				int ptB_i = pMatchPairs[n].ptB_i - vecAccFixed[pMatchPairs[n].ptB_i]; // ptB所在的图像的序号

				// 2*n行, 8*i列
				// ptA
				int ptA_i = pMatchPairs[n].ptA_i - vecAccFixed[pMatchPairs[n].ptA_i]; // ptA所在的图像的序号
				
				double hA[9];
				for(int i=0; i<8; i++)
				{
					hA[i] = cvGetReal2D(pX, 8*ptA_i+i, 0);
				}
				hA[8] = 1;
				double xA = pMatchPairs[n].ptA.x, yA = pMatchPairs[n].ptA.y;
				double h6x_h7y_1_a  = hA[6]*xA + hA[7]*yA + 1;
				double h0x_h1y_h2_a = hA[0]*xA + hA[1]*yA + hA[2];
				cvSetReal2D(pA, 2*n, 8*ptA_i + 0, xA/h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n, 8*ptA_i + 1, yA/h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n, 8*ptA_i + 2, 1/h6x_h7y_1_a);
				cvSetReal2D(pA, 2*n, 8*ptA_i + 6, -h0x_h1y_h2_a*xA / (h6x_h7y_1_a*h6x_h7y_1_a));
				cvSetReal2D(pA, 2*n, 8*ptA_i + 7, -h0x_h1y_h2_a*yA / (h6x_h7y_1_a*h6x_h7y_1_a));

				double h3x_h4y_h5_a = hA[3]*xA + hA[4]*yA + hA[5];
				cvSetReal2D(pA, (2*n+1), 8*ptA_i + 3, xA/h6x_h7y_1_a);
				cvSetReal2D(pA, (2*n+1), 8*ptA_i + 4, yA/h6x_h7y_1_a);
				cvSetReal2D(pA, (2*n+1), 8*ptA_i + 5, 1/h6x_h7y_1_a);
				cvSetReal2D(pA, (2*n+1), 8*ptA_i + 6, -h3x_h4y_h5_a*xA/(h6x_h7y_1_a*h6x_h7y_1_a));
				cvSetReal2D(pA, (2*n+1), 8*ptA_i + 7, -h3x_h4y_h5_a*yA/(h6x_h7y_1_a*h6x_h7y_1_a));

				// 用固定的参数对B点进行变换
				DfPoint ptB_Bar;
				double hB64F[9];
				for(int t=0; t<9; t++)
				{
					hB64F[t] = pImagesTransform0[ptB_i0].h.m[t];
				}
				ApplyProject9(hB64F, pMatchPairs[n].ptB.x, pMatchPairs[n].ptB.y, ptB_Bar.x, ptB_Bar.y);
				// 给B赋值
				cvSetReal2D(pB, 2*n, 0, ptB_Bar.x - h0x_h1y_h2_a/h6x_h7y_1_a);
				cvSetReal2D(pB, 2*n+1, 0, ptB_Bar.y - h3x_h4y_h5_a/h6x_h7y_1_a);
			}
		}

		// 整理计算结果
		vecTransformRefined.clear();
		nUnFixed = 0;
		for(int i=0; i<nImages; i++)
		{
			if(pImagesTransform0[i].fixed==0) // 未固定参数情况
			{
				ImageTransform temp;
				temp.fixed = 0;
				for(int j=0; j<8; j++)
				{
					temp.h.m[j] = (float)cvGetReal2D(pX, 8*nUnFixed+j, 0); // a
				}
				temp.h.m[8] = 1;
				nUnFixed++;
				vecTransformRefined.push_back(temp);
			}
			else
			{
				vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
			}
		}
		
		double errorOld = CalMatchError(&vecTransformRefined[0], nImages, pMatchPairs, nPairs);

		// CV解线性方程组
		cout<<"解线性方程组"<<endl<<endl;
		//int ret = cvSolve(pA, pB, pDX, CV_SVD);

		// cholmod解稀疏线性方程组
		SolveSparseSystem(pA, pB, pDX);

		// 更新
		for(int j=0; j<rowX; j++)
		{
			double x= cvGetReal2D(pX, j, 0);
			double dx = cvGetReal2D(pDX, j, 0);
			x += dx;
			cvSetReal2D(pX, j, 0, x);
		}

		// 整理计算结果
		vecTransformRefined.clear();
		nUnFixed = 0;
		for(int i=0; i<nImages; i++)
		{
			if(pImagesTransform0[i].fixed==0) // 未固定参数情况
			{
				ImageTransform temp;
				temp.fixed = 0;
				for(int j=0; j<8; j++)
				{
					temp.h.m[j] = (float)cvGetReal2D(pX, 8*nUnFixed+j, 0); // a
				}
				temp.h.m[8] = 1;
				nUnFixed++;
				vecTransformRefined.push_back(temp);
			}
			else
			{
				vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
			}
		}

		double errorNew = CalMatchError(&vecTransformRefined[0], nImages, pMatchPairs, nPairs);
		cout<<"errorOld "<<errorOld<<" errorNew "<<errorNew<<endl;
	}

	// 整理计算结果
	vecTransformRefined.clear();
	nUnFixed = 0;
	for(int i=0; i<nImages; i++)
	{
		if(pImagesTransform0[i].fixed==0) // 未固定参数情况
		{
			ImageTransform temp;
			temp.fixed = 0;
			for(int j=0; j<8; j++)
			{
				temp.h.m[j] = (float)cvGetReal2D(pX, 8*nUnFixed+j, 0); // a
			}
			temp.h.m[8] = 1;
			nUnFixed++;
			vecTransformRefined.push_back(temp);
		}
		else
		{
			vecTransformRefined.push_back(pImagesTransform0[i]); // 固定的变换参数
		}
	}

	cvReleaseMat(&pA);
	cvReleaseMat(&pB);
	cvReleaseMat(&pX);
	cvReleaseMat(&pDX);

	return 0;
}

// 对序列图像进行分段
int CMosaicByPose::SegmentSequenceImages(ImagePoseInfo* pImagePoses, int nImages, 
										 RectifiedImage* pRectified, vector<Segment> &vecSegments)
{
	if(NULL==pRectified)
		return -1;

	// 第一张图像固定
	vecSegments.push_back(Segment(0,nImages-1));
	pImagePoses[0].fixed = 1;
	pImagePoses[nImages-1].fixed = 1;

	int nPreviousFixed = 0;
	for(int n=0; n<nImages; n++)
	{
		// 计算每幅图像的物理宽度
		double hCam = pImagePoses[nPreviousFixed].camPose.pos.z;
		double wPhy = pImagePoses[nPreviousFixed].pImg->width* hCam/ m_inParams.fx;
		double hPhy = pImagePoses[nPreviousFixed].pImg->height* hCam/ m_inParams.fx;
		double imgWPhy = max(wPhy, hPhy); // 图像的物理宽度
		// 
		
		// 计算当前图像的主点到上一张图像的主点之间的距离
		double distToPre;
		DistanceOfTwoPoints(pRectified[n].imgCxPys, pRectified[n].imgCyPys, 
							pRectified[nPreviousFixed].imgCxPys, pRectified[nPreviousFixed].imgCyPys, distToPre);
		if(distToPre>=imgWPhy*1.5)
		//if(distToPre>=imgWPhy*150)
		{
			// 找到一个固定点
			vecSegments[(int)vecSegments.size()-1].end = n;
			vecSegments.push_back(Segment(n, nImages-1));
			nPreviousFixed = n;
			pImagePoses[n].fixed = 1;
		}
	}

	if(vecSegments[vecSegments.size()-1].end == vecSegments[vecSegments.size()-1].beg)
	{
		vecSegments.pop_back();
	}
	
	return 0;
}

void CMosaicByPose::MatchThreadOver_Plus()
{
	m_lock.Lock(); // 加锁

	m_nMatchThreadOver++;

	m_lock.Unlock(); // 解锁
}

void CMosaicByPose::PushMatchPairs(const vector<MatchPointPairs> &src)
{
	m_lock.Lock();

	m_vecMatchPairs.insert(m_vecMatchPairs.begin()+m_vecMatchPairs.size(), 
		src.begin(), src.end());

	m_lock.Unlock();
}

// 无人机图像拼接
int MosaicVavImages( IplImage** pImages, int nImages, 
					GPS_Info *pGpsDatas, int gpsValid, 
					UavMatchParam surfParam, 
					IplImage* &pMosaicResult,
					int &numMosaiced,
					float scale)
{
	numMosaiced = 1;
	cout<<"MosaicVavImages..."<<endl;
	if(NULL==pImages)
		return -1;
	if(nImages<2)
		return -1;
	if(gpsValid==1)
	{
		if(pGpsDatas==NULL)
			return -1;
	}
	
	int nRet = 0;

	pMosaicResult = NULL;

	vector<ImagePoseInfo> vecImagePoses(nImages);
	for(int n=0; n<nImages; n++)
	{
		vecImagePoses[n].pImg = pImages[n];
	}

	CMosaicByPose mosaic;
	mosaic.m_blending = surfParam.blending;
	mosaic.m_matchDist = surfParam.matchDist;
	mosaic.m_maxFeatureNum = surfParam.maxFeatruesNum;
	mosaic.m_minHessian = surfParam.minHessian;
	mosaic.m_ransacDist = surfParam.ransacDist;
	mosaic.m_fLoadMatchPairs = surfParam.loadMatchPairs;
	mosaic.m_scale = scale;
	mosaic.m_width = pImages[0]->width;
	mosaic.m_height = pImages[0]->height;

	if(0==gpsValid) // gps invalid
	{
		nRet = mosaic.MosaicWithoutPose(&vecImagePoses[0], nImages, numMosaiced);

		if(mosaic.GetMosaicResult())
		{
			pMosaicResult = cvCloneImage(mosaic.GetMosaicResult());
		}
		
		if(mosaic.m_blending==2)
		{
			for(int i=0; i<numMosaiced; i++)
			{
				if(vecImagePoses[i].pImg==NULL)
				{
					pImages[i] = NULL;
				}
			}
		}
	}
	else  // gps有效
	{
		
	}

	return nRet;
}

// 功能: 视频序列图像拼接
// 返回值含义: 0正常, -1输入参数错误, -2拼接失败
// pVideoPath---------------avi视频完整地址  XXX.avi
// pGpsDataPath-------------gps数据完整路径  XXX.txt
// surfParam----------------拼接参数
// maxOnceMosaicNum---------每一次最多拼接的图像数量 200
// imageInterval------------图像间隔
// mosaicResultDirectory----图像拼接结果保存目录 
int MosaicUavVideo(char* pVideoPath, 
				   char* pGpsDataPath,
				   UavMatchParam surfParam,
				   int maxOnceMosaicNum,
				   int imageInterval,
				   CutInfo cut,
				   string mosaicResultDirectory,
				   int indexStartFrame, 
				   int indexEndFrame, 
				   float scale)
{
	if(pVideoPath==NULL)
		return -1;

	//CreateFolder( mosaicResultDirectory.c_str() );

	CreateDirectoryA( mosaicResultDirectory.c_str(), NULL);

	if(pGpsDataPath==NULL) // 不用GPS信息
	{
		// 解析AVI视频, 并获取所有解析出的图像的路径
		vector<FilePath> vecImageList;
		ConvertVideo2Bmp(pVideoPath, imageInterval, cut, vecImageList, indexStartFrame, indexEndFrame);
		int nTotalImages = (int)vecImageList.size();
		if(nTotalImages<=1)
			return -2;
		
		// 拼接
		for(int n=0; n<nTotalImages;)
		{
			cout<<n<<" / "<<nTotalImages<<endl;
			int nBeg = n; 
			int nEnd = Min(n + maxOnceMosaicNum-1, nTotalImages-1);
			vector<IplImage*> vecImages;
			for(int i=nBeg; i<=nEnd; i++)
			{
				// 读取图像
				IplImage* pSrc = cvLoadImage(vecImageList[i].path, CV_LOAD_IMAGE_COLOR);
				if(pSrc)
				{
					vecImages.push_back(pSrc);
				}
			}
			int nSrcImages = nEnd - nBeg + 1;
			IplImage* pMosaicResult = NULL;
			int gpsValid = 0;
			int numMosaic = 0;
			int nRet = 0;
			if(!vecImages.empty())
			{
				nRet = MosaicVavImages(&vecImages[0], (int)vecImages.size(), 
										NULL, gpsValid, 
										surfParam, 
										pMosaicResult, 
										numMosaic, scale);
			}		
			if(pMosaicResult) // 如果拼出了图像, 保存
			{
				SYSTEMTIME sysTime;    
				GetLocalTime( &sysTime );
				char fileName[200];
				sprintf_s(fileName, "\\mosRes_%d_%d_%d_%d_%d_%d_%d.jpg", 
					sysTime.wYear, sysTime.wMonth, sysTime.wDay, // 年月日
					sysTime.wHour, sysTime.wMinute, sysTime.wSecond,  // 时分秒
					sysTime.wMilliseconds);
				string strName(fileName);
				string strSavePath = mosaicResultDirectory + strName;
				cvSaveImage(strSavePath.c_str(), pMosaicResult);
				cvReleaseImage(&pMosaicResult);
			}
			// 释放图像
			for(int i=0; i<nSrcImages; i++)
			{
				cvReleaseImage(&vecImages[i]);
			}
			n = nBeg + numMosaic;
		}
	}
	else// 用GPS信息
	{
		// 未完成
	}

	return 0;
}

// 将AVI视频转换成BMP
int ConvertVideo2Bmp(char* pVideoPath, 
					 int imageInterval, // 图像间隔
					 CutInfo cut,
					 vector<FilePath> &vecImagePathList,
					 int indexStartFrame, 
					 int indexEndFrame)
{
	if(pVideoPath==NULL)
		return -1;

	cout<<"ConvertVideo2Bmp"<<endl;
	//int nImage = 1000;
	//for(int i=0; i<nImage; i++)
	//{
	//	FilePath filePath;
	//	sprintf_s(filePath.path, "c:\\temp\\v_%d.bmp", i);
	//	vecImagePathList.push_back(filePath);
	//}
	//return 0;

	CvCapture* pCapture = cvCaptureFromAVI(pVideoPath);
	if(!pCapture)
	{
		cout<<"capture failed"<<endl;
	}

	//CreateFolder(_T("c:\\temp"));
	CreateDirectory(_T("c:\\temp"),   NULL   );

	int nRround = 0;
	FilePath savePath;
	IplImage* img = NULL;
	int i=0;
	int nFrames = 0;
	do
	{
		img = cvQueryFrame(pCapture);
		
		if( (indexEndFrame>0) && (indexStartFrame>=0) ) // 如果指定了首帧 和 尾帧
		{
			if( (nFrames<indexStartFrame) || (nFrames>indexEndFrame) )
			{
				nFrames++;
				continue;
			}
		}
		nFrames++;
		if(img)
		{
			if(nRround==0)
			{
				// 裁剪图像
				IntPoint begPt, endPt;
				begPt.x = cut.cutL; endPt.x = img->width - 1 - cut.cutR;
				begPt.y = cut.cutU; endPt.y = img->height - 1 - cut.cutD;
				sprintf_s(savePath.path, "c:\\temp\\v_%d.bmp", i);
				IplImage* pCut = NULL;
				pCut = CutPatchImage(img, begPt, endPt);
				cvSaveImage(savePath.path, pCut);
				cvReleaseImage(&pCut);
				i++;
				cout<<i<<endl;
				vecImagePathList.push_back(savePath);
			}
			nRround++;
			if(nRround==imageInterval)
			{
				nRround = 0;
			}
		}
	}
	while(img);

	cvReleaseCapture(&pCapture);

	return 0;
}

// 将AVI视频转换成BMP
int ConvertVideo2Bmp(char* pVideoPath, 
					 string imagesDirectory,
					 int imageInterval, // 图像间隔
					 CutInfo cut)
{
	if(pVideoPath==NULL)
		return -1;

	cout<<"ConvertVideo2Bmp"<<endl;

	CvCapture* pCapture = cvCaptureFromAVI(pVideoPath);
	if(!pCapture)
	{
		cout<<"Capture avi failed"<<endl;
	}

	int nRround = 0;
	FilePath savePath;
	IplImage* img = NULL;
	int i=0;
	int nFrames = 0;
	do
	{
		img = cvQueryFrame(pCapture);

		nFrames++;

		if(img)
		{
			if(nRround==0)
			{
				// 裁剪图像
				IntPoint begPt, endPt;
				begPt.x = cut.cutL; endPt.x = img->width - 1 - cut.cutR;
				begPt.y = cut.cutU; endPt.y = img->height - 1 - cut.cutD;
				//sprintf_s(savePath.path, "c:\\temp\\v_%d.bmp", i);

				string strNum;
				stringstream ost(strNum);
				ost<<i;
				strNum = ost.str();

				string imagePath;
				imagePath = imagesDirectory + "/v_" + strNum + ".bmp";
				
				IplImage* pCut = NULL;
				pCut = CutPatchImage(img, begPt, endPt);
				cvSaveImage(imagePath.c_str(), pCut);
				cvReleaseImage(&pCut);
				i++;
				cout<<i<<endl;
			}
			nRround++;
			if(nRround==imageInterval)
			{
				nRround = 0;
			}
		}
	}
	while(img);

	cvReleaseCapture(&pCapture);

	return 0;
}

IplImage* ShowMatchPairs(IplImage* pImgA, IplImage* pImgB, const vector<MatchPointPairs> &vecPairs)
{
	int w1, h1, ws1;
	CvGetImageSize(pImgA, w1, h1, ws1);
	int w2, h2, ws2;
	CvGetImageSize(pImgB, w2, h2, ws2);


	IplImage* pShow = cvCreateImage(cvSize(max(w1, w2), h1+h2), 8, 3);
	int wsShow = pShow->widthStep;

	for(int r=0; r<h1; r++)
	{
		memcpy(pShow->imageData+r*wsShow, pImgA->imageData+r*ws1, ws1);
	}
	for(int r=0; r<h2; r++)
	{
		memcpy(pShow->imageData+(r+h1)*wsShow, pImgB->imageData+r*ws2, ws2);
	}
	//double []
	for(int n=0; n<vecPairs.size(); n++)
	{
		CvPoint pt1, pt2;
		pt1.x = vecPairs[n].ptA.x;
		pt1.y = vecPairs[n].ptA.y;

		pt2.x = vecPairs[n].ptB.x;
		pt2.y = vecPairs[n].ptB.y + h1;

		CvScalar color;
		if(n%2==0)
		{
			color.val[0] = 0; 
			color.val[1] = 0; 
			color.val[2] = 255; 
		}
		else
		{
			color.val[0] = 255; 
			color.val[1] = 0; 
			color.val[2] = 0; 
		}

		cvLine(pShow, pt1, pt2, color);
	}
	
	return pShow;
}

// 显示多视图匹配对
int ShowMultiViewMatchPairs(ImagePoseInfo* pImgs, int nImages, const vector<MatchPointPairs> &vecPairs)
{
	int nPairs = vecPairs.size();
	bool startShow = false;

	vector<MatchPointPairs> vecTwoImagesPairs;
	for(int n=1; n<nPairs; )
	{
		if( (vecPairs[n].ptA_i!=vecPairs[n-1].ptA_i) || (vecPairs[n].ptB_i!=vecPairs[n-1].ptB_i) )
		{
			// 显示匹配对

			IplImage* pImg1 = pImgs[vecTwoImagesPairs[0].ptA_i].pImg;
			IplImage* pImg2 = pImgs[vecTwoImagesPairs[0].ptB_i].pImg;

			IplImage* pShow = ShowMatchPairs(pImg1, pImg2, vecTwoImagesPairs);
			cout<<vecTwoImagesPairs[0].ptA_i<<" - "<<vecTwoImagesPairs[0].ptB_i<<"  "<<vecTwoImagesPairs.size()<<endl;
			ShowImageCv(pShow, "match-pairs"); cvWaitKey(-1);
			//if( (vecTwoImagesPairs[0].ptA_i==260) || (vecTwoImagesPairs[0].ptB_i==260) )
			//	cvWaitKey(-1);
			//if( (vecTwoImagesPairs[0].ptA_i==46) || (vecTwoImagesPairs[0].ptB_i==46) )
			//	cvWaitKey(-1);
			//if( (vecTwoImagesPairs[0].ptA_i==97) || (vecTwoImagesPairs[0].ptB_i==97) )
			//	cvWaitKey(-1);
			//if( (vecTwoImagesPairs[0].ptA_i==573) || (vecTwoImagesPairs[0].ptB_i==573) )
			//	cvWaitKey(-1);
			//if( (vecTwoImagesPairs[0].ptA_i==513) || (vecTwoImagesPairs[0].ptB_i==513) )
			//	cvWaitKey(-1);
			//if( (vecTwoImagesPairs[0].ptA_i==383) || (vecTwoImagesPairs[0].ptB_i==383) )
			//	cvWaitKey(-1);
			//if( (vecTwoImagesPairs[0].ptA_i==575) || (vecTwoImagesPairs[0].ptB_i==575) )
			//	cvWaitKey(-1);
			//if( (vecTwoImagesPairs[0].ptA_i==448) || (vecTwoImagesPairs[0].ptB_i==448) )
			//	cvWaitKey(-1);
			//if( (vecTwoImagesPairs[0].ptA_i==321) || (vecTwoImagesPairs[0].ptB_i==321) )
			//	cvWaitKey(-1);
				

			vecTwoImagesPairs.clear();
			vecTwoImagesPairs.push_back(vecPairs[n]);
			cvReleaseImage(&pShow);
			n++;
		}
		else
		{
			vecTwoImagesPairs.push_back(vecPairs[n]);
			n++;
		}
	}
	return 0;
}