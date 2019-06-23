/* ***********************************************************
Xu Yuhua, 2012-07-08
**************************************************************/
#pragma once
#include "stdafx.h"
#include "opencv2\opencv.hpp"
#include "Bitmap.h"
#include "Mosaic1By1.h"
//#include "atlstr.h"


using namespace pool;

struct CutInfo // 图像裁剪信息
{
	int cutL, cutR, cutU, cutD; // 左边的裁剪量, 右边的裁剪量,上边的裁剪量, 下边的裁剪量

	CutInfo(int cutL, int cutR, int cutU, int cutD)
	{
		this->cutL = cutL;
		this->cutR = cutR;
		this->cutU = cutU;
		this->cutD = cutD;
	}

	CutInfo()
	{
		cutL = 20;
		cutR = 25;
		cutU = 20;
		cutD = 100;
	}
};

struct FilePath
{
	char path[200];
};

// 像机内参
struct IntrinsicParams
{
	int width, height;
	double cx, cy;	// 主点
	double fx, fy;	// 有效焦距
	double k[9];		// 畸变系数
	double B;
	IntrinsicParams()
	{
		cx = 0;
		cy = 0;
		fx = 0;
		fy = 0;
	}
};

struct UavMatchParam
{
	IntrinsicParams cam; // 像机内参

	int minHessian;
	int maxFeatruesNum;
	float matchDist;
	float ransacDist;
	int blending; // 0不融合 1加权融合 2多频带融合
	int downViewConstraint; // 1使用正下视约束 0不使用正下视约束
	int loadMatchPairs;

	UavMatchParam()
	{
		minHessian = 50;
		maxFeatruesNum = 200;
		matchDist = 0.5f;
		ransacDist = 2.5f;
		blending = 2;
		downViewConstraint = 1;
		loadMatchPairs = 0;
	}
};

struct GPS_Info
{
	int valid; // GPS数据是否有效标记 0为无效 1为有效 

	double ax; // 俯仰角 度
	double ay; // 滚动角
	double az; // 偏航角度

	double H1; // 海拔高度
	double H2; // 到地面高度
	double B; // 纬度
	double L; // 经度

	GPS_Info()
	{
		valid = 0;
	}
};

struct GPS_Data
{
	int valid; // GPS数据是否有效标记 0为无效 1为有效 

	double ax; // 俯仰角
	double ay; // 滚动角
	double az; // 偏航角度

	double h; // 高度
	double x; // 纬度
	double y; // 经度

	GPS_Data()
	{
		valid = 0;
	}
};

struct Segment
{
	int beg;
	int end;
	Segment(int beg, int end)
	{
		this->beg = beg;
		this->end = end;
	}
	Segment()
	{
		beg = 0;
		end = 0;
	}
};

// 匹配的点对
struct MatchPointPairs
{
	// 点A
	SfPoint ptA;	
	// 点A所在的图像的序号
	int ptA_i;
	// 点A是否是固定点
	int ptA_Fixed;

	// 点B
	SfPoint ptB;
	// 点B所在的图像的序号
	int ptB_i;
	// 点B是否是固定点
	int ptB_Fixed;

	// 世界坐标点的编号
	//int wpIdx; // world Point Index
};

// 匹配的点对
struct MatchPointPairs2
{
	// 点A
	SfPoint ptA;	
	// 点A所在的图像的序号
	int ptA_i;
	// 点A是否是固定点
	int ptA_Fixed;

	// 点B
	SfPoint ptB;
	// 点B所在的图像的序号
	int ptB_i;
	// 点B是否是固定点
	int ptB_Fixed;

	// 世界坐标点的编号
	int wpIdx; // world Point Index

	MatchPointPairs2()
	{
		wpIdx = -1;
	}
};

struct ImagePoint
{
	float x;
	float y;
	int imIdx;
	int id;
};

// 重建的点
struct ReconstructedPoint
{
	float x; // 在世界坐标系中的坐标x
	float y; // 在世界坐标系中的坐标y
	float z;
	int idx;
	int fixed;
	vector<ImagePoint> imgPt; // 该点在能观测到它的图像中的坐标 (可能会被多幅图像观测到)

	ReconstructedPoint()
	{
		idx = -1;
		fixed = 0;
	}
};

// 匹配的点对
struct MatchPointPairs_CV32F
{
	// 点A
	CvPoint3D32f ptA;	
	// 点A所在的图像的序号
	int ptA_i;
	// 点A是否是固定点
	int ptA_Fixed;

	// 点B
	CvPoint3D32f ptB;
	// 点B所在的图像的序号
	int ptB_i;
	// 点B是否是固定点
	int ptB_Fixed;
};

struct ImageTransform
{
	ProjectMat h;
	int fixed;
};

struct MatchNeighbourError
{
	double error;
	int iImage;
	int nMatch;
	bool   operator <  (const MatchNeighbourError& match)  const   //升序排序时必须写的函数
	{   
		return error<match.error;
	}
};

struct Translation32F
{
	float dx,dy;
	int fixed;
};

struct ImageTransform64F
{
	ProjectMat64F h;
	int fixed;
};

//// 像机姿态
//struct CameraPose
//{
//	// 位置
//	CvPoint3D32f pos;
//	// 角度
//	float AX, AY, AZ;
//	// 姿态是否有效, 1为有效
//	int isValid;
//	float R[9];
//	float T[3];
//
//};

// 像机姿态
struct CameraPose64F
{
	// 位置
	CvPoint3D64f pos;
	// 角度
	float AX, AY, AZ;
	// 姿态是否有效, 1为有效
	int isValid;
	double R[9]; // 旋转矩阵
	double T[3]; // 平移矩阵
	double k[2]; // 畸变系数
	double F; // 有效焦距
};

// 图像和像机姿态信息
struct ImagePoseInfo
{
	//图像指针
	IplImage* pImg;

	//像机姿态
	CameraPose64F camPose;

	int fixed;

	ImagePoseInfo()
	{
		fixed = 0;
	}
};

// 纠正后的图像
struct RectifiedImage
{
	ProjectMat64F H;
	double imgCxPys,imgCyPys; // 图像主点的物理坐标(m)
	double imgCxPix,imgCyPix; // 图像主点的图像坐标(pixel)
};

struct MatchEdge
{
	int idxImg1, idxImg2;
};

// 多帧特征跟踪
int TrackMultiFrames(vector<MatchPointPairs> &matched, int nImages);

// 使用bundler的输出结果
// vecMatchPoints--------特征点对
int ImportBundlerOut(char* pPath, vector<CameraPose64F> &camPose, vector<CvPoint3D64f> &points,
					 vector<MatchPointPairs> &vecMatchPoints);

int ImportVisualSFM_Match(char* pPath, vector<MatchPointPairs> &vecMatchPoints, float cx, float cy);

int ImportVisualSFM_Match2(char* pPath, vector<MatchPointPairs> &vecMatchPoints, float cx, float cy);

// 使用cmvs的输出结果 wu-changchang
int ImportCMVSOut(char* pPath, vector<CameraPose64F> &camPose, vector<CvPoint3D64f> &points,
				  vector<MatchPointPairs> &vecMatchPoints);

// 读取list中的图像的名字
int ReadImageList(char* pPath, vector<string> &vecList);

template<class T1, class T2, class T3>
void ApplyProject9(const T1 h[9], const T2 xSrc, const T2 ySrc, T3 &xDst, T3 &yDst)
{
	xDst = (h[0]*xSrc + h[1]*ySrc + h[2]) / (h[6]*xSrc + h[7]*ySrc + h[8]);
	yDst = (h[3]*xSrc + h[4]*ySrc + h[5]) / (h[6]*xSrc + h[7]*ySrc + h[8]);
}

// 单应矩阵 + 平移(dx,dy)
template<class T1>
void HomographyPlusTranslation(T1 H[9], T1 dx, T1 dy)
{
	H[0] = H[0] + dx * H[6];
	H[1] = H[1] + dx * H[7];
	H[2] = H[2] + dx * H[8];

	H[3] = H[3] + dy * H[6];
	H[4] = H[4] + dy * H[7];
	H[5] = H[5] + dy * H[8];

	//H[6] = H[6];
	//H[7] = H[7];
	//H[8] = H[8];
}

// 根据像机的姿态对图像进行拼接
class CMosaicByPose
{
public:
	CMosaicByPose();

	~CMosaicByPose();

private:
	// 像机内参
	IntrinsicParams m_inParams;
public:

	// Bundler's out
	// 由像机参数计算每幅图像的单应变换矩阵
	// 适用于近似平面场景的图像拼接
	int CamPose2Homo3(ImagePoseInfo *pImagePose, int nImages, vector<CvPoint3D64f> &vecPoints, 
		vector<ProjectMat64F> &homos, vector<MatchPointPairs> vecMatchPairs);

	// 没有位姿信息的图像匹配
	int MosaicWithoutPose(ImagePoseInfo *pImgPoses, const int nImages, int &numMosaiced);


	// 单航带SURF图像特征点对应
	// 帧间匹配
	int GetMatchedPairsSingleStripSurf(const ImagePoseInfo *pImgPoses, const int nImages, 
										vector<MatchPointPairs> &vecMatchPairs,
										int &nSuccess);

	// 特征匹配
	// 没有位姿先验信息
	// one-to-all
	int GetMatchedPairsOneToAllSurf(const ImagePoseInfo *pImgPoses, const int nImages, 
									vector<MatchPointPairs> &vecMatchPairs,
									int &nSuccess);

	int GetMatchedPairsOneToAllSIFT_MultiThread();

	// 单航带图像特征点对应
	int GetMatchedPairsSingleStrip(const ImagePoseInfo *pImgPoses, const int nImages, 
										vector<MatchPointPairs> &vecMatchPairs, int &nFixedImages);
	
	// 多航带图像特征点对应
	int GetMatchedPairsMultiStrip(const ImagePoseInfo *pImgPoses, const int nImages, 
										vector<MatchPointPairs> &vecMatchPairs, int &nFixedImages);

	int GetMatchedPairsMultiStripSurf(const ImagePoseInfo *pImgPoses, const int nImages, 
										vector<MatchPointPairs> &vecMatchPairs, int &nFixedImages);

	// 对序列图像进行分段
	int SegmentSequenceImages(ImagePoseInfo* pImagePoses, int nImages, 
								RectifiedImage* pRectified, vector<Segment> &vecSegments);

	// 对每段图像进行拼接计算
	int MosaicBySegment(ImagePoseInfo *pImgPoses, const int nImages, ImageTransform* pImageTransformInit,
						vector<ImageTransform> &vecImageTransformRefined);
private:
	double m_heightNorm;// 规范的相机高度
public:
	double HeightNorm() const { return m_heightNorm; }
	void HeightNorm(double val) { m_heightNorm = val; }
public:
	IntrinsicParams InParams() const { return m_inParams; }
	void InParams(IntrinsicParams val) { m_inParams = val; }

public:

	// 用特征点对再一次调整变换参数
	int AdjustByFeaturePairs(ImagePoseInfo *pImgPoses, 
				vector<ImageTransform> &vecTrans, const vector<MatchPointPairs> &vecMatchedPoints, int nImages, int adjust);

	int Mosaic(ImagePoseInfo *pImages, int nImage);

	int Mosaic2( ImagePoseInfo *pImgPoses, int nImages, vector<MatchPointPairs> vecMatchedPoints);

public:
	// 用GPS信息和图像配准, 来做图像拼接
	int MosaicByPose_Image(ImagePoseInfo *pImgPoses, const int nImages);

	int MosaicByBundler(ImagePoseInfo *pImgPoses, const int nImages, 
		vector<CvPoint3D64f> &vecPoints, vector<MatchPointPairs> vecMatchPairs);

private:
	// 正下视校正
	int RectifyImage2(const ImagePoseInfo *pImagePose, ProjectMat64F &H, double &imgCxPix, double &imgCyPix);

	// 正下视校正
	int RectifyImage3(const ImagePoseInfo *pImagePose, ProjectMat64F &H, double &imgCxPix, double &imgCyPix);

	// 图像镶嵌
	int MosaicImagesByPoses(const ImagePoseInfo *pImgPoses, const int nImages, const RectifiedImage* pRectified);

	// 由像机姿态得到每幅图像的拼接变换参数
	int MosaicImagesByPoses(const ImagePoseInfo *pImgPoses, const int nImages, const RectifiedImage* pRectified,
							vector<ImageTransform> &vecTrans);

	int MosaicImagesRefined(const ImagePoseInfo *pImgPoses, const int nImages, const ImageTransform* pRectified);

	int MosaicImagesRefined(const ImagePoseInfo *pImgPoses, const int nImages, const ImageTransform64F* pRectified);

	int MergeImagesRefined(ImagePoseInfo *pImgPoses, const int nImages, const ImageTransform* pRectified);

private:
	// 拼接结果图
	IplImage* m_pMosaicResult;
public:
	IplImage* GetMosaicResult() const { return m_pMosaicResult; }
	
private:
	// 根据GPS和惯导数据计算图像的变换参数
	int CalImgTransformByPosIMU(const ImagePoseInfo *pImgPoses, const int nImages, const RectifiedImage* pRectified,
								vector<ProjectMat> &vecImagesTranform);
	
	// 用图像配准的方法调整由GPS/IMU计算得到的图像变换参数
	int AdjustmentByImage(const ImagePoseInfo *pImgPoses, const int nImages, ImageTransform* pTransformInit,
						vector<ImageTransform> &vecTransformRefined);

	// QuadraticProgram方法
	int QuadraticProgram(const ImagePoseInfo *pImgPoses,
						MatchPointPairs* pMatchPairs, int nPairs,
						const ImageTransform* pImagesTransform0, int nImages, 
						int nFixedImages,
						vector<ImageTransform> &vecTransformRefined);

	// 捆集调整
	int BundleAdjustment(MatchPointPairs* pMatchPairs, int nPairs,
						const ImageTransform* pImagesTransform0, int nImages,
						int nFixedImages,
						vector<ImageTransform> &vecTransformRefined);

	// 线性捆集调整
	int BundleAdjustmentSparse(MatchPointPairs* pMatchPairs, int nPairs,
						const ImageTransform* pImagesTransform0, int nImages, 
						int nFixedImages,
						vector<ImageTransform> &vecTransformRefined);

	// 捆集调整---平移模型
	int SBA_Translation(MatchPointPairs* pMatchPairs, int nPairs,
						const Translation32F* pImagesTransform0, int nImages, 
						int nFixedImages,
						vector<Translation32F> &vecTransformRefined);

	// 带约束的线性捆集调整
	int BundleAdjustmentSparseConstraint(MatchPointPairs* pMatchPairs, int nPairs,
						const ImageTransform* pImagesTransform0, int nImages, 
						int nFixedImages,
						vector<ImageTransform> &vecTransformRefined);

	// 带rot约束的非线性捆集调整
	// wRot为rot约束的权重
	int SparseAffineRotConstraint(MatchPointPairs* pMatchPairs, int nPairs,
						const ImageTransform* pImagesTransform0, int nImages, 
						int nFixedImages, float weight, 
						vector<ImageTransform> &vecTransformRefined);

	// 非线性稀疏捆集调整 + rot约束 (参考图像的参数不做优化)
	int SparseNonlinearRotConstraint(MatchPointPairs* pMatchPairs, int nPairs,
						ImageTransform* pImagesTransform0, int nImages, 
						int nFixedImages, float weight,
						vector<ImageTransform> &vecTransformRefined,
						int label[10000]);

	// 非线性稀疏捆集调整 + rot约束 - v2 (参考图像的参数也做优化)
	int SparseNonlinearRotConstraint_V2(MatchPointPairs* pMatchPairs, int nPairs,
						ImageTransform* pImagesTransform0, int nImages, 
						int nFixedImages, int refImgIdx0,
						int weight,
						vector<ImageTransform> &vecTransformRefined,
						int label[10000]);

	// Lagrange方法
	int BundleAdjustmentLagrange(const ImagePoseInfo *pImgPoses,
						MatchPointPairs* pMatchPairs, int nPairs,
						const ImageTransform* pImagesTransform0, int nImages, 
						int nFixedImages,
						vector<ImageTransform> &vecTransformRefined);

	int BundleAdjustmentFixPoint(MatchPointPairs* pMatchPairs, int nPairs,
								const ImageTransform* pImagesTransform0, int nImages, 
								int nFixedImages,
								vector<ImageTransform> &vecTransformRefined);

	// 固定的图像的中心点不变, 其余4个自由度可变, 固定的中心点, 直接加入矩阵A
	int BundleAdjustmentLS(const ImagePoseInfo *pImgPoses,
						MatchPointPairs* pMatchPairs, int nPairs,
						const ImageTransform* pImagesTransform0, int nImages, 
						int nFixedImages,
						vector<ImageTransform> &vecTransformRefined);

	// 非线性捆集调整
	int BundleAdjustmentNonlinear(MatchPointPairs* pMatchPairs, int nPairs,
						ImageTransform* pImagesTransform0, int nImages, 
						int nFixedImages,
						vector<ImageTransform> &vecTransformRefined);

	// 非线性稀疏捆集调整
	int BANonlinearSparse(MatchPointPairs* pMatchPairs, int nPairs,
						ImageTransform* pImagesTransform0, int nImages, 
						int nFixedImages,
						vector<ImageTransform> &vecTransformRefined);


private:
	CMosaic mosaic;

	vector<RectifiedImage> vecRectified;

public:
	int m_blending;
	int m_maxFeatureNum;
	int m_minHessian;
	float m_matchDist;
	float m_ransacDist;
	int m_downViewConstraint; // 使用正下视约束
	int m_fLoadMatchPairs;
	float m_scale;

	ImagePoseInfo *m_pImgPoses;
	
	int m_nImages;

	int m_nMatchThread;

	int m_idxMatchThread;

	int m_nMatchThreadOver; // 特征提取和匹配当前的状态(完成了几个线程)

	int m_width;
	
	int m_height;

	void MatchThreadOver_Plus();

private:
	CMutexLock m_lock;

private:
	vector<MatchPointPairs> m_vecMatchPairs;
public:
	void PushMatchPairs(const vector<MatchPointPairs> &src);
};

// 将AVI视频转换成BMP
int ConvertVideo2Bmp(char* pVideoPath, 
					 int imageInterval, 
					 CutInfo cut,
					 vector<FilePath> &vecImagePathList,
					 int indexStartFrame, 
					 int indexEndFrame);

// 将AVI视频转换成BMP
int ConvertVideo2Bmp(char* pVideoPath, 
					 string pImagesDirectory,
					 int imageInterval, // 图像间隔
					 CutInfo cut);

int GetInnerPoints(SfPoint* &pInner1, SfPoint* &pInner2, int &nInner);

// 显示多视图匹配对
int ShowMultiViewMatchPairs(ImagePoseInfo* pImgs, int nImages, const vector<MatchPointPairs> &vecPairs);

IplImage* ShowMatchPairs(IplImage* pImgA, IplImage* pImgB, const vector<MatchPointPairs> &vecPairs);

// Bundler's out
// 由像机参数计算每幅图像的单应变换矩阵
// 适用于近似平面场景的图像拼接
int CamPose2Homo(const ImagePoseInfo *pImagePose, int nImages, const vector<CvPoint3D64f> &vecPoints, 
				 vector<ProjectMat64F> &homos);

// Bundler's out
// 由像机参数计算每幅图像的单应变换矩阵
// 适用于近似平面场景的图像拼接
int CamPose2Homo2(const ImagePoseInfo *pImagePose, int nImages, const vector<CvPoint3D64f> &vecPoints, 
				  vector<ProjectMat64F> &homos);

// 无人机图像拼接图像
// 返回值含义, 0为成功, -1输入参数错误, -2拼接失败
// pImages------------原始图像
// nImages------------图像的数量
// pGpsDatas----------GPS数据
// gpsValid-----------gps数据是否有效，1-valid, 0-invalid
// pMosaicResult------拼接的结果, 其内存由程序自动分配, 由用户释放
// numMosaiced--------拼接成功图像数, 显然, numMosaiced<=nImages
extern "C" __declspec(dllexport)
int MosaicVavImages(IplImage** pImages, 
					int nImages, 
					GPS_Info *pGpsDatas, 
					int gpsValid, 
					UavMatchParam surfParam,
					IplImage* &pMosaicResult,
					int &numMosaiced,float scale);

// 功能: AVI视频序列图像拼接
// pVideoPath---------------avi视频完整地址,  XXX.avi
// pGpsDataPath-------------gps数据完整路径,  XXX.txt, GPS信息无效时, 给NULL
// mosaicParam--------------图像拼接参数,
// maxOnceMosaicNum---------每次拼接最多处理的图像数, 默认100
// imageInterval------------图像间隔数量, 默认5
// cut----------------------裁剪信息, (对于现在AVI视频取 20, 25, 20, 100)
// mosaicResultDirectory----图像拼接结果保存目录, 示例 "c:\\temp"
// indexStartFrame----------指定的拼接起始帧号, indexStartFrame>=0, 如果不指定, 请赋-1
// indexEndFrame------------指定的拼接尾帧号, indexEndFrame<视频总帧数, 如果不指定, 请赋-1
// 如果不指定拼接的起始帧和和尾帧号, 则对视频中的所有帧进行拼接
extern "C" __declspec(dllexport)
int MosaicUavVideo( char* pVideoPath, 
					char* pGpsDataPath,
					UavMatchParam mosaicParam,
					int maxOnceMosaicNum,
					int imageInterval,
					CutInfo cut,
					string mosaicResultDirectory,
					int indexStartFrame, 
					int indexEndFrame,
					float scale);