// mosaicing.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <direct.h>
#include "MosaicWithoutPos.h"


int _tmain(int argc, _TCHAR* argv[])
{
	mkdir("feature_temp");

	UavMatchParam surfParam;
	surfParam.blending = 2;
	////surfParam.loadMatchPairs = m_loadMatchPairs;
	//cout<<"blending "<<surfParam.blending<<endl;

	int nTotalImages = 20;
	float scale = 1.0; // scale for the mosaicing result

	// ÐòÁÐÍ¼Ïñ±£´æÄ¿Â¼
	std:string imagesDirectory = "test_data";
	//CStringA stra1(m_SequentialImagesDirectory.GetBuffer(0));
	//imagesDirectory = stra1.GetBuffer(0);

	// Æ´½ÓÍ¼Ïñ±£´æÄ¿Â¼
	std::string mosaicDirectory = "test_data";
	//CStringA stra2(m_mosaicDirectory.GetBuffer(0));
	//mosaicDirectory = stra2.GetBuffer(0);

	//CreateDirectory(m_mosaicDirectory, NULL);

	//// image path
	std::vector<std::string> vecImageList;
	int startIndex  = 4;
	for(int n=0; n<nTotalImages; n++)
	{
		string strNum;
		stringstream ost(strNum);
		ost<<startIndex + n;
		strNum = ost.str();

		char numChar[100];
		string numString, imagePath;
		
		sprintf_s(numChar, "%05d", n + startIndex); numString = numChar; 
		imagePath = imagesDirectory + "/DSC" + numString + ".jpg";
		
		vecImageList.push_back(imagePath);
	}


	std::cout<<"Load Images"<<endl;
	vector<IplImage*> vecImages;
	for(int n=0; n<nTotalImages; n++)
	{
		// ¶ÁÈ¡Í¼Ïñ
		IplImage* pSrc = cvLoadImage(vecImageList[n].c_str(), CV_LOAD_IMAGE_COLOR);
		if(pSrc)
		{
			cout<<n<<" ";
			vecImages.push_back(pSrc);
		}
		else
			cout<<vecImageList[n].c_str()<<endl;
	}
	cout<<endl;
	cout<<"number of images "<<vecImages.size()<<endl;

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

	if(pMosaicResult) // save result
	{
		SYSTEMTIME sysTime;    
		GetLocalTime( &sysTime );
		char fileName[200];
		sprintf_s(fileName, "\\mosRes_%04d_%02d_%02d_%02d_%02d_%02d_%03d.png", 
			sysTime.wYear, 
			sysTime.wMonth, 
			sysTime.wDay, // 
			sysTime.wHour, 
			sysTime.wMinute, 
			sysTime.wSecond,  //
			sysTime.wMilliseconds // 
			);
		string strName(fileName);
		string strSavePath = mosaicDirectory + strName;
		cvSaveImage(strSavePath.c_str(), pMosaicResult);
		cvReleaseImage(&pMosaicResult);
	}
	// release momery
	for(int i=0; i<(int)vecImages.size(); i++)
	{
		if(vecImages.empty())
			break;

		cvReleaseImage(&vecImages[i]);
	}
	
	cout<<"mosaicing done!"<<endl;

	return 0;
}

