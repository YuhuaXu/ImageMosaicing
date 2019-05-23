#include "opencv2\opencv.hpp"
#include "mosaicing.h"

void main()
{
	IplImage* src = cvLoadImage("d:/xu.png");

	cvShowImage("xu", src);
	cvWaitKey(-1);

	cvReleaseImage(&src);

	int res = Add(3, 4);

	printf("res %d \n", res);
}