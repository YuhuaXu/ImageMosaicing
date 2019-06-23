#pragma once
//#include <windows.h>

#ifndef NULL
#define NULL 0 
#endif

namespace pool
{
	struct InterestRegion3D
	{
		float minX, minY, minZ;
		float maxX, maxY, maxZ;
	};

	struct Point3DNormal
	{
		float x, y, z;
		float nx, ny, nz;
	};

	struct Point3D32I
	{
		int x,y,z;
	};

	struct SfPoint
	{
		float x;
		float y;
		//float z;
		int id;
		SfPoint()
		{
		}
		SfPoint(float x, float y)
		{
			this->x = x;
			this->y = y;
		}
		SfPoint(float x, float y, float z)
		{
			this->x = x;
			this->y = y;
			//this->z = z;
		}
	};

	struct CornerDescriptor
	{
		float x;
		float y;
		int id;
		float* pD; // 归一化图像
		int length;
		int valid;
		CornerDescriptor()
		{
			this->pD = NULL;
		}
		static void Release(CornerDescriptor* pD, int num)
		{
			for(int n=0; n<num; n++)
			{
				delete[] pD[n].pD; pD[n].pD;
			}
			delete[] pD;
		}
	};

	struct fPoint
	{
		double x;
		double y;
		int seq; 
		fPoint()
		{
			seq=0;
		}
	};

	struct sfPoint
	{
		float x;
		float y;
		int seq;
		sfPoint()
		{
			seq=0;
		}
	};


	struct DfPoint
	{
		double x;
		double y;
		int id;
	};

	struct Point
	{
		int x; 
		int y; 
		int seq; // 该点的属性
		Point()
		{
			x=0;
			y=0;
			seq=1;
		}
		Point(int x, int y, int seq)
		{
			this->x=x;
			this->y=y;
			this->seq=seq;
		}
	};

	struct ShortPoint
	{
		short x; 
		short y; 
	};

	struct IntPoint
	{
		int x; 
		int y;

		IntPoint()
		{
			x=0;
			y=0;
		}

		IntPoint(int x, int y)
		{
			this->x=x;
			this->y=y;
		}
	};

}