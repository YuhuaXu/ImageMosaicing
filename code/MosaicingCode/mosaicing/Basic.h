#pragma once

namespace pool
{

struct InterestRegion2
{
	int xbeg; // 起点x
	int xend; // 终点x
	int ybeg; // 起点y
	int yend; // 终点y
	int valid;

	InterestRegion2()
	{
		xbeg=0;
		xend=0;
		ybeg=0;
		yend=0;
		valid = 0;
	}

	InterestRegion2(int xbeg, int ybeg, int xend, int yend)
	{
		this->xbeg = xbeg;
		this->xend = xend;
		this->ybeg = ybeg;
		this->yend = yend;
	}
};

struct MotionMatrix44
{
	float M[16];

	void eye()
	{
		for(int i=0; i<16; i++)
		{
			M[i] = 0;
		}
		M[0] = M[5] = M[10] = M[15] = 1;
	}
};

}

struct Plane64F
{
	double A,B,C,D;
};

struct Plane32F
{
	float A,B,C,D;
	Plane32F()
	{}
	Plane32F(float A, float B, float C, float D)
	{
		this->A = A;
		this->B = B;
		this->C = C;
		this->D = D;
	}
};