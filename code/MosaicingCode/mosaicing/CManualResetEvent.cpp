//#include "StdAfx.h"
#include "CManualResetEvent.h"
#include <winbase.h>

CManualResetEvent::CManualResetEvent()
{
	hevent = CreateEvent(NULL,FALSE,FALSE,NULL);
}

BOOL CManualResetEvent::Wait(int timeout)
{
	return Wait(timeout,true);
}

BOOL CManualResetEvent::Wait(int timeout,BOOL autoReset)
{
	DWORD ret;
	if(timeout <= 0)
	{
		ret = WaitForSingleObject(hevent,INFINITE);
	}
	else
	{
		ret = WaitForSingleObject(hevent,timeout);
	}
	//if(autoReset)
	//{
	//	Reset();
	//}
	return ret == 0;
}

BOOL CManualResetEvent::Wait()
{
	return Wait(0,false);//,true);
}

BOOL CManualResetEvent::Set()
{
	return SetEvent(hevent);
}


BOOL CManualResetEvent::Reset()
{
	return ResetEvent(hevent);
}

CManualResetEvent::~CManualResetEvent(void)
{
	CloseHandle(hevent);
	hevent = NULL;
}
