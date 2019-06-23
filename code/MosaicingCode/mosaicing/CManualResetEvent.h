#pragma once
#include <windows.h>
//#include "afxwin.h"

class CManualResetEvent
{
private:
	HANDLE hevent;

public:
	CManualResetEvent(void);
	CManualResetEvent(LPTSTR mutexName);
	~CManualResetEvent(void);

	BOOL Reset();
	BOOL Set(); 
	BOOL Wait();
	BOOL Wait(int timeout);
	BOOL Wait(int timeout,BOOL autoReset);
};
