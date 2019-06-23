
#pragma once


#include <windows.h>


#ifdef WIN32
	// 定义锁变量类型
	#define  MUTEX CRITICAL_SECTION
	// 定义初始化锁的功能
	#define MUTEXINIT(m) InitializeCriticalSection(m)
	// 定义加锁
	#define MUTEXLOCK(m) EnterCriticalSection(m)
	// 定义解锁
	#define MUTEXUNLOCK(m) LeaveCriticalSection(m)
	// 定义摧毁锁变量操作
	#define MUTEXDESTROY(m) DeleteCriticalSection(m)
#endif

class CMutexLock
{
public:
	// 初始化锁
	CMutexLock(void) 
	{ 
		MUTEXINIT(&m_Lock); 
	}
	// 摧毁锁
	~CMutexLock(void) 
	{ 
		MUTEXDESTROY(&m_Lock); 
	}

public:
	// 加锁
	void Lock() 
	{ 
		MUTEXLOCK(&m_Lock); 
	}
	// 解锁
	void Unlock() 
	{ 
		MUTEXUNLOCK(&m_Lock); 
	}

private:

	MUTEX m_Lock;
};