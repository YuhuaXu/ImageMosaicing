// "MemoryPool.h"
// 内存池
// 非多线程安全(2011-01-16)
#pragma once

//#include "afxwin.h"

#include <vector>
#include "Lock.h"

//#pragma warning(push)
#pragma warning(disable:4251) // your declarations that cause 4251
//#pragma warning(pop)


#if defined (_WINDLL) || defined(_CONSOLE) || defined(_WINDOWS)
	#ifndef MV_MODULE_TYPE
		#define MV_MODULE_TYPE _declspec(dllexport)
	#else
		#define MV_MODULE_TYPE _declspec(dllimport)
	#endif
#else
	#define MV_MODULE_TYPE 
#endif

#ifndef _MemoryPool
	#if _DEBUG
		//#pragma comment(lib,"memoryPoold.lib")
	#else
		//#pragma comment(lib,"memoryPool.lib")
	#endif
#endif

typedef unsigned long ulong;

typedef unsigned short ushort;

struct MV_MODULE_TYPE MemoryBlockHeader
{
	ulong m_blockLevel;
	MemoryBlockHeader* m_pNext;
};

// 内存登记
struct MemoryRegistrator
{
	int id;
	void* pAddress;
};

// ***************************************************
// 内存池类
// ***************************************************
class MV_MODULE_TYPE CMemoryPool
{
public:

	CMemoryPool();

	~CMemoryPool();

public:
	// 初始化内存池
	void InitialPool();

	// 内存操作
	void* Operate(void* pDataBlock, int nSize=0);

	// 申请内存
	void* NewIn(int nSize); // 应用程序申请的内存块尺寸

	// 释放内存
	bool DeleteIn(void* pDataBlock );
	
	// 销毁所有的内存块
	static bool Destroy();

private:
	inline void WritePoolHead(ulong nLevel, MemoryBlockHeader* pHead);

	ulong CalValidBlockSize(ulong userNeedSize, ulong &nLevel);

	// 计算合适的内存块尺寸
	unsigned long CalValidBlockSizeTab(unsigned long userNeedSize, ulong &nLevel);

	inline ulong Pow2(ulong n);

	ulong Pow2Inv(ulong val);
#ifdef _DEBUG
	void RegNew(void*);

	void RegDelete(void*);

	static void PrintLeakInfo();
#endif

private:
	 //CMultiReadSingleWriteLock m_Lock;
	CMutexLock m_lock;

	// 树枝数
	static int m_branchNumber;

	// 声明
	static std::vector<MemoryBlockHeader*> m_internalPool;
};




