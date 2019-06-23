#include "MemoryPool.h"&
#include "math.h"
#include <iostream>

// 定义
std::vector<MemoryBlockHeader*> CMemoryPool::m_internalPool;
int CMemoryPool::m_branchNumber = 0;

static int s_DeleteNum = 0;

static int s_NewNum = 0;
								// 0  1   2   3     4    5
const ulong aBlockSize[100] = {16, 32, 64, 128, 256, 512, 
			// 6    7       8       9       10      11        12          13     14         15
			1024, 1024*2, 1024*4, 1024*8, 1024*16, 1024*32, 1024*64, 1024*128, 1024*256, 1024*512, 
			//  16       17            18           19            20            21
			1024*1024, 1024*1024*2, 1024*1024*4, 1024*1024*8, 1024*1024*16, 1024*1024*32 };

// 许可检查
int MV_MODULE_TYPE PermissionCheck()
{
	// 在线获取相机序列号
	//GetCameraSerialNumbersOnline(vector<string> &serialNumbers);

	// 获取授权序列号

	// 匹配

	return 1;
}
const ulong SMemoryBlockHeadSize = sizeof(MemoryBlockHeader);

// (5)最小内存块长度，16bytes - 8bytes, 
const unsigned long MIN_MEMORY_STACK_BLOCK_USER =  16 - SMemoryBlockHeadSize;

// (5)最小内存块长度，16bytes, 
const unsigned long MIN_MEMORY_STACK_BLOCK =  16;

// (6)内存块尺寸的最大等级
const ulong MAX_BLOCK_SIZE_LEVEL = 19;

// (7)这是管理的最大内存块长度,超过此限制，内存池停止服务, 改为直接向系统申请和释放
const unsigned long MAX_MEMORY_STACK_BLOCK_SIZE  = aBlockSize[MAX_BLOCK_SIZE_LEVEL];

const float fInvlog2 = 1/log(2.0f);

#ifdef _DEBUG
#endif
// 内存注册管理, 跟踪内存泄漏
static std::vector<MemoryRegistrator> s_regVector;


// 内存池构造
CMemoryPool::CMemoryPool()
{
	//CWinThread *pPrsThread = ::AfxBeginThread(MosaicImagesThread, this);
}

// 初始化内存池
void CMemoryPool::InitialPool()
{
	if((int)m_internalPool.size()==0)
	{	
		ulong currentBlockSize = MIN_MEMORY_STACK_BLOCK;
		//// 加锁
		//m_lock.Lock();
		ulong MAX_MEMORY_STACK_BLOCK_SIZE2  = aBlockSize[MAX_BLOCK_SIZE_LEVEL];
		while(currentBlockSize<=MAX_MEMORY_STACK_BLOCK_SIZE2)
		{
			MemoryBlockHeader* pHead = NULL;

			m_internalPool.push_back(pHead);
			currentBlockSize *= 2;

			m_branchNumber++;
		}	
		
		//// 解锁
		//m_lock.Unlock();
	}
}

CMemoryPool::~CMemoryPool()
{
	
}

#ifdef _DEBUG
// 注册分配
void CMemoryPool::RegNew(void* pAddress)
{
	//MemoryRegistrator regTemp;
	//regTemp.id = s_NewNum;
	//regTemp.pAddress = pAddress;
	//s_regVector.push_back(regTemp);
}
// 注册释放
void CMemoryPool::RegDelete(void* pAddress)
{
	//for(int i=0; i<(int)s_regVector.size(); i++)
	//{
	//	if(s_regVector[i].pAddress==pAddress)
	//	{
	//		s_regVector[i] = s_regVector[s_regVector.size()-1];
	//		s_regVector.pop_back();		
	//	}
	//}
}


// 打印泄漏信息
void CMemoryPool::PrintLeakInfo()
{
	for(int i=0; i<(int)s_regVector.size(); i++)
	{
		std::cout<<"Leak id "<<s_regVector[i].id<<" address "<<s_regVector[i].pAddress<<std::endl;
	}
}
#endif

// 计算2的n次方的值
unsigned long CMemoryPool::Pow2(unsigned long n)
{
	return 1<<n;
}

// 根据2的n次方的值，计算n
unsigned long CMemoryPool::Pow2Inv(unsigned long val)
{
	unsigned long n = 0;
	unsigned long valTemp = val;
	while(valTemp>1)
	{
		valTemp /= 2;
		n++;
	}
	return n;
}

// 计算合适的内存块尺寸
unsigned long CMemoryPool::CalValidBlockSize(unsigned long userNeedSize, ulong &nLevel)
{
	if(userNeedSize<=MIN_MEMORY_STACK_BLOCK_USER)
	{
		nLevel = 0; 
		return MIN_MEMORY_STACK_BLOCK;
	}
	
	float fN = log(userNeedSize + 8.0f)*fInvlog2 - 4;

	if( (fN-ulong(fN))==0)
	{
		nLevel = (ulong)fN;
	}
	else
	{
		nLevel = (ulong)fN + 1;
	}
	
	//return 16*Pow2(nLevel);
	return 1<<(nLevel+4);
}

// 计算合适的内存块尺寸
unsigned long CMemoryPool::CalValidBlockSizeTab(unsigned long userNeedSize, ulong &nLevel)
{
	// 折半查找
	if(userNeedSize<=aBlockSize[10])
	{
		for(int i=0; i<=10; i++)
		{
			if(userNeedSize<=aBlockSize[i])
			{
				nLevel = i;
				return aBlockSize[i];
			}
		}
	}
	else
	{
		for(int i=11; i<=19; i++)
		{
			if(userNeedSize<=aBlockSize[i])
			{
				nLevel = i;
				return aBlockSize[i];
			}
		}
	}

	return 0;
}

// 写内存池"头"
inline void CMemoryPool::WritePoolHead(ulong nLevel, MemoryBlockHeader* pHead)
{
	// 加锁
	m_lock.Lock();
	
	m_internalPool[nLevel] = pHead;
	
	// 解锁
	m_lock.Unlock();
}

// *****************************************************************
// 功能：申请内存
// nSize------------用户申请的内存块的尺寸, 单位为byte
// *****************************************************************
void* CMemoryPool::NewIn(int nSize) // 应用程序申请的内存块尺寸
{
	if(nSize<=0)
		return NULL;

	// 如果内存池为空，则做初始化
	if(m_internalPool.empty())
		InitialPool();

	// 计算合适的内存块的大小，16, 32, 64, 128, 256, 512，......
	ulong nLevel = 0;
	ulong blockSize = CalValidBlockSize(nSize, nLevel);
	
#ifdef _DEBUG
	//std::cout<<"s_NewNum "<<s_NewNum<<" blockSize "<<blockSize<<std::endl;
	s_NewNum++;
#endif
	
	// 如果大于定义的最大的内存块的尺寸，则由系统直接分配，不由内存池来管理
	if(blockSize<=MAX_MEMORY_STACK_BLOCK_SIZE)
	{
		// 查找内存池中是否有该量级的剩余内存
		if(m_internalPool[nLevel]!=NULL)
		{
			// 如果有剩余的内存，则从内存池中分配
			// 拆链
			void* pBlock = (char*)m_internalPool[nLevel] + SMemoryBlockHeadSize;
			//m_sVecPool[nLevel] = m_sVecPool[nLevel]->m_pNext; // "写"
			WritePoolHead( nLevel, m_internalPool[nLevel]->m_pNext);
#ifdef _DEBUG
			RegNew(pBlock);
#endif
			return pBlock;
		}
		else
		{
			// 如果没有剩余的内存，则从OS申请新的内存
			char* pBlock = new char[blockSize]; // 从OS申请内存
			MemoryBlockHeader* pHead = (MemoryBlockHeader*)(pBlock);
			pHead->m_blockLevel = nLevel;
			pHead->m_pNext = NULL;
#ifdef _DEBUG
			RegNew(pBlock + SMemoryBlockHeadSize);
#endif
			return pBlock + SMemoryBlockHeadSize;
		}
	}
	else
	{
		// 如果大于定义的最大的内存块的尺寸，则由系统直接分配，不由内存池来管理
		char* pBlock = new char[nSize+SMemoryBlockHeadSize]; // 从OS申请内存
		MemoryBlockHeader* pHead = (MemoryBlockHeader*)(pBlock);
		pHead->m_blockLevel = nLevel;
		pHead->m_pNext = NULL;
#ifdef _DEBUG
		RegNew(pBlock + SMemoryBlockHeadSize);
#endif
		return pBlock + SMemoryBlockHeadSize;
	}

	return NULL;
}

// *****************************************************************
// 释放内存
// *****************************************************************
bool CMemoryPool::DeleteIn(void* pDataBlock )
{
	if(pDataBlock==NULL)
		return true;

#ifdef _DEBUG
	RegDelete(pDataBlock);
	//std::cout<<"s_DeleteNum "<<s_DeleteNum<<std::endl;
	s_DeleteNum++;
#endif

	MemoryBlockHeader* pHead = (MemoryBlockHeader*)((char*)(pDataBlock) - SMemoryBlockHeadSize);

	// 计算该内存块大小的量级
	ulong nLevel = pHead->m_blockLevel;

	// 如果该内存块的尺寸比内存池管理的最大尺寸要小，则挂到相应的量级的树枝上去
	if(pHead->m_blockLevel<=MAX_BLOCK_SIZE_LEVEL)
	{
		// "挂链"
		pHead->m_pNext = m_internalPool[nLevel]; 
		//m_sVecPool[nLevel] = pHead; // "写"
		WritePoolHead( nLevel, pHead);
	}
	// 如果该内存块的尺寸比内存池管理的最大尺寸要大，则直接释放
	else
	{
		delete[] (char*)pHead; pHead = NULL;
	}

	return true;
}


// 销毁内存池
bool CMemoryPool::Destroy()
{
	//// 加锁
	//CMutexLock lock;
	//lock.Lock();

#ifdef _DEBUG
	PrintLeakInfo();
#endif

	for(int i=0; i<m_branchNumber; i++)
	{
		while(m_internalPool[i]!=NULL)
		{
			MemoryBlockHeader* pBlock = m_internalPool[i]->m_pNext;
			delete[] (char*)m_internalPool[i]; m_internalPool[i] = NULL;
			m_internalPool[i] = pBlock;
		}
	}

	//// 解锁
	//lock.Unlock();

	return true;
}

void* CMemoryPool::Operate( void* pDataBlock, int nSize/*=0*/ )
{
	CMutexLock lock;
	lock.Lock();

	if(nSize>0) // 申请内存
	{
		return NewIn(nSize);
	}
	else if(pDataBlock) // 释放内存
	{
		DeleteIn(pDataBlock);
	}

	// 解锁
	lock.Unlock();
	return NULL;
}