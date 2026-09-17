#ifndef __THREAD_H  
#define __THREAD_H
#include "common.h"
using namespace std;  

typedef void (*FunType)(void*agr ); 


class CTask  
{  
protected:  
    string m_strTaskName;
	void* m_ptrData;
	FunType Taskfunc;
	bool runStatus;
public:  
	CTask();
	virtual int Run()= 0;
    CTask(string taskName) ;    
	void SetData(void* data);
	void SetFunc(const FunType pf);
	bool Status(){return runStatus;}
  
public:  
	virtual ~CTask(){}  
};  

class CMyTask: public CTask  
{  
public:  
	CMyTask(){}
public:
	inline int Run()  
	{  
		runStatus=false;
		Taskfunc(this->m_ptrData);
		runStatus=true; 
		return 0;  
	}  
}; 
  
#ifndef CTHREADPOOL
#define CTHREADPOOL

class CThreadPool  
{  
private:
    static  vector<CTask*> m_vecTaskList;
    static  bool bShutDown;
	pthread_t   *pthread_id;  
	int m_iThreadNum;
	static int* ptDone;
	
      
    static pthread_mutex_t m_pthreadMutex;
    static pthread_cond_t m_pthreadCond;
protected:  
    static void* ThreadFunc(void * threadData);
    static int MoveToIdle(pthread_t tid);
    static int MoveToBusy(pthread_t tid);
      
    int CreateThread();
  
public:   
	static int TaskNum;
	
    CThreadPool(int threadNum=1);
    int AddTask(CTask *task);
    int StopAll();
    int GetTaskSize();
    bool ShutDownAllThread();
	bool WaitAllTastDone(int tasknum);
};  

#endif
#endif  