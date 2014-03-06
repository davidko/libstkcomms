/*
   Copyright 2013 Barobo, Inc.

   This file is part of BaroboLink.

   BaroboLink is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   BaroboLink is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with BaroboLink.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _THREAD_MACROS_H_
#define _THREAD_MACROS_H_

/* * * * * * * * * * * * * * * */
/* W I N D O W S   M A C R O S */
/* * * * * * * * * * * * * * * */
#if defined (_WIN32) ||  defined (MSYS)
/* ******* *
 * THREADS *
 * ******* */
#ifndef THREAD_T
#define THREAD_T HANDLE
#endif

#define THREAD_CREATE(thread_handle, function, arg) \
  *(thread_handle) = CreateThread( \
      NULL, \
      0, \
      (LPTHREAD_START_ROUTINE)function, \
      arg, \
      0, \
      NULL \
      )

#define THREAD_CANCEL(thread_handle)  \
  TerminateThread( thread_handle, 0)

#define THREAD_JOIN(thread_handle) \
  WaitForSingleObject(thread_handle, INFINITE)

#define THREAD_EXIT() \
  ExitThread(0)

#define THREAD_DETACH() 

#define THREAD_YIELD() SwitchToThread()

/* ***** */
/* MUTEX */
/* ***** */
/* Typedef */
#define MUTEX_T HANDLE
/* Init */
#define MUTEX_INIT(mutex) \
  *mutex = CreateMutex(NULL, FALSE, NULL)
/* Destroy */
#define MUTEX_DESTROY(mutex)
/* Functions */
#define MUTEX_LOCK(mutex)           \
  WaitForSingleObject(            \
      *mutex ,                 \
      INFINITE)
#define MUTEX_UNLOCK(mutex)         \
  ReleaseMutex( *mutex )
#define MUTEX_NEW(mutex) \
  mutex = (HANDLE*)malloc(sizeof(HANDLE)); \
  if(mutex == NULL) \
    fprintf(stderr, "Memory Error. %s:%d\n", __FILE__, __LINE__)


/* **** *
 * COND *
 * **** */
/* Typedef */
#define COND_T HANDLE
/* New */
#define COND_NEW(cond) \
  cond = (HANDLE*)malloc(sizeof(HANDLE)); \
  if(cond == NULL) \
    fprintf(stderr, "Memory Error. %s:%d\n", __FILE__, __LINE__)
/* Init */
#define COND_INIT(cond) \
  *cond = CreateEvent(NULL, TRUE, TRUE, NULL);\
  ResetEvent(*cond)
/* Destroy */
#define COND_DESTROY(cond)
/* Functions */

#define COND_WAIT( cond , mutex ) \
ResetEvent(*cond); \
ReleaseMutex(*mutex); \
WaitForSingleObject( *cond, INFINITE)
#define COND_SLEEP( cond, mutex, test )   \
  ResetEvent( *cond );             \
if (!test){ \
  WaitForSingleObject( *cond, INFINITE); \
}
#define COND_RESET( cond, mutex ) \
	ResetEvent(*cond)
#define COND_SLEEP_ACTION(cond, mutex, action) \
  ResetEvent( *cond ); \
action; \
WaitForSingleObject( *cond, INFINITE)
#define SIGNAL(cond, mutex, action) \
  action; \
SetEvent( *cond )
#define COND_BROADCAST(cond) \
  PulseEvent(*cond)
#define COND_SIGNAL(cond) \
  SetEvent(*cond)


/* ********* *
 * SEMAPHORE *
 * ********* */
/* Typedef */
#define SEMAPHORE_T HANDLE
/* Init */
#define SEMAPHORE_INIT(sem) \
  *sem = CreateSemaphore( \
      NULL, \
      0, \
      1024, \
      NULL ) 
/* Destroy */
#define SEMAPHORE_DESTROY(sem) 
/* Functions */
#define SEMAPHORE_WAIT(sem) \
  WaitForSingleObject(sem, INFINITE)
#define SEMAPHORE_POST(sem) \
  ReleaseSemaphore(sem, 1, NULL)


/* ******* *
 * RW_LOCK *
 * ******* */
/* Typedef */
#define RWLOCK_T mc_rwlock_t
/* Init */
#define RWLOCK_INIT(rwlock) \
  mc_rwlock_init(rwlock)
/* Destroy */
#define RWLOCK_DESTROY(rwlock) mc_rwlock_destroy(rwlock)
/* Functions */
#define RWLOCK_RDLOCK(rwlock) \
  mc_rwlock_rdlock(rwlock)
#define RWLOCK_RDUNLOCK(rwlock) \
  mc_rwlock_rdunlock(rwlock)
#define RWLOCK_WRLOCK(rwlock) \
  mc_rwlock_wrlock(rwlock)
#define RWLOCK_WRUNLOCK(rwlock) \
  mc_rwlock_wrunlock(rwlock)



/* * * * * * * * * * * * */
/* U N I X   M A C R O S */
/* * * * * * * * * * * * */
#else
#include <pthread.h>
#define THREAD_T pthread_t
#define THREAD_CREATE( thread_handle, function, arg ) \
  while(pthread_create( \
      thread_handle, \
      NULL, \
      function, \
      (void*) arg \
      ) < 0) { \
			printf("pthread_create failed. Trying again...\n"); \
	}

#define THREAD_CANCEL( thread_handle ) \
  pthread_cancel( thread_handle )

#define THREAD_JOIN(thread_handle ) \
  pthread_join( thread_handle, NULL )

#define THREAD_DETACH(thread_handle) \
	if(pthread_detach(thread_handle) < 0) { \
		printf("pthread_detach failed. %s:%d\n", __FILE__, __LINE__); \
	}

#define THREAD_EXIT() \
  pthread_exit(NULL)

#define THREAD_YIELD() pthread_yield()

/* ***** */
/* MUTEX */
/* ***** */

/* Typedef */
#define MUTEX_T pthread_mutex_t
/* Init */
#define MUTEX_INIT(mutex) \
  pthread_mutex_init(mutex, NULL)
/* Destroy */
#define MUTEX_DESTROY(mutex) \
  pthread_mutex_destroy(mutex)
/* functions */
#define MUTEX_LOCK(mutex)                                                   \
  if (pthread_mutex_lock( mutex ))                                        \
fprintf(stderr, "pthread lock error: %s:%d\n", __FILE__, __LINE__)
#define MUTEX_UNLOCK(mutex) \
  pthread_mutex_unlock( mutex )
#define MUTEX_NEW(mutex) \
  mutex = (pthread_mutex_t*)malloc(sizeof(pthread_mutex_t)); \
  if (mutex == NULL)  \
    fprintf(stderr, "Memory Error. %s:%d\n", __FILE__,__LINE__); \

/* **** *
 * COND *
 * **** */
/* Typedef */
#define COND_T pthread_cond_t
/* Init */
#define COND_INIT(cond) \
  pthread_cond_init(cond, NULL)
/* New */
#define COND_NEW(cond) \
  cond = (pthread_cond_t*)malloc(sizeof(pthread_cond_t)); \
  if (cond == NULL)  \
    fprintf(stderr, "Memory Error. %s:%d\n", __FILE__,__LINE__); \
/* Destroy */
#define COND_DESTROY(cond) \
  pthread_cond_destroy(cond)
/* functions */
#define COND_WAIT( cond , mutex ) \
  pthread_cond_wait(cond, mutex )
/* Wait until 'test' is true */
#define COND_SLEEP( cond, mutex, test )       \
  if (pthread_mutex_lock( mutex ))    \
printf("pthread lock error: %s:%d\n", __FILE__, __LINE__); \
if (!test) { \
  pthread_cond_wait( cond, mutex ); \
} 
#define COND_RESET( cond, mutex ) \
  pthread_mutex_unlock( mutex );
#define COND_SLEEP_ACTION(cond, mutex, action) \
  if (pthread_mutex_lock( mutex )) \
printf("pthread lock error: %s:%d\n", __FILE__, __LINE__); \
action; \
pthread_cond_wait( cond, mutex ); 
#define CONDSIGNAL(cond, mutex, action) \
  pthread_mutex_lock( mutex ); \
action; \
pthread_cond_signal( cond ); \
pthread_mutex_unlock( mutex )
#define COND_BROADCAST(cond) \
  pthread_cond_broadcast( cond )
#define COND_SIGNAL(cond) \
  pthread_cond_signal( cond )

/* ********* *
 * SEMAPHORE *
 * ********* */
/* Typedef */
#define SEMAPHORE_T sem_t
/* Init */
#define SEMAPHORE_INIT(sem) \
  sem_init(sem, 0, 0)
/* Destroy */
#define SEMAPHORE_DESTROY(sem) \
  sem_destroy(sem)
/* Functions */
#define SEMAPHORE_WAIT(sem) \
  sem_wait(sem)
#define SEMAPHORE_POST(sem) \
  sem_post(sem)

/* ******* *
 * RW_LOCK *
 * ******* */
/* Typedef */
#ifdef HAVE_PTHREAD_RWLOCK_T
#define RWLOCK_T pthread_rwlock_t
#else
#define RWLOCK_T mc_rwlock_t
#endif
/* Init */
#ifdef HAVE_PTHREAD_RWLOCK_T
#define RWLOCK_INIT(rwlock) \
  pthread_rwlock_init(rwlock, NULL)
#else
#define RWLOCK_INIT(rwlock) \
  mc_rwlock_init(rwlock)
#endif
/* Destroy */
#ifdef HAVE_PTHREAD_RWLOCK_T
#define RWLOCK_DESTROY(rwlock) \
  pthread_rwlock_destroy(rwlock)
#else
#define RWLOCK_DESTROY(rwlock) \
  mc_rwlock_destroy(rwlock)
#endif
/* Functions */
#ifdef HAVE_PTHREAD_RWLOCK_T
#define RWLOCK_RDLOCK(rwlock) \
  if (pthread_rwlock_rdlock(rwlock)) \
fprintf(stderr, "rwlock error: %s:%d\n", __FILE__, __LINE__)
#define RWLOCK_RDUNLOCK(rwlock) \
  if (pthread_rwlock_unlock(rwlock)) \
fprintf(stderr, "rwunlock error: %s:%d\n", __FILE__, __LINE__)
#define RWLOCK_WRLOCK(rwlock) \
  if (pthread_rwlock_wrlock(rwlock)) \
fprintf(stderr, "rwlock error: %s:%d\n", __FILE__, __LINE__)
#define RWLOCK_WRUNLOCK(rwlock) \
  if (pthread_rwlock_unlock(rwlock)) \
fprintf(stderr, "rwunlock error: %s:%d\n", __FILE__, __LINE__)
#else
#define RWLOCK_RDLOCK(rwlock) \
  mc_rwlock_rdlock(rwlock)
#define RWLOCK_RDUNLOCK(rwlock) \
  mc_rwlock_rdunlock(rwlock)
#define RWLOCK_WRLOCK(rwlock) \
  mc_rwlock_wrlock(rwlock)
#define RWLOCK_WRUNLOCK(rwlock) \
  mc_rwlock_wrunlock(rwlock)
#endif

#endif /* Unix/Windows macros */

#endif
