#ifndef _ksemaphore_h
#define _ksemaphore_h


#ifdef __cplusplus
extern "C" {
#endif


#define INFINITE     0xFFFFFFFF  // Infinite timeout
enum
{
    ObjSignaled,
    ObjTimeout,
    ObjError,
    ObjMax
};

typedef struct SKSemaphore TKSemaphore;

extern TKSemaphore* create_sema( int inicount, int limit );
extern int destroy_sema( TKSemaphore* sem );
extern int wait_sema( TKSemaphore* sem, int timeoutmsec );
extern int signal_sema( TKSemaphore* sem, int inc );
extern void reset_sema( TKSemaphore* sem );


#ifdef __cplusplus
}
#endif

#endif
