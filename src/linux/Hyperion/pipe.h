#ifndef pipeH
#define pipeH pipeH

#define DEFAULT_ITEMS   256

//-------------------------------------------------------------------------------------
#define _PIPE_INITIALIZE(pipe,items)\
    {\
        if ( ( pipe = (TPipe*)kmalloc( sizeof(TPipe), GFP_KERNEL ) ) != NULL )\
        {\
            pipe->Items = items > 0 ? items : DEFAULT_ITEMS ;\
            pipe->ItemsW = pipe->ItemsR = 0;\
            pipe->sizeBytes = pipe->Items * sizeof(TItem);\
            pipe->Data = (TItem *)kmalloc(pipe->sizeBytes, GFP_KERNEL);\
            memset( (PVOID)pipe->Data, 0, pipe->sizeBytes );\
            pipe->Head = pipe->Data;\
            pipe->Tail = pipe->Data;\
            spin_lock_init( &pipe->lock_w );\
            spin_lock_init( &pipe->lock_r );\
        }\
    }

//-------------------------------------------------------------------------------------
#define _PIPE_DESTROY(pipe)\
    {\
        if ( pipe != NULL )\
        {\
            if ( pipe->Data != NULL )\
                kfree ( pipe->Data );\
            kfree ( pipe ) ;\
            pipe = 0;\
        }\
    }


//-------------------------------------------------------------------------------------
#define _PIPE_RESET(pipe)\
    {\
        pipe->Head = pipe->Data ;\
        pipe->Tail = pipe->Data ;\
        pipe->ItemsW = pipe->ItemsR = 0;\
    }

//-------------------------------------------------------------------------------------
#define _PIPE_WRITE(pipe,item,ok)\
    {\
        if ( pipe->Head == &pipe->Data[pipe->Items] )\
            pipe->Head = pipe->Data ;\
        {\
            ULONG diff = pipe->ItemsW - pipe->ItemsR;\
            if( diff < pipe->Items )\
            {\
                *(pipe->Head)++ = item ;\
                pipe->ItemsW++;\
                ok = 1;\
            }\
            else\
                ok = 0;\
        }\
    }

//-------------------------------------------------------------------------------------
#define _PIPE_WRITE_LOCK(pipe,item,ok)\
    {\
        unsigned long irqflags;\
        spin_lock_irqsave( &pipe->lock_w, irqflags );\
        if ( pipe->Head == &pipe->Data[pipe->Items] )\
            pipe->Head = pipe->Data ;\
        {\
            ULONG diff = pipe->ItemsW - pipe->ItemsR;\
            if( diff < pipe->Items )\
            {\
                *(pipe->Head)++ = item ;\
                pipe->ItemsW++;\
                ok = 1;\
            }\
            else\
                ok = 0;\
        }\
        spin_unlock_irqrestore( &pipe->lock_w, irqflags );\
    }

//-------------------------------------------------------------------------------------
#define _PIPE_READ(pipe,item,ok)\
    {\
        unsigned int diff;\
        if ( pipe->Tail == &pipe->Data[pipe->Items] )\
            pipe->Tail = pipe->Data ;\
        diff = pipe->ItemsW - pipe->ItemsR;\
        if( diff != 0 )\
        {\
            *item = *(pipe->Tail) ;\
            memset( pipe->Tail, 0, sizeof( TItem ) );\
            pipe->Tail++;\
            pipe->ItemsR++;\
            ok = 1;\
        }\
        else\
            ok = 0;\
    }

//-------------------------------------------------------------------------------------
#define _PIPE_READ_LOCK(pipe,item,ok)\
    {\
        unsigned int diff;\
        unsigned long irqflags;\
        spin_lock_irqsave( &pipe->lock_r, irqflags );\
        if ( pipe->Tail == &pipe->Data[pipe->Items] )\
            pipe->Tail = pipe->Data ;\
        diff = pipe->ItemsW - pipe->ItemsR;\
        if( diff != 0 )\
        {\
            *item = *(pipe->Tail) ;\
            memset( pipe->Tail, 0, sizeof( TItem ) );\
            pipe->Tail++;\
            pipe->ItemsR++;\
            ok = 1;\
        }\
        else\
            ok = 0;\
        spin_unlock_irqrestore( &pipe->lock_r, irqflags );\
    }
//-------------------------------------------------------------------------------------
#define _ITEMS_IN_PIPE(pipe) (pipe->ItemsW - pipe->ItemsR)

//-------------------------------------------------------------------------------------
/// \brief item definition
typedef struct SItem TItem;

//-------------------------------------------------------------------------------------
/// \brief struct SPipe includes all elements for handling a wraparound-buffer
typedef struct SPipe
//-------------------------------------------------------------------------------------
{
    unsigned int Items, sizeBytes, ItemsW, ItemsR;
    TItem*   Data ;
    TItem*   Head, *Tail ;
    spinlock_t lock_w, lock_r;
} TPipe;

#endif //pipeH
