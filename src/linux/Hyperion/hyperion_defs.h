#ifndef hyperiondefsH
#define hyperiondefsH hyperiondefsH

//-------------------------------------------------------------------------------------------
/// \brief item definition
struct SItem
//-------------------------------------------------------------------------------------------
{
    struct dma_transfer_object* dto; ///< pointer to dma_transfer_object
    //UART_OBJECT* pua;
    void* pua;
    unsigned long int_stat; ///< current interruptstatus infos of dma_controller
    unsigned long int_src; ///< which interrupt src was handled
    unsigned char dma_error;
    DMA_CONTROLLER* controller;
    POCL_OBJECT* ppo;
};

typedef struct _physical_address
{
    unsigned long low_addr;
    unsigned long high_addr;
} physical_address_t;

#endif //hyperiondefsH
