#ifndef hyperionregisterH
#define hyperionregisterH hyperionregisterH

#include "i2c_register.h"
#include "HyperionVendorDeviceID.h"
//------------------------------------------------------------------------------------
// DeviceID
//-------------------------------------------------------------------------------------
#ifndef PCI_DEVICE_ID_HYPERION
#define PCI_DEVICE_ID_HYPERION_CLE PCI_DEVICE_ID_HYPERION_CL_1X
#define PCI_DEVICE_ID_HYPERION_CL4E PCI_DEVICE_ID_HYPERION_CL_4X
#endif

//------------------------------------------------------------------------------------
// offset
//-------------------------------------------------------------------------------------
#ifndef BIT
#define BIT(n) (1<<n)
#endif
#define VERSION_ADDR_SPACE_64 BIT(15)

//------------------------------------------------------------------------------------
// offset 32bit Avalon System
//-------------------------------------------------------------------------------------
#define A32_NII51006                0x0
#define A32_SYSTEM_REGISTER         0x40
#define A32_IIC_READ                0x20
#define A32_ASMI_INTERFACE          0x60
#define A32_VIDEO_IN_DMA_0          0x100
#define A32_VIDEO_IN_DMA_1          0x200
#define A32_DDR_PCI_DMA_0           0x300
#define A32_DDR_PCI_DMA_1           0x400
#define A32_POCL_CONTROL_0          0x80
#define A32_POCL_CONTROL_1          0xA0
#define A32_UART_0                  0x1F0
#define A32_UART_1                  0x2F0
#define A32_CL_CONTROLLER_0         0x8000
#define A32_CL_CONTROLLER_1         0xc000
#define A32_HRT_CONTROLLER_0        0x10000
#define A32_HRT_CONTROLLER_1        0x20000
#define A32_TRIGGER_HRT_CONTROLLER_0    0x18000
#define A32_TRIGGER_HRT_CONTROLLER_1    0x28000
//------------------------------------------------------------------------------------
// offset 64bit Avalon System
//-------------------------------------------------------------------------------------
#define A64_NII51006                0x0
#define A64_SYSTEM_REGISTER         0x40
#define A64_IIC_READ                0x20
#define A64_ASMI_INTERFACE          0x60
#define A64_VIDEO_IN_DMA_0          0x100
#define A64_VIDEO_IN_DMA_1          0x200
#define A64_DDR_PCI_DMA_0           0x300
#define A64_DDR_PCI_DMA_1           0x400
#define A64_POCL_CONTROL_0          0x80
#define A64_POCL_CONTROL_1          0xA0
#define A64_UART_0                  0x800
#define A64_UART_1                  0x900
#define A64_CL_CONTROLLER_0         0x8000
#define A64_CL_CONTROLLER_1         0x10000
#define A64_HRT_CONTROLLER_0        0x10000
#define A64_HRT_CONTROLLER_1        0x20000
#define A64_TRIGGER_HRT_CONTROLLER_0    0x18000
#define A64_TRIGGER_HRT_CONTROLLER_1    0x28000
//------------------------------------------------------------------------------------
// offset 64bit Arria System
//-------------------------------------------------------------------------------------
#define CL4E_NII51006               0x0
#define CL4E_SYSTEM_REGISTER        0x0
#define CL4E_IIC_READ               0x100
#define CL4E_ASMI_INTERFACE         0x50400
#if NIOS_SUPPORTED_TRANSFER
#define CL4E_VIDEO_IN_DMA_0         0x2008400
#define CL4E_VIDEO_IN_DMA_1         0x2008600
#else
#define CL4E_VIDEO_IN_DMA_0         0x400
#define CL4E_VIDEO_IN_DMA_1         0x600
#endif
#define CL4E_DDR_PCI_DMA_0          0x700
#define CL4E_DDR_PCI_DMA_1          0x800
#define CL4E_POCL_CONTROL_0         0x200
#define CL4E_POCL_CONTROL_1         0x300
#define CL4E_UART_0                 0x900
#define CL4E_UART_1                 0xA00
#define CL4E_CL_CONTROLLER_0        0x10000
#define CL4E_CL_CONTROLLER_1        0x20000
#define CL4E_HRT_CONTROLLER_0       0x30000
#define CL4E_HRT_CONTROLLER_1       0x40000
#if NIOS_SUPPORTED_TRANSFER
#define CL4E_TRIGGER_HRT_CONTROLLER_0   0x2038000
#define CL4E_TRIGGER_HRT_CONTROLLER_1   0x2048000
#else
#define CL4E_TRIGGER_HRT_CONTROLLER_0   0x38000
#define CL4E_TRIGGER_HRT_CONTROLLER_1   0x48000
#endif
//------------------------------------------------------------------------------------
// offset 64bit Arria System
//-------------------------------------------------------------------------------------
#define HDSDI_4E_SYSTEM_REGISTER        0x0
#define HDSDI_4E_IIC_READ               0x100
#define HDSDI_4E_NICE_UART              0x900
#define HDSDI_4E_SPI_SIMPLE             0xA00
#define HDSDI_4E_DEC_CTRL_0             0x200
#define HDSDI_4E_DEC_CTRL_1             0x300
#define HDSDI_4E_CL_CONTROLLER_0        0x10000
#define HDSDI_4E_CL_CONTROLLER_1        0x20000
#define HDSDI_4E_HRT_CONTROLLER_0           0x30000
#define HDSDI_4E_HRT_CONTROLLER_1           0x40000
#define HDSDI_4E_TRIGGER_HRT_CONTROLLER_0   0x2038000
#define HDSDI_4E_TRIGGER_HRT_CONTROLLER_1   0x2048000
#define HDSDI_4E_EPCS_CONTROLLER        0x50000
#define HDSDI_4E_VIDEO_IN_DMA_0         0x2008400
#define HDSDI_4E_VIDEO_IN_DMA_1         0x2008600
//------------------------------------------------------------------------------------
// relative register offset definition
//-------------------------------------------------------------------------------------
#define OFF_INTERRUPT_STATUS        0x0
#define OFF_SYSTEM_CONTROL          0x4
#define OFF_SYSTEM_VERSION          0xC
#define OFF_VIDEO_IN_CONTROL        0x0
#define OFF_VIDEO_IN_CTRL_VERSION       0x4
#define OFF_VIDEO_IN_DIGIO_CONTROL      0x8
#define OFF_VIDEO_IN_DEBUG_SIGNALS      0x28
#define OFF_VIDEO_IN_CTRL_STAT          0x40
#define OFF_VIDEO_IN_INTR_ENABLE        0x44
#define OFF_VIDEO_IN_INTR_STAT_CLEAR    0x48
#define OFF_VIDEO_IN_DMA_CMD_STAT       0x50
#define OFF_VIDEO_IN_DMA_XFER_REMAIN    0x54
#define OFF_VIDEO_IN_DMA_TOTAL_XFER     0x58
#define OFF_VIDEO_IN_DMA_CMD_STAT2      0x60
#define OFF_VIDEO_IN_DMA_XFER_REMAIN2   0x64
#define OFF_VIDEO_IN_DMA_TOTAL_XFER2    0x68
#define OFF_UART_CONTROL            0x0
#define OFF_UART_DATA               0x4
#define OFF_UART_INTR_ENABLE        0x8
#define OFF_UART_INTR_CLEAR_STAT    0xC
#define OFF_CHANNEL0_LOWER_LUT0     0x0
#define OFF_CHANNEL0_LOWER_LUT1     0x400
#define OFF_CHANNEL0_UPPER_LUT0     0x1000
#define OFF_CHANNEL0_UPPER_LUT1     0x1400
#define OFF_MUX_RAM_REGISTER        0x2000
#define OFF_MUX_CONTROLLER          0x3000
#define OFF_SCAN_PIX_LINE0          0x3800
#define OFF_SCAN_PIX_LINE1          0x3804
#define OFF_SCAN_LINES              0x3808
#define OFF_FRAME_LINE_COUNTER      0x380C
#define OFF_AOI_XSTART              0x3400
#define OFF_AOI_XSTOP               0x3404
#define OFF_AOI_YSTART              0x3408
#define OFF_AOI_YSTOP               0x340C
#define OFF_EXTENDED_MODE           0x3C00
#define OFF_HRT_CONTROLLER_CTRL     0x0
#define OFF_HRT_CONTROLLER_RAM      0x2000
#define OFF_HRT_CONTROLLER_CLK      0xFF0
#define OFF_HRT_CONTROLLER_VER      0xFFC
#define OFF_POCL_CONTROL            0
#define OFF_POCL_STATUS             0x4
#define OFF_POCL_VERSION            0x8
#define OFF_POCL_INTR_ENABLE        0x10
#define OFF_POCL_INTR_CLEAR_STAT    0x14
#define OFF_NII51006_STAT           0x0
#define OFF_NII51006_READ_ADDR      0x4
#define OFF_NII51006_WRITE_ADDR     0x8
#define OFF_NII51006_LENGTH         0xC
#define OFF_NII51006_CONTROL        0x18

//------------------------------------------------------------------------------------
// Altera PCICore definitions
//-------------------------------------------------------------------------------------
#define PCI_CORE_REGISTER           0x4000
#define OFF_AVALON_TRANSLATION_TABLE 0x1000
#define AVALON_MEM_PCI_ADDR_TRANSLATION_TABLE 0x0
#define AVALON_DDR_MEMORY           0x20000000
#define AVALON_DDR_MEMORY_SIZE      (64*MB)
//------------------------------------------------------------------------------------
// Altera PCIeCore definitions
//-------------------------------------------------------------------------------------
#if NIOS_SUPPORTED_TRANSFER
#define PCI_EXPRESS_CORE_REGISTER   0x200000
#else
#define PCI_EXPRESS_CORE_REGISTER   0x4000
#endif
#define OFF_PCI_EXPRESS_INTERRUPT_STATUS 0x40
#define OFF_PCI_EXPRESS_INTERRUPT_ENABLE 0x50

#define OFF_P2A_MAILBOX0            0x800
#define OFF_P2A_MAILBOX1            0x804
#define OFF_P2A_MAILBOX2            0x808
#define OFF_P2A_MAILBOX3            0x80C
#define OFF_P2A_MAILBOX4            0x810
#define OFF_P2A_MAILBOX5            0x814
#define OFF_P2A_MAILBOX6            0x818
#define OFF_P2A_MAILBOX7            0x81C

#define OFF_A2P_MAILBOX0            0x900
#define OFF_A2P_MAILBOX1            0x904
#define OFF_A2P_MAILBOX2            0x908
#define OFF_A2P_MAILBOX3            0x90C
#define OFF_A2P_MAILBOX4            0x910
#define OFF_A2P_MAILBOX5            0x914
#define OFF_A2P_MAILBOX6            0x918
#define OFF_A2P_MAILBOX7            0x91C

//------------------------------------------------------------------------------------
// hyperion interrup vector
//-------------------------------------------------------------------------------------
#define VIDEO_IN_INT_VEC_0          (0x0<<8)
#define VIDEO_IN_INT_VEC_1          (0x1<<8)
#define UART_INT_VEC_0              (0x2<<8)
#define UART_INT_VEC_1              (0x3<<8)
#define EPCS_CTRL_INT_VEC           (0x8<<8)
#define POCL_CTRL_INT_VEC_0         (0x9<<8)
#define POCL_CTRL_INT_VEC_1         (0xA<<8)
//------------------------------------------------------------------------------------
// additional defines
//-------------------------------------------------------------------------------------
#define BIT_OFF_64 2
#define HYPERION_MSI_X_MAX_VECTORS  1
#define UART_NUM            0x2
#define MAX_PARALLEL_TRANSFER 0x2

#define KB 1024
#define MB (KB*KB)
#define kHz (1000)
#define MHz (kHz * kHz)
#define LUT_SIZE KB

#define TRANSLATION_TABLE_ELEMENTS 512
#define PAGE_SIZE_4K 0x1000
#define PAGE_SIZE_64K 0x10000
#define TRANSLATION_TABLE_SIZE (TRANSLATION_TABLE_ELEMENTS*PAGE_SIZE_4K)
#define TRANSLATION_TABLE_SIZE_64K (TRANSLATION_TABLE_ELEMENTS*PAGE_SIZE_64K)
#define TRANSFERBLOCKS_PER_CHANNEL 4
#define ONBOARD_MEMORY_LEN 640
#define DMA_RESULT_QUEUE_LEN 2048

#define NUMBER_PCI_BASE_ADDRESS 6
#define REGISTER_BASE_CLE 0
#define REGISTER_BASE_CL4E 2
#define REGISTER_BASE_CL4E_WIN 1
#define DDR_BASE 1

#define MUX_CONTROLLER_SEQUENCE_SIZE_BYTES (32*sizeof(unsigned int))

#define RESET_HYPERION_DATA 0xA4
#define HRTC_VER1_FREQ_HZ 53333333
//-------------------------------------------------------------------------------------
// register
//-------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------
struct memory_space
//-------------------------------------------------------------------------------------
{
    unsigned char* base; ///< pointer to mapped physical_address
    unsigned long physical_address; ///< physical address of this memory
    unsigned long size; ///< mapped size
};

#define HYPERION_OFFSET_MUL 0xF
//-------------------------------------------------------------------------------------
typedef struct _HyperionBaseRegisterDef
//-------------------------------------------------------------------------------------
{
    int index, offset, off_mul;
} HYPERION_BASE_REGISTER_DEF;

//-------------------------------------------------------------------------------------
enum eBaseRegisterHyperion
//-------------------------------------------------------------------------------------
{
    ebrhDMACtrlNII51006 = 0,
    ebrhSystemRegister,
    ebrhI2CRead,
    ebrhAsmiInterface,
    ebrhDMACtrlVideoIn0,
    ebrhDMACtrlVideoIn1,
    ebrhDMACtrlPCI0,
    ebrhDMACtrlPCI1,
    ebrhPoCLCtrl0,
    ebrhPoCLCtrl1,
    ebrhUart0,
    ebrhUart1,
    ebrhCLController0,
    ebrhCLController1,
    ebrhHrtController0,
    ebrhHrtController1,
    ebrhPCICore,
    ebrhTriggerHrtController0,
    ebrhTriggerHrtController1,
    ebrhOnChipMemData,
    ebrhSPISimple,
    ebrhDecCtrl0,
    ebrhDecCtrl1,
    ebrhMax
};

//-------------------------------------------------------------------------------------
typedef struct _InterruptStatusReg
//-------------------------------------------------------------------------------------
{
    volatile unsigned vdma0_int : 1;
    volatile unsigned vdma1_int : 1;
    volatile unsigned uart0_int : 1;
    volatile unsigned uart1_int : 1;
    volatile unsigned reserved  : 4;
    _fillbits( 24 );
} INTERRUPT_STATUS_REG;

//-------------------------------------------------------------------------------------
// system control
//-------------------------------------------------------------------------------------
#define ENABE_BASE_MODE_CH1 (1<<0)
#define RESET_CL_RECEIVER (1<<1)
#define PRODUCT_TYPE_MSK (0xf0)
#define HYPERION_PCI_PAGE_SIZE_64K (1<<17)
#define LOCAL_RESET (0xa4<<24)
#define ENABLE_VOLTAGE_12V (1<<31)

//-------------------------------------------------------------------------------------
typedef struct _SYSTEM_CONTROL
//-------------------------------------------------------------------------------------
{
    volatile unsigned base_ch1 : 1;
    volatile unsigned reserved : 23;
    volatile unsigned local_reset : 8;
} SYSTEM_CONTROL_REG;

//-------------------------------------------------------------------------------------
enum eInterruptStatus
//-------------------------------------------------------------------------------------
{
    eIntVideoInDma0 = 0,
    eIntVideoInDma1,
    eIntUart0,
    eIntUart1,
    eIntDdrToPciDma0,
    eIntDdrToPciDma1,
    eIntPCITableWriter = 7,
    eIntAsmiController,
    eIntPoCL0,
    eIntPoCL1,
    eIntHrtc0 = 11,
    eIntHrtc1,
    eIntHrtc2 = 13,
    eIntHrtc3,
    eIntMax,
};

#define INTR_STAT_VIDEO_IN_DMA0 (1<<eIntVideoInDma0)
#define INTR_STAT_VIDEO_IN_DMA1 (1<<eIntVideoInDma1)
#define INTR_STAT_UART0 (1<<eIntUart0)
#define INTR_STAT_UART1 (1<<eIntUart1)
#define INTR_STAT_DDRTOPCI_DMA0 (1<<eIntDdrToPciDma0)
#define INTR_STAT_DDRTOPCI_DMA1 (1<<eIntDdrToPciDma1)
#define INTR_STAT_DMA_TO_FPGA (1<<eIntPCITableWriter)
#define INTR_STAT_ASMI_CTRL (1<<eIntAsmiController)
#define INTR_STAT_POCL0 (1<<eIntPoCL0)
#define INTR_STAT_POCL1 (1<<eIntPoCL1)
#define INTR_STAT_HRTC0 (1<<eIntHrtc0)
#define INTR_STAT_HRTC1 (1<<eIntHrtc1)
#define INTR_STAT_HRTC2 (1<<eIntHrtc2)
#define INTR_STAT_HRTC3 (1<<eIntHrtc3)

//-------------------------------------------------------------------------------------
// uart control register
//-------------------------------------------------------------------------------------
#define UART_CTRL_DATA          0
#define UART_CTRL_RXD_PRES      8
#define UART_CTRL_RX_FIFO       9
#define UART_CTRL_RX_STOP_DETECT 9
#define UART_CTRL_TX_FIFO       10
#define UART_CTRL_TEST_LOOP     11
#define UART_CTRL_RXD_IRQ       12
#define UART_CTRL_TX_FIFO_IRQ   13
#define UART_CTRL_PARITY        14
#define UART_CTRL_PARITY_MSK    0x3
#define UART_CTRL_FIFO_RESET    16
#define UART_CTRL_BAUDRATE      17
#define UART_CTRL_BAUDRATE_MSK  0x7
#define UART_CTRL_ENABLE_NTRISTATE 20
#define UART_CTRL_DISABLE_ECHO  21
#define UART_CTRL_TX_SIGNAL_ENABLE 23
#define UART_CTRL_VERSION       24

#define UART_PARITY_EVEN        0x0
#define UART_PARITY_ODD         0x1
#define UART_PARITY_DISABLE     0x2

#define UART_BAUDRATE_9600      0x0
#define UART_BAUDRATE_19200     0x1
#define UART_BAUDRATE_38400     0x2
#define UART_BAUDRATE_57600     0x3
#define UART_BAUDRATE_115200    0x4
#define UART_BAUDRATE_230400    0x5
#define UART_BAUDRATE_460800    0x6
#define UART_BAUDRATE_921600    0x7

#define CTRL_READ_DATA      (0xff<<UART_CTRL_DATA)
#define CTRL_READ_RXD_PRES  (1<<UART_CTRL_RXD_PRES)
#define CTRL_READ_RX_FIFO_FULL  (1<<UART_CTRL_RX_FIFO)
#define CTRL_READ_RX_STOP_DETECT (1<<UART_CTRL_RX_STOP_DETECT)
#define CTRL_READ_TX_FIFO_FULL  (1<<UART_CTRL_TX_FIFO)
#define CTRL_READ_TEST_LOOP     (1<<UART_CTRL_TEST_LOOP)
#define CTRL_READ_PARITY    (0x3<<UART_CTRL_PARITY)
#define CTRL_READ_FIFO_RESET    (1<<UART_CTRL_FIFO_RESET)
#define CTRL_READ_BAUDRATE  (0x3<<UART_CTRL_BAUDRATE)
#define CTRL_READ_UART_VERSION  (0xff<<UART_CTRL_VERSION)

#define CTRL_WRITE_TEST_LOOP    (1<<UART_CTRL_TEST_LOOP)
#define CTRL_WRITE_RXD_IRQ  (1<<UART_CTRL_RXD_IRQ)
#define CTRL_WRITE_TX_FIFO_IRQ  (1<<UART_CTRL_TX_FIFO_IRQ)
#define CTRL_WRITE_PARITY   (0x3<<UART_CTRL_PARITY)
#define CTRL_WRITE_FIFO_RESET   (1<<UART_CTRL_FIFO_RESET)
#define CTRL_WRITE_BAUDRATE     (0x3<<UART_CTRL_BAUDRATE)

//-------------------------------------------------------------------------------------
// uart data register
//-------------------------------------------------------------------------------------
#define UART_RX_DATA (0xff)
#define UART_TX_DATA (0xff)

//-------------------------------------------------------------------------------------
typedef struct _UART_REG
//-------------------------------------------------------------------------------------
{
    struct control_reg
    {
        union
        {
            volatile unsigned int l;
            struct
            {
                volatile unsigned int rx_data       : 8;
                volatile unsigned int rxd_pres      : 1;
                volatile unsigned int rx_ffull      : 1;
                volatile unsigned int tx_ffull      : 1;
                volatile unsigned int test_loop     : 1;
                volatile unsigned int reserved0     : 2;
                volatile unsigned int parity_mode   : 2;
                volatile unsigned int fifo_reset    : 1;
                volatile unsigned int baud_rate     : 3;
                volatile unsigned int reserved1     : 4;
                volatile unsigned int version       : 8;
            } read;
            struct
            {
                volatile unsigned int reserved      : 11;
                volatile unsigned int test_loop     : 1;
                volatile unsigned int rxd_pres_irq  : 1;
                volatile unsigned int tx_ffull_irq  : 1;
                volatile unsigned int parity_mode   : 2;
                volatile unsigned int fifo_reset    : 1;
                volatile unsigned int baud_rate     : 3;
                volatile unsigned int reserved1     : 12;
            } write;
        } u; //offset 0
    } uart_control;
    union
    {
        volatile unsigned int l;
        struct
        {
            volatile unsigned int rx_data       : 8;
            volatile unsigned int rxd_pres      : 1;
            volatile unsigned int rx_ffull      : 1;
            volatile unsigned int tx_ffull      : 1;
            volatile unsigned int test_loop     : 1;
            volatile unsigned int reserved0     : 2;
            volatile unsigned int parity_mode       : 2;
            volatile unsigned int fifo_reset        : 1;
            volatile unsigned int baud_rate     : 3;
            volatile unsigned int reserved1     : 12;
        } read;
        struct
        {
            volatile unsigned char tx_data;
            volatile unsigned char reserved[3];
        } write;
    } data; //offset 0x4
    union
    {
        volatile unsigned int l;
        struct
        {
            volatile unsigned int rxd_pres_en   : 1;
            volatile unsigned int tx_ffull_en   : 1;
            volatile unsigned int reserved  : 30;
        } b;
    } interrupt_enable; //offset 0x8
    union
    {
        volatile unsigned int l;
        struct
        {
            volatile unsigned int rxd_pres  : 1;
            volatile unsigned int tx_ffull  : 1;
            volatile unsigned int reserved  : 30;
        } b;
    } interrupt_status; //offset 0xc
} UART_REG;

//-------------------------------------------------------------------------------------
// hrt_control
//-------------------------------------------------------------------------------------
#define HRTC_ENABLE (1<<0)
#define HRTC_INTERRUPT_ENABLE (1<<1)
#define HRTC_INTERRUPT_CLEAR (1<<2)

//-------------------------------------------------------------------------------------
// multiplexer_control
//-------------------------------------------------------------------------------------
#define MEDIUM_MODE_BIT 6
#define DATA_VALID_ENABLE   (1<<0)
#define AOI_ENABLE      (1<<1)
#define FIRST_LINE_CUT_ENABLE   (1<<2)
#define LINE_SCAN_ENABLE    (1<<3)
#define COUNTER_MODE_FALLING_EDGE (2<<4)
#define COUNTER_MODE_RISING_EDGE (3<<4)
#define FRAME_LINE_COUNTER_MODE (3<<4)
#define MEDIUM_MODE_ENABLE  (0x3<<MEDIUM_MODE_BIT)
#define PHASE_ERROR_DETECT (1<<9)
#define RESET_MEDIUM_CHANNELS (1<<8)
#define PHASE_ERROR_DETECT_VERSION1 (1<<8)
#define EXPAND_LVAL 16
#define EXPAND_LVAL_MSK (0xf)

//-------------------------------------------------------------------------------------
typedef struct _MUX_CTRL
//-------------------------------------------------------------------------------------
{
    volatile unsigned int dval      : 1;
    volatile unsigned int aoi_en        : 1;
    volatile unsigned int first_line_cut : 1;
    volatile unsigned int line_scan : 1;
    volatile unsigned int frame_line : 2;
    volatile unsigned int medium_mode : 1;
    _fillbits( 24 );
} MUX_CTRL_REG;

//-------------------------------------------------------------------------------------
typedef struct _AOI_XSTART
//-------------------------------------------------------------------------------------
{
    volatile unsigned int xstart : 24;
    _fillbits( 8 );
} AOI_XSTART_REG;

//-------------------------------------------------------------------------------------
typedef struct _AOI_XSTOP
//-------------------------------------------------------------------------------------
{
    volatile unsigned int xstop : 24;
    _fillbits( 8 );
} AOI_XSTOP_REG;

//-------------------------------------------------------------------------------------
typedef struct _AOI_YSTART
//-------------------------------------------------------------------------------------
{
    volatile unsigned int ystart : 24;
    _fillbits( 8 );
} AOI_YSTART_REG;

//-------------------------------------------------------------------------------------
typedef struct _AOI_YSTOP
//-------------------------------------------------------------------------------------
{
    volatile unsigned int ystop : 24;
    _fillbits( 8 );
} AOI_YSTOP_REG;

//-------------------------------------------------------------------------------------
typedef struct _SCAN_PIX
//-------------------------------------------------------------------------------------
{
    volatile unsigned int value : 24;
    _fillbits( 12 );
} SCAN_PIX_REG;


//-------------------------------------------------------------------------------------
typedef struct _POCL_CONTROL
//-------------------------------------------------------------------------------------
{
    volatile unsigned int control;
    volatile unsigned int status;
    volatile unsigned int version;
    volatile unsigned int dummy_reg0;
    volatile unsigned int interrupt_enable;
    volatile unsigned int interrupt_status;
} POCL_CONTROL;

//-------------------------------------------------------------------------------------
//PoCL control register
//-------------------------------------------------------------------------------------
#define NON_POCL_CAMERA_ENABLE  (1<<0)
#define POCL_CAMERA_ENABLE      (1<<1)
#define CL_CLK_OBSERVE          (1<<2)

//-------------------------------------------------------------------------------------
//PoCL status register
//-------------------------------------------------------------------------------------
#define STAT_THRESHOLD          (3<<0)
#define STAT_OVERCURRENT        (1<<2)
#define STAT_CH0_ENABLE         (1<<3)
#define STAT_CH1_ENABLE         (1<<4)
#define STAT_OVERLOAD           (1<<5)
#define STAT_ON_SEQ             (1<<6)
#define STAT_OVERLOAD_TIMER     (0xFF<<8)
#define STAT_CL_CLK_OFF_TIMER   (0xFF<<16)

//-------------------------------------------------------------------------------------
//PoCL interrupt enable and status
//-------------------------------------------------------------------------------------
#define OVERLOAD_INTR           (1<<0)
#define CL_CLK_HALT_INTR        (1<<1)

#define CRITICAL_CL_CLK_OFF_TIMER 0x660000

//-------------------------------------------------------------------------------------
//PCI_EXPRESS_CORE_INTERRUPT_ENABLE
//-------------------------------------------------------------------------------------
#define AVL_IRQ                 (1<<7)
#define AVL_IRQ_INPUT_VECTOR    (0x3f<<8)
#define A2P_MB_IRQ              (0xff<<16)

//-------------------------------------------------------------------------------------
//PCI_EXPRESS_CORE_INTERRUPT_STATUS
//-------------------------------------------------------------------------------------
#define AV_IRQ_ASSERTED         (1<<7)
#define AVL_IRQ_INPUT_VECTOR    (0x3f<<8)
#define BIT_A2P_MAILBOX_INT0    16
#define BIT_A2P_MAILBOX_INT7    23
#define A2P_MAILBOX_INT0        (1<<16)
#define A2P_MAILBOX_INT1        (1<<17)
#define A2P_MAILBOX_INT2        (1<<18)
#define A2P_MAILBOX_INT3        (1<<19)
#define A2P_MAILBOX_INT4        (1<<20)
#define A2P_MAILBOX_INT5        (1<<21)
#define A2P_MAILBOX_INT6        (1<<22)
#define A2P_MAILBOX_INT7        (1<<23)


//-------------------------------------------------------------------------------------
#define UART0_IRQ_VEC           (2<<8)
#define UART1_IRQ_VEC           (3<<8)
#define POCL0_IRQ_VEC           (9<<8)
#define POCL1_IRQ_VEC           (10<<8)


//-------------------------------------------------------------------------------------
typedef struct _INTERRRUPT_COUNTER_BASE
//-------------------------------------------------------------------------------------
{
    long long changed_counter;
    long long dpc_changed_counter;
} INTERRRUPT_COUNTER_BASE;

//-------------------------------------------------------------------------------------
#define SDI_DECODER_RESET       (1<<15)

#define VERSION_NUMBER_VIDEOIN_ROTARY_DECODER 0x2F
#endif //hyperionregisterH
