#if defined(linux) || defined(__linux) || defined(__linux__)
#   include <linux/pci.h>
#   include "matrix_types.h"
#   include "HyperionProp.h"
#else
#   include "stddcls.h"
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
#include "DigitalIO.h"
#include "DMACtrl.h"

#define VIDEO_IN_CTRL_0 0
#define VIDEO_IN_CTRL_1 1


//-----------------------------------------------------------------------------
static DIGITAL_IN_OUT DigitalOutputSystemDefinition[edoMaxDigitalOutput] =
//-----------------------------------------------------------------------------
{
    { edoCC1_J1, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CONTROL, 4, 6, 0, NOT_INVERTED, 0x12 },
    { edoCC2_J1, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CONTROL, 9, 11, 0, NOT_INVERTED, 0x12 },
    { edoCC3_J1, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CONTROL, 14, 16, 0, NOT_INVERTED, 0x12 },
    { edoCC4_J1, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CONTROL, 19, 21, 0, NOT_INVERTED, 0x12 },
    { edoCC1_J2, ebrhDMACtrlVideoIn1, OFF_VIDEO_IN_CONTROL, 4, 6, 0, NOT_INVERTED, 0x12 },
    { edoCC2_J2, ebrhDMACtrlVideoIn1, OFF_VIDEO_IN_CONTROL, 9, 11, 0, NOT_INVERTED, 0x12 },
    { edoCC3_J2, ebrhDMACtrlVideoIn1, OFF_VIDEO_IN_CONTROL, 14, 16, 0, NOT_INVERTED, 0x12 },
    { edoCC4_J2, ebrhDMACtrlVideoIn1, OFF_VIDEO_IN_CONTROL, 19, 21, 0, NOT_INVERTED, 0x12 },
    { edoFlashOut_J1, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_CONTROL, 24, 26, 0, INVERTED, 0x12 },
    { edoFlashOut_J2, ebrhDMACtrlVideoIn1, OFF_VIDEO_IN_CONTROL, 24, 26, 0, INVERTED, 0x12 },
    { edoDigitalOut0_J6, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DIGIO_CONTROL, 0, 2, 0, NOT_INVERTED, 0x1a },
    { edoDigitalOut1_J6, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DIGIO_CONTROL, 5, 7, 0, NOT_INVERTED, 0x1a },
    { edoDigitalOut2_J6, ebrhDMACtrlVideoIn1, OFF_VIDEO_IN_DIGIO_CONTROL, 0, 2, 0, NOT_INVERTED, 0x1a },
    { edoDigitalOut3_J6, ebrhDMACtrlVideoIn1, OFF_VIDEO_IN_DIGIO_CONTROL, 5, 7, 0, NOT_INVERTED, 0x1a }
};


//-----------------------------------------------------------------------------
static DIGITAL_IN_OUT DigitalInputSystemDefinition[ediMaxDigitalInput] =
//-----------------------------------------------------------------------------
{
    { ediTriggerIn_J3, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DIGIO_CONTROL, 0, 0, 31, INVERTED, 0x1a },
    { ediSyncIn_J3, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DIGIO_CONTROL, 0, 0, 30, INVERTED, 0x1a },
    { ediTriggerIn_J4, ebrhDMACtrlVideoIn1, OFF_VIDEO_IN_DIGIO_CONTROL, 0, 0, 31, INVERTED, 0x1a },
    { ediSyncIn_J4, ebrhDMACtrlVideoIn1, OFF_VIDEO_IN_DIGIO_CONTROL, 0, 0, 30, INVERTED, 0x1a },
    { ediDigIn0_J6, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DIGIO_CONTROL, 0, 0, 28, NOT_INVERTED, 0x1a },
    { ediDigIn1_J6, ebrhDMACtrlVideoIn0, OFF_VIDEO_IN_DIGIO_CONTROL, 0, 0, 29, NOT_INVERTED, 0x1a },
    { ediDigIn2_J6, ebrhDMACtrlVideoIn1, OFF_VIDEO_IN_DIGIO_CONTROL, 0, 0, 28, NOT_INVERTED, 0x1a },
    { ediDigIn3_J6, ebrhDMACtrlVideoIn1, OFF_VIDEO_IN_DIGIO_CONTROL, 0, 0, 29, NOT_INVERTED, 0x1a }
};

#if defined(linux) || defined(__linux) || defined(__linux__)
#   define READ_REG32(MEMBASE,REGDEF,REGID,OFFSET) ioread32( (void __iomem *)(MEMBASE+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)))
#   define WRITE_REG32(MEMBASE,REGDEF,REGID,OFFSET,DATA) iowrite32(DATA, (void __iomem *)(MEMBASE+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)))
#else
#   define READ_REG32(MEMBASE,REGDEF,REGID,OFFSET) READ_REGISTER_ULONG( (PULONG)(MEMBASE+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)))
#   define WRITE_REG32(MEMBASE,REGDEF,REGID,OFFSET,DATA) WRITE_REGISTER_ULONG((PULONG)(MEMBASE+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)), DATA)
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)

//-----------------------------------------------------------------------------
int GetControllerVersion( unsigned char* base, HYPERION_BASE_REGISTER_DEF* reg_def, DIGITAL_IN_OUT* piodescr )
//-----------------------------------------------------------------------------
{
    int version = READ_REG32( base, reg_def, piodescr->ctrl_index, OFF_VIDEO_IN_CTRL_VERSION );
    return ( version & VERSIONS_NUMBER_MSK );
}

//-----------------------------------------------------------------------------
int IsDigitalInputAvailable( unsigned char* base, HYPERION_BASE_REGISTER_DEF* reg_def, unsigned int input )
//-----------------------------------------------------------------------------
{
    if( input < ediMaxDigitalInput )
    {
        int version = GetControllerVersion( base, reg_def, &DigitalInputSystemDefinition[input] );
        if( version != VERSIONS_NUMBER_MSK )
        {
            return ( version >= DigitalInputSystemDefinition[input].firmware_version );
        }
    }
    return FALSE;
}

//-----------------------------------------------------------------------------
int IsDigitalOutputAvailable( unsigned char* base, HYPERION_BASE_REGISTER_DEF* reg_def, unsigned int output )
//-----------------------------------------------------------------------------
{
    if( output < edoMaxDigitalOutput )
    {
        int version = GetControllerVersion( base, reg_def, &DigitalOutputSystemDefinition[output] );
        if( version != VERSIONS_NUMBER_MSK )
        {
            return ( version >= DigitalOutputSystemDefinition[output].firmware_version );
        }
    }
    return FALSE;
}

//-----------------------------------------------------------------------------
void ConfigureDigitalOutput( unsigned char* base, HYPERION_BASE_REGISTER_DEF* reg_def, unsigned int output, unsigned int passthrough_inv, unsigned int input_to_passthrough )
//-----------------------------------------------------------------------------
{
    DIGITAL_IN_OUT* pdigital_output_descr;
    unsigned int sync_mode, sync_sel, control = 0;
    if( output < edoMaxDigitalOutput )
    {
        pdigital_output_descr = &DigitalOutputSystemDefinition[output];
        sync_mode = passthrough_inv == TRUE ? dsmSyncSelInv : dsmSyncSel;
        sync_sel = input_to_passthrough;
        control = READ_REG32( base, reg_def, pdigital_output_descr->ctrl_index, pdigital_output_descr->register_offset );
        control &= ~( ( SYNC_SEL << pdigital_output_descr->nbit_sync_select ) | ( SYNC_MODE << pdigital_output_descr->nbit_sync_mode ) );
        control |= ( ( sync_sel << pdigital_output_descr->nbit_sync_select ) | ( sync_mode << pdigital_output_descr->nbit_sync_mode ) );
        WRITE_REG32( base, reg_def, pdigital_output_descr->ctrl_index, pdigital_output_descr->register_offset, control );
    }
}


//-----------------------------------------------------------------------------
void WriteDigitalOutput( unsigned char* base, HYPERION_BASE_REGISTER_DEF* reg_def, unsigned int output, unsigned int state )
//-----------------------------------------------------------------------------
{
    DIGITAL_IN_OUT* pdigital_output_descr;
    unsigned int control = 0;
    if( output < edoMaxDigitalOutput )
    {
        pdigital_output_descr = &DigitalOutputSystemDefinition[output];
        control = READ_REG32( base, reg_def, pdigital_output_descr->ctrl_index, pdigital_output_descr->register_offset );
        control &= ~( ( SYNC_MODE << pdigital_output_descr->nbit_sync_mode ) );
        control |= ( ( ( state ^ pdigital_output_descr->inverted ) << pdigital_output_descr->nbit_sync_mode ) );
        WRITE_REG32( base, reg_def, pdigital_output_descr->ctrl_index, pdigital_output_descr->register_offset, control );
    }
}

//-----------------------------------------------------------------------------
unsigned int ReadDigitalInput( unsigned char* base, HYPERION_BASE_REGISTER_DEF* reg_def, unsigned int input )
//-----------------------------------------------------------------------------
{
    DIGITAL_IN_OUT* pdigital_input_descr;
    unsigned int control;
    if( input < ediMaxDigitalInput )
    {
        pdigital_input_descr = &DigitalInputSystemDefinition[input];
        control = READ_REG32( base, reg_def, pdigital_input_descr->ctrl_index, pdigital_input_descr->register_offset ) & ( 1 << pdigital_input_descr->nbit_input );
        return ( control > 0 ) ^ pdigital_input_descr->inverted;
    }
    else
    {
        return 0;
    }
}
