//-----------------------------------------------------------------------------
#ifndef hyperionpropH
#define hyperionpropH hyperionpropH
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
enum TPropertyType
//-----------------------------------------------------------------------------
{
    prtInt = 0,
    prtPointer
};

//-----------------------------------------------------------------------------
typedef struct _PropertyElement
//-----------------------------------------------------------------------------
{
    int PropertyID;
    union
    {
        int intElement;
        unsigned long long vPtrElement;
    } u;
    int Changed;
    int Type;
} TPropertyElement;

//-----------------------------------------------------------------------------
typedef struct _PropertyElement32
//-----------------------------------------------------------------------------
{
    int PropertyID;
    union
    {
        int intElement;
        int vPtrElement;
    } u;
    int Changed;
    int Type;
} TPropertyElement32;

#define _SET_PROP_ELEM_I(elemn,prop,val,changed)\
    elemn->PropertyID = prop;\
    elemn->u.intElement = val;\
    elemn->Changed = changed;\
    elemn->Type = prtInt;\
    elemn++;

//-----------------------------------------------------------------------------
enum TPropertyCategories
//-----------------------------------------------------------------------------
{
    pcInfo = 1,
    pcCameraCL = 2,
    pcConnector = 3,
    pcDMA = 4,
    pcSnapRequestResult = 5,
    pcDigitalIO = 6,
    pcHRTCtrl = 7,
    pcImageSettings = 8,
    pcMax
};

#define TRANSFER_PROPERTY_MAX_SIZE (64*1024)
#define CODE_FATAL_ERROR 0x80000000
#define CODE_WARNING 0x40000000
//-----------------------------------------------------------------------------
enum TErrorFacilityHyperion
//-----------------------------------------------------------------------------
{
    efhGeneric = 0,
    efhNiosStatus = ( 1 << 16 )
};

//-----------------------------------------------------------------------------
enum TCommunicationError
//-----------------------------------------------------------------------------
{
    cerrSuccess = 0,
    cerrDMACancelled = 1,
    cerrTimeout = 2,
    cerrDMAAborted = 3,
    cerrResultQueueEmpty = 4,
    cerrResultPacketAbort = 5,
    cerrDMAScatterGatherList = 6,
    cerrDMADataOverflow = 7,
    cerrNoPropertyStartCode = CODE_FATAL_ERROR
};

// from ntdefs.h
//  Values are 32 bit values using the following layout:
//
//   3 3 2 2 2 2 2 2 2 2 2 2 1 1 1 1 1 1 1 1 1 1
//   1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0 9 8 7 6 5 4 3 2 1 0
//  +---+-+-+-----------------------+-------------------------------+
//  |Sev|C|R|     Facility          |               Code            |
//  +---+-+-+-----------------------+-------------------------------+
//
//  where
//
//      Sev - is the severity code
//
//          00 - Success
//          01 - Informational
//          10 - Warning
//          11 - Error
//
//      C - is the Customer code flag
//
//      R - is a reserved bit
//
//      Facility - is the facility code
//
//      Code - is the facility's status code

//-----------------------------------------------------------------------------
enum TNiosErrorCode
//-----------------------------------------------------------------------------
{
    cnerrSuccess = 0,
    cnerrNiosRunning = 0,
    cnerrNiosStopped = ( 0xE0000000 | efhNiosStatus ),
    cnerrStartNiosNoEntryPointFound = ( 0xE0000001 | efhNiosStatus ),
    cnerrCommunicationTimeout = ( 0xE0000002 | efhNiosStatus )
};

//-----------------------------------------------------------------------------
enum TRequestActionInterface
//-----------------------------------------------------------------------------
{
    raiSnapRequest = 0,
    raiSnapResult = 1,
    raiSnapAbort = 2,
    raiMuxControllerData = 3,
    raiSnapRequestTrial = 4
};

//----------------------------------------------------------------------------
enum TDigitalOutput
//----------------------------------------------------------------------------
{
    edoCC1_J1 = 0,
    edoCC2_J1,
    edoCC3_J1,
    edoCC4_J1,
    edoCC1_J2,
    edoCC2_J2,
    edoCC3_J2,
    edoCC4_J2,
    edoFlashOut_J1,
    edoFlashOut_J2,
    edoDigitalOut0_J6,
    edoDigitalOut1_J6,
    edoDigitalOut2_J6,
    edoDigitalOut3_J6,
    edoID0,
    edoID2,
    edoMaxDigitalOutput
};

//----------------------------------------------------------------------------
enum TPulseStartEvent
//----------------------------------------------------------------------------
{
    epseSyncPassThrough = 0,
    epseSyncPassThroughInv,
    epseTrigPassThrough,
    epseTrigPassThroughInv
};

//----------------------------------------------------------------------------
enum TPassThroughSignal
//----------------------------------------------------------------------------
{
    eptsTriggerIn_J3 = 0,
    eptsSyncIn_J3,
    eptsTriggerIn_J4,
    eptsSyncIn_J4,
    eptsDigIn0,
    eptsDigIn1
};

//----------------------------------------------------------------------------
enum TDigitalInput
//----------------------------------------------------------------------------
{
    ediTriggerIn_J3 = 0,
    ediSyncIn_J3,
    ediTriggerIn_J4,
    ediSyncIn_J4,
    ediDigIn0_J6,
    ediDigIn1_J6,
    ediDigIn2_J6,
    ediDigIn3_J6,
    ediMaxDigitalInput
};

//-----------------------------------------------------------------------------
enum THardwareRealTimeController
//-----------------------------------------------------------------------------
{
    ehrtc0 = 0,
    ehrtc1,
    ehrtc2,
    ehrtc3,
    ehrtcMax
};

//-----------------------------------------------------------------------------
enum TDigitalIOControlMode
//-----------------------------------------------------------------------------
{
    edcmSoftware = 0,
    edcmRTC
};

//-----------------------------------------------------------------------------
enum TMuxMediumCtrl
//-----------------------------------------------------------------------------
{
    emmcBase = 0,
    emmcMedium = 1,
    emmcFull = 3,
    emmcMax
};

//-----------------------------------------------------------------------------
enum TTriggerSignals
//-----------------------------------------------------------------------------
{
    etsStartSignal = 1,
    etsStartSignalOverlap = 2,
    etsStopSignal = 4,
    etsStopSignalOverlap = 8
};

//-----------------------------------------------------------------------------
enum TTriggerActivation
//-----------------------------------------------------------------------------
{
    taEdge = 0,
    taLevel = 1
};

//-----------------------------------------------------------------------------
enum TMaxValueDefinition
//-----------------------------------------------------------------------------
{
    emvdHRTCTriggerCommand = 128
};

//-----------------------------------------------------------------------------
enum EQueryInfo
//-----------------------------------------------------------------------------
{
    eqiEepromContent = 0,
    eqiHRTCFrequency,
    eqiDigitalOutputAvailable,
    eqiDigitalInputAvailable,
    eqiTriggerModeAvailable,
    eqiMaxRequestObjectCount,
    eqiPermanentDMABufferUserVirtualAddress,
    eqiPermanentDMABufferSize,
    eqiHyperionProduct,
    eqiPermanentDMABufferUnmapUserVA,
    eqiMaxInfo
};

//-----------------------------------------------------------------------------
enum EHyperionDriverCapabilities
//-----------------------------------------------------------------------------
{
    edcSupportsLeaderTrailerStruct = 0x00000001
};

//-----------------------------------------------------------------------------
enum EHyperionProduct
//-----------------------------------------------------------------------------
{
    eHyperionProductUnknown = 0,
    eHyperionProductCLm = 0xc0,
    eHyperionProductCLf = 0xd0,
    eHyperionProductCLb = 0xe0,
    eHyperionProductReserved = 0xf0,
    eHyperionProductMax
};

//-----------------------------------------------------------------------------
enum EConnectorDefinition
//-----------------------------------------------------------------------------
{
    ecdConnectorJ1 = 0x1,
    ecdConnectorJ2 = 0x2,
    ecdConnectorJ3 = 0x4,
    ecdConnectorJ4 = 0x8,
    ecdConnectorJ6 = 0x10,
    ecdConnectorJ5 = 0x20
};

//-----------------------------------------------------------------------------
struct SPropertyInterface
//-----------------------------------------------------------------------------
{
    TPropertyElement* propList;
};

#define PROPERTY_TYPE_SHIFT 16
#define PROP_MASK 0x0000ffffUL
#define PROP_TYPE_MASKS ( PROP_MASK << PROPERTY_TYPE_SHIFT )
#define PROP_CATEGORY(prop) (prop >> PROPERTY_TYPE_SHIFT)

#ifdef PROP_DRIVER
#   define prStart( t, u ) pr##t##First = pr##u##Max + ( pc##t << PROPERTY_TYPE_SHIFT ) - 1
#else
#   define prStart( t, u ) pr##t##First =  ( pc##t << PROPERTY_TYPE_SHIFT ) - 1
#endif
#define prEnd( t ) pr##t##Last, pr##t##Max =  pr##t##Last & PROP_MASK

//----------------------------------------------------------------------------
enum TPropCommon
//----------------------------------------------------------------------------
{
    prCount = 0,
    prDebugLevel,
    prCommonMax
};

//----------------------------------------------------------------------------
//should be first located in prop-handler
enum TPropInfoCategory
//----------------------------------------------------------------------------
{
    prStart( Info, Common ),

    prInfoPropertyCount,
    prInfoFooterOffset,
    prInfoFooterSize,
    prInfoRequestID,
    prInfoTimeout,
    prInfoRequestAction,
    prInfoPropertiesChanged,

    prEnd( Info )
};

//----------------------------------------------------------------------------
enum TPropConnectorCategory
//----------------------------------------------------------------------------
{
    prStart( Connector, Common ),

    prCnInput,
    prCnMuxCtrlCount,
    prCnMuxCtrlRamData,

    prEnd( Connector )
};

//----------------------------------------------------------------------------
enum TPropImageSettingsCategory
//----------------------------------------------------------------------------
{
    prStart( ImageSettings, Common ),

    prIsInputChannel,
    prIsFrameStartMode,
    prIsFrameStartOverlap,
    prIsFrameStopMode,
    prIsFrameStopOverlap,
    prIsTriggerConfiguration,
    prIsTriggerRestartHRTC,
    prIsFrameStartTriggerMomemt,

    prEnd( ImageSettings )
};

//----------------------------------------------------------------------------
enum TPropCameraCLCategory
//----------------------------------------------------------------------------
{
    prStart( CameraCL, Common ),

    prClXPos,
    prClYPos,
    prClWidth,
    prClHeight,
    prClInterlaced,
    prClStartField,
    prClPixelClockFreq,
    prClScanMode,
    prClDataValidEnable,
    prClLUTEnable,
    prClMediumSource,
    prCLEnableAOIMode,
    prCLExpandLineValid,
    prCLLinePitch,
    prCLFramerateDivider,

    prEnd( CameraCL )
};

//----------------------------------------------------------------------------
enum TPropDMACategory
//----------------------------------------------------------------------------
{
    prStart( DMA, Common ),

    prDMATransferLength,
    prDMAResultPacket,
    prDMAVideoInDirectTransfer,
    prDMAVideoInWaitOfFrameEdge,
    prDMASGListPhysLowAddress,
    prDMASGListPhysHighAddress,
    prDMAPageSizeHost,
    prDMALineScanStartCondition,

    prEnd( DMA )
};

//----------------------------------------------------------------------------
enum TPropSnapRequestResultCategory
//----------------------------------------------------------------------------
{
    prStart( SnapRequestResult, Common ),

    prSResPropertyCount,
    prSResStatus,
    prSResFrameDelay,
    prSResTimeStampLowPart,
    prSResTimeStampHighPart,
    prSResRequestID,
    prSResFrameNr,
    prSResScanPixLine0,
    prSResScanPixLine1,
    prSResScanLines,
    prSresBytesTransferredSoFar,
    prSresTriggerFrameStartCounter,
    prSresTriggerAcquisitionStartCounter,
    prEnd( SnapRequestResult )
};

//----------------------------------------------------------------------------
enum TPropDigitalIOCategory
//----------------------------------------------------------------------------
{
    prStart( DigitalIO, Common ),

    prIOPropertyCount,
    prIOOutput,
    pdIOControlMode,
    prIOState,
    prPulsStartEvent,
    prDigInPassThrough,
    prPassThroughInverted,
    prIOAvailable,

    prEnd( DigitalIO )
};

//----------------------------------------------------------------------------
enum TPropHRTCtrlCategory
//----------------------------------------------------------------------------
{
    prStart( HRTCtrl, Common ),

    prHRTCIndex,
    prHRTCControl,
    prHRTCCommandCount,
    prHRTCRAMData,

    prEnd( HRTCtrl )
};

#endif //hyperionpropH
