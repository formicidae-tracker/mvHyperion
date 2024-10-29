#ifdef DRIVER
//includes for kernel compilation
#   include "stddcls.h"
#   include "HyperionRegister.h"
#   include "i2c_access.h"
#   define DELAY_EXECUTION(delay)\
    if(delay < 50)\
    {\
        KeStallExecutionProcessor( delay );\
    }\
    else\
    {\
        LARGE_INTEGER Interval;\
        Interval.QuadPart = ((delay * -10000) / 1000);\
        KeDelayExecutionThread( KernelMode, FALSE, &Interval );\
    }
#else
#   include "i2c_register.h"
#   include "i2c_access.h"
#   include <asm/delay.h>
#   define DELAY_EXECUTION(delay) udelay(delay)
#   include <linux/pci.h>
#endif

#define NO_SUB_ADDR             -1

static long i2c_delay_m  = 10000L; //eeprom specification at least 5000usec
static long i2c_t_buf_m  = 49L;
static long i2c_t_low_m  = 49L;
static long i2c_t_high_m = 49L;


//-----------------------------------------------------------------------------
#define WAIT()                  DELAY_EXECUTION( i2c_delay_m )
#define TBUF()                  DELAY_EXECUTION( i2c_t_buf_m )   /* 30 milliseconds */
#define TLOW()                  DELAY_EXECUTION( i2c_t_low_m )
#define THIGH()                 DELAY_EXECUTION( i2c_t_high_m )

volatile unsigned int* IICRead;
unsigned int I2CPort;

#if defined(linux) || defined(__linux) || defined(__linux__)
#   define WRITE_REG32(preg,val) iowrite32( val, (void __iomem*)preg )
#   define READ_REG32(preg) ioread32( (void __iomem*)preg )
#else
#   define WRITE_REG32(preg,val) *preg = val
#   define READ_REG32(preg) *preg
#endif // #if defined(linux) || defined(__linux) || defined(__linux__)
//-----------------------------------------------------------------------------

#define SDAH()\
    {\
        I2CPort |= 2;\
        WRITE_REG32( IICRead, I2CPort );\
    }
#define SDAL()\
    {\
        I2CPort &= ~2;\
        WRITE_REG32( IICRead, I2CPort );\
    }
#define SCLH()\
    {\
        I2CPort |= 1;\
        WRITE_REG32( IICRead, I2CPort );\
    }
#define SCLL()\
    {\
        I2CPort &= ~1;\
        WRITE_REG32( IICRead, I2CPort );\
    }
#define SDAinput() ((READ_REG32(IICRead) & 0x2) >> 1)


//-----------------------------------------------------------------------------
void  SetI2CDelay( long delay, long tbuf, long tlow, long thigh )
//-----------------------------------------------------------------------------
{
    i2c_delay_m = delay;
    i2c_t_buf_m = tbuf;
    i2c_t_low_m = tlow;
    i2c_t_high_m = thigh;
}


//-----------------------------------------------------------------------------
void I2CStart( void )
//-----------------------------------------------------------------------------
{
    SCLH();
    THIGH();
    SDAH();
    THIGH();
    SDAL();                             /* Startbedingung auf I2C-Bus */
    TLOW();
    SCLL();
    TLOW();
}


//-----------------------------------------------------------------------------
void I2CStop( void )
//-----------------------------------------------------------------------------
{
    SDAL();                             /* Stoppbedingung auf I2C-Bus */
    TLOW();
    SCLH();
    THIGH();
    SDAH();
    WAIT();
}

//-----------------------------------------------------------------------------
//Ein Datenbyte ueber den I2C-Bus schicken
//Returnwert: 1 - Fehler, keine Quttierung
//            0 - Alles OK
int I2CSendByte( int Data )
//-----------------------------------------------------------------------------
{
    int Pos, err = 0, retry = 0, sda;

    /* Das Byte seriell ausgeben */
    for( Pos = 7; Pos >= 0; Pos-- )
    {
        if( ( ( Data >> Pos ) & 1 ) == 1 )
        {
            SDAH();
        }
        else
        {
            SDAL();
        }
        TBUF();
        SCLH();
        THIGH();
        SCLL();
        TLOW();
    }

    SDAH();
    THIGH();
    SCLH();
    THIGH();
    /* Quittierungsbit lesen */
    sda = *IICRead;
    if( SDAinput() )
    {
        SCLL();
        err = 1;
    }
    else
    {
        SCLL();
        err = 0;
    }

    retry++;
    return err;
}

//-----------------------------------------------------------------------------
int I2CSendData( unsigned int* preg, int Addr, int SubAddr, int Length, unsigned char* pData )
//-----------------------------------------------------------------------------
{
    int Pos;
    IICRead = preg;
    I2CPort = 0x3;

    I2CStart();                            /* Uebertragung starten */

    /* Adresse senden */
    if( I2CSendByte( ( short )( Addr & 0xfe ) ) )      /* Alles OK ? */
    {
        I2CStop();                           /* Nein Fehler aufgetreten */
        //KdPrint(("%s send address byte missing acknowledge\n", __FUNCTION__ ));
        return( 1 );
    }

    if( SubAddr != NO_SUB_ADDR )           /* Subadresse senden ? */
    {
        if( I2CSendByte( SubAddr ) )           /* Ja, Alles OK ? */
        {
            if( ( Addr & 1 ) == 0 )            /* kein Testmode ? */
            {
                I2CStop();                       /* Nein, Fehler aufgetreten */
                //KdPrint(("%s send subaddress byte error\n", __FUNCTION__ ));
                return( 2 );
            }
        }
    }
    TLOW();
    /* Daten senden */
    for( Pos = 0; Pos < ( int ) Length; Pos++ )
    {
        if( I2CSendByte( pData[Pos] ) )         /* Alles OK ? */
        {
            if( ( Addr & 1 ) == 0 )            /* kein Testmode ? */
            {
                I2CStop();                       /* Nein Fehler aufgetreten */
                //KdPrint(("%s send data byte error\n", __FUNCTION__ ));
                return( 3 );
            }
        }
        TLOW();
    }


    I2CStop();

    return( 0 );
}

//-----------------------------------------------------------------------------
void I2CReceiveByte( char* pData, int Last )
//-----------------------------------------------------------------------------
{
    int Pos, sda;

    *pData = 0;
    SDAH();
    THIGH();
    SCLL();
    TBUF();
    sda = *IICRead;
    for( Pos = 7; Pos >= 0; Pos-- )
    {
        SCLH();
        THIGH();
        *pData *= 2;
        sda = *IICRead;
        if( SDAinput() )
        {
            *pData |= 1;
        }
        SCLL();
        TLOW();
    }
    if( Last )
    {
        SDAH();
        THIGH();
    }
    else
    {
        SDAL();
        TLOW();
    }
    SCLH();
    THIGH();
    SCLL();
    TLOW();
    SDAH();
    THIGH();
}


//-----------------------------------------------------------------------------
//implemented as random read, if lenght > 1 then sequential read will be performed
int I2CReceiveData( unsigned int* preg, int Addr, int SubAddr, int Length, unsigned char* pData )
//-----------------------------------------------------------------------------
{
    int Pos;

    IICRead = preg;
    I2CPort = 0x3;

    I2CStart();                            /* bertragung starten */

    /* Adresse senden */
    if( I2CSendByte( ( int )( Addr ) ) )   /* Alles OK ? */
    {
        I2CStop();                           /* Nein Fehler aufgetreten */
        return( 1 );
    }

    if( SubAddr != NO_SUB_ADDR )           /* Subadresse senden ? */
    {
        if( I2CSendByte( SubAddr ) )            /* Ja, Alles OK ? */
        {
            // if ((Addr & 1) == 0)               /* kein Testmode ? */
            if( 0 )
            {
                I2CStop();                       /* Nein, Fehler aufgetreten */
                return( 2 );
            }
        }
    }
    TLOW();
    I2CStart();                            /* bertragung starten */
    SDAH();                                /* SDA-Leitung freigeben */
    if( I2CSendByte( ( int )( Addr | 1 ) ) )         /* Alles OK ? */
    {
        I2CStop();                           /* Nein Fehler aufgetreten */
        return( 3 );
    }

    /* Daten lesen */
    for( Pos = 0; Pos < ( int ) Length; Pos++ )
    {
        I2CReceiveByte( ( char* )&pData[Pos], ( int )( Pos == ( int )( Length - 1 ) ) );
    }
    I2CStop();
    return( 0 );
}
