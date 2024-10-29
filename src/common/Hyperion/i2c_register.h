#ifndef i2c_registerH
#define i2c_registerH i2c_registerH

#ifndef _fillbits
#define _fillbits(n)    volatile unsigned int : (n)/3; volatile unsigned int : (n)-(n)/3
#endif

//-------------------------------------------------------------------------------------
typedef struct _IIC_READ
//-------------------------------------------------------------------------------------
{
    unsigned scl : 1;
    unsigned sda : 1;
    _fillbits( 30 );
} IIC_READ_REG;


#endif //i2c_registerH
