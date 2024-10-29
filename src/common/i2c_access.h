//-----------------------------------------------------------------------------
#ifndef I2C_ACCESS_H
#define I2C_ACCESS_H I2C_ACCESS_H
//-----------------------------------------------------------------------------

#define EEPROM_ADDR  0xA0

int I2CReceiveData( unsigned int* preg, int Addr, int SubAddr, int Length, unsigned char* pData );
int I2CSendData( unsigned int* preg, int Addr, int SubAddr, int Length, unsigned char* pData );

#ifdef DRIVER
// {4D4F4B6F-1C19-49e6-B2FD-3CFA51672AAA}
DEFINE_GUID( EEPROM_ACCESS_MANUFACTURER_DATA, 0x4d4f4b6f, 0x1c19, 0x49e6, 0xb2, 0xfd, 0x3c, 0xfa, 0x51, 0x67, 0x2a, 0xaa );

// {739668E9-04BB-427b-9CDD-ACBE41003B1A}
DEFINE_GUID( EEPROM_ACCESS_USER_DATA, 0x739668e9, 0x4bb, 0x427b, 0x9c, 0xdd, 0xac, 0xbe, 0x41, 0x0, 0x3b, 0x1a );
#endif
#define EEPROM_USER_DATA_OFFSET 128


#endif // I2C_ACCESS_H
