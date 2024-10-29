//-----------------------------------------------------------------------------
#ifndef hyperioneepromH
#define hyperioneepromH hyperioneepromH
//-----------------------------------------------------------------------------

//hyperion eeprom size complete = 256 bytes, user 128, mv 128

#define EEPROM_SIZE 256
#define USER_SIZE 128
#define MANUFACTORER_SIZE (EEPROM_SIZE-USER_SIZE)
#define CRC32_CSUM_SIZE 4
#define INFORMATION_SIZE 1
#define SERIAL_SIZE 12
#define TYPE_SIZE 16
#define REVISION_SIZE 6
#define DATA_MANUFACTORER_SIZE (CRC32_CSUM_SIZE + INFORMATION_SIZE + SERIAL_SIZE + TYPE_SIZE + REVISION_SIZE)
#define RESERVED_SIZE (MANUFACTORER_SIZE - DATA_MANUFACTORER_SIZE)

//-----------------------------------------------------------------------------
struct _HYPERION_EEPROM_CONTENT
//-----------------------------------------------------------------------------
{
    unsigned int crc32_checksum;
    unsigned char data_manufactorer_size;
    char serial[SERIAL_SIZE]; //actual [11] HC000530610
    char type[TYPE_SIZE]; //Hyperion-CLe
    char revision[REVISION_SIZE]; //1.00
    unsigned char reserved[RESERVED_SIZE];
    unsigned char user_information[USER_SIZE];
};

#endif //hyperioneepromH
