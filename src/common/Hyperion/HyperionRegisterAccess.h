#ifndef HyperionRegisterAccessH
#define HyperionRegisterAccessH HyperionRegisterAccessH

#define IO_READ_8(MEMBASE,REGDEF,REGID,OFFSET) READ_REGISTER_UCHAR( (PUCHAR)(MEMBASE+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)))
#define IO_WRITE_8(MEMBASE,REGDEF,REGID,OFFSET,DATA) WRITE_REGISTER_UCHAR((PUCHAR)(MEMBASE+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)),DATA)
#define IO_READ_32(MEMBASE,REGDEF,REGID,OFFSET) READ_REGISTER_ULONG( (PULONG)(MEMBASE+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)))
#define IO_READ_32_PRINT(MEMBASE,REGDEF,REGID,OFFSET) KdPrint(("ioread32(base %p, regid %d, off %x) regdef.off %x regdef.off_mul %d *%p = %x\n", (void*)MEMBASE, REGID, OFFSET, REGDEF[REGID].offset, REGDEF[REGID].off_mul, (void*)(MEMBASE+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)), READ_REGISTER_ULONG((PULONG)(MEMBASE+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)))))
#define IO_WRITE_32(MEMBASE,REGDEF,REGID,OFFSET,DATA) WRITE_REGISTER_ULONG((PULONG)(MEMBASE+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET)),DATA)
#define REG_POINTER(MEMBASE,REGDEF,REGID,OFFSET) (MEMBASE+REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET))
#define REG_OFFSET(REGDEF,REGID,OFFSET) (REGDEF[REGID].offset+(REGDEF[REGID].off_mul*OFFSET))

#define SET_BIT(preg,bit,enable)\
    {\
        ULONG reg_value = READ_REGISTER_ULONG( (PULONG)preg );\
        if( enable )\
            reg_value |= bit;\
        else\
            reg_value &= ~bit;\
        WRITE_REGISTER_ULONG( (PULONG)preg, reg_value );\
    }

#endif //HyperionRegisterAccessH