#ifndef IOControlH
#define IOControlH IOControlH

#if HAVE_UNLOCKED_IOCTL
long hyperion_ioctl( struct file* file, unsigned int cmd, unsigned long arg );
#else
int hyperion_ioctl( struct inode* inode, struct file* file, unsigned int cmd, unsigned long arg );
#endif

#endif //IOControlH
