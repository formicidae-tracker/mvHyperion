#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.
AC_PREREQ(2.57)
PACKAGE=hyperion2
MODULE_VERSION_MAJOR=1
MODULE_VERSION_MINOR=1
MODULE_VERSION=$MODULE_VERSION_MAJOR.$MODULE_VERSION_MINOR
VERSION=$MODULE_VERSION
AC_INIT([PACKAGE], [VERSION])
AC_PREFIX_DEFAULT([/usr])
AC_CANONICAL_HOST
AC_CANONICAL_TARGET

AC_SUBST(MODULE_VERSION_MAJOR)
AC_SUBST(MODULE_VERSION_MINOR)
AC_SUBST(MODULE_VERSION)


AM_INIT_AUTOMAKE
AC_SUBST(VERSION)
AC_SUBST(PACKAGE)

AC_CONFIG_SRCDIR(drivermain.c)
AC_CONFIG_HEADER(../common/mv_kconfig.h)
###AC_PRESERVE_HELP_ORDER

# Support for maintainer mode
AC_MSG_CHECKING(for maintainer mode)
AC_ARG_ENABLE(maintainer-mode,
  AS_HELP_STRING(--enable-maintainer-mode, enable make rules and dependencies not useful
                          (and sometimes confusing) to the casual installer),
	[ac_cv_enable_maintainer=$enableval], [ac_cv_enable_maintainer="no"])
MAINTAINER_MODE=$ac_cv_enable_maintainer
AC_SUBST(MAINTAINER_MODE)
AC_MSG_RESULT($ac_cv_enable_maintainer)

# Checks for programs.
AC_PROG_CC

# Options
AC_CHECK_TOOL([use_strip], [strip], [:])
AC_ARG_WITH([strip], 
			AS_HELP_STRING([--with-strip=location], [how to call the 'strip' program [[strip]]]),
			[STRIP="$withval"],
			[STRIP=$use_strip])

AC_CHECK_TOOL([use_depmod], [depmod], [:])
AC_ARG_WITH([depmod], 
			AS_HELP_STRING([--with-depmod=location], [how to call the 'depmod' program [[depmod]]]),
			[DEPMOD="$withval"],
			[DEPMOD=$use_depmod])

# Check for directory with kernel source...
AC_MSG_CHECKING(for directory with kernel source)
DEFAULT1_KSRC=/lib/modules/`uname -r`/source
DEFAULT2_KSRC=/lib/modules/`uname -r`/build
# try source before build
if test -e $DEFAULT1_KSRC ; then
	DEFAULT_KSRC=$DEFAULT1_KSRC
else
	DEFAULT_KSRC=$DEFAULT2_KSRC
fi

AC_ARG_WITH([kernel],
  			AS_HELP_STRING([--with-kernel=DIR], [directory with kernel sources (DIR=/lib/modules/`uname -r`/source)]),
  			[kerneldir="$withval"],
			[kerneldir=$DEFAULT_KSRC])
KERNEL_SOURCE=$kerneldir
AC_MSG_RESULT($kerneldir)

# Check for directory with kernel objects...
AC_MSG_CHECKING(for directory with kernel objects)
DEFAULT2_KOBJ=/lib/modules/`uname -r`/build
if test x"$KERNEL_SOURCE" == x"$DEFAULT_KSRC" ; then
	# using default source directory so try default build directory too
	if test -e $DEFAULT2_KOBJ ; then
		DEFAULT_KOBJ=$DEFAULT2_KOBJ
	fi
else
	# use same, user-provided directory for both source and objects 
	DEFAULT_KOBJ=$kerneldir
fi
AC_ARG_WITH([kernelobj],
  			AS_HELP_STRING([--with-kernelobj=DIR], [directory with kernel objects (DIR=/lib/modules/`uname -r`/build)]),
  			[kerneldir="$withval"],
			[kernelobjdir=$DEFAULT_KOBJ])
KERNEL_OBJECT=$kernelobjdir
AC_MSG_RESULT($kernelobjdir)

# don't use default CFLAGS
CFLAGS=

# check kernel properties
AC_MSG_CHECKING(Red Hat kernel source)
if test -e $kerneldir/include/linux/rhconfig.h ; then
	RH_KERNEL=yes
else
	RH_KERNEL=no
fi
AC_MSG_RESULT($RH_KERNEL)

AC_MSG_CHECKING(kernel version)

if test x$kerneldir == x$DEFAULT_KSRC -o x$RH_KERNEL = xyes; then
	LINUXVERSION=`uname -r`
elif test -e $kerneldir/include/generated/utsrelease.h; then
	LINUXVERSION=`cat $kerneldir/include/generated/utsrelease.h | sed -ne 's/[[#a-z ]]*UTS_RELEASE[[ ]]*\"\([[\.0-9]]*[[\.a-zA-Z0-9\-]]*\)\"/\1/p'`
elif test -e $kerneldir/include/linux/utsrelease.h; then
	LINUXVERSION=`cat $kerneldir/include/linux/utsrelease.h | sed -ne 's/[[#a-z ]]*UTS_RELEASE[[ ]]*\"\([[\.0-9]]*[[\.a-zA-Z0-9\-]]*\)\"/\1/p'`
else
	LINUXVERSION=`cat $kerneldir/include/linux/version.h | sed -ne 's/[[#a-z ]]*UTS_RELEASE[[ ]]*\"\([[\.0-9]]*[[\.a-zA-Z0-9\-]]*\)\"/\1/p'`
fi
AC_MSG_RESULT($LINUXVERSION)

KERNEL_MAJOR=`echo $LINUXVERSION | cut -d'"' -f2 | cut -d'.' -f1`
KERNEL_MINOR=`echo $LINUXVERSION | cut -d'"' -f2 | cut -d'.' -f2`
KERNEL_BUILD=`echo $LINUXVERSION | cut -d'"' -f2 | cut -d'.' -f3 | cut -d'-' -f1`
AC_MSG_CHECKING(numerical kernel version)
NUMERICAL_VERSION=$KERNEL_MAJOR.$KERNEL_MINOR.$KERNEL_BUILD
AC_MSG_RESULT($NUMERICAL_VERSION)
AC_MSG_CHECKING(kernel patchlevel)
KERNEL_PATCH_LEVEL=$KERNEL_MINOR
AC_MSG_RESULT($KERNEL_PATCH_LEVEL)

if test x$cross_compiling = xno; then
	KMODULES=/lib/modules/$LINUXVERSION
fi

AC_MSG_CHECKING([whether do_munmap takes an extra parameter])
CFLAGS1=$CFLAGS
CFLAGS="$CFLAGS -Wall -Werror -D__KERNEL__ -DMODULE -DKBUILD_BASENAME=\"dummy\" -I$kerneldir/include -I$kerneldir/include/asm/mach-default"
AC_TRY_COMPILE([#include <linux/mm.h>],
[
    struct mm_struct *mms = NULL;
    unsigned long addr = 0;
    size_t len = 0;

    do_munmap (mms, addr, len, 1);
], dm_extra=yes, dm_extra=no)
AC_MSG_RESULT($dm_extra)
if test x$dm_extra = xyes; then
    AC_DEFINE(DOMUNMAP_EXTRA, [1], [do_munmap takes an extra parameter])
fi
CFLAGS=$CFLAGS1


AC_SUBST(RH_KERNEL)
AC_SUBST(LINUXVERSION)
AC_SUBST(NUMERICAL_VERSION)
AC_SUBST(KERNEL_PATCH_LEVEL)
AC_SUBST(KMODULES)
AC_SUBST(KERNEL_SOURCE)
AC_SUBST(KERNEL_OBJECT)
AC_SUBST(STRIP)
AC_SUBST(DEPMOD)
AC_SUBST(KERNEL_MAJOR)
AC_SUBST(KERNEL_MINOR)
AC_SUBST(KERNEL_BUILD)

AC_CONFIG_FILES(Makefile)
AC_OUTPUT

