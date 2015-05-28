#ifndef	__PLAT_PART_TABLE_H
#define	__PLAT_PART_TABLE_H

#define DECLARE_LAB_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "lab", \
			.offset      = 0, \
			.size        = MTDPART_SIZ_FULL,     /* massstorage*/ \
		}, \
	}

#define DECLARE_ANDROID_128M_V75_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "Bootloader", \
			.offset      = 0, \
			.size        = 0x100000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "NVM", \
			.offset      = 0x100000, \
			.size        = 0x020000, \
		}, \
		[2] = { \
			.name        = "Arbel and Greyback Image", \
			.offset      = 0x120000,  \
			.size        = 0x800000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		},	 \
		[3] = { \
			.name        = "Kernel", \
			.offset      = 0x920000, \
			.size        = 0x300000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name        = "system", \
			.offset      = 0x0c20000, \
			.size        = 0x4000000,     /* mount 64M fs */ \
		}, \
		[5] = { \
			.name        = "userdata", \
			.offset      = 0x4c20000, \
			.size        = 0x20E0000,     /* mount 32.875M */ \
		}, \
		[6] = { \
			.name        = "filesystem", \
			.offset      = 0x6d00000, \
			.size        = 0x1000000,     /* mount 16M fs */ \
		}, \
		[7] = { \
			.name        = "BBT", \
			.offset      = 0x7d00000, \
			.size        = 0x0080000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[8] = { \
			.name        = "resevred_for_bbm", \
			.offset      = 0x7d80000, \
			.size        = MTDPART_SIZ_FULL,      \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
	/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.*/ \
		/* And the max relocation blocks is not same on different platform. */\
		/* Please take care it when define the partition table.*/ \
	}
	 
#define DECLARE_128M_V75_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "Bootloader", \
			.offset      = 0, \
			.size        = 0x100000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "NVM", \
			.offset      = 0x100000, \
			.size        = 0x020000, \
		}, \
		[2] = { \
			.name        = "Arbel and Greyback Image", \
			.offset      = 0x120000,  \
			.size        = 0x800000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		},	 \
		[3] = { \
			.name        = "Kernel", \
			.offset      = 0x920000, \
			.size        = 0x300000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name        = "Filesystem", \
			.offset      = 0x0c20000, \
			.size        = 0x3000000,     /* only mount 48M fs */ \
		}, \
		[5] = { \
			.name        = "MassStorage", \
			.offset      = 0x3c20000, \
			.size        = 0x40e0000,		/* 64.875M */ \
		}, \
		[6] = { \
			.name        = "BBT", \
			.offset      = 0x7d00000, \
			.size        = 0x0080000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[7] = { \
			.name        = "resevred_for_bbm", \
			.offset      = 0x7d80000, \
			.size        = MTDPART_SIZ_FULL,      \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.*/ \
		/* And the max relocation blocks is not same on different platform. */ \
		/* Please take care it when define the partition table.*/ \
	}

#define DECLARE_ANDROID_512M_V75_PARTITIONS(partitions) \
static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "Bootloader", \
			.offset      = 0x00040000, \
			.size        = 0x000b0000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "Reserve", \
			.offset      = 0x00100000, \
			.size        = 0x00020000, \
		}, \
		[2] = { \
			.name        = "Reserve", \
			.offset      = 0x00120000, \
			.size        = 0x00800000, \
		}, \
		[3] = { \
			.name        = "Kernel", \
			.offset      = 0x00920000, \
			.size        = 0x00300000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name        = "system", \
			.offset      = 0x00c20000, \
			.size        = 0x10000000, /* only mount 256M fs */ \
		}, \
		[5] = { \
			.name        = "userdata", \
			.offset      = 0x10c20000, \
			.size        = 0x0e920000,	/* 233.125M */ \
		}, \
		[6] = { \
			.name        = "BBT", \
			.offset      = 0x1f540000,     /* Last 81 Blocks reserved for BBM relocation */ \
			.size        = 0x00080000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[7] = { \
			.name        = "resevred_for_bbm", \
			.offset      = 0x1f5c0000, \
			.size        = MTDPART_SIZ_FULL,      \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.*/ \
		/* And the max relocation blocks is not same on different platform. */ \
		/* Please take care it when define the partition table.*/ \
	}

#define DECLARE_ANDROID_256M_V75_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "init", \
			.offset      = 0xc0000, \
			.size        = 0x40000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "NVM", \
			.offset      = 0x100000, \
			.size        = 0x020000, \
		}, \
		[2] = { \
			.name        = "Arbel and Greyback Image", \
			.offset      = 0x120000,  \
			.size        = 0x800000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		},	 \
		[3] = { \
			.name        = "Kernel", \
			.offset      = 0x920000, \
			.size        = 0x300000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name        = "system", \
			.offset      = 0x0c20000, \
			.size        = 0x7000000,     /* mount 112M fs */ \
		}, \
		[5] = { \
			.name        = "userdata", \
			.offset      = 0x7c20000, \
			.size        = 0x7000000,     /* mount 112M */ \
		}, \
		[6] = { \
			.name        = "filesystem", \
			.offset      = 0xec20000, \
			.size        = 0xEE0000,     /* mount 14.875M fs */ \
		}, \
		[7] = { \
			.name        = "resevred_for_bbm", \
			.offset      = 0xfb00000, \
			.size        = MTDPART_SIZ_FULL,      \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.*/ \
		/* And the max relocation blocks is not same on different platform. */\
		/* Please take care it when define the partition table.*/ \
	}
	 
#define DECLARE_256M_V75_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "Bootloader", \
			.offset      = 0, \
			.size        = 0x100000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "NVM", \
			.offset      = 0x100000, \
			.size        = 0x020000, \
		}, \
		[2] = { \
			.name        = "Arbel and Greyback Image", \
			.offset      = 0x120000,  \
			.size        = 0x800000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		},	 \
		[3] = { \
			.name        = "Kernel", \
			.offset      = 0x920000, \
			.size        = 0x300000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name        = "Filesystem", \
			.offset      = 0x0c20000, \
			.size        = 0x3000000,     /* only mount 48M fs */ \
		}, \
		[5] = { \
			.name        = "MassStorage", \
			.offset      = 0x3c20000, \
			.size        = 0x40e0000,		/* 64.875M */ \
		}, \
		[6] = { \
			.name        = "BBT", \
			.offset      = 0x7d00000, \
			.size        = 0x0080000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[7] = { \
			.name        = "SWAP", \
			.offset      = 0x8000000, \
			.size        = 0x6000000,   \
		}, \
		[8] = { \
			.name        = "resevred_for_bbm", \
			.offset      = 0xe000000, \
			.size        = MTDPART_SIZ_FULL,      \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.*/ \
		/* And the max relocation blocks is not same on different platform. */\
		/* Please take care it when define the partition table.*/ \
	}

#define DECLARE_512M_V75_PARTITIONS(partitions) \
static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "Bootloader", \
			.offset      = 0x00040000, \
			.size        = 0x000b0000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "Reserve", \
			.offset      = 0x00100000, \
			.size        = 0x00020000, \
		}, \
		[2] = { \
			.name        = "Reserve", \
			.offset      = 0x00120000, \
			.size        = 0x00800000, \
		}, \
		[3] = { \
			.name        = "Kernel", \
			.offset      = 0x00920000, \
			.size        = 0x00300000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name        = "Filesystem", \
			.offset      = 0x00c20000, \
			.size        = 0x03000000, /* only mount 48M fs */ \
		}, \
		[5] = { \
			.name        = "MassStorage", \
			.offset      = 0x03c20000, \
			.size        = 0x1b960000,	/* 441.375M */ \
		}, \
		[6] = { \
			.name        = "BBT", \
			.offset      = 0x1f540000,     /* Last 81 Blocks reserved 
						  for BBM relocation */ \
			.size        = 0x00080000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[7] = { \
			.name        = "resevred_for_bbm", \
			.offset      = 0x1f5c0000, \
			.size        = MTDPART_SIZ_FULL,      \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		/* NOTES: We reserve some blocks for PXA3xx BBM at the end of NAND.*/ \
		/* And the max relocation blocks is not same on different platform. */\
		/* Please take care it when define the partition table.*/ \
	}

#define DECLARE_32G_V75_PARTITIONS(partitions) \
static struct mtd_partition partitions[] = { \
		[0] = { \
			.name	     = "Bootloader", \
			.offset	     = 0x00040000, \
			.size	     = 0x000b0000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name	     = "Reserve", \
			.offset	     = 0x00100000, \
			.size	     = 0x00080000, \
		}, \
		[2] = { \
			.name	     = "Reserve", \
			.offset	     = 0x00180000, \
			.size	     = 0x00800000, \
		}, \
		[3] = { \
			.name	     = "Kernel", \
			.offset	     = 0x00980000, \
			.size	     = 0x00300000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name	     = "Filesystem", \
			.offset	     = 0x00c80000, \
			.size	     = 0x10000000, /* only mount 256M fs */ \
		}, \
		[5] = { \
			.name	     = "MassStorage", \
			.offset	     = 0x10c80000, \
			.size	     = 0x0e900000,	/* 233.125M */ \
		}, \
		[6] = { \
			.name	     = "BBT", \
			.offset	     = 0x1f540000,     \
			.size	     = 0x00080000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[7] = { \
			.name        = "resevred_for_bbm", \
			.offset      = 0x1f5c0000, \
			.size        = MTDPART_SIZ_FULL,      \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		/* NOTES: We reserve some blocks for			\
		 *  PXA3xx BBM at the end of NAND.			\
		 * And the max relocation blocks is not same		\
		 * on different platform.				\
		 * Please take care it when define the partition table.*/\
}

#define DECLARE_ANDROID_32G_V75_PARTITIONS(partitions) \
static struct mtd_partition partitions[] = { \
		[0] = { \
			.name	     = "Bootloader", \
			.offset	     = 0x00040000, \
			.size	     = 0x000b0000, \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name	     = "Reserve", \
			.offset	     = 0x00100000, \
			.size	     = 0x00080000, \
		}, \
		[2] = { \
			.name	     = "Reserve", \
			.offset	     = 0x00180000, \
			.size	     = 0x00800000, \
		}, \
		[3] = { \
			.name	     = "Kernel", \
			.offset	     = 0x00980000, \
			.size	     = 0x00300000, \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[4] = { \
			.name	     = "system", \
			.offset	     = 0x00c80000, \
			.size	     = 0x10000000, /* only mount 256M fs */ \
		}, \
		[5] = { \
			.name	     = "userdata", \
			.offset	     = 0x10c80000, \
			.size	     = 0x0e980000,	/* 233.125M */ \
		}, \
		[6] = { \
		.name	     = "BBT", \
		.offset	     = 0x1f540000,	\
		.size	     = 0x00080000, \
		.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		[7] = { \
			.name        = "resevred_for_bbm", \
			.offset      = 0x1f5c0000, \
			.size        = MTDPART_SIZ_FULL,      \
			.mask_flags  = MTD_WRITEABLE,  /* force read-only */ \
		}, \
		/* Please take care it when define the partition table.*/ \
}

#define DECLARE_AVENGERSLITE_SLC_PARTITIONS(partitions)     \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "Bootloader", \
			.offset      = 0, \
			.size        = 0x100000, \
		}, \
		[1] = { \
			.name        = "MassStorage0", \
			.offset      = 0x100000, \
			.size        = MTDPART_SIZ_FULL, /* use the rest of flash*/ \
		}, \
	}

#define DECLARE_AVENGERSLITE_MLC_4G_ANDROID_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "resereved", \
			.offset      = 0, \
			.size        = 0x80000,     /* reserved for relocation table */ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "Kernel",        \
			.offset      = 0x80000,         \
			.size        = 0x380000,        \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[2] = { \
			.name        = "Kernel_recovery", \
			.offset      = 0x400000, \
			.size        = 0x400000,     /* massstorage*/ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[3] = { \
			.name        = "System", \
			.offset      = 0x800000, \
			.size        = 0xf800000,     /* massstorage*/ \
		}, \
		[4] = { \
			.name        = "Userdata", \
			.offset      = 0x10000000, \
			.size        = 0x70000000,     /* massstorage*/ \
		}, \
		[5] = { \
			.name        = "MassStorage1", \
			.offset      = 0x80000000, \
			.size        = MTDPART_SIZ_FULL,     /* use the rest of flash*/ \
		}, \
	}

#ifdef CONFIG_ESI_EDGE
#define DECLARE_EDGE_MLC_4G_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		/* will not contain UBI devices/volumes */ \
		[0] = { \
			.name        = "BBM_reserve1", \
			.offset      = 0x00000000, \
			.size        = 0x00a00000, /* reserved */ \
			/* reserved space for next gen BBM scheme (20 blocks = 10M) */ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "bootloader-alt", \
			.offset      = 0x00a00000, \
			.size        = 0x00080000, /* reserved */ \
			/* reserved space for possible use by SPI NOR contents: */ \
			/* ntim   @ 0x00000000-0x00020000 */ \
			/* loader @ 0x00020000-0x00040000 */ \
			/* u-boot @ 0x00040000-0x00080000 */ \
		}, \
		[2] = { \
			.name        = "environment-alt", \
			.offset      = 0x00a80000, \
			.size        = 0x00080000, /* reserved */ \
			/* reserved space for possible use by SPI NOR contents: */ \
			/* env    @ 0x00080000-0x00100000 */ \
		}, \
		[3] = { \
			.name        = "kernel-alt",        \
			.offset      = 0x00b00000,      \
			.size        = 0x00300000,      \
			/* reserved space for possible use by SPI NOR contents: */ \
			/* kernel @ 0x00100000-0x00400000 */ \
		}, \
		[4] = { \
			.name        = "kernelrecovery-alt", \
			.offset      = 0x00e00000, \
			.size        = 0x00400000, \
			/* reserved space for possible use by SPI NOR contents: */ \
			/* kernelrecovery @ 0x00400000-0x00800000 */ \
		}, \
		[5] = { \
			.name        = "maintenance", \
			.offset      = 0x01200000, \
			.size        = 0x02e00000, \
			/* holds ramdisks for android and maintenance modes */ \
		}, \
		/* will contain UBI device and volumes */ \
		[6] = { \
			.name        = "Android", \
			.offset      = 0x04000000, \
			.size        = MTDPART_SIZ_FULL, \
			/* reserve space for relocated blocks: */ \
			/* 2% at end of device = 2% * 4G = 82M = 0x5200000 OR */ \
			/* 200 blocks per Samsung datasheet = 100M = 0x06400000 */ \
			/* THIS SPACE IS NOW AUTOMATICALLY ACCOUNTED FOR IN BBM DRIVER: */ \
			/* an additional partition "MRVL_BBM" is automatically created */ \
		}, \
	}

#define DECLARE_EDGE_8MNOR_PARTITIONS(partitions) \
struct mtd_partition partitions[] = { \
	{ \
		.name		= "bootloader", \
		.offset		= 0x00000000, \
		.size		= 0x00040000, \
		.mask_flags	= MTD_WRITEABLE,  /* force read-only */ \
	} , { \
		.name		= "bootcode", \
		.offset		= 0x00040000, \
		.size		= 0x00080000, \
	} , { \
		.name		= "environment", \
		.offset		= 0x000c0000, \
		.size		= 0x00040000, \
	} , { \
		.name		= "kernel", \
		.offset		= 0x00100000, \
		.size		= 0x00300000, \
	} , { \
		.name		= "kernelrecovery", \
		.offset		= 0x00400000, \
		.size		= MTDPART_SIZ_FULL, \
	} \
}; 
#endif

#define DECLARE_AVENGERSLITE_MLC_1G_ANDROID_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "resereved", \
			.offset      = 0, \
			.size        = 0x80000,     /* reserved for relocation table */ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "Kernel",        \
			.offset      = 0x80000,         \
			.size        = 0x380000,        \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[2] = { \
			.name        = "Kernel_recovery", \
			.offset      = 0x400000, \
			.size        = 0x400000,     /* massstorage*/ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[3] = { \
			.name        = "System", \
			.offset      = 0x800000, \
			.size        = 0x8000000,     /* massstorage*/ \
		}, \
		[4] = { \
			.name        = "Userdata", \
			.offset      = 0x8800000, \
			.size        = 0x13800000,     /* massstorage*/ \
		}, \
		[5] = { \
			.name        = "MassStorage1", \
			.offset      = 0x1c000000, \
			.size        = MTDPART_SIZ_FULL,     /* use the rest of flash*/ \
		}, \
	}

#define DECLARE_AVENGERSLITE_MLC_4G_MAEMO_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "resereved", \
			.offset      = 0, \
			.size        = 0x80000,     /* reserved for relocation table */ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "Kernel",        \
			.offset      = 0x80000,         \
			.size        = 0x380000,        \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[2] = { \
			.name        = "Kernel_recovery", \
			.offset      = 0x400000, \
			.size        = 0x400000,     /* massstorage*/ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[3] = { \
			.name        = "System", \
			.offset      = 0x800000, \
			.size        = 0x3f800000,     /* massstorage*/ \
		}, \
		[4] = { \
			.name        = "Userdata", \
			.offset      = 0x40000000, \
			.size        = 0x40000000,     /* massstorage*/ \
		}, \
		[5] = { \
			.name        = "MassStorage1", \
			.offset      = 0x80000000, \
			.size        = MTDPART_SIZ_FULL,     /* use the rest of flash*/ \
		}, \
	}

#define DECLARE_AVENGERSLITE_MLC_1G_MAEMO_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "resereved", \
			.offset      = 0, \
			.size        = 0x80000,     /* reserved for relocation table */ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[1] = { \
			.name        = "Kernel",        \
			.offset      = 0x80000,         \
			.size        = 0x380000,        \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[2] = { \
			.name        = "Kernel_recovery", \
			.offset      = 0x400000, \
			.size        = 0x400000,     /* massstorage*/ \
			.mask_flags  = MTD_WRITEABLE, /* force read-only */ \
		}, \
		[3] = { \
			.name        = "System", \
			.offset      = 0x800000, \
			.size        = 0x1b800000,     /* massstorage*/ \
		}, \
		[4] = { \
			.name        = "Userdata", \
			.offset      = 0x1c000000, \
			.size        = MTDPART_SIZ_FULL,     /* massstorage*/ \
		},\
	}

#define DECLARE_SPI_PARTITIONS(partitions) \
	static struct mtd_partition partitions[] = { \
		[0] = { \
			.name        = "Bootloader", \
			.offset      = 0, \
			.size        = 0x100000, \
		}, \
		[1] = { \
			.name        = "MassStorage0", \
			.offset      = 0x100000, \
			.size        = MTDPART_SIZ_FULL,  /* rest of flash*/ \
		}, \
	}

#endif /*__PLAT_PART_TABLE_H*/

