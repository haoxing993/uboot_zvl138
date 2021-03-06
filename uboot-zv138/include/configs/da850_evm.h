/*
 * Copyright (C) 2008 Texas Instruments, Inc <www.ti.com>
 * 
 * Based on davinci_dvevm.h. Original Copyrights follow:
 *
 * Copyright (C) 2007 Sergey Kubushyn <ksi@koi8.net>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H
#include <asm/sizes.h>
#include <asm/arch/hardware.h>

/*=======*/
/* Board */
/*=======*/
#undef	CONFIG_USE_SPIFLASH
#define	CONFIG_SYS_USE_NAND
#undef	CONFIG_SYS_USE_NOR 

#define CONFIG_SYS_I2C_EXPANDER_ADDR	0x20

/*===================*/
/* I2C Configuration */
/*===================*/
#define CONFIG_HARD_I2C
#define CONFIG_DRIVER_DAVINCI_I2C
#define CONFIG_SYS_I2C_SPEED		25000
/* Bogus, master-only in U-Boot */
#define CONFIG_SYS_I2C_SLAVE		10

/*===================*/
/* SoC Configuration */
/*===================*/
#define CONFIG_ARM926EJS	
/* Arm Clock */
#define CONFIG_SYS_CLK_FREQ		clk_get(DAVINCI_ARM_CLKID)
#define CFG_OSCIN_FREQ			24000000
/* timer to be used */
#define CONFIG_SYS_TIMERBASE		DAVINCI_TIMER0_BASE	
/* Timer Input clock freq */
#define CONFIG_SYS_HZ_CLOCK		clk_get(DAVINCI_AUXCLK_CLKID)
#define CONFIG_SYS_HZ			1000
/* U-Boot is _always_ loaded by a bootloader... */
#undef CONFIG_SKIP_LOWLEVEL_INIT
/*... to a proper address, init done */
#define CONFIG_SKIP_RELOCATE_UBOOT

/*=============*/
/* Memory Info */
/*=============*/
/* malloc() len */
#define CONFIG_SYS_MALLOC_LEN		(1 * 1024 * 1024)
/* reserved for initial data */
#define CONFIG_SYS_GBL_DATA_SIZE	128
/* DDR Start */
#define PHYS_SDRAM_1			DAVINCI_DDR_EMIF_DATA_BASE
/* SDRAM size 64MB */
#define PHYS_SDRAM_1_SIZE		0x04000000
/* memtest start address */
#define CONFIG_SYS_MEMTEST_START	PHYS_SDRAM_1
/* 16MB RAM test */
#define CONFIG_SYS_MEMTEST_END		(PHYS_SDRAM_1 + 16*1024*1024)
/* we have 1 bank of DRAM */
#define CONFIG_NR_DRAM_BANKS		1
/* regular stack */
#define CONFIG_STACKSIZE		(256*1024)	

/*====================*/
/* Serial Driver info */
/*====================*/
#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	4		
/* use first UART for console */
#define CONFIG_CONS_INDEX		1
/* Base address of console UART */
#define CONFIG_SYS_NS16550_COM1		DAVINCI_UART2_BASE//DAVINCI_UART1_BASE
/* Input clock to NS16550 */
#define CONFIG_SYS_NS16550_CLK		clk_get(DAVINCI_UART2_CLKID)
/* Default baud rate */
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }


/*==================================*/
/* SPI Flash Configuration	    */
/*==================================*/
#define CONFIG_SPI
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_DAVINCI_SPI
#define CFG_SPI_BASE			DAVINCI_SPI1_BASE
#define CFG_SPI_CLK			clk_get(DAVINCI_SPI1_CLKID)
#define CONFIG_SF_DEFAULT_SPEED		50000000

/*==================================*/
/* Network & Ethernet Configuration */
/*==================================*/
#define CONFIG_DRIVER_TI_EMAC
#undef CONFIG_DRIVER_TI_EMAC_USE_RMII
#define CONFIG_MII
#define CONFIG_BOOTP_DEFAULT
#define CONFIG_BOOTP_DNS
#define CONFIG_BOOTP_DNS2
#define CONFIG_BOOTP_SEND_HOSTNAME
#define CONFIG_NET_RETRY_COUNT		10
#define CONFIG_NET_MULTI
//#define CONFIG_NETCONSOLE

/*=====================*/
/* Flash & Environment */
/*=====================*/
#ifdef CONFIG_SYS_USE_NAND
#undef CONFIG_ENV_IS_IN_FLASH
#define CONFIG_SYS_NO_FLASH
#define CONFIG_ENV_IS_IN_NAND
#define CONFIG_ENV_SIZE			SZ_128K
#define CONFIG_NAND_CS			1
#define CONFIG_SYS_NAND_BASE		DAVINCI_ASYNC_EMIF_DATA_CE3_BASE
#define CONFIG_CLE_MASK			0x10
#define CONFIG_ALE_MASK			0x8
#undef CONFIG_SYS_NAND_HW_ECC
#define CONFIG_SYS_NAND_4BIT_HW_ECC
#define CONFIG_SYS_NAND_USE_FLASH_BBT
#define CFG_DAVINCI_STD_NAND_LAYOUT
/* Max number of NAND devices */
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define NAND_MAX_CHIPS			1
/* Block 0--not used by bootcode */
#define CONFIG_ENV_OFFSET		0x0	
#define CONFIG_JFFS2_NAND
#endif

#ifdef CONFIG_SYS_USE_NOR
#define CONFIG_ENV_IS_IN_FLASH
#undef CONFIG_SYS_NO_FLASH
#define CONFIG_FLASH_CFI_DRIVER
#define CONFIG_SYS_FLASH_CFI
#define CONFIG_SYS_FLASH_PROTECTION
#define CONFIG_SYS_MAX_FLASH_BANKS	1
#define CONFIG_SYS_FLASH_SECT_SZ	0x20000	
#define CONFIG_ENV_OFFSET		(CONFIG_SYS_FLASH_SECT_SZ * 3)
#define CONFIG_SYS_FLASH_BASE		DAVINCI_ASYNC_EMIF_DATA_CE2_BASE 
#define PHYS_FLASH_SIZE			0x800000	
#define CONFIG_SYS_MAX_FLASH_SECT	(PHYS_FLASH_SIZE/CONFIG_SYS_FLASH_SECT_SZ) + 3
#define CONFIG_ENV_SECT_SIZE		CONFIG_SYS_FLASH_SECT_SZ	
#endif

#ifdef CONFIG_USE_SPIFLASH
#undef CONFIG_ENV_IS_IN_FLASH
#undef CONFIG_ENV_IS_IN_NAND
#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_SIZE			SZ_64K
#define CONFIG_ENV_OFFSET		SZ_256K
#define CONFIG_ENV_SECT_SIZE		SZ_64K
#define CONFIG_SYS_NO_FLASH
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#endif


/*==============================*/
/* USB  configuration           */
/*==============================*/
#define CONFIG_MUSB		/* Generic MUSB controller */
#define CONFIG_USB_DA8XX	/* Platform hookup to MUSB controller */ 
#define CONFIG_USB_STORAGE	/* MSC class support */

/*==============================*/
/* U-Boot general configuration */
/*==============================*/
/* No IRQ/FIQ in U-Boot */
#undef 	CONFIG_USE_IRQ
#define CONFIG_MISC_INIT_R
#undef CONFIG_BOOTDELAY
#define CONFIG_BOOTFILE			"uImage"
/* Monitor Command Prompt */
#define CONFIG_SYS_PROMPT		"U-Boot-zv138 > "
/* Console I/O Buffer Size */
#define CONFIG_SYS_CBSIZE		1024
/* Print buffer sz */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
						sizeof(CONFIG_SYS_PROMPT) + 16)
/* max number of command args */
#define CONFIG_SYS_MAXARGS		16
/* Boot Argument Buffer Size */
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE
/* default Linux kernel load address */
#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_MEMTEST_START + 0x700000)
#define CONFIG_VERSION_VARIABLE
/* Won't work with hush so far, may be later */
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_CMDLINE_EDITING
#define CONFIG_SYS_LONGHELP
#define CONFIG_CRC32_VERIFY
#define CONFIG_MX_CYCLIC

/*===================*/
/* Linux Information */
/*===================*/
#define LINUX_BOOT_PARAM_ADDR		(CONFIG_SYS_MEMTEST_START + 0x100)
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_BOOTCOMMAND	"nboot.e 0xc0700000 0 0x300000; bootm"
#define CONFIG_BOOTARGS			"noinitrd root=/dev/mtdblock4 console=ttyS2,115200n8 rootfstype=jffs2 ip=192.168.253.123:192.168.253.1:255.255.255.0"
#define CONFIG_BOOTDELAY		3
#define CONFIG_SERVERIP			192.168.253.61
#define CONFIG_IPADDR				192.168.253.123

/*=================*/
/* U-Boot commands */
/*=================*/
#include <config_cmd_default.h>
#define CONFIG_CMD_ENV
#define CONFIG_CMD_ASKENV
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_DIAG
#define CONFIG_CMD_MII
#define CONFIG_CMD_PING
#define CONFIG_CMD_SAVES
#define CONFIG_CMD_MEMORY
#define CONFIG_CMD_BDI    
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_SETGETDCR
#define CONFIG_CMD_EEPROM
#define CONFIG_CMD_NET

#ifdef CONFIG_SYS_USE_NAND
#undef CONFIG_CMD_FLASH
#undef CONFIG_CMD_IMLS
#define CONFIG_CMD_NAND
#endif

#ifdef CONFIG_USE_SPIFLASH
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_FLASH
#define CONFIG_CMD_SF
#endif

#if !defined(CONFIG_SYS_USE_NAND) && !defined(CONFIG_SYS_USE_NOR) && !defined(CONFIG_USE_SPIFLASH)
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_SYS_NO_FLASH
#define CONFIG_ENV_SIZE				SZ_16K
#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_FLASH
#undef CONFIG_CMD_ENV
#endif

#ifdef CONFIG_USB_DA8XX
#define CONFIG_CMD_USB		/* inclue support for usb       */
#define CONFIG_CMD_STORAGE	/* inclue support for usb       */
#define CONFIG_CMD_FAT		/* inclue support for FAT/storage*/
#define CONFIG_DOS_PARTITION	/* inclue support for FAT/storage*/
#endif


#endif /* __CONFIG_H */
