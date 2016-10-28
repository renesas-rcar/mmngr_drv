/*************************************************************************/ /*
 MMNGR

 Copyright (C) 2015-2016 Renesas Electronics Corporation

 License        Dual MIT/GPLv2

 The contents of this file are subject to the MIT license as set out below.

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 Alternatively, the contents of this file may be used under the terms of
 the GNU General Public License Version 2 ("GPL") in which case the provisions
 of GPL are applicable instead of those above.

 If you wish to allow use of your version of this file only under the terms of
 GPL, and not to allow others to use your version of this file under the terms
 of the MIT license, indicate your decision by deleting the provisions above
 and replace them with the notice and other provisions required by GPL as set
 out in the file called "GPL-COPYING" included in this distribution. If you do
 not delete the provisions above, a recipient may use your version of this file
 under the terms of either the MIT license or GPL.

 This License is also included in this distribution in the file called
 "MIT-COPYING".

 EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
 PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


 GPLv2:
 If you wish to use this file under the terms of GPL, following terms are
 effective.

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; version 2 of the License.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/ /*************************************************************************/
#ifndef __MMNGR_PRIVATE_H__
#define __MMNGR_PRIVATE_H__

#include "mmngr_private_cmn.h"

struct MM_DRVDATA {
	struct device *mm_dev;
	struct device *mm_dev_reserve;
	unsigned long	reserve_size;
	phys_addr_t	reserve_phy_addr;
	unsigned long	reserve_kernel_virt_addr;
};

struct BM {
	phys_addr_t	top_phy_addr;
	unsigned long	order;
	unsigned long	end_bit;
	unsigned long	*bits;
};

struct LOSSY_INFO {
	uint32_t magic;
	uint32_t a0;
	uint32_t b0;
};

struct LOSSY_DATA {
	uint32_t fmt;
	struct BM *bm_lossy;
};

#ifdef MMNGR_IPMMU_PMB_ENABLE
enum {
	DO_IOREMAP,
	DO_IOUNMAP,
	ENABLE_PMB,
	DISABLE_PMB,
	ENABLE_UTLB,
	DISABLE_UTLB,
	SET_PMB_AREA,
	CLEAR_PMB_AREA,
	BACKUP_PMB_REGS,
	RESTORE_PMB_REGS,
	PRINT_PMB_DEBUG,
};

struct hw_register {
	char *reg_name;
	unsigned int reg_offset;
	unsigned int reg_val;
};

struct ip_master {
	char *ip_name;
	unsigned int utlb_no;
};

struct p2v_map {
	unsigned int impmba;
	unsigned int impmbd;
};

struct pmb_p2v_map {
	struct p2v_map *p2v_map;
	unsigned int map_count;
};

struct pmb_table_map {
	unsigned long table_size;
	unsigned int multiple_of_16;
	unsigned int impmbd_sz;
	unsigned int table_count;
};

struct rcar_ipmmu {
	char *ipmmu_name;
	unsigned int base_addr;
	void __iomem *virt_addr;
	unsigned int reg_count;
	unsigned int masters_count;
	struct hw_register *ipmmu_reg;
	struct ip_master *ip_masters;
};

struct rcar_ipmmu_data {
	struct rcar_ipmmu **ipmmu_data;
};

#endif

extern struct cma *rcar_gen3_dma_contiguous;

#ifdef CONFIG_COMPAT
struct COMPAT_MM_PARAM {
	compat_size_t	size;
	compat_u64	phy_addr;
	compat_uint_t	hard_addr;
	compat_ulong_t	user_virt_addr;
	compat_ulong_t	kernel_virt_addr;
	compat_uint_t	flag;
};

#define COMPAT_MM_IOC_ALLOC	_IOWR(MM_IOC_MAGIC, 0, struct COMPAT_MM_PARAM)
#define COMPAT_MM_IOC_FREE	_IOWR(MM_IOC_MAGIC, 1, struct COMPAT_MM_PARAM)
#define COMPAT_MM_IOC_SET	_IOWR(MM_IOC_MAGIC, 2, struct COMPAT_MM_PARAM)
#define COMPAT_MM_IOC_GET	_IOWR(MM_IOC_MAGIC, 3, struct COMPAT_MM_PARAM)
#define COMPAT_MM_IOC_ALLOC_CO	_IOWR(MM_IOC_MAGIC, 4, struct COMPAT_MM_PARAM)
#define COMPAT_MM_IOC_FREE_CO	_IOWR(MM_IOC_MAGIC, 5, struct COMPAT_MM_PARAM)
#define COMPAT_MM_IOC_SHARE	_IOWR(MM_IOC_MAGIC, 6, struct COMPAT_MM_PARAM)
#endif

#define DEVNAME		"rgnmm"
#define DRVNAME		DEVNAME
#define CLSNAME		DEVNAME
#define DEVNUM		1

static int mm_ioc_alloc(struct device *mm_dev,
			int __user *in,
			struct MM_PARAM *out);
static void mm_ioc_free(struct device *mm_dev, struct MM_PARAM *p);
static int mm_ioc_set(int __user *in, struct MM_PARAM *out);
static int mm_ioc_get(struct MM_PARAM *in, int __user *out);
static int alloc_bm(struct BM *pb,
		phys_addr_t top_phy_addr,
		unsigned long size,
		unsigned long order);
static void free_bm(struct BM *pb);
static int mm_ioc_alloc_co(struct BM *pb, int __user *in, struct MM_PARAM *out);
static int mm_ioc_alloc_co_select(int __user *in, struct MM_PARAM *out);
static void mm_ioc_free_co(struct BM *pb, struct MM_PARAM *p);
static void mm_ioc_free_co_select(struct MM_PARAM *p);
static int mm_ioc_share(int __user *in, struct MM_PARAM *out);
static void mmngr_dev_set_cma_area(struct device *dev, struct cma *cma);
static int init_lossy_info(void);
static int find_lossy_entry(unsigned int flag, int *entry);
static int _parse_reserved_mem_dt(char *dt_path,
			u64 *addr, u64 *size);
static int parse_reserved_mem_dt(void);
#ifdef MMNGR_IPMMU_PMB_DISABLE
static int validate_memory_map(void);

#if defined(MMNGR_SALVATORX) || defined(MMNGR_KRIEK)
#define MM_OMXBUF_ADDR		(0x70000000UL)
#define MM_OMXBUF_SIZE		(256 * 1024 * 1024)
#endif
#endif /* MMNGR_IPMMU_PMB_DISABLE */

#define	MM_CO_ORDER		(12)

#ifdef MMNGR_SSP_ENABLE
#if defined(MMNGR_SALVATORX) || defined(MMNGR_KRIEK)
#define MM_SSPBUF_ADDR		(0x53000000UL)
#define MM_SSPBUF_SIZE		(16 * 1024 * 1024)
#endif
#endif

#define MM_LOSSY_INFO_MAGIC		(0x12345678UL)
#define MM_LOSSY_ADDR_MASK		(0x0003FFFFUL)  /* [17:0] */
#define MM_LOSSY_FMT_MASK		(0x60000000UL)  /* [30:29] */
#define MM_LOSSY_ENABLE_MASK		(0x80000000UL)  /* [31] */
#define MM_LOSSY_SHARED_MEM_ADDR	(0x47FD7000UL)
#define MM_LOSSY_SHARED_MEM_SIZE	(16 * sizeof(struct LOSSY_INFO))

#if defined(MMNGR_SSP_ENABLE) && defined(MMNGR_IPMMU_PMB_ENABLE)
#error "Have not support IPMMU(PMB) for SSPBUF yet"
#endif

#ifdef MMNGR_IPMMU_PMB_ENABLE
/* IPMMU (PMB mode) */
static int map_register(void);
static void unmap_register(void);
static void enable_pmb(void);
static void enable_utlb(void);
static void set_pmb_area(void);
static void pmb_debuginfo(void);
static int pmb_init(void);
static void pmb_exit(void);
static int __handle_registers(struct rcar_ipmmu *ipmmu, unsigned int handling);
static int handle_registers(struct rcar_ipmmu **ipmmu, unsigned int handling);
static phys_addr_t pmb_virt2phys(unsigned int ipmmu_virt_addr);

#define IPMMUVC0_BASE		(0xFE6B0000)
#define IPMMUVC1_BASE		(0xFE6F0000)
#define IPMMUVP_BASE		(0xFE990000)
#define IPMMUVI_BASE		(0xFEBD0000)

#define MAX_PMB_TABLE		(16)
#define IMPCTR_OFFSET		(0x200)
#define IMPSTR_OFFSET		(0x208)
#define IMPEAR_OFFSET		(0x20C)
#define IMPMBAn_OFFSET(n)	(0x280 + 0x4  * n)
#define IMPMBDn_OFFSET(n)	(0x2C0 + 0x4  * n)
#define MAX_UTLB		(32)
#define IMUCTRn_OFFSET(n)	(0x300 + 0x10 * n)

#define REG_SIZE		IMUCTRn_OFFSET(MAX_UTLB)

#define IMPCTR_VAL		(0x00000001)
#define IMUCTR_VAL		(0x00000081)
#define IMPMBAn_V_BIT		(0x00000100)
#define IMPMBDn_V_BIT		(0x00000100)

#define LOWER_PPN_MASK		(0x00FF000000UL)
#define UPPER_PPN_MASK		(0xFF00000000UL)
#define IMPMBA_VALUE(virt_addr)	(virt_addr)
#define IMPMBD_VALUE(phys_addr) ((phys_addr & LOWER_PPN_MASK) \
				| ((phys_addr & UPPER_PPN_MASK) >> 16))

#define MM_OMXBUF_ADDR		(0xC0000000)

/* IPMMU virtual address */
#define CMA_1ST_VIRT_BASE_ADDR	(mm_common_reserve_addr)
#define CMA_2ND_VIRT_BASE_ADDR	(MM_OMXBUF_ADDR)
#define CMA_LOSSY_VIRT_BASE_ADDR	(mm_lossybuf_addr)

#endif /* MMNGR_IPMMU_PMB_ENABLE */

#endif	/* __MMNGR_PRIVATE_H__ */
