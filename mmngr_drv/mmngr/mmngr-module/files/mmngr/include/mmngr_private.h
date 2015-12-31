/*************************************************************************/ /*
 MMNGR

 Copyright (C) 2015 Renesas Electronics Corporation

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

#define DEVFILE "/dev/rgnmm"

struct MM_DRVDATA {
	struct device *mm_dev;
	struct device *mm_dev_reserve;
	unsigned long	reserve_size;
	phys_addr_t	reserve_phy_addr;
	unsigned long	reserve_kernel_virt_addr;
};

struct MM_PARAM {
	size_t		size;
	phys_addr_t	phy_addr;
	unsigned int	hard_addr;
	unsigned long	user_virt_addr;
	unsigned long	kernel_virt_addr;
	unsigned int	flag;
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
	uint32_t ena;
};

struct LOSSY_DATA {
	uint32_t fmt;
	struct BM *bm_lossy;
};

extern struct cma *rcar_gen3_dma_contiguous;

#define MM_IOC_MAGIC 'm'
#define MM_IOC_ALLOC	_IOWR(MM_IOC_MAGIC, 0, struct MM_PARAM)
#define MM_IOC_FREE	_IOWR(MM_IOC_MAGIC, 1, struct MM_PARAM)
#define MM_IOC_SET	_IOWR(MM_IOC_MAGIC, 2, struct MM_PARAM)
#define MM_IOC_GET	_IOWR(MM_IOC_MAGIC, 3, struct MM_PARAM)
#define MM_IOC_ALLOC_CO	_IOWR(MM_IOC_MAGIC, 4, struct MM_PARAM)
#define MM_IOC_FREE_CO	_IOWR(MM_IOC_MAGIC, 5, struct MM_PARAM)
#define MM_IOC_SHARE	_IOWR(MM_IOC_MAGIC, 6, struct MM_PARAM)

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
static int find_lossy_entry(unsigned int flag, int entry);

#ifdef MMNGR_SALVATORX
	#define MM_OMXBUF_ADDR	(0x70000000UL)
	#define MM_OMXBUF_SIZE	(256 * 1024 * 1024)
	#define MM_LOSSYBUF_ADDR	(0x55000000UL)
	#define MM_LOSSYBUF_SIZE	(48 * 1024 * 1024)
#endif

#ifdef MMNGR_SSP_ENABLE
#ifdef MMNGR_SALVATORX
#define MM_SSPBUF_ADDR	(0x54000000)
#define MM_SSPBUF_SIZE	(16 * 1024 * 1024)
#endif
#endif

#define MM_KERNEL_RESERVE_SIZE	(256 * 1024 * 1024)
#define	MM_CO_ORDER		(12)

#define MM_LOSSY_INFO_MAGIC		(0x12345678UL)
#define MM_LOSSY_ADDR_MASK		(0x0003FFFFUL)  /* [17:0] */
#define MM_LOSSY_FMT_MASK		(0x60000000UL)  /* [30:29] */
#define MM_LOSSY_ENABLE_MASK		(0x80000000UL)  /* [31] */

#endif	/* __MMNGR_PRIVATE_H__ */
