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
#include <linux/compat.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/bitmap.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/dma-contiguous.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/sizes.h>
#include <linux/sys_soc.h>

#include "mmngr_public.h"
#include "mmngr_private.h"

static spinlock_t		lock;
static struct BM		bm;
static struct BM		bm_ssp;
static struct LOSSY_DATA	lossy_entries[16];
static struct MM_DRVDATA	*mm_drvdata;
static struct cma		*mm_cma_area;
static u64			mm_common_reserve_addr;
static u64			mm_common_reserve_size;
static u64			mm_kernel_reserve_addr;
static u64			mm_kernel_reserve_size;
static u64			mm_lossybuf_addr;
static u64			mm_lossybuf_size;
static bool			have_lossy_entries;
#ifdef MMNGR_SSP_ENABLE
static bool			is_sspbuf_valid = false;
#endif

/* Attribute structs describing Salvator-X revisions */
/* H3 WS1.0 and WS1.1 */
static const struct soc_device_attribute r8a7795es1[]  = {
	{ .soc_id = "r8a7795", .revision = "ES1.*" },
	{}
};

#ifdef MMNGR_IPMMU_PMB_ENABLE
/* IPMMU (PMB mode) */
static struct p2v_map p2v_mapping[MAX_PMB_TABLE];
static struct pmb_p2v_map pmb_p2v_mapping = {
	.p2v_map = p2v_mapping,
	/* Actual map_count will be set during MMNGR init */
};

static struct rcar_ipmmu **rcar_gen3_ipmmu;

static struct pmb_table_map pmb_table_mapping[] = {
	{SZ_512M,	32, 0x90, 0}, /* 512MB table */
	{SZ_128M,	 8, 0x80, 0}, /* 128MB table */
	{SZ_64M,	 4, 0x10, 0}, /*  64MB table */
	{SZ_16M,	 1, 0x00, 0}, /*  16MB table */
};

static struct hw_register ipmmu_ip_regs[] = {
	{"IMPCTR",	IMPCTR_OFFSET,		0},
	{"IMPSTR",	IMPSTR_OFFSET,		0},
	{"IMPEAR",	IMPEAR_OFFSET,		0},
	{"IMPMBA0",	IMPMBAn_OFFSET(0),	0},
	{"IMPMBD0",	IMPMBDn_OFFSET(0),	0},
	{"IMPMBA1",	IMPMBAn_OFFSET(1),	0},
	{"IMPMBD1",	IMPMBDn_OFFSET(1),	0},
	{"IMPMBA2",	IMPMBAn_OFFSET(2),	0},
	{"IMPMBD2",	IMPMBDn_OFFSET(2),	0},
	{"IMPMBA3",	IMPMBAn_OFFSET(3),	0},
	{"IMPMBD3",	IMPMBDn_OFFSET(3),	0},
	{"IMPMBA4",	IMPMBAn_OFFSET(4),	0},
	{"IMPMBD4",	IMPMBDn_OFFSET(4),	0},
	{"IMPMBA5",	IMPMBAn_OFFSET(5),	0},
	{"IMPMBD5",	IMPMBDn_OFFSET(5),	0},
	{"IMPMBA6",	IMPMBAn_OFFSET(6),	0},
	{"IMPMBD6",	IMPMBDn_OFFSET(6),	0},
	{"IMPMBA7",	IMPMBAn_OFFSET(7),	0},
	{"IMPMBD7",	IMPMBDn_OFFSET(7),	0},
	{"IMPMBA8",	IMPMBAn_OFFSET(8),	0},
	{"IMPMBD8",	IMPMBDn_OFFSET(8),	0},
	{"IMPMBA9",	IMPMBAn_OFFSET(9),	0},
	{"IMPMBD9",	IMPMBDn_OFFSET(9),	0},
	{"IMPMBA10",	IMPMBAn_OFFSET(10),	0},
	{"IMPMBD10",	IMPMBDn_OFFSET(10),	0},
	{"IMPMBA11",	IMPMBAn_OFFSET(11),	0},
	{"IMPMBD11",	IMPMBDn_OFFSET(11),	0},
	{"IMPMBA12",	IMPMBAn_OFFSET(12),	0},
	{"IMPMBD12",	IMPMBDn_OFFSET(12),	0},
	{"IMPMBA13",	IMPMBAn_OFFSET(13),	0},
	{"IMPMBD13",	IMPMBDn_OFFSET(13),	0},
	{"IMPMBA14",	IMPMBAn_OFFSET(14),	0},
	{"IMPMBD14",	IMPMBDn_OFFSET(14),	0},
	{"IMPMBA15",	IMPMBAn_OFFSET(15),	0},
	{"IMPMBD15",	IMPMBDn_OFFSET(15),	0},
	/*
	 * IMUCTRn_OFFSET(n) is not defined here
	 * The register is calculated base on IP utlb_no value
	 */
};

/* R-Car H3 (R8A7795 ES1.x) */
static struct ip_master r8a7795es1_ipmmuvc0_masters[] = {
	{"FCP-CS", 0},
};

static struct rcar_ipmmu r8a7795es1_ipmmuvc0 = {
	.ipmmu_name	= "IPMMUVC0",
	.base_addr	= IPMMUVC0_BASE,
	.reg_count	= ARRAY_SIZE(ipmmu_ip_regs),
	.masters_count	= ARRAY_SIZE(r8a7795es1_ipmmuvc0_masters),
	.ipmmu_reg	= ipmmu_ip_regs,
	.ip_masters	= r8a7795es1_ipmmuvc0_masters,
};

static struct ip_master r8a7795es1_ipmmuvc1_masters[] = {
	{"FCP-CI ch0",	4},
	{"FCP-CI ch1",	5},
};

static struct rcar_ipmmu r8a7795es1_ipmmuvc1 = {
	.ipmmu_name	= "IPMMUVC1",
	.base_addr	= IPMMUVC1_BASE,
	.reg_count	= ARRAY_SIZE(ipmmu_ip_regs),
	.masters_count	= ARRAY_SIZE(r8a7795es1_ipmmuvc1_masters),
	.ipmmu_reg	= ipmmu_ip_regs,
	.ip_masters	= r8a7795es1_ipmmuvc1_masters,
};

static struct ip_master r8a7795es1_ipmmuvp_masters[] = {
	{"FCP-F ch0",	0},
	{"FCP-F ch1",	1},
	{"FCP-F ch2",	2},
};

static struct rcar_ipmmu r8a7795es1_ipmmuvp = {
	.ipmmu_name	= "IPMMUVP",
	.base_addr	= IPMMUVP_BASE,
	.reg_count	= ARRAY_SIZE(ipmmu_ip_regs),
	.masters_count	= ARRAY_SIZE(r8a7795es1_ipmmuvp_masters),
	.ipmmu_reg	= ipmmu_ip_regs,
	.ip_masters	= r8a7795es1_ipmmuvp_masters,
};

static struct rcar_ipmmu *r8a7795es1_ipmmu[] = {
	&r8a7795es1_ipmmuvp,
	&r8a7795es1_ipmmuvc0,
	&r8a7795es1_ipmmuvc1,
	NULL, /* End of list */
};

/* R-Car H3 (R8A7795 ES2.0) */
static struct ip_master r8a7795_ipmmuvc1_masters[] = {
	{"FCP-CS osid0", 8},
	{"FCP-CS osid4", 12},
};

static struct rcar_ipmmu r8a7795_ipmmuvc1 = {
	.ipmmu_name	= "IPMMUVC1",
	.base_addr	= IPMMUVC1_BASE,
	.reg_count	= ARRAY_SIZE(ipmmu_ip_regs),
	.masters_count	= ARRAY_SIZE(r8a7795_ipmmuvc1_masters),
	.ipmmu_reg	= ipmmu_ip_regs,
	.ip_masters	= r8a7795_ipmmuvc1_masters,
};

static struct ip_master r8a7795_ipmmuvp0_masters[] = {
	{"FCP-F ch0",	0},
};

static struct rcar_ipmmu r8a7795_ipmmuvp0 = {
	.ipmmu_name	= "IPMMUVP0",
	.base_addr	= IPMMUVP0_BASE,
	.reg_count	= ARRAY_SIZE(ipmmu_ip_regs),
	.masters_count	= ARRAY_SIZE(r8a7795_ipmmuvp0_masters),
	.ipmmu_reg	= ipmmu_ip_regs,
	.ip_masters	= r8a7795_ipmmuvp0_masters,
};

static struct ip_master r8a7795_ipmmuvp1_masters[] = {
	{"FCP-F ch1",	1},
};

static struct rcar_ipmmu r8a7795_ipmmuvp1 = {
	.ipmmu_name	= "IPMMUVP1",
	.base_addr	= IPMMUVP1_BASE,
	.reg_count	= ARRAY_SIZE(ipmmu_ip_regs),
	.masters_count	= ARRAY_SIZE(r8a7795_ipmmuvp1_masters),
	.ipmmu_reg	= ipmmu_ip_regs,
	.ip_masters	= r8a7795_ipmmuvp1_masters,
};

static struct rcar_ipmmu *r8a7795_ipmmu[] = {
	&r8a7795_ipmmuvp0,
	&r8a7795_ipmmuvp1,
	&r8a7795_ipmmuvc1,
	NULL, /* End of list */
};

/* R-Car M3 (R8A7796) */
static struct ip_master r8a7796_ipmmuvc0_masters[] = {
	{"FCP-CI",  4},
	{"FCP-CS",  8},
	{"FCP-F",  16},
};

static struct rcar_ipmmu r8a7796_ipmmuvc0 = {
	.ipmmu_name	= "IPMMUVC0",
	.base_addr	= IPMMUVC0_BASE,
	.reg_count	= ARRAY_SIZE(ipmmu_ip_regs),
	.masters_count	= ARRAY_SIZE(r8a7796_ipmmuvc0_masters),
	.ipmmu_reg	= ipmmu_ip_regs,
	.ip_masters	= r8a7796_ipmmuvc0_masters,
};

static struct rcar_ipmmu *r8a7796_ipmmu[] = {
	&r8a7796_ipmmuvc0,
	NULL, /* End of list */
};

#endif

static int mm_ioc_alloc(struct device *mm_dev,
			int __user *in,
			struct MM_PARAM *out)
{
	int		ret = 0;
	struct MM_PARAM	tmp;

	if (copy_from_user(&tmp, (void __user *)in, sizeof(struct MM_PARAM))) {
		pr_err("%s EFAULT\n", __func__);
		ret = -EFAULT;
		return ret;
	}
	out->size = tmp.size;
	out->kernel_virt_addr = (unsigned long)dma_alloc_coherent(mm_dev,
						out->size,
						(dma_addr_t *)&out->phy_addr,
						GFP_KERNEL);
	if (!out->kernel_virt_addr) {
		pr_err("%s ENOMEM\n", __func__);
		ret = -ENOMEM;
		out->phy_addr = 0;
		out->hard_addr = 0;
		return ret;
	}
	out->hard_addr = (unsigned int)out->phy_addr;

	out->flag = tmp.flag;

	return ret;
}

static void mm_ioc_free(struct device *mm_dev, struct MM_PARAM *p)
{
	dma_free_coherent(mm_dev, p->size, (void *)p->kernel_virt_addr,
			(dma_addr_t)p->phy_addr);
	memset(p, 0, sizeof(struct MM_PARAM));
}

static int mm_ioc_set(int __user *in, struct MM_PARAM *out)
{
	int		ret = 0;
	struct MM_PARAM	tmp;

	if (copy_from_user(&tmp, in, sizeof(struct MM_PARAM))) {
		ret = -EFAULT;
		return ret;
	}

	out->user_virt_addr = tmp.user_virt_addr;

	return ret;
}

static int mm_ioc_get(struct MM_PARAM *in, int __user *out)
{
	int	ret = 0;

	if (copy_to_user(out, in, sizeof(struct MM_PARAM))) {
		ret = -EFAULT;
		return ret;
	}

	return ret;
}

static int alloc_bm(struct BM *pb,
		phys_addr_t top_phy_addr,
		unsigned long size,
		unsigned long order)
{
	unsigned long	nbits;
	unsigned long	nbytes;

	pr_debug("MMNGR: CO from 0x%llx to 0x%llx\n",
	top_phy_addr, top_phy_addr + size - 1);

	if (pb == NULL)
		return -1;

	nbits = (size + ((1UL << order) - 1)) >> order;
	nbytes = (nbits + BITS_PER_BYTE - 1) / BITS_PER_BYTE;
	pb->bits = kzalloc(nbytes, GFP_KERNEL);
	if (pb->bits == NULL)
		return -1;
	pb->order = order;
	pb->top_phy_addr = top_phy_addr;
	pb->end_bit = nbits;

	return 0;
}

static void free_bm(struct BM *pb)
{
	kfree(pb->bits);
}

static int mm_ioc_alloc_co(struct BM *pb, int __user *in, struct MM_PARAM *out)
{
	int		ret = 0;
	unsigned long	nbits;
	unsigned long	start_bit;
	struct MM_PARAM	tmp;

	if (copy_from_user(&tmp, in, sizeof(struct MM_PARAM))) {
		pr_err("%s EFAULT\n", __func__);
		ret = -EFAULT;
		return ret;
	}

	out->size = tmp.size;
	nbits = (out->size + (1UL << pb->order) - 1) >> pb->order;

	spin_lock(&lock);
	start_bit = bitmap_find_next_zero_area(pb->bits, pb->end_bit, 0,
						nbits, 0);
	if (start_bit >= pb->end_bit) {
		pr_err("start(%ld), end(%ld)\n", start_bit,
			pb->end_bit);
		spin_unlock(&lock);
		out->phy_addr = 0;
		out->hard_addr = 0;
		return -ENOMEM;
	}
	bitmap_set(pb->bits, start_bit, nbits);
	spin_unlock(&lock);

	out->phy_addr = pb->top_phy_addr + (start_bit << pb->order);
	out->hard_addr = (unsigned int)out->phy_addr;
	out->flag = tmp.flag;

	return 0;
}

static int find_lossy_entry(unsigned int flag, int *entry)
{
	uint32_t	i, fmt;
	int		ret;

	if (!have_lossy_entries)
		return -EINVAL; /* Not supported */

	fmt = ((flag & 0xF0) >> 4) - 1;
	pr_debug("Requested format 0x%x.\n", fmt);

	for (i = 0; i < 16; i++) {
		if (lossy_entries[i].fmt == fmt)
			break;
	}

	if (i < 16) {
		*entry = i;
		ret = 0;
		pr_debug("Found entry no.%d\n", i);
	} else {
		*entry = -1;
		ret = -EINVAL; /* Not supported */
		pr_debug("No entry is found!\n");
	}

	return ret;
}

static int mm_ioc_alloc_co_select(int __user *in, struct MM_PARAM *out)
{
	int		ret = 0;
	int		entry = 0;
	struct MM_PARAM	tmp;
	struct device	*mm_dev;

	mm_dev = mm_drvdata->mm_dev;

	if (copy_from_user(&tmp, in, sizeof(struct MM_PARAM))) {
		pr_err("%s EFAULT\n", __func__);
		ret = -EFAULT;
		return ret;
	}

	if (tmp.flag == MM_CARVEOUT)
		ret = mm_ioc_alloc_co(&bm, in, out);
#ifndef MMNGR_SSP_ENABLE
	else if (tmp.flag == MM_CARVEOUT_SSP) {
		pr_err("%s EINVAL\n", __func__);
		ret = -EINVAL;
	}
#else
	else if (tmp.flag == MM_CARVEOUT_SSP)
	if (is_sspbuf_valid)
		ret = mm_ioc_alloc_co(&bm_ssp, in, out);
	else
		ret = -ENOMEM;
#endif
	else if ((tmp.flag & 0xF) == MM_CARVEOUT_LOSSY) {
		ret = find_lossy_entry(tmp.flag, &entry);
		if (ret)
			return ret;
		ret = mm_ioc_alloc_co(lossy_entries[entry].bm_lossy, in, out);
	}

	return ret;
}

static void mm_ioc_free_co(struct BM *pb, struct MM_PARAM *p)
{
	unsigned long	nbits;
	unsigned long	start_bit;

	start_bit = (p->phy_addr - pb->top_phy_addr) >> pb->order;
	nbits = (p->size + (1UL << pb->order) - 1) >> pb->order;

	spin_lock(&lock);
	bitmap_clear(pb->bits, start_bit, nbits);
	spin_unlock(&lock);
	memset(p, 0, sizeof(struct MM_PARAM));
}

static void mm_ioc_free_co_select(struct MM_PARAM *p)
{
	struct device	*mm_dev;
	int		entry = 0;

	mm_dev = mm_drvdata->mm_dev;

	if (p->flag == MM_CARVEOUT)
		mm_ioc_free_co(&bm, p);
	else if (p->flag == MM_CARVEOUT_SSP)
		mm_ioc_free_co(&bm_ssp, p);
	else if ((p->flag & 0xF) == MM_CARVEOUT_LOSSY) {
		find_lossy_entry(p->flag, &entry);
		if (entry >= 0)
			mm_ioc_free_co(lossy_entries[entry].bm_lossy, p);
	}
}

static int mm_ioc_share(int __user *in, struct MM_PARAM *out)
{
	int		ret = 0;
	struct MM_PARAM	tmp;

	if (copy_from_user(&tmp, in, sizeof(struct MM_PARAM))) {
		ret = -EFAULT;
		return ret;
	}

	out->phy_addr = tmp.phy_addr;
	out->size = tmp.size;

	return ret;
}

static void mmngr_dev_set_cma_area(struct device *dev, struct cma *cma)
{
	if (dev)
		dev->cma_area = cma;
}

static int open(struct inode *inode, struct file *file)
{
	struct MM_PARAM	*p;

	p = kzalloc(sizeof(struct MM_PARAM), GFP_KERNEL);
	if (!p)
		return -1;

	file->private_data = p;

	return 0;
}

static int close(struct inode *inode, struct file *file)
{
	struct MM_PARAM	*p = file->private_data;
	struct BM	*pb;
	struct device	*mm_dev;
	int		entry = 0;

	if (p) {
		if ((p->flag == MM_KERNELHEAP)
		&& (p->kernel_virt_addr != 0)) {
			pr_err("MMD close kernelheap\n");
			mm_dev = mm_drvdata->mm_dev;
			dma_free_coherent(mm_dev, p->size,
					(void *)p->kernel_virt_addr,
					(dma_addr_t)p->phy_addr);
		} else if ((p->flag == MM_CARVEOUT)
		&& (p->phy_addr != 0)) {
			pr_err("MMD close carveout\n");
			pb = &bm;
			mm_ioc_free_co(pb, p);
		} else if ((p->flag == MM_CARVEOUT_SSP)
		&& (p->phy_addr != 0)) {
#ifdef MMNGR_SSP_ENABLE
		if (is_sspbuf_valid) {
			pr_err("MMD close carveout SSP\n");
			pb = &bm_ssp;
			mm_ioc_free_co(pb, p);
		}
#endif
		} else if (((p->flag & 0xF) == MM_CARVEOUT_LOSSY)
		&& (p->phy_addr != 0)) {
			pr_err("MMD close carveout LOSSY\n");
			find_lossy_entry(p->flag, &entry);
			if (entry >= 0) {
				pb = lossy_entries[entry].bm_lossy;
				mm_ioc_free_co(pb, p);
			}
		}

		kfree(p);
		file->private_data = NULL;
	}

	return 0;
}

static long ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int		ercd;
	int		ret;
	struct MM_PARAM	*p = file->private_data;
	struct device	*mm_dev;

	mm_dev = mm_drvdata->mm_dev;

	switch (cmd) {
	case MM_IOC_ALLOC:
		ercd = mm_ioc_alloc(mm_dev, (int __user *)arg, p);
		if (ercd) {
			pr_err("MMD ALLOC ENOMEM\n");
			ret = ercd;
			goto exit;
		}
		break;
	case MM_IOC_FREE:
		mm_ioc_free(mm_dev, p);
		break;
	case MM_IOC_SET:
		ercd = mm_ioc_set((int __user *)arg, p);
		if (ercd) {
			pr_err("MMD SET EFAULT\n");
			ret = ercd;
			goto exit;
		}
		break;
	case MM_IOC_GET:
		ercd = mm_ioc_get(p, (int __user *)arg);
		if (ercd) {
			pr_err("MMD GET EFAULT\n");
			ret = ercd;
			goto exit;
		}
		break;
	case MM_IOC_ALLOC_CO:
		ercd = mm_ioc_alloc_co_select((int __user *)arg, p);
		if (ercd) {
			pr_err("MMD C ALLOC ENOMEM\n");
			ret = ercd;
			goto exit;
		}
		break;
	case MM_IOC_FREE_CO:
		mm_ioc_free_co_select(p);
		break;
	case MM_IOC_SHARE:
		ercd = mm_ioc_share((int __user *)arg, p);
		if (ercd) {
			pr_err("MMD C SHARE EFAULT\n");
			ret = ercd;
			goto exit;
		}
		break;
	default:
		pr_err("MMD CMD EFAULT\n");
		ret = -EFAULT;
		goto exit;
	}

	return 0;

exit:
	return ret;
}

#ifdef CONFIG_COMPAT
static long compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;
	struct MM_PARAM	__user *tmp;
	struct COMPAT_MM_PARAM tmp32;
	struct COMPAT_MM_PARAM __user *argp = (void __user *)arg;

	tmp = compat_alloc_user_space(sizeof(*tmp));

	/* Convert 32-bit data to 64-bit data */
	if (cmd != COMPAT_MM_IOC_FREE_CO && cmd != COMPAT_MM_IOC_FREE) {
		/* Convert 32-bit data to 64-bit data */
		if (copy_from_user(&tmp32, argp, sizeof(tmp32))) {
			ret = -EFAULT;
			return ret;
		}
	}

	switch (cmd) {
	case COMPAT_MM_IOC_ALLOC:
		cmd = MM_IOC_ALLOC;
		if (!access_ok(VERIFY_WRITE, tmp, sizeof(*tmp))
		    || __put_user(tmp32.size, &tmp->size)
		    || __put_user(tmp32.flag, &tmp->flag))
			return -EFAULT;
		break;
	case COMPAT_MM_IOC_SET:
		cmd = MM_IOC_SET;
		if (!access_ok(VERIFY_WRITE, tmp, sizeof(*tmp))
		    || __put_user(tmp32.user_virt_addr, &tmp->user_virt_addr))
			return -EFAULT;
		break;
	case COMPAT_MM_IOC_ALLOC_CO:
		cmd = MM_IOC_ALLOC_CO;
		if (!access_ok(VERIFY_WRITE, tmp, sizeof(*tmp))
		    || __put_user(tmp32.size, &tmp->size)
		    || __put_user(tmp32.flag, &tmp->flag))
			return -EFAULT;
		break;
	case COMPAT_MM_IOC_SHARE:
		cmd = MM_IOC_SHARE;
		if (!access_ok(VERIFY_WRITE, tmp, sizeof(*tmp))
		    || __put_user(tmp32.size, &tmp->size)
		    || __put_user(tmp32.phy_addr, &tmp->phy_addr))
			return -EFAULT;
		break;
	case COMPAT_MM_IOC_GET:
		cmd = MM_IOC_GET;
		break;
	case COMPAT_MM_IOC_FREE_CO:
		cmd = MM_IOC_FREE_CO;
		break;
	case COMPAT_MM_IOC_FREE:
		cmd = MM_IOC_FREE;
		break;

	default:
		break;
	}

	ret = ioctl(file, cmd, (unsigned long)tmp);
	if (ret)
		return ret;

	if (cmd == MM_IOC_GET) {
		/* Convert 64-bit data to 32-bit data */
		if (__get_user(tmp32.size, &tmp->size)
		    || __get_user(tmp32.phy_addr, &tmp->phy_addr)
		    || __get_user(tmp32.hard_addr, &tmp->hard_addr)
		    || __get_user(tmp32.user_virt_addr,
					&tmp->user_virt_addr)
		    || __get_user(tmp32.kernel_virt_addr,
					&tmp->kernel_virt_addr)
		    || __get_user(tmp32.flag, &tmp->flag))
			return -EFAULT;

		if (copy_to_user(argp, &tmp32, sizeof(tmp32)))
			ret = -EFAULT;
	}

	return ret;
}
#endif

static int mmap(struct file *filp, struct vm_area_struct *vma)
{
	phys_addr_t	start;
	phys_addr_t	off;
	unsigned long	len;
	struct MM_PARAM	*p = filp->private_data;

	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;

	off = vma->vm_pgoff << PAGE_SHIFT;
#ifdef MMNGR_IPMMU_PMB_DISABLE
	start = p->phy_addr;
#else
	start = pmb_virt2phys((unsigned int)p->phy_addr);
	if (!start)
		return -EINVAL;
#endif

	len = PAGE_ALIGN((start & ~PAGE_MASK) + p->size);

	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;

	off += start;
	vma->vm_pgoff = off >> PAGE_SHIFT;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	vma->vm_flags |= (VM_IO | VM_DONTEXPAND | VM_DONTDUMP);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

#ifdef MMNGR_IPMMU_PMB_DISABLE
static int validate_memory_map(void)
{
	int ret = 0;
#ifdef MMNGR_SSP_ENABLE
	unsigned long buf_size;
	char *buf_name;
#endif

	if (mm_kernel_reserve_size < MM_OMXBUF_SIZE) {
		pr_warn("The size (0x%x) of OMXBUF is over "\
			"the kernel reserved size (0x%llx) for Multimedia.\n",
			MM_OMXBUF_SIZE, mm_kernel_reserve_size);
		pr_warn("Failed to initialize MMNGR.\n");
		ret = -1;
	}

#ifdef MMNGR_SSP_ENABLE
	buf_size = MM_OMXBUF_SIZE + MM_SSPBUF_SIZE;
	buf_name = "OMXBUF and SSPBUF";

	if (mm_kernel_reserve_size >= buf_size) {
		if ((MM_SSPBUF_ADDR >= mm_kernel_reserve_addr) &&
		    (MM_SSPBUF_ADDR <= (mm_kernel_reserve_addr
					+ mm_kernel_reserve_size
					- MM_SSPBUF_SIZE))) {
			is_sspbuf_valid = true;
		} else {
			pr_warn("The SSPBUF (0x%lx - 0x%lx) is out of range of"\
				"the kernel reserved size (0x%llx - 0x%llx) for Multimedia.\n",
				MM_SSPBUF_ADDR, MM_SSPBUF_ADDR + MM_SSPBUF_SIZE,
				mm_kernel_reserve_addr,
				mm_kernel_reserve_addr
				+ mm_kernel_reserve_size);
			pr_warn("Not able to allocate buffer in SSPBUF.\n");

			is_sspbuf_valid = false;
		}
	} else {
		pr_warn("The total size (0x%lx) of %s is over "\
			"the kernel reserved size (0x%llx) for Multimedia.\n",
			buf_size, buf_name, mm_kernel_reserve_size);
		pr_warn("Not able to allocate buffer in SSPBUF.\n");

		is_sspbuf_valid = false;
	}
#endif
	return ret;
}
#endif

static int _parse_reserved_mem_dt(char *dt_path,
			u64 *addr, u64 *size)
{
	const u32 *regaddr_p;
	struct device_node *node;

	node = of_find_node_by_path(dt_path);
	if (!node)
		return -1;

	regaddr_p = of_get_address(node, 0, size, NULL);
	if (!regaddr_p) {
		of_node_put(node);
		return -1;
	}

	*addr = of_translate_address(node, regaddr_p);

	of_node_put(node);

	return 0;
}

#ifdef MMNGR_IPMMU_PMB_ENABLE
static phys_addr_t pmb_virt2phys(unsigned int ipmmu_virt_addr)
{
	phys_addr_t cpu_phys_addr;

	/*
	 * For Common CMA and CMA for Lossy,
	 *   assign physical address equal to virtual address.
	 * For CMA for MMP,
	 *   do address conversion.
	 */
	if (((ipmmu_virt_addr >= CMA_LOSSY_VIRT_BASE_ADDR) &&
	     (ipmmu_virt_addr < (CMA_LOSSY_VIRT_BASE_ADDR
				 + mm_lossybuf_size))) ||
	    ((ipmmu_virt_addr >= CMA_1ST_VIRT_BASE_ADDR) &&
	     (ipmmu_virt_addr < (CMA_1ST_VIRT_BASE_ADDR
				 + mm_common_reserve_size)))) {
		cpu_phys_addr = ipmmu_virt_addr;
	} else if ((ipmmu_virt_addr >= CMA_2ND_VIRT_BASE_ADDR) &&
		(ipmmu_virt_addr < (CMA_2ND_VIRT_BASE_ADDR
					+ mm_kernel_reserve_size))) {
		cpu_phys_addr = ((ipmmu_virt_addr - CMA_2ND_VIRT_BASE_ADDR)
				+ mm_kernel_reserve_addr);
	} else {
		pr_err("Invalid IPMMU virtual address 0x%08x\n",
					ipmmu_virt_addr);
		return 0;
	}

	return cpu_phys_addr;
}

static unsigned int pmb_get_table_type(u64 phys_addr, u64 size)
{
	unsigned int i;

	/*
	 * Find the 1st table type
	 * which the phys start address is aligned to its table size.
	 */
	for (i = 0; i < ARRAY_SIZE(pmb_table_mapping); i++) {
		if ((phys_addr % pmb_table_mapping[i].table_size) ||
			(size < pmb_table_mapping[i].table_size))
			continue;
		else
			break;
	}

	pr_debug("%s: Select table size %ldMB\n",
		  __func__, pmb_table_mapping[i].table_size >> 20);

	return i;
}

static void pmb_update_table_info(struct p2v_map *p2v_map,
			unsigned int impmbd_sz, u64 phys_addr,
					unsigned int virt_addr)
{
	p2v_map->impmba = IMPMBA_VALUE(virt_addr);
	p2v_map->impmbd = IMPMBD_VALUE(phys_addr) | impmbd_sz;

	pr_debug("%s: virt_addr 0x%08x phys_addr 0x%llx\n",
	__func__, virt_addr, phys_addr);


	pr_debug("%s: IMPMBA_VALUE(virt_addr) 0x%08x IMPMBD_VALUE(phys_addr) 0x%llx\n",
	__func__, IMPMBA_VALUE(virt_addr), IMPMBD_VALUE(phys_addr));
}

static int __pmb_create_phys2virt_map(char *dt_path, u64 phys_addr,
				u64 size, unsigned int *table_count)
{
	int ret = 0;
	unsigned int table_type, virt_addr, size_in_MB, table_entry, impmbd_sz;
	u64 tmp_size, tmp_phys_addr;

	/* Sanity tests */
	size_in_MB = size >> 20;

	if (size_in_MB % 16) {
		pr_warn("Reserved area %s (%dMB) is not multiple of 16MB",
						dt_path, size_in_MB);
		return -1;
	}

	if (phys_addr % SZ_16M) {
		pr_warn("Physical start address (0x%llx) of reserved area %s"\
			" is not 16MB aligned.", phys_addr, dt_path);
		return -1;
	}

	/* Set the base IPMMU virt address */
	if (!strcmp(dt_path, "/reserved-memory/linux,cma")) {
		virt_addr = CMA_1ST_VIRT_BASE_ADDR;
	} else if (!strcmp(dt_path, "/reserved-memory/linux,multimedia")) {
		virt_addr = CMA_2ND_VIRT_BASE_ADDR;
	} else { /* /reserved-memory/linux,lossy_decompress */
		virt_addr = CMA_LOSSY_VIRT_BASE_ADDR;
	}

	table_entry = *table_count;
	tmp_size = size;
	tmp_phys_addr = phys_addr;

	while (tmp_size > 0) {
		if (table_entry == MAX_PMB_TABLE) {
			pr_err("Over 16 PMB tables. 16 is maximum.");
			ret = -1;
		}

		/* Proceed a table entry */
		table_type = pmb_get_table_type(tmp_phys_addr, tmp_size);
		impmbd_sz = pmb_table_mapping[table_type].impmbd_sz;

		pmb_update_table_info(&p2v_mapping[table_entry],
					impmbd_sz, tmp_phys_addr, virt_addr);

		/* Update for next table entry */
		tmp_phys_addr += pmb_table_mapping[table_type].table_size;
		virt_addr += pmb_table_mapping[table_type].table_size;
		tmp_size -= pmb_table_mapping[table_type].table_size;

		table_entry++;
	}

	pmb_p2v_mapping.map_count += (table_entry - *table_count);
	*table_count = table_entry;

	return ret;
}

static int pmb_create_phys2virt_map(void)
{
	int ret = 0;
	unsigned int table_count = 0;

	ret = __pmb_create_phys2virt_map(
			"/reserved-memory/linux,cma",
			mm_common_reserve_addr, mm_common_reserve_size,
			&table_count);
	if (ret)
		return ret;

	ret = __pmb_create_phys2virt_map(
			"/reserved-memory/linux,multimedia",
			mm_kernel_reserve_addr, mm_kernel_reserve_size,
			&table_count);
	if (ret)
		return ret;

	ret = __pmb_create_phys2virt_map(
			"/reserved-memory/linux,lossy_decompress",
			mm_lossybuf_addr, mm_lossybuf_size, &table_count);

	return ret;
}

#endif

static int parse_reserved_mem_dt(void)
{
	int ret = 0;

	ret = _parse_reserved_mem_dt(
			"/reserved-memory/linux,cma",
			&mm_common_reserve_addr, &mm_common_reserve_size);
	if (ret) {
		pr_warn("Failed to parse common CMA reserved area" \
			 "(linux,cma) from DT\n");
		return ret;
	}

	ret = _parse_reserved_mem_dt(
			"/reserved-memory/linux,multimedia",
			&mm_kernel_reserve_addr, &mm_kernel_reserve_size);
	if (ret) {
		pr_warn("Failed to parse MMP reserved area" \
			 "(linux,multimedia) from DT\n");
		return ret;
	}

	ret = _parse_reserved_mem_dt(
			"/reserved-memory/linux,lossy_decompress",
			&mm_lossybuf_addr, &mm_lossybuf_size);
	if (ret) {
		pr_warn("Failed to parse Lossy reserved area" \
			"(linux,lossy_decompress) from DT\n");
		ret = 0; /* Let MMNGR support other features */
	}

	return ret;
}

static int init_lossy_info(void)
{
	int ret = 0;
	void __iomem *mem;
	uint32_t i, start, end, fmt;
	struct BM *bm_lossy;
	struct LOSSY_INFO *p;
	uint32_t total_lossy_size = 0;

	have_lossy_entries = false;

	mem = ioremap_nocache(MM_LOSSY_SHARED_MEM_ADDR,
			MM_LOSSY_SHARED_MEM_SIZE);
	if (mem == NULL)
		return -1;

	p = (struct LOSSY_INFO __force *)mem;

	for (i = 0; i < 16; i++) {
		/* Validate the entry */
		if ((p->magic != MM_LOSSY_INFO_MAGIC)
		|| (p->a0 == 0) || (p->b0 == 0)
		|| ((p->a0 & MM_LOSSY_ENABLE_MASK) == 0))
			break;

		/* Parse entry information */
		start = (p->a0 & MM_LOSSY_ADDR_MASK) << 20;
		end = (p->b0 & MM_LOSSY_ADDR_MASK) << 20;
		fmt = (p->a0 & MM_LOSSY_FMT_MASK) >> 29;

		/* Validate kernel reserved mem for Lossy */
		if (i == 0 && start != mm_lossybuf_addr) {
			pr_warn("Mismatch between the start address (0x%llx) "\
				"of reserved mem and start address (0x%x) "\
				"of Lossy enabled area.\n", mm_lossybuf_addr,
				start);
			break;
		}

		total_lossy_size += end - start;
		if (total_lossy_size > mm_lossybuf_size) {
			pr_warn("Size of Lossy enabled areas (0x%x) is over "\
				"the size of reserved mem(0x%x).\n",
				total_lossy_size,
				(unsigned int)mm_lossybuf_size);
			break;
		}

		/* Allocate bitmap for entry */
		bm_lossy = kzalloc(sizeof(struct BM), GFP_KERNEL);
		if (bm_lossy == NULL)
			break;

		ret = alloc_bm(bm_lossy, start, end - start, MM_CO_ORDER);
		if (ret)
			break;

		pr_debug("Support entry %d with format 0x%x.\n", i, fmt);

		lossy_entries[i].fmt = fmt;
		lossy_entries[i].bm_lossy = bm_lossy;

		p++;
	}

	if (i > 0)
		have_lossy_entries = true;

	iounmap(mem);
	return ret;
}

#ifdef MMNGR_IPMMU_PMB_ENABLE
/* IPMMU (PMB mode) */
static int __handle_registers(struct rcar_ipmmu *ipmmu, unsigned int handling)
{
	int ret = 0;
	unsigned int j, k;
	phys_addr_t base_addr = ipmmu->base_addr;
	void __iomem *virt_addr = ipmmu->virt_addr;
	unsigned int reg_count = ipmmu->reg_count;
	unsigned int masters_count = ipmmu->masters_count;
	struct hw_register *ipmmu_reg = ipmmu->ipmmu_reg;
	struct ip_master *ip_masters = ipmmu->ip_masters;

	if (handling == DO_IOREMAP) { /* ioremap */
		/* IOREMAP registers in an IPMMU */
		ipmmu->virt_addr = ioremap_nocache(base_addr, REG_SIZE);
		if (ipmmu->virt_addr == NULL)
			ret = -1;

		pr_debug("\n%s: DO_IOREMAP: %s, virt_addr 0x%lx\n",
			__func__, ipmmu->ipmmu_name,
			(unsigned long) ipmmu->virt_addr);

	} else if (handling == DO_IOUNMAP) { /* iounmap*/
		/* IOUNMAP registers in an IPMMU */
		iounmap(ipmmu->virt_addr);
		ipmmu->virt_addr = NULL;
		pr_debug("%s: DO_IOUNMAP: %s, virt_addr 0x%lx\n",
			__func__, ipmmu->ipmmu_name,
			(unsigned long) ipmmu->virt_addr);

	} else if (handling == ENABLE_PMB) { /* Enable PMB of IPMMU */
		for (j = 0; j < reg_count; j++) {
			if (!strcmp(ipmmu_reg[j].reg_name, "IMPCTR"))
				break;
		}

		if (j < reg_count) /* Found IMPCTR */
			iowrite32(IMPCTR_VAL |
				ioread32(virt_addr + ipmmu_reg[j].reg_offset),
				virt_addr + ipmmu_reg[j].reg_offset);
		else
			ret = -1;

	} else if (handling == DISABLE_PMB) { /* Disable PMB of IPMMU */
		for (j = 0; j < reg_count; j++) {
			if (!strcmp(ipmmu_reg[j].reg_name, "IMPCTR"))
				break;
		}

		if (j < reg_count) /* Found IMPCTR */
			iowrite32(~IMPCTR_VAL & ioread32(
				virt_addr + ipmmu_reg[j].reg_offset),
				virt_addr + ipmmu_reg[j].reg_offset);
		else
			ret = -1;

	} else if (handling == ENABLE_UTLB) { /* Enable utlb for IP master */
		for (j = 0; j < masters_count; j++)
			iowrite32(IMUCTR_VAL,
				virt_addr +
				IMUCTRn_OFFSET(ip_masters[j].utlb_no));

	} else if (handling == DISABLE_UTLB) { /* Disable utlb for IP master */
		for (j = 0; j < masters_count; j++)
			iowrite32(~IMUCTR_VAL & ioread32(
				virt_addr + IMUCTRn_OFFSET(
						ip_masters[j].utlb_no)),
				virt_addr + IMUCTRn_OFFSET(
						ip_masters[j].utlb_no));

	} else if (handling == SET_PMB_AREA) { /* Enable PMB area for IPMMU */
		for (j = 0; j < reg_count; j++) {
			if (!strcmp(ipmmu_reg[j].reg_name, "IMPMBA0"))
				break;
		}

		if (j < reg_count) /* Found IMPMBA0 */
			for (k = 0; k < pmb_p2v_mapping.map_count; k++) {
				pr_debug("k=%d: impmba 0x%08x impmbd 0x%08x\n",
					k, p2v_mapping[k].impmba,
					p2v_mapping[k].impmbd);

				iowrite32(IMPMBAn_V_BIT | p2v_mapping[k].impmba,
					virt_addr + ipmmu_reg[j].reg_offset);
				iowrite32(IMPMBDn_V_BIT | p2v_mapping[k].impmbd,
					virt_addr + ipmmu_reg[j+1].reg_offset);
				j += 2; /* Move to next PMB entry */
			}
		else
			ret = -1;

	} else if (handling == CLEAR_PMB_AREA) { /* Clear PMB area for IPMMU */
		for (j = 0; j < reg_count; j++) {
			if (!strcmp(ipmmu_reg[j].reg_name, "IMPMBA0"))
				break;
		}

		if (j < reg_count) /* Found IMPMBA0 */
			for (k = 0; k < pmb_p2v_mapping.map_count; k++) {
				iowrite32(0x0, virt_addr +
					 ipmmu_reg[j].reg_offset);
				iowrite32(0x0, virt_addr +
					 ipmmu_reg[j+1].reg_offset);
				j += 2; /* Move to next PMB entry */
			}
		else
			ret = -1;

	} else if (handling == BACKUP_PMB_REGS) { /* Backup IPMMU(PMB) regs */
		for (j = 0; j < reg_count; j++) {
			if (!strcmp(ipmmu_reg[j].reg_name, "IMPMBA0"))
				break;
		}

		if (j < reg_count) /* Found IMPMBA0 */
			for (k = 0; k < pmb_p2v_mapping.map_count; k++) {
				/* For IMPMBAn */
				ipmmu_reg[j].reg_val = ioread32(virt_addr +
						       ipmmu_reg[j].reg_offset);
				pr_debug("%s: reg value 0x%08x\n",
					ipmmu_reg[j].reg_name,
					ipmmu_reg[j].reg_val);

				/* For IMPMBDn */
				ipmmu_reg[j+1].reg_val = ioread32(virt_addr +
						ipmmu_reg[j+1].reg_offset);
				pr_debug("%s: reg value 0x%08x\n",
					  ipmmu_reg[j+1].reg_name,
					  ipmmu_reg[j+1].reg_val);

				j += 2; /* Move to next PMB entry */
			}
		else
			ret = -1;

	} else if (handling == RESTORE_PMB_REGS) { /* Restore IPMMU(PMB) regs */
		for (j = 0; j < reg_count; j++) {
			if (!strcmp(ipmmu_reg[j].reg_name, "IMPMBA0"))
				break;
		}

		if (j < reg_count) /* Found IMPMBA0 */
			for (k = 0; k < pmb_p2v_mapping.map_count; k++) {
				/* For IMPMBAn */
				iowrite32(ipmmu_reg[j].reg_val,
					virt_addr + ipmmu_reg[j].reg_offset);
				pr_debug("%s: reg value 0x%08x\n",
					ipmmu_reg[j].reg_name,
					ioread32(virt_addr +
					ipmmu_reg[j].reg_offset));

				/* For IMPMBDn */
				iowrite32(ipmmu_reg[j+1].reg_val,
					 virt_addr + ipmmu_reg[j+1].reg_offset);
				pr_debug("%s: reg value 0x%08x\n",
					ipmmu_reg[j+1].reg_name,
					ioread32(virt_addr +
					ipmmu_reg[j+1].reg_offset));

				j += 2; /* Move to next PMB entry */
			}
		else
			ret = -1;

	} else if (handling == PRINT_PMB_DEBUG) { /* Print PMB status info. */
		for (j = 0; j < reg_count; j++) {
			if (j == 0)
				pr_debug("---\n"); /* delimiter */
			if (!strcmp(ipmmu_reg[j].reg_name, "IMPSTR") ||
			    !strcmp(ipmmu_reg[j].reg_name, "IMPEAR"))
				pr_err("%s: %s(%08x)\n", ipmmu->ipmmu_name,
					ipmmu_reg[j].reg_name,
					ioread32(virt_addr +
						ipmmu_reg[j].reg_offset));
			else
				pr_debug("%s: %s(%08x)\n", ipmmu->ipmmu_name,
					  ipmmu_reg[j].reg_name,
					  ioread32(virt_addr +
						ipmmu_reg[j].reg_offset));
		}
	} else { /* Invalid */
		pr_info("%s: Invalid parameters\n", __func__);
		ret = -1;
	}

	return ret;
}

/*
 * Handle the ioremap/iounmap of IP registers
 *   handling: Flag of processing
 *     0: ioremap
 *     1: iounmap
 */
static int handle_registers(struct rcar_ipmmu **ipmmu, unsigned int handling)
{
	struct rcar_ipmmu *working_ipmmu;
	unsigned int i = 0;
	unsigned int ret = 0;

	while (ipmmu[i] != NULL) {
		working_ipmmu = ipmmu[i];
		ret = __handle_registers(working_ipmmu, handling);
		i++;
	}

	return ret;
}
static int map_register(void)
{
	int ret = 0;

	ret = handle_registers(rcar_gen3_ipmmu, DO_IOREMAP);

	return ret;
}

static void unmap_register(void)
{
	handle_registers(rcar_gen3_ipmmu, DO_IOUNMAP);
}

static void enable_pmb(void)
{
	handle_registers(rcar_gen3_ipmmu, ENABLE_PMB);
}

static void enable_utlb(void)
{
	handle_registers(rcar_gen3_ipmmu, ENABLE_UTLB);
}

static void set_pmb_area(void)
{
	handle_registers(rcar_gen3_ipmmu, SET_PMB_AREA);
}

static void pmb_debuginfo(void)
{
	handle_registers(rcar_gen3_ipmmu, PRINT_PMB_DEBUG);
}

static void backup_pmb_registers(void)
{
	handle_registers(rcar_gen3_ipmmu, BACKUP_PMB_REGS);
}

static void restore_pmb_registers(void)
{
	handle_registers(rcar_gen3_ipmmu, RESTORE_PMB_REGS);
}

static int pmb_init(void)
{
	int			ret = 0;

	ret = map_register();
	if (ret != 0) {
		pr_err("%s: map_register() NG\n", __func__);
		return -1;
	}

	set_pmb_area();
	enable_pmb();
	enable_utlb();

	return 0;
}

static void pmb_exit(void)
{
	pmb_debuginfo();

	/* Disable all uTLB and PMB support */
	handle_registers(rcar_gen3_ipmmu, DISABLE_UTLB);
	handle_registers(rcar_gen3_ipmmu, DISABLE_PMB);
	handle_registers(rcar_gen3_ipmmu, CLEAR_PMB_AREA);

	unmap_register();
}

static int ipmmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct rcar_ipmmu_data *data;

	data = of_device_get_match_data(dev);
	if (!data)
		return -1;

	if (soc_device_match(r8a7795es1))
		rcar_gen3_ipmmu = r8a7795es1_ipmmu;
	else
		rcar_gen3_ipmmu = data->ipmmu_data;

	return 0;
}

static int ipmmu_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct rcar_ipmmu_data r8a7795_ipmmu_data = {
	.ipmmu_data = r8a7795_ipmmu,
};

static const struct rcar_ipmmu_data r8a7796_ipmmu_data = {
	.ipmmu_data = r8a7796_ipmmu,
};

static const struct of_device_id ipmmu_of_match[] = {
	{
	  .compatible	= "renesas,ipmmu-pmb-r8a7795",
	  .data		= &r8a7795_ipmmu_data
	},
	{
	  .compatible	= "renesas,ipmmu-pmb-r8a7796",
	  .data = &r8a7796_ipmmu_data
	},
	{ },
};

#ifdef CONFIG_PM_SLEEP
static int mm_ipmmu_suspend(struct device *dev)
{
	backup_pmb_registers();

	return 0;
}

static int mm_ipmmu_resume(struct device *dev)
{
	restore_pmb_registers();
	enable_pmb();
	enable_utlb();

	return 0;
}

static SIMPLE_DEV_PM_OPS(mm_ipmmu_pm_ops,
			mm_ipmmu_suspend, mm_ipmmu_resume);
#define DEV_PM_OPS (&mm_ipmmu_pm_ops)
#else
#define DEV_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver ipmmu_driver = {
	.driver = {
		.name = DEVNAME "_ipmmu_drv",
		.pm	= DEV_PM_OPS,
		.owner = THIS_MODULE,
		.of_match_table = ipmmu_of_match,
	},
	.probe = ipmmu_probe,
	.remove = ipmmu_remove,
};
#endif /* MMNGR_IPMMU_PMB_ENABLE */

static const struct file_operations fops = {
	.owner		= THIS_MODULE,
	.open		= open,
	.release	= close,
	.unlocked_ioctl	= ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= compat_ioctl,
#endif
	.mmap		= mmap,
};

static struct miscdevice misc = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= DEVNAME,
	.fops		= &fops,
};

static int mm_probe(struct platform_device *pdev)
{
	int			ret = 0;
	struct MM_DRVDATA	*p = NULL;
	phys_addr_t		phy_addr;
	void			*pkernel_virt_addr;
	struct device		*dev = &pdev->dev;
	unsigned long		mm_omxbuf_size;

	ret = parse_reserved_mem_dt();
	if (ret) {
		pr_err("MMD mm_probe ERROR\n");
		return -1;
	}

#ifdef MMNGR_IPMMU_PMB_DISABLE
	ret = validate_memory_map();
	if (ret) {
		pr_err("MMD mm_probe ERROR\n");
		return -1;
	}
#endif

#ifndef MMNGR_SSP_ENABLE
	mm_omxbuf_size = mm_kernel_reserve_size;
#else
	mm_omxbuf_size = mm_kernel_reserve_size - MM_SSPBUF_SIZE;
#endif
	ret = alloc_bm(&bm, MM_OMXBUF_ADDR, mm_omxbuf_size, MM_CO_ORDER);
	if (ret) {
		pr_err("MMD mm_probe ERROR\n");
		return -1;
	}

#ifdef MMNGR_SSP_ENABLE
	if (is_sspbuf_valid) {
		ret = alloc_bm(&bm_ssp, MM_SSPBUF_ADDR, MM_SSPBUF_SIZE,
				MM_CO_ORDER);
		if (ret) {
			pr_err("MMD mm_probe ERROR\n");
			return -1;
		}
	}
#endif

	ret = init_lossy_info();
	if (ret) {
		pr_err("MMD mm_init ERROR\n");
		return -1;
	}

	p = kzalloc(sizeof(struct MM_DRVDATA), GFP_KERNEL);
	if (p == NULL)
		return -1;

#ifdef MMNGR_IPMMU_PMB_ENABLE
	ret = pmb_create_phys2virt_map();
	if (ret) {
		pr_err("MMD mm_init ERROR\n");
		return -1;
	}

	pmb_init();
#endif

	misc_register(&misc);

	/* Handler for mem alloc in 2nd CMA area */
	p->mm_dev_reserve = dev;
	of_reserved_mem_device_init(p->mm_dev_reserve);

	pkernel_virt_addr = dma_alloc_coherent(p->mm_dev_reserve,
					mm_kernel_reserve_size,
					(dma_addr_t *)&phy_addr,
					GFP_KERNEL);
	if (pkernel_virt_addr == NULL) {
		pr_err("MMD mm_init ERROR\n");
		return -1;
	}

	p->reserve_size = mm_kernel_reserve_size;
	p->reserve_kernel_virt_addr = (unsigned long)pkernel_virt_addr;
	p->reserve_phy_addr = (unsigned long)phy_addr;
	pr_debug("MMD reserve area from 0x%pK to 0x%pK at virtual\n",
		pkernel_virt_addr,
		pkernel_virt_addr + mm_kernel_reserve_size - 1);
	pr_debug("MMD reserve area from 0x%lx to 0x%llx at physical\n",
		(unsigned long)phy_addr,
		(unsigned long)phy_addr + mm_kernel_reserve_size - 1);

	/* Handler for mem alloc in 1st CMA area */
	mm_cma_area = dev->cma_area;
	dev->cma_area = NULL;
	p->mm_dev = dev;
	mm_drvdata = p;


	spin_lock_init(&lock);

	return 0;
}

static int mm_remove(struct platform_device *pdev)
{
	uint32_t i;

	misc_deregister(&misc);

#ifdef MMNGR_IPMMU_PMB_ENABLE
	pmb_exit();
#endif

#ifdef MMNGR_SSP_ENABLE
	if (is_sspbuf_valid)
		free_bm(&bm_ssp);
#endif

	for (i = 0; i < 16; i++) {
		if (lossy_entries[i].bm_lossy == NULL)
			break;
		free_bm(lossy_entries[i].bm_lossy);
	}

	free_bm(&bm);

	mmngr_dev_set_cma_area(mm_drvdata->mm_dev_reserve,
				mm_cma_area);
	dma_free_coherent(mm_drvdata->mm_dev_reserve,
			mm_drvdata->reserve_size,
			(void *)mm_drvdata->reserve_kernel_virt_addr,
			(dma_addr_t)mm_drvdata->reserve_phy_addr);

	kfree(mm_drvdata);

	return 0;
}

static const struct of_device_id mm_of_match[] = {
	{ .compatible = "renesas,mmngr" },
	{ },
};

static struct platform_driver mm_driver = {
	.driver = {
		.name = DEVNAME "_drv",
		.owner = THIS_MODULE,
		.of_match_table = mm_of_match,
	},
	.probe = mm_probe,
	.remove = mm_remove,
};

static int mm_init(void)
{
#ifdef MMNGR_IPMMU_PMB_ENABLE
	platform_driver_register(&ipmmu_driver);
#endif
	return platform_driver_register(&mm_driver);
}

static void mm_exit(void)
{
	platform_driver_unregister(&mm_driver);
#ifdef MMNGR_IPMMU_PMB_ENABLE
	platform_driver_unregister(&ipmmu_driver);
#endif
}

module_init(mm_init);
module_exit(mm_exit);

MODULE_LICENSE("Dual MIT/GPL");
