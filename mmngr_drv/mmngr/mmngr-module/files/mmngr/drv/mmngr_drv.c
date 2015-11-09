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
#include <linux/version.h>
#include <linux/dma-attrs.h>
#include <linux/dma-contiguous.h>

#include "mmngr_public.h"
#include "mmngr_private.h"

static spinlock_t		lock;
static struct BM		bm;
static struct BM		bm_ssp;
static struct BM		bm_lossy;
static struct MM_DRVDATA	*mm_drvdata;

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

static int mm_ioc_alloc_co_select(int __user *in, struct MM_PARAM *out)
{
	int		ret = 0;
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
		ret = mm_ioc_alloc_co(&bm_ssp, in, out);
#endif
	else if (tmp.flag == MM_CARVEOUT_LOSSY)
		ret = mm_ioc_alloc_co(&bm_lossy, in, out);

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

	mm_dev = mm_drvdata->mm_dev;

	if (p->flag == MM_CARVEOUT)
		mm_ioc_free_co(&bm, p);
	else if (p->flag == MM_CARVEOUT_SSP)
		mm_ioc_free_co(&bm_ssp, p);
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
			pr_err("MMD close carveout SSP\n");
			pb = &bm_ssp;
			mm_ioc_free_co(pb, p);
#endif
		} else if ((p->flag == MM_CARVEOUT_LOSSY)
		&& (p->phy_addr != 0)) {
			    pr_err("MMD close carveout LOSSY\n");
			    pb = &bm_lossy;
			    mm_ioc_free_co(pb, p);
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
	start = p->phy_addr;
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

	ret = alloc_bm(&bm, MM_OMXBUF_ADDR, MM_OMXBUF_SIZE, MM_CO_ORDER);
	if (ret) {
		pr_err("MMD mm_probe ERROR");
		return -1;
	}

#ifdef MMNGR_SSP_ENABLE
	ret = alloc_bm(&bm_ssp, MM_SSPBUF_ADDR, MM_SSPBUF_SIZE, MM_CO_ORDER);
	if (ret) {
		pr_err("MMD mm_probe ERROR");
		return -1;
	}
#endif

#ifdef MM_FUNC_LOSSY_SUPPORT
	ret = alloc_bm(&bm_lossy, MM_LOSSYBUF_ADDR, MM_LOSSYBUF_SIZE, MM_CO_ORDER);
	if (ret) {
		pr_err("MMD mm_init ERROR");
		return -1;
	}
#endif

	p = kzalloc(sizeof(struct MM_DRVDATA), GFP_KERNEL);
	if (p == NULL)
		return -1;

	misc_register(&misc);

	/* Handler for mem alloc in 2nd CMA area */
	p->mm_dev_reserve = dev;

	mmngr_dev_set_cma_area(p->mm_dev_reserve, rcar_gen3_dma_contiguous);
	pkernel_virt_addr = dma_alloc_coherent(p->mm_dev_reserve,
					MM_KERNEL_RESERVE_SIZE,
					(dma_addr_t *)&phy_addr,
					GFP_KERNEL);
	p->reserve_size = MM_KERNEL_RESERVE_SIZE;
	p->reserve_kernel_virt_addr = (unsigned long)pkernel_virt_addr;
	p->reserve_phy_addr = (unsigned long)phy_addr;
	pr_debug("MMD reserve area from 0x%pK to 0x%pK at virtual\n",
		pkernel_virt_addr,
		pkernel_virt_addr + MM_KERNEL_RESERVE_SIZE - 1);
	pr_debug("MMD reserve area from 0x%08x to 0x%08x at physical\n",
		(unsigned int)phy_addr,
		(unsigned int)phy_addr + MM_KERNEL_RESERVE_SIZE - 1);

	/* Handler for mem alloc in 1st CMA area */
	dev->cma_area = 0;
	p->mm_dev = dev;
	mm_drvdata = p;

	spin_lock_init(&lock);

	return 0;
}

static int mm_remove(struct platform_device *pdev)
{
	misc_deregister(&misc);

#ifdef MMNGR_SSP_ENABLE
	free_bm(&bm_ssp);
#endif
#ifdef MM_FUNC_LOSSY_SUPPORT
	free_bm(&bm_lossy);
#endif
	free_bm(&bm);

	mmngr_dev_set_cma_area(mm_drvdata->mm_dev_reserve,
				rcar_gen3_dma_contiguous);
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
	return platform_driver_register(&mm_driver);
}

static void mm_exit(void)
{
	platform_driver_unregister(&mm_driver);
}

module_init(mm_init);
module_exit(mm_exit);

MODULE_LICENSE("Dual MIT/GPL");
