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
#ifndef __MMNGR_BUF_PRIVATE_H__
#define __MMNGR_BUF_PRIVATE_H__

#include "mmngr_buf_private_cmn.h"

struct MM_BUF_DRVDATA {
	struct device *mm_buf_dev;
};

struct MM_BUF_PRIVATE {
	size_t		size;
	unsigned int	hard_addr;
	int		buf;
	struct dma_buf	*dma_buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
};

#define DEVNAME		"rgnmmbuf"
#define DRVNAME		DEVNAME
#define CLSNAME		DEVNAME
#define DEVNUM		1

#ifdef CONFIG_COMPAT
struct COMPAT_MM_BUF_PARAM {
	compat_size_t	size;
	compat_uint_t	hard_addr;
	compat_int_t	buf;
};

#define COMPAT_MM_IOC_EXPORT_START \
		_IOWR(MM_IOC_MAGIC, 0, struct COMPAT_MM_BUF_PARAM)
#define COMPAT_MM_IOC_EXPORT_END \
		_IOWR(MM_IOC_MAGIC, 1, struct COMPAT_MM_BUF_PARAM)
#define COMPAT_MM_IOC_IMPORT_START \
		_IOWR(MM_IOC_MAGIC, 2, struct COMPAT_MM_BUF_PARAM)
#define COMPAT_MM_IOC_IMPORT_END \
		_IOWR(MM_IOC_MAGIC, 3, struct COMPAT_MM_BUF_PARAM)

static long compat_ioctl(struct file *file, unsigned int cmd,
			unsigned long arg);
#endif

static int open(struct inode *inode, struct file *file);
static int close(struct inode *inode, struct file *file);
static long ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int dmabuf_attach(struct dma_buf *buf,
			 struct dma_buf_attachment *attach);
static void dmabuf_detach(struct dma_buf *buf,
			struct dma_buf_attachment *attach);
static struct sg_table *dmabuf_map_dma_buf(struct dma_buf_attachment *attach,
					enum dma_data_direction dir);
static void dmabuf_unmap_dma_buf(struct dma_buf_attachment *attach,
				struct sg_table *sgt,
				enum dma_data_direction dir);
static void dmabuf_release(struct dma_buf *buf);
static int dmabuf_begin_cpu_access(struct dma_buf *buf,
				enum dma_data_direction direction);
static int dmabuf_end_cpu_access(struct dma_buf *buf,
				enum dma_data_direction direction);
static int dmabuf_mmap(struct dma_buf *buf, struct vm_area_struct *vma);
static void *dmabuf_vmap(struct dma_buf *buf);
static void dmabuf_vunmap(struct dma_buf *buf, void *vaddr);
static int mm_probe(struct platform_device *pdev);
static int mm_remove(struct platform_device *pdev);
static int mm_init(void);
static void mm_exit(void);
static int mm_ioc_export_start(int __user *arg, struct MM_BUF_PRIVATE *priv);
static int mm_ioc_export_end(int __user *arg, struct MM_BUF_PRIVATE *priv);
static int mm_ioc_import_start(int __user *arg, struct MM_BUF_PRIVATE *priv);
static int mm_ioc_import_end(struct MM_BUF_PRIVATE *priv);

#endif	/* __MMNGR_BUF_PRIVATE_H__ */
