CFILES = mmngr_buf_drv.c

obj-m += mmngrbuf.o
mmngrbuf-objs := $(CFILES:.c=.o)

ccflags-y	+= -I$(KERNELSRC)/include

all:
	$(CP) ../include/mmngr_buf_private.h $(KERNELSRC)/include
	make -C $(KERNELDIR) M=$(PWD) modules
clean:
	make -C $(KERNELDIR) M=$(PWD) clean

