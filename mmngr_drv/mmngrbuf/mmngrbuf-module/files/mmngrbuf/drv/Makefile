CFILES = mmngr_buf_drv.c

obj-m += mmngrbuf.o
mmngrbuf-objs := $(CFILES:.c=.o)

ccflags-y	+= -I$(KERNELSRC)/include

all:
	$(CP) ../include/mmngr_buf_private.h $(KERNELSRC)/include
	$(CP) ../include/mmngr_buf_private_cmn.h $(KERNELSRC)/include
	$(CP) ../include/mmngr_buf_private_cmn.h $(INCSHARED)
	make -C $(KERNELSRC) M=$(PWD) modules
clean:
	make -C $(KERNELSRC) M=$(PWD) clean
	rm -f $(KERNELSRC)/include/mmngr_buf_private.h
	rm -f $(KERNELSRC)/include/mmngr_buf_private_cmn.h

