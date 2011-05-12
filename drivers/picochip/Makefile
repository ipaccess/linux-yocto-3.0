# $picoChipHeaderSubst$
#
# Makefile for pico driver

obj-$(CONFIG_PICOIF)	      	+= picoif.o
picoif-y := picoif_main.o resource.o \
	    debug.o gpr_interrupt.o dma.o \
	    dma_fifo.o hwif2.o utilities_internal.o \
	    directdma.o

picoifktest-y := kerneltest.o

picoif-$(CONFIG_PICOIF_PC3XX)	+= pc3xx.o
