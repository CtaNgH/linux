#
# Makefile for the Novatek NT36xxx touchscreen driver.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_TOUCHSCREEN_NT36xxx_HOSTDL_SPI) += nt36xxx.o nt36xxx_fw_update.o nt36xxx_stubs.o

# the vendor drm notifier header lives under the display stack
ccflags-y += -I$(srctree)/drivers/gpu/drm/mediatek/mediatek_v2/vendor-include
