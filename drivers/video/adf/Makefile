ccflags-y := -Idrivers/staging/android

CFLAGS_adf.o := -I$(src)

<<<<<<< HEAD
obj-$(CONFIG_ADF) += adf.o \
=======
obj-$(CONFIG_ADF) += adf_core.o

adf_core-y := adf.o \
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	adf_client.o \
	adf_fops.o \
	adf_format.o \
	adf_sysfs.o

<<<<<<< HEAD
obj-$(CONFIG_COMPAT) += adf_fops32.o
=======
adf_core-$(CONFIG_COMPAT) += adf_fops32.o
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012

obj-$(CONFIG_ADF_FBDEV) += adf_fbdev.o

obj-$(CONFIG_ADF_MEMBLOCK) += adf_memblock.o
