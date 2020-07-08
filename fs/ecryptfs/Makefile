#
# Makefile for the Linux eCryptfs
#

<<<<<<< HEAD
ccflags-y += -Ifs/sdcardfs/m

obj-$(CONFIG_ECRYPT_FS) += ecryptfs.o
obj-$(CONFIG_SDP) += ecryptfs_dek.o mm.o ecryptfs_sdp_chamber.o
=======
obj-$(CONFIG_ECRYPT_FS) += ecryptfs.o
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012

ecryptfs-y := dentry.o file.o inode.o main.o super.o mmap.o read_write.o \
	      crypto.o keystore.o kthread.o debug.o

ecryptfs-$(CONFIG_ECRYPT_FS_MESSAGING) += messaging.o miscdev.o
