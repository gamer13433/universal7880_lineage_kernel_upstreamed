obj-y := core.o
<<<<<<< HEAD
=======
CFLAGS_core.o += $(call cc-disable-warning, override-init)

>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
obj-$(CONFIG_BPF_SYSCALL) += syscall.o verifier.o
ifdef CONFIG_TEST_BPF
obj-$(CONFIG_BPF_SYSCALL) += test_stub.o
endif
