TARGETS = breakpoints
TARGETS += cpu-hotplug
TARGETS += efivarfs
TARGETS += kcmp
TARGETS += memfd
TARGETS += memory-hotplug
TARGETS += mqueue
TARGETS += mount
TARGETS += net
TARGETS += ptrace
TARGETS += timers
TARGETS += vm
TARGETS += powerpc
TARGETS += user
TARGETS += sysctl
TARGETS += firmware
TARGETS += ftrace

TARGETS_HOTPLUG = cpu-hotplug
TARGETS_HOTPLUG += memory-hotplug

# Clear LDFLAGS and MAKEFLAGS if called from main
# Makefile to avoid test build failures when test
# Makefile doesn't have explicit build rules.
ifeq (1,$(MAKELEVEL))
undefine LDFLAGS
override MAKEFLAGS =
endif

all:
	for TARGET in $(TARGETS); do \
		make -C $$TARGET; \
	done;

run_tests: all
	for TARGET in $(TARGETS); do \
		make -C $$TARGET run_tests; \
	done;

hotplug:
	for TARGET in $(TARGETS_HOTPLUG); do \
		make -C $$TARGET; \
	done;

run_hotplug: hotplug
	for TARGET in $(TARGETS_HOTPLUG); do \
		make -C $$TARGET run_full_test; \
	done;

clean_hotplug:
	for TARGET in $(TARGETS_HOTPLUG); do \
		make -C $$TARGET clean; \
	done;

clean:
	for TARGET in $(TARGETS); do \
		make -C $$TARGET clean; \
	done;
