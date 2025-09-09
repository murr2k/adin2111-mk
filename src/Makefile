#
# Makefile for the Analog Devices ADIN2111-MK network device drivers.
# Author: Murray Kopit <murr2k@gmail.com>
#

obj-$(CONFIG_ADIN2111_MK) += adin2111-mk.o

# Build dependencies
adin2111-mk-y := adin2111-mk.o

# Additional flags for compilation
ccflags-y += -I$(src)

# Enable additional debugging if needed
ifdef CONFIG_DYNAMIC_DEBUG
ccflags-y += -DDEBUG
endif