-include $(TOPDIR)/Make.defs

APPNAME = bmi055_example
PRIORITY   = SCHED_PRIORITY_DEFAULT
STACKSIZE  = 1024

ASRCS =
CSRCS =
MAINSRC = bmi055_example_main.c

CFLAGS += -I../../../nuttx/arch/arm/src -I../../../nuttx/arch/arm/src/common

CONFIG_EXAMPLES_BMI055_EXAMPLE_PROGNAME ?= bmi055_example$(EXEEXT)
PROGNAME = $(CONFIG_EXAMPLES_BMI055_EXAMPLE_PROGNAME)

include $(APPDIR)/Application.mk
