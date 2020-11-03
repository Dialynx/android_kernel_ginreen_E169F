#add by liupeng 201500721 begin
include $(srctree)/drivers/misc/mediatek/Makefile.custom


# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/ft5x46/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ft5x46/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/


obj-y	+= focaltech_ctl.o
obj-y	+= focaltech_driver.o
obj-y	+= focaltech_ex_fun.o

#path := $(if $(filter yes,$(KBUILD_OUTPUT_SUPPORT)),$(srctree)/$(obj),$(obj))
#obj-y := $(patsubst %.c,%.o,$(subst $(path)/,,$(call wildcard2,$(path)/*.c)))
#obj-y += ft_gesture_lib.a
#add by liupeng 201500721 end