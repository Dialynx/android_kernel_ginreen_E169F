#//huangzhaosong 201500713 s--
include $(srctree)/drivers/misc/mediatek/Makefile.custom

#tuwenzan modify at 20150901 begin
# Linux driver folder
#ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/ft5446i_pop3_5/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/ft5446/
#ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ft5446i_pop3_5/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
#tuwenzan modify at 20150901 begin

obj-y	+= focaltech_ctl.o
obj-y	+= focaltech_driver.o
obj-y	+= focaltech_ex_fun.o

#path := $(if $(filter yes,$(KBUILD_OUTPUT_SUPPORT)),$(srctree)/$(obj),$(obj))
#obj-y := $(patsubst %.c,%.o,$(subst $(path)/,,$(call wildcard2,$(path)/*.c)))
#obj-y += ft_gesture_lib.a
#//huangzhaosong 201500713 e--