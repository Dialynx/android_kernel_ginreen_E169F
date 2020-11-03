
include $(srctree)/drivers/misc/mediatek/Makefile.custom

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y   += core/
obj-y   += dct/dct/
endif

#obj-y   +=  lcm/
obj-y   +=  camera/camera/
obj-$(CONFIG_MTK_VIBRATOR)   +=  vibrator/
obj-$(CONFIG_MTK_LEDS)   +=  leds/
#obj-y   +=  keypad/
#obj-y   +=  sound/
obj-y   +=  accdet/

ifeq ($(CONFIG_MTK_SENSOR_SUPPORT),y)
ifeq ($(CONFIG_MTK_CM36652_NEW),y)
obj-y	+=  alsps/cm36652-new/
endif
#guohongjin debug sensor begin
ifeq ($(CONFIG_MTK_EPL2182_NEW),y)
obj-y	+=  alsps/epl2182-new/
endif
ifeq ($(CONFIG_MTK_MC3410_NEW),y)
obj-y   +=  accelerometer/mc3410-new/
endif

ifeq ($(CONFIG_MTK_LIS3DH_NEW),y)
obj-y   +=  accelerometer/lis3dh-new/
endif
#tuwenzan add at 20151102 begin
ifeq ($(CONFIG_MTK_KXTJ2_1009_NEW),y)
obj-y   +=  accelerometer/KXTJ2_1009-new/
endif
#tuwenzan add at 20151102 end
#guohongjin debug sensor end
ifeq ($(CONFIG_MTK_ITG1010_NEW),y)
obj-y   +=  gyroscope/ITG1010-new/
endif

ifeq ($(CONFIG_MTK_AKM09911_NEW),y)
obj-y	+=  magnetometer/akm09911-new/
endif
endif
