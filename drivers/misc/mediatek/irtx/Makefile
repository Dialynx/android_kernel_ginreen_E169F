$(info IRTX: PWM+IRTX:$(CONFIG_MTK_IRTX_HW_SUPPORT))
$(info IRTX: PWM ONLY:$(CONFIG_MTK_IRTX_SUPPORT))

ifeq ($(CONFIG_MTK_IRTX_HW_SUPPORT), y)

include $(srctree)/drivers/misc/mediatek/Makefile.custom
obj-y := mt_irtx.o

endif

ifeq ($(CONFIG_MTK_IRTX_SUPPORT), y)

include $(srctree)/drivers/misc/mediatek/Makefile.custom
obj-y := mt_irtx_pwm.o

endif
