include $(srctree)/drivers/misc/mediatek/Makefile.custom

#zhenglihong 20150707 add begin
#liuying 20150806 add begin
ifeq ($(CONFIG_MTK_LENS_E200L_SUPPORT),y)
obj-y += $(subst ",,e200)/
else
ifeq ($(CONFIG_ARCH_MTK_PROJECT),"E165L")
obj-y += $(subst ",,e165)/
else
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
endif
endif
#liuying 20150806 add end
#zhenglihong 20150707 add end


