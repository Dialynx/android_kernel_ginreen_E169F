include $(srctree)/drivers/misc/mediatek/Makefile.custom
#liuying 20150901 add begin
CONFIG_MTK_PLATFORM_SENSOR := $(CONFIG_MTK_PLATFORM)

ifeq ($(CONFIG_ARCH_MT6735M),y)
CONFIG_MTK_PLATFORM_SENSOR := mt6735m
endif

ifeq ($(CONFIG_ARCH_MT6753),y)
CONFIG_MTK_PLATFORM_SENSOR := mt6753
endif

ifeq ($(CONFIG_ARCH_MT6735),y)
CONFIG_MTK_PLATFORM_SENSOR := mt6735
endif

# hebiao 20151203 begin
#wangkangmin 20150729 add begin
#zhenglihong 20150707 add begin
ifeq ($(CONFIG_ARCH_MTK_PROJECT),"A165")
obj-y += $(subst ",,a165)/
else
ifeq ($(CONFIG_MTK_LENS_E200L_SUPPORT),y)
obj-y += $(subst ",,e200)/
else
ifeq ($(CONFIG_ARCH_MTK_PROJECT),"E167L")
obj-y += $(subst ",,e167)/
else
ifeq ($(CONFIG_ARCH_MTK_PROJECT),"E167L_CU_35M")
obj-y += $(subst ",,e167_cmcc)/
else
ifeq ($(CONFIG_MTK_LENS_E169_SUPPORT),y)
obj-y += $(subst ",,e169)/
else
ifeq ($(CONFIG_ARCH_MTK_PROJECT),"E165L_CMCC")
obj-y += $(subst ",,e165_cmcc)/
else
ifeq ($(CONFIG_MTK_LENS_E169_TW_SUPPORT),y)
obj-y += $(subst ",,e169_tw)/
else
ifeq ($(CONFIG_ARCH_MTK_PROJECT),"V310F")
obj-y += $(subst ",,v310)/
else
ifeq ($(CONFIG_ARCH_MTK_PROJECT),"E165L")
obj-y += $(subst ",,e165)/
else
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM_SENSOR))/
endif
endif
endif
endif
endif
endif
endif
endif
endif
#liuying 20150901 add end
#zhenglihong 20150707 add end
#wangkangmin 20150729 add end
# hebiao 20151203 end

