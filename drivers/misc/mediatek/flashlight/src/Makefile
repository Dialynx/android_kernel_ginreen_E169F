include $(srctree)/drivers/misc/mediatek/Makefile.custom
#hebiao 20151204 begin
#wangkangmin 20150729 add begin
#liuying 20150914 add begin
#zhenglihong 20150721 add begin
ifeq ($(CONFIG_MTK_LENS_E169_SUPPORT),y)
obj-y += $(subst ",,e169)/
else
ifeq ($(CONFIG_MTK_LENS_E200L_SUPPORT),y)
obj-y += $(subst ",,e200)/
else
ifeq ($(CONFIG_ARCH_MTK_PROJECT),"A165")
obj-y += $(subst ",,a165)/
else
ifeq ($(CONFIG_ARCH_MTK_PROJECT),"E167L")
obj-y += $(subst ",,e167)/
else
ifeq ($(CONFIG_ARCH_MTK_PROJECT),"E167L_CU_35M")
obj-y += $(subst ",,e167_cu_35m)/
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
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/
endif
endif
endif
endif
endif
endif
endif
endif
endif
#wangkangmin 20150729 add end
#liuying 20150914 add end
#zhenglihong 20150721 add end
#hebiao 20151204 end

