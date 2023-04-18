# Android makefile for BT kernel modules

LOCAL_PATH := $(call my-dir)

## Build/Package only in case of supported target
ifeq ($(call is-board-platform-in-list, kalama), true)

JS_SELECT := CONFIG_AKS_ADC_JOYSTICK=m
##ifdef CONFIG_SLIMBUS
JS_SELECT += CONFIG_AKS_ADS1015=m
##endif
#BT_SELECT += CONFIG_I2C_RTC6226_QCA=m

LOCAL_PATH := $(call my-dir)

# This makefile is only for DLKM
ifneq ($(findstring vendor,$(LOCAL_PATH)),)

ifneq ($(findstring opensource,$(LOCAL_PATH)),)
	JS_BLD_DIR := $(abspath .)/vendor/qcom/opensource/ecosw/aks_js_drivers
endif # opensource

DLKM_DIR := $(TOP)/device/qcom/common/dlkm

LOCAL_ADDITIONAL_DEPENDENCIES := $(wildcard $(LOCAL_PATH)/**/*) $(wildcard $(LOCAL_PATH)/*)

# Build
###########################################################
# This is set once per LOCAL_PATH, not per (kernel) module
KBUILD_OPTIONS := JS_KERNEL_ROOT=$(JS_BLD_DIR)
KBUILD_OPTIONS += $(foreach js_select, \
       $(JS_SELECT), \
       $(js_select))
JS_SRC_FILES := \
	$(wildcard $(LOCAL_PATH)/*) \
	$(wildcard $(LOCAL_PATH)/*/*) \
	
KBUILD_OPTIONS += MODNAME=aks_js_dlkm
KBUILD_OPTIONS += BOARD_PLATFORM=$(TARGET_BOARD_PLATFORM)


# Below are for Android build system to recognize each module name, so
# they can be installed properly. Since Kbuild is used to compile these
# modules, invoking any of them will cause other modules to be compiled
# as well if corresponding flags are added in KBUILD_OPTIONS from upper
# level Makefiles.

################################ aks-adc-joystick ################################
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(JS_SRC_FILES)
LOCAL_MODULE              := aks-adc-joystick.ko
LOCAL_MODULE_KBUILD_NAME  := aks-adc-joystick/aks-adc-joystick.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk
################################## aks-ads1015 ###################################
include $(CLEAR_VARS)
LOCAL_SRC_FILES           := $(JS_SRC_FILES)
LOCAL_MODULE              := aks-ads1015.ko
LOCAL_MODULE_KBUILD_NAME  := aks-ads1015/aks-ads1015.ko
LOCAL_MODULE_TAGS         := optional
LOCAL_MODULE_DEBUG_ENABLE := true
LOCAL_MODULE_PATH         := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/Build_external_kernelmodule.mk

endif # DLKM check
endif # target check
