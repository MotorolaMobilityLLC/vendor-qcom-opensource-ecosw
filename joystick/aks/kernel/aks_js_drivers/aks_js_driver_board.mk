ifeq ($(call is-board-platform-in-list, kalama), true)
AKS_JS_DLKM_ENABLE := true
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	AKS_JS_DLKM_ENABLE := false
endif

ifeq ($(AKS_JS_DLKM_ENABLE),  true)
	ifneq ($(TARGET_BOARD_AUTO),true)
		ifeq ($(call is-board-platform-in-list,$(TARGET_BOARD_PLATFORM)),true)
			BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/aks-adc-joystick.ko \
				$(KERNEL_MODULES_OUT)/aks-ads1015.ko
		endif
	endif
endif
endif
