ifeq ($(call is-board-platform-in-list, kalama), true)
AKS_JS_DLKM_ENABLE := true
ifeq ($(TARGET_KERNEL_DLKM_DISABLE), true)
	AKS_JS_DLKM_ENABLE := false
endif

ifeq ($(AKS_JS_DLKM_ENABLE),  true)
	PRODUCT_PACKAGES += $(KERNEL_MODULES_OUT)/aks-adc-joystick.ko \
		$(KERNEL_MODULES_OUT)/aks-ads1015.ko

	PRODUCT_COPY_FILES += vendor/qcom/opensource/ecosw/joystick/aks/kernel/aks_js_drivers/Vendor_2212_Product_0010.kl:$(TARGET_COPY_OUT_VENDOR)/usr/keylayout/Vendor_2212_Product_0010.kl
endif
endif
