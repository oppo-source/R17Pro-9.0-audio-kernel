# Android makefile for audio kernel modules
MY_LOCAL_PATH := $(call my-dir)

UAPI_OUT := $(OUT)/obj/vendor/qcom/opensource/audio-kernel/include

ifeq ($(call is-board-platform-in-list,msm8953 sdm845 sdm710 qcs605),true)
$(shell mkdir -p $(UAPI_OUT)/linux;)
$(shell mkdir -p $(UAPI_OUT)/sound;)
$(shell rm -rf $(OUT)/obj/vendor/qcom/opensource/audio-kernel/ipc/Module.symvers)
$(shell rm -rf $(OUT)/obj/vendor/qcom/opensource/audio-kernel/dsp/Module.symvers)
$(shell rm -rf $(OUT)/obj/vendor/qcom/opensource/audio-kernel/dsp/codecs/Module.symvers)
$(shell rm -rf $(OUT)/obj/vendor/qcom/opensource/audio-kernel/soc/Module.symvers)
$(shell rm -rf $(OUT)/obj/vendor/qcom/opensource/audio-kernel/asoc/Module.symvers)
$(shell rm -rf $(OUT)/obj/vendor/qcom/opensource/audio-kernel/asoc/codecs/Module.symvers)
$(shell rm -rf $(OUT)/obj/vendor/qcom/opensource/audio-kernel/asoc/codecs/wcd934x/Module.symvers)

include $(MY_LOCAL_PATH)/ipc/Android.mk
include $(MY_LOCAL_PATH)/dsp/Android.mk
include $(MY_LOCAL_PATH)/dsp/codecs/Android.mk
include $(MY_LOCAL_PATH)/soc/Android.mk
include $(MY_LOCAL_PATH)/asoc/Android.mk
include $(MY_LOCAL_PATH)/asoc/codecs/Android.mk
include $(MY_LOCAL_PATH)/asoc/codecs/wcd934x/Android.mk
endif

ifeq ($(call is-board-platform-in-list,sdm710 qcs605),true)
$(shell rm -rf $(PRODUCT_OUT)/obj/vendor/qcom/opensource/audio-kernel/asoc/codecs/aqt1000/Module.symvers)
include $(MY_LOCAL_PATH)/asoc/codecs/aqt1000/Android.mk
endif

ifeq ($(call is-board-platform-in-list,msm8953 sdm710 qcs605),true)
$(shell rm -rf $(OUT)/obj/vendor/qcom/opensource/audio-kernel/asoc/codecs/sdm660_cdc/Module.symvers)
$(shell rm -rf $(OUT)/obj/vendor/qcom/opensource/audio-kernel/asoc/codecs/msm_sdw/Module.symvers)
include $(MY_LOCAL_PATH)/asoc/codecs/sdm660_cdc/Android.mk
include $(MY_LOCAL_PATH)/asoc/codecs/msm_sdw/Android.mk
endif

#ifdef VENDOR_EDIT
#Jianfeng.Qiu@PSW.MM.AudioDriver.Codec, 2018/04/20, Add for tfa9894 codec
include $(MY_LOCAL_PATH)/asoc/codecs/tfa98xx-v6/Android.mk
#endif /* VENDOR_EDIT */

#ifdef VENDOR_EDIT
#Jianfeng.Qiu@PSW.MM.AudioDriver.Codec, 2018/04/20, Add for ak43xx codec
include $(MY_LOCAL_PATH)/asoc/codecs/ak4376/Android.mk
#endif /* VENDOR_EDIT */

#ifdef VENDOR_EDIT
#Jianfeng.Qiu@PSW.MM.AudioDriver.Codec.1263116, 2018/05/15, Add for smart mic
include $(MY_LOCAL_PATH)/asoc/codecs/audience/Android.mk
#endif /* VENDOR_EDIT */

#ifdef VENDOR_EDIT
#Huiqun.Han@PSW.MM.AudioDriver.Codec, 2018/06/28, Add for max20328
ifneq ($(filter MSM_18181 MSM_18385, $(OPPO_TARGET_DEVICE)),)
include $(MY_LOCAL_PATH)/asoc/codecs/max20328/Android.mk
endif
#endif /* VENDOR_EDIT */
