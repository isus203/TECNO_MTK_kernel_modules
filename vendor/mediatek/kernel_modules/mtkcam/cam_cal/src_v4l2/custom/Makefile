# SPDX-License-Identifier: GPL-2.0-only

include $(wildcard $(KERNEL_SRC)/$(DEVICE_MODULES_REL_DIR)/Makefile.include)

# add for imgsensor.ko Shared variables to camera_eeprom_v4l2.ko by haofeng.liang 2024.11.12 start
EXTRA_SYMBOLS += $(OUT_DIR)/../vendor/mediatek/kernel_modules/mtkcam/imgsensor/src-v4l2/Module.symvers
# add for imgsensor.ko Shared variables to camera_eeprom_v4l2.ko by haofeng.liang 2024.11.12 end
modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" $(@)
