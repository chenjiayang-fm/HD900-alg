TARGET_LIB  = libalg_zoom.a

include ../../../Makefile.param

ifneq ($(BOARD),$(findstring $(BOARD),HDW845V1))
IGNORE_THIS_BUILD = zoom
endif

ifeq ($(findstring -DMAKE_STORAGE,$(CFLAGS) $(CPPFLAGS)), )
IGNORE_THIS_BUILD = storage
endif

CPPFLAGS += -O2
CPPFLAGS += -I$(INC_PATH) -I$(INC_PATH)/cjson -I$(INC_PATH)/libdrm -I$(INC_PATH)/libdrm/drm -I../include -I../include/rknn -I../include/ncnn

_TARGET_DIR_ = $(TOP_LIB)

include $(AUTO_DEP_MK) 