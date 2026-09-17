
TARGET_LIB  = libalg_dmm.a

include ../../../Makefile.param

ifeq ($(BOARD), DMS31V2)
ifeq ($(BOARD), ADA42V1)
ifeq ($(BOARD), ADA42PTZV1)
else
IGNORE_THIS_BUILD = dmm
endif

CPPFLAGS += -O2
CPPFLAGS += -I$(INC_PATH) -I$(INC_PATH)/libdrm -I$(INC_PATH)/libdrm/drm -I../include -I../include/rknn -I../include/ncnn

_TARGET_DIR_ = $(TOP_LIB)

include $(AUTO_DEP_MK) 
