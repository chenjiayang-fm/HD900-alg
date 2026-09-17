
TARGET_LIB  = libalg_track.a

include ../../../Makefile.param

ifeq ( ,$(findstring $(BOARD), ADA42PTZV1))
IGNORE_THIS_BUILD = track
endif

CPPFLAGS += -O2
CPPFLAGS += -I$(INC_PATH) -I$(INC_PATH)/libdrm -I$(INC_PATH)/libdrm/drm -I$(INC_PATH)/cjson -I../include -I../include/rknn -I../include/ncnn

_TARGET_DIR_ = $(TOP_LIB)

include $(AUTO_DEP_MK) 
