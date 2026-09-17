
TARGET_LIB  = libalg_apc.a

include ../../../Makefile.param

ifneq ($(BOARD),$(findstring $(BOARD),ADA900V1 ADA47V1))
IGNORE_THIS_BUILD = apc
endif

CPPFLAGS += -O2
CPPFLAGS += -I$(INC_PATH) -I$(INC_PATH)/cjson -I$(INC_PATH)/libdrm -I$(INC_PATH)/libdrm/drm -I../include -I../include/rknn -I../include/ncnn

_TARGET_DIR_ = $(TOP_LIB)

include $(AUTO_DEP_MK) 
