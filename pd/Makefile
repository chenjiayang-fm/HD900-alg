
TARGET_LIB  = libalg_pd.a

include ../../../Makefile.param

ifneq ($(BOARD),$(findstring $(BOARD),ADA32V2,ADA32IR,ADA32N1,ADA42V1,ADA42PTZV1,ADA32C4))
IGNORE_THIS_BUILD = pd
endif

CPPFLAGS += -O2
CPPFLAGS += -I$(INC_PATH) -I$(INC_PATH)/cjson -I$(INC_PATH)/libdrm -I$(INC_PATH)/libdrm/drm -I../include -I../include/rknn -I../include/ncnn -I../../media/rockchip/rv1126/include2/rkmedia/rkmedia_api.h

_TARGET_DIR_ = $(TOP_LIB)

include $(AUTO_DEP_MK) 
