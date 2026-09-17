#
# Link all library, and build the final excutable file
#

include ../../../../Makefile.param

ifneq ($(PLATFORM), RV1126)
IGNORE_THIS_BUILD = gui_demo
endif

SRCPPS += main.cpp
# Link SV Common library
SV_COM_LIBS +=  -lalarm -lcjson -lsharefifo -llog -lstorage -lmsg -lcjson -lutils -lboard -lconfig -lmxml -luuid -lsafefunc

# Link other SV libs
OTHER_SV_LIBS	= -lrknn_api -lv4l2 -lv4lconvert -lrga -ldrm -lrockchip_mpp

SYSTEM_LIB	= -lssl -lcrypto -ldl -lrt -lpthread -lm -lstdc++

CFLAGS += -I$(INC_PATH)/libdrm -I$(INC_PATH)/libdrm/drm -I./include -I./include/rknn -I./include/ncnn -I$(INC_PATH)/cjson
CPPFLAGS = $(CFLAGS)

TARGET_BIN	= gui_demo

LIB_DEPEND	= $(COMP_DEPEND)
LD_FLAGS	+= -L$(LIB_PATH) -L$(TOP_LIB) 
LD_FLAGS	+= $(SV_COM_LIBS) $(OTHER_SV_LIBS) $(SYSTEM_LIB)

include $(BIN_AUTO_DEP_MK)

# vim:noet:sw=4:ts=4
