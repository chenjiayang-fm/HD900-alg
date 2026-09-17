#
# Link all library, and build the final excutable file
#

include ../../Makefile.param

ifneq ($(PLATFORM),$(findstring $(PLATFORM),RV1126,RV1106))
IGNORE_THIS_BUILD = alg
endif

ifeq ($(BOARD),$(findstring $(BOARD),DMS31V2,DMS31SDK))
SRCPPS = $(wildcard ./dmm/*.cpp)
SV_COM_LIBS = -ldmm
endif
ifeq ($(BOARD),$(findstring $(BOARD),ADA32V2,ADA32SDK,ADA32N1))
SRCPPS = $(wildcard ./pd/*.cpp)
SV_COM_LIBS = -lpds_general_rv1126  -lcalibrate_radar -ljpeg
endif
ifeq ($(BOARD), ADA32V3)
SRCPPS = $(wildcard ./pd/*.cpp)
SV_COM_LIBS = -lpds_general_rv1106 -ljpeg
endif
ifeq ($(BOARD), ADA32IR)
SRCPPS = $(wildcard ./pd/*.cpp)
SV_COM_LIBS = -lpds_general_rv1126 -lcalibrate_radar -ljpeg
endif
ifeq ($(BOARD), ADA42V1)
SRCPPS = $(wildcard ./pd/*.cpp)
SRCPPS += $(wildcard ./dmm/*.cpp)
SRCPPS += $(wildcard ./adas/*.cpp)
SV_COM_LIBS = -lpds_general_rv1126 -lcalibrate_radar -ljpeg -ldmm -lLANE -lLANEPROCESS 
endif
ifeq ($(BOARD), ADA42PTZV1)
SRCPPS = $(wildcard ./pd/*.cpp)
SRCPPS += $(wildcard ./dmm/*.cpp)
SRCPPS += $(wildcard ./adas/*.cpp)
SRCPPS += $(wildcard ./track/*.cpp)
SV_COM_LIBS = -lpds_general_rv1126 -lcalibrate_radar -ljpeg -ldmm -lLANE -lLANEPROCESS -ltrack
endif
ifeq ($(BOARD), ADA47V1)
SRCPPS = $(wildcard ./dmm/*.cpp)
SRCPPS += $(wildcard ./pd/*.cpp)
SV_COM_LIBS = -ldmm -lpds_general_rv1126 -lcalibrate_radar -ljpeg
endif
ifeq ($(BOARD), ADA900V1)
SRCPPS = $(wildcard ./apc/*.cpp)
SV_COM_LIBS = -lpds_general_rv1126 -ljpeg
endif
ifeq ($(BOARD), HDW845V1)
SV_COM_LIBS =-lpds_general_rv1126 -lcalibrate_radar -ljpeg
endif

ifeq ($(PLATFORM), RV1126)
CFLAGS += -DPLATFORM=PLATFORM_RV1126 -I../ipserver/someip/include -std=c++11
CFLAGS += -DPLATFORM=PLATFORM_RV1126 -D_LARGEFILE_SOURCE -D_LARGEFILE64_SOURCE -D_FILE_OFFSET_BITS=64 -DUSE_UPDATEENGINE=ON -DSUCCESSFUL_BOOT=ON  -DNDEBUG  -rdynamic
HISIL_LIBS = -leasymedia -ldrm -lrockchip_mpp -lavformat -lavcodec -lswresample -lavutil \
			 -lliveMedia -lgroupsock -lBasicUsageEnvironment -lUsageEnvironment -lpthread \
			 -lasound -lRKAP_3A -lRKAP_ANR -lRKAP_Common -lrga \
			 -lsqlite3 -lmd_share -lod_share -lrkaiq -lssl -lcrypto -lz \
			 -lv4l2 -lv4lconvert \
			 -Wl,--as-needed

ifneq ($(findstring $(BOARD), ADA42V1 ADA42PTZV1 ADA47V1 ADA900V1 HDW845V1),)
HISIL_LIBS += -leasymedia -ldrm -lrockchip_mpp -lavformat -lavcodec -lswresample -lavutil\
			 -lpthread -lasound -lRKAP_3A -lRKAP_ANR -lRKAP_Common -lrga \
			 -lsqlite3 -lmd_share -lod_share -lrkaiq -lssl -lcrypto -lz \
			 -lv4l2 -lv4lconvert
endif

endif

SRCPPS += alg.cpp
# Link SV Common library
SV_COM_LIBS +=  -lalarm -lcjson -lsharefifo -llog -lstorage -lmsg -lcjson -lutils -lboard -lconfig -lmxml -luuid -lsafefunc -lavcodec -lavformat -lavutil -lswresample  -lthpool 

################## optional library ######################
ifneq ($(findstring -DMAKE_LED,$(CFLAGS) $(CPPFLAGS)), )
SV_COM_LIBS += -lled
endif

##########################################################
# Link other SV libs
OTHER_SV_LIBS	= -lrknn_api -lrga -lrockchip_mpp -lopticalflow

ifneq ($(BOARD), ADA32V3)
OTHER_SV_LIBS	+=  -lv4l2 -lv4lconvert -ldrm -lmedia
endif

SYSTEM_LIB	= -lssl -lcrypto -ldl -lrt -lpthread -lm -lstdc++ -lz\

#FFMPEG_DIR = ffmpeg4.1.3
PROTOCOL_DIR = $(shell pwd)/protocol


CFLAGS += -I$(INC_PATH)/libdrm -I$(INC_PATH)/libdrm/drm -I./include -I./include/rknn -I./include/ncnn -I$(INC_PATH)/cjson -I$(INC_PATH)/jpeg -I../media/include -I../media/rockchip/rv1126/rkmedia/\
-I../media/rockchip/rv1126/include2/
#CFLAGS += -I./ffmpeg4.1.3/include
CPPFLAGS = $(CFLAGS)

TARGET_BIN	= alg
COPY_TO_DIR = $(ROOT_PATH)

LIB_DEPEND	= $(COMP_DEPEND)
LD_FLAGS	+= -L$(LIB_PATH) -L$(TOP_LIB) 
LD_FLAGS	+= $(SV_COM_LIBS) $(OTHER_SV_LIBS) $(SYSTEM_LIB) $(HISIL_LIBS)

include $(BIN_AUTO_DEP_MK)

# vim:noet:sw=4:ts=4
