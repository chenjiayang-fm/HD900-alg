#
# Link all library, and build the final excutable file
#

include ../../Makefile.param
SRCS = $(wildcard ./common/*.c)
SRCS += $(wildcard ./*.c)

ifneq ($(BOARD), IPCR20S2)
IGNORE_THIS_BUILD = DynamicTrace
endif

# link pthread and math library
SYSTEM_LIB	= -lpthread -lm 

# Link SV Common library
SV_COM_LIBS = -lcontrol -lmedia -lfreetype -lpng -lz -lnetwork -lboard -llog -lconfig -lmxml -lhttp \
			-lsharefifo -lmsg -lsafefunc -lcjson  -luuid -lonvif -lrtsp -lliveMedia -lgroupsock -lBasicUsageEnvironment -lUsageEnvironment

ifeq ($(BOARD), WFCR10S1)
SV_COM_LIBS += -lmculink -lwf697 -lwifi -lzbar
endif
ifeq ($(BOARD), WFCR20S1)
SV_COM_LIBS += -lmculink -lwf697 -lwifi -lzbar
endif
ifeq ($(BOARD), WFTR20S1)
SV_COM_LIBS += -lwf697 -lwifi
endif
SV_COM_LIBS += -lssl -lcrypto -ldl
# Link other SV libs
OTHER_SV_LIBS 	= 
TARGET_LIB  = libDynamicTrace.a

# Link other Hisi libs
ifeq ($(PLATFORM), HI3516C)
CFLAGS += -DPLATFORM=PLATFORM_HI3516C
HISIL_LIBS 	= -lmpi -live -lmd -ldnvqe -lupvqe -lVoiceEngine -l_hiae -lisp -lsns_imx32x -l_hiae -l_hiawb -l_hiaf -l_hidefog
ifeq ($(BOARD), WFTR20S1)
HISIL_LIBS := $(HISIL_LIBS:-lsns_imx32x=)
endif
endif

ifeq ($(PLATFORM), HI3518E)
CFLAGS += -DPLATFORM=PLATFORM_HI3518E
HISIL_LIBS 	= -lmpi -live -lmd -ldnvqe -lupvqe -lVoiceEngine -lisp -lsns_imx22x -l_cmoscfg -l_iniparser -l_hiae -l_hiawb -l_hiaf -l_hidefog
endif

ifeq ($(PLATFORM), HI3516E)
CFLAGS += -DPLATFORM=PLATFORM_HI3516E
HISIL_LIBS 	= -lmpi -live -lmd -ldnvqe -lupvqe -lVoiceEngine -l_hiae -lisp -l_hidehaze -l_hidrc -l_hildci -l_hiawb -lsns_imx307 -lsecurec -lmpi -ltde
endif

CFLAGS += -L$(TOP_LIB)
CFLAGS += -I./ -I./include -I$(INC_PATH)/freetype -I../scene_auto/include -I../scene_auto/tools/configaccess/include -I../scene_auto/tools/iniparser/include

CPPFLAGS = $(CFLAGS)

TARGET_BIN	= DynamicTrace
LIB_DEPEND	= $(COMP_DEPEND)
LD_FLAGS	+=$(SYSTEM_LIB) $(OTHER_SV_LIBS) $(SV_COM_LIBS) $(HISIL_LIBS)

COPY_TO_DIR = $(ROOT_PATH)
include $(BIN_AUTO_DEP_MK)

# vim:noet:sw=4:ts=4

