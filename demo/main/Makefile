#
# Link all library, and build the final excutable file
#

include ../../../../Makefile.param

ifneq ($(PLATFORM), RV1126)
IGNORE_THIS_BUILD = main_demo
endif

SRCPPS += main.cpp
SRCPPS += ../rknpu/examples/rknn_yolov5_demo/src/alglib.cpp
SRCPPS += ../rknpu/examples/rknn_yolov5_demo/src/postprocess.cpp
OBJS := $(SRCPPS:%.c=%.o)

# Link SV Common library
SV_COM_LIBS +=  -lalarm -lcjson -lsharefifo -llog -lstorage -lmsg -lcjson -lutils -lboard -lconfig -lmxml -luuid -lsafefunc

# Link other SV libs
OTHER_SV_LIBS	= -lrknn_api -lv4l2 -lv4lconvert -lrga -ldrm -lrockchip_mpp \
				  -lopencv_calib3d -lopencv_features2d -lopencv_imgcodecs -lopencv_video -lopencv_imgproc -lopencv_core   \
				  -llibjpeg-turbo -llibwebp -llibpng -llibtiff -llibjasper -lIlmImf -ltegra_hal -lzlib

SYSTEM_LIB	= -lssl -lcrypto -ldl -lrt -lpthread -lm -lstdc++

CFLAGS += -fpermissive -I$(INC_PATH)/libdrm -I$(INC_PATH)/libdrm/drm -I../rknpu/librknn_api/include \
			-I../rknpu/examples/3rdparty/opencv/opencv-linux-armhf/include \
			-I../rknpu/examples/rknn_yolov5_demo/include -I$(INC_PATH)/cjson
CPPFLAGS = $(CFLAGS)

TARGET_BIN	= main_demo
COPY_TO_DIR = ./install/rknn_yolov5_demo

LIB_DEPEND	= $(COMP_DEPEND)
LD_FLAGS	+= -L$(LIB_PATH) -L$(TOP_LIB)
LD_FLAGS	+= $(SV_COM_LIBS) $(OTHER_SV_LIBS) $(SYSTEM_LIB)
LD_FLAGS	+= -L../rknpu/examples/3rdparty/opencv/opencv-linux-armhf/lib
LD_FLAGS	+= -L../rknpu/examples/3rdparty/opencv/opencv-linux-armhf/share/OpenCV/3rdparty/lib

.PHONY:	clean strip all

all: $(COPY_TO_DIR)/$(TARGET_BIN)

$(COPY_TO_DIR)/$(TARGET_BIN): $(TARGET_BIN)
	@$(MKDIR) -p $(COPY_TO_DIR)
	@$(CP) run_rv1109_rv1126.sh $(COPY_TO_DIR)/
	@$(CP) -rf ../rknpu/examples/rknn_yolov5_demo/model $(COPY_TO_DIR)/
	@$(MV) $(TARGET_BIN) $(COPY_TO_DIR)/

$(TARGET_BIN):$(OBJS) $(LIB_DEPEND)
	@$(ECHO) Compiling $@, changed dep:$?
	
ifeq ($(SRCPPS), )	
	$(CC) -o $@ $(OBJS) $(CFLAGS) $(LD_FLAGS)
else
	$(CXX) -o $@ $(OBJS) $(CFLAGS) $(CPPFLAGS) $(LD_FLAGS)
endif	

ifneq ($(DEBUG), y)
	$(STRIP) -S $(TARGET_BIN)
endif
	@$(ECHO)  "==========Excutable binary" $@ "Build OK ==========" 
	
$(OBJS):$(DEPEND_FILE_NAME)
$(DEPEND_FILE_NAME):$(SRCPPS)
	-$(RM)	$@
	
ifneq ($(SRCPPS), )
	for x in $(SRCPPS);	\
	do			\
		$(CXX) -MM -MT "$${x%.cpp}.o" $(CPPFLAGS) $$x >> $@;\
	done	
endif

clean:
	@$(RM) $(MAKE_LOG_FILE)
	@$(RM) $(DEPEND_FILE_NAME)
	@$(RM) -rf $(COPY_TO_DIR)

strip:
	$(STRIP) -s $(AUTOUPDATE)

include $(BUILD_MK)

# vim:noet:sw=4:ts=4