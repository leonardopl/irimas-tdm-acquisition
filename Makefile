# TDM Holographic Tomography Acquisition
# Requires: rc_genicam_api, LabJack, OpenCV 4, Boost

CXX        = g++
OBJDIR     = obj
BINDIR     = bin

# Use pkg-config for OpenCV; fall back to manual flags if .pc not found
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null)
OPENCV_LIBS   := $(shell pkg-config --libs opencv4 2>/dev/null)
ifeq ($(OPENCV_LIBS),)
  OPENCV_CFLAGS := -I/usr/include/opencv4
  OPENCV_LIBS   := -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc
endif

# Detect rc_genicam_api: search known prefixes for headers and library
RC_GENICAM_SEARCH_PREFIXES = /usr /usr/local

# Find the prefix that contains the genicam GenApi headers
RC_GENICAM_PREFIX := $(firstword $(foreach p,$(RC_GENICAM_SEARCH_PREFIXES),\
  $(if $(wildcard $(p)/include/rc_genicam_api/genicam/GenApi/GenApi.h),$(p))))

# Find the library directory (multi-arch or plain lib/)
RC_GENICAM_LIBDIR := $(firstword \
  $(wildcard $(RC_GENICAM_PREFIX)/lib/$(shell dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null)/librc_genicam_api.so) \
  $(wildcard $(RC_GENICAM_PREFIX)/lib/librc_genicam_api.so))
RC_GENICAM_LIBDIR := $(patsubst %/librc_genicam_api.so,%,$(RC_GENICAM_LIBDIR))

ifneq ($(RC_GENICAM_PREFIX),)
  RC_GENICAM_CFLAGS := -I$(RC_GENICAM_PREFIX)/include -I$(RC_GENICAM_PREFIX)/include/rc_genicam_api/genicam
  RC_GENICAM_LIBS   := $(if $(RC_GENICAM_LIBDIR),-L$(RC_GENICAM_LIBDIR)) -lrc_genicam_api
else
  $(warning rc_genicam_api not found — install it or set RC_GENICAM_PREFIX manually)
  RC_GENICAM_CFLAGS :=
  RC_GENICAM_LIBS   := -lrc_genicam_api
endif

CXXFLAGS = -D_UNIX_ -D_LINUX_ \
           -Iinclude -Iinclude/LabJack -Iinclude/john \
           $(OPENCV_CFLAGS) \
           $(RC_GENICAM_CFLAGS) \
           -std=c++11

LDFLAGS = -L/opt/lib_tomo/labjack \
          -lpthread -lm \
          $(OPENCV_LIBS) \
          -ltiff -lboost_system -lboost_filesystem -lboost_chrono -lboost_thread \
          $(RC_GENICAM_LIBS) \
          -lLjack -llabjackusb -llabjackusbcpp -lmsleep

TARGET = $(BINDIR)/tdm_acquisition
OBJS   = $(OBJDIR)/main.o $(OBJDIR)/functions.o $(OBJDIR)/scan_functions.o

$(TARGET): $(OBJS) | $(BINDIR)
	$(CXX) -o $@ $^ $(LDFLAGS)

$(OBJDIR)/main.o: main.cpp | $(OBJDIR)
	$(CXX) -c $< $(CXXFLAGS) -o $@

$(OBJDIR)/functions.o: src/functions.cpp | $(OBJDIR)
	$(CXX) -c $< $(CXXFLAGS) -o $@

$(OBJDIR)/scan_functions.o: src/scan_functions.cpp | $(OBJDIR)
	$(CXX) -c $< $(CXXFLAGS) -o $@

$(OBJDIR) $(BINDIR):
	mkdir -p $@

clean:
	rm -f $(OBJDIR)/*.o $(TARGET)

.PHONY: clean
