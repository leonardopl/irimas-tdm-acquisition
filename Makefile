# TDM Holographic Tomography Acquisition
# Requires: Basler Pylon SDK, LabJack, OpenCV 4, Boost

PYLON_ROOT ?= /opt/pylon
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

CXXFLAGS = -D_UNIX_ -D_LINUX_ \
           -Iinclude -Iinclude/LabJack -Iinclude/john \
           $(OPENCV_CFLAGS) \
           $(shell $(PYLON_ROOT)/bin/pylon-config --cflags) \
           -std=c++11

LDFLAGS = -L/opt/lib_tomo/labjack \
          -lpthread -lm \
          $(OPENCV_LIBS) \
          -ltiff -lboost_system -lboost_filesystem -lboost_chrono -lboost_thread \
          $(shell $(PYLON_ROOT)/bin/pylon-config --libs) \
          -lLjack -llabjackusb -llabjackusbcpp -lmsleep

TARGET = $(BINDIR)/tdm_acquisition
OBJS   = $(OBJDIR)/main.o $(OBJDIR)/fonctions.o $(OBJDIR)/scan_functions.o

$(TARGET): $(OBJS) | $(BINDIR)
	$(CXX) -o $@ $^ $(LDFLAGS)

$(OBJDIR)/main.o: main.cpp | $(OBJDIR)
	$(CXX) -c $< $(CXXFLAGS) -o $@

$(OBJDIR)/fonctions.o: src/fonctions.cpp | $(OBJDIR)
	$(CXX) -c $< $(CXXFLAGS) -o $@

$(OBJDIR)/scan_functions.o: src/scan_functions.cpp | $(OBJDIR)
	$(CXX) -c $< $(CXXFLAGS) -o $@

$(OBJDIR) $(BINDIR):
	mkdir -p $@

clean:
	rm -f $(OBJDIR)/*.o $(TARGET)

.PHONY: clean
