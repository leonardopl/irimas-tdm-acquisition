# TDM Holographic Tomography Acquisition
# Requires: Aravis, LabJack, OpenCV 4

CXX        = g++
OBJDIR     = obj
BINDIR     = bin

# Use pkg-config for OpenCV
OPENCV_CFLAGS := $(shell pkg-config --cflags opencv4 2>/dev/null)
OPENCV_LIBS   := $(shell pkg-config --libs opencv4 2>/dev/null)

# Aravis via pkg-config
ARAVIS_CFLAGS := $(shell pkg-config --cflags aravis-0.8 2>/dev/null)
ARAVIS_LIBS   := $(shell pkg-config --libs aravis-0.8 2>/dev/null)

CXXFLAGS = -D_UNIX_ -D_LINUX_ \
           -Iinclude -Iinclude/LabJack -Iinclude/john \
           $(OPENCV_CFLAGS) \
           $(ARAVIS_CFLAGS) \
           -std=c++11

LDFLAGS = -L/opt/lib_tomo/labjack \
          -lpthread -lm \
          $(OPENCV_LIBS) \
          -ltiff \
          $(ARAVIS_LIBS) \
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
