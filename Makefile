# TDM Holographic Tomography Acquisition
# Requires: Basler Pylon SDK, LabJack, OpenCV 4, Boost

PYLON_ROOT ?= /opt/pylon
CXX        = g++
OBJDIR     = obj
BINDIR     = bin

CXXFLAGS = -D_UNIX_ -D_LINUX_ \
           -Iinclude -Iinclude/LabJack -Iinclude/john \
           -I/usr/include/opencv4 \
           $(shell $(PYLON_ROOT)/bin/pylon-config --cflags) \
           -std=c++11

LDFLAGS = -L/opt/lib_tomo/labjack \
          -lpthread -lm \
          -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc \
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
