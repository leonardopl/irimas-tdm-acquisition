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
OBJS   = $(OBJDIR)/main.o $(OBJDIR)/functions.o $(OBJDIR)/scan_functions.o $(OBJDIR)/scan_schedule.o
TEST_TARGET = $(BINDIR)/test_scan_schedule
CONFIG_TEST_TARGET = $(BINDIR)/test_config_resolution
FRAME_LOOP_TEST_TARGET = $(BINDIR)/test_frame_loop
CAMERA_DISCOVERY_TEST_TARGET = $(BINDIR)/test_camera_discovery
ACQUISITION_PROTOCOL_TEST_TARGET = $(BINDIR)/test_acquisition_protocol
SESSION_TEST_TARGET = $(BINDIR)/test_session

$(TARGET): $(OBJS) | $(BINDIR)
	$(CXX) -o $@ $^ $(LDFLAGS)

$(OBJDIR)/main.o: main.cpp include/acquisition_protocol.h include/functions.h include/camera_discovery.h include/frame_loop.h include/frame_timing.h include/scan_schedule.h include/scan_functions.h | $(OBJDIR)
	$(CXX) -c $< $(CXXFLAGS) -o $@

$(OBJDIR)/functions.o: src/functions.cpp include/functions.h include/acquisition_protocol.h | $(OBJDIR)
	$(CXX) -c $< $(CXXFLAGS) -o $@

$(OBJDIR)/scan_functions.o: src/scan_functions.cpp | $(OBJDIR)
	$(CXX) -c $< $(CXXFLAGS) -o $@

$(OBJDIR)/scan_schedule.o: src/scan_schedule.cpp include/scan_schedule.h include/scan_functions.h | $(OBJDIR)
	$(CXX) -c $< $(CXXFLAGS) -o $@

$(TEST_TARGET): tests/test_scan_schedule.cpp src/scan_schedule.cpp src/scan_functions.cpp include/scan_schedule.h include/scan_functions.h include/frame_timing.h | $(BINDIR)
	$(CXX) $(CXXFLAGS) tests/test_scan_schedule.cpp src/scan_schedule.cpp src/scan_functions.cpp -o $@

$(CONFIG_TEST_TARGET): tests/test_config_resolution.cpp src/functions.cpp include/functions.h include/acquisition_protocol.h | $(BINDIR)
	$(CXX) $(CXXFLAGS) tests/test_config_resolution.cpp src/functions.cpp -o $@

$(FRAME_LOOP_TEST_TARGET): tests/test_frame_loop.cpp include/frame_loop.h include/frame_timing.h | $(BINDIR)
	$(CXX) $(CXXFLAGS) $< -o $@

$(CAMERA_DISCOVERY_TEST_TARGET): tests/test_camera_discovery.cpp include/camera_discovery.h | $(BINDIR)
	$(CXX) $(CXXFLAGS) $< -o $@

$(ACQUISITION_PROTOCOL_TEST_TARGET): tests/test_acquisition_protocol.cpp include/acquisition_protocol.h | $(BINDIR)
	$(CXX) $(CXXFLAGS) $< -o $@

# Includes main.cpp with a renamed entry point so only the fixture can inject simulated hardware.
$(SESSION_TEST_TARGET): tests/test_session.cpp main.cpp include/acquisition_protocol.h include/functions.h include/camera_discovery.h include/frame_loop.h include/frame_timing.h include/scan_schedule.h include/scan_functions.h $(OBJDIR)/functions.o $(OBJDIR)/scan_schedule.o $(OBJDIR)/scan_functions.o | $(BINDIR)
	$(CXX) $(CXXFLAGS) $< $(OBJDIR)/functions.o $(OBJDIR)/scan_schedule.o $(OBJDIR)/scan_functions.o -o $@ $(LDFLAGS)

test-argument-keys:
	python3 tests/test_argument_keys.py

test: test-argument-keys $(TARGET) $(TEST_TARGET) $(CONFIG_TEST_TARGET) $(FRAME_LOOP_TEST_TARGET) $(CAMERA_DISCOVERY_TEST_TARGET) $(ACQUISITION_PROTOCOL_TEST_TARGET) $(SESSION_TEST_TARGET)
	./$(TEST_TARGET)
	./$(CONFIG_TEST_TARGET)
	./$(FRAME_LOOP_TEST_TARGET)
	./$(CAMERA_DISCOVERY_TEST_TARGET)
	./$(ACQUISITION_PROTOCOL_TEST_TARGET)
	./$(SESSION_TEST_TARGET) || test $$? -eq 77
	test "$$(grep -c TDM_SESSION_PROTOCOL=1 $(TARGET))" -ge 1

fermat-fixture: $(TEST_TARGET)
	./$(TEST_TARGET) --write-fermat-fixture tests/fixtures/fermat_rings_rig_400.csv

$(OBJDIR) $(BINDIR):
	mkdir -p $@

clean:
	rm -f $(OBJDIR)/*.o $(TARGET) $(TEST_TARGET) $(CONFIG_TEST_TARGET) $(FRAME_LOOP_TEST_TARGET) $(CAMERA_DISCOVERY_TEST_TARGET) $(ACQUISITION_PROTOCOL_TEST_TARGET) $(SESSION_TEST_TARGET)

.PHONY: clean test test-argument-keys fermat-fixture
