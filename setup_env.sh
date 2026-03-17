#!/bin/bash
# Environment setup for TDM acquisition

echo "================================================"
echo "  TDM Acquisition - Environment Setup"
echo "================================================"
echo ""

# Set library paths
export PYLON_ROOT=/opt/pylon
export LD_LIBRARY_PATH=$PYLON_ROOT/lib:/opt/lib_tomo/labjack:$LD_LIBRARY_PATH

echo "[OK] Library paths configured:"
echo "  - /opt/pylon/lib"
echo "  - /opt/lib_tomo/labjack"
echo ""

# Check if Pylon is installed
if [ -f "/opt/pylon/bin/pylon-config" ]; then
    echo "[OK] Pylon SDK found: $(/opt/pylon/bin/pylon-config --version 2>/dev/null || echo 'installed')"
else
    echo "[!!] Warning: Pylon SDK not found at /opt/pylon"
    echo "  Install from: https://www.baslerweb.com/en/downloads/software-downloads/"
fi

# Check if LabJack libraries exist
if [ -d "/opt/lib_tomo/labjack" ]; then
    echo "[OK] LabJack libraries found"
else
    echo "[!!] Warning: LabJack libraries not found at /opt/lib_tomo/labjack"
fi

echo ""
echo "================================================"
echo "  To make permanent, add to ~/.bashrc:"
echo "================================================"
echo "export PYLON_ROOT=/opt/pylon"
echo "export LD_LIBRARY_PATH=\$PYLON_ROOT/lib:/opt/lib_tomo/labjack:\$LD_LIBRARY_PATH"
echo ""
echo "================================================"
echo "  Ready to compile and run:"
echo "================================================"
echo "  make"
echo "  ./bin/tdm_acquisition"
echo ""
