#!/bin/bash
# Run script for TDM acquisition

# Set environment
export PYLON_ROOT=/opt/pylon
export LD_LIBRARY_PATH=$PYLON_ROOT/lib:/opt/lib_tomo/labjack:$LD_LIBRARY_PATH

# Run the application
cd "$(dirname "$0")"
./bin/tdm_acquisition "$@"
