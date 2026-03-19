#!/bin/bash
# Run script for TDM acquisition

cd "$(dirname "$0")"
./bin/tdm_acquisition "$@"
