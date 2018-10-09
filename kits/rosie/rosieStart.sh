#!/bin/sh
#This script launches the rosie matlab script with the correct path.
cd $(dirname $(readlink -f $0))
x-terminal-emulator -e "matlab -r \"rosieDemoRunner('omni')\""
