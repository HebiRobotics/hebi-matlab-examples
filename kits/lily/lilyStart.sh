#!/bin/sh
#This script launches the daisy matlab script with the correct path.
cd $(dirname $(readlink -f $0))
x-terminal-emulator -e "matlab -r \"runLily\""
