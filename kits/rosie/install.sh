#!/bin/sh
#Run this script to create a dynamic link to the rosieStart script.
#Once this has executed, you can run the program anywhere by typing "rosieStart"
ROSIE_FILE="/rosieStart.sh"
sudo ln -s $(dirname $(readlink -f $0))$ROSIE_FILE /usr/bin/rosieStart
chmod +x rosieStart.sh
echo "you can now run the program by typing: rosieStart"

