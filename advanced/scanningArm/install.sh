#!/bin/sh
#Run this script to create a dynamic link to the robotStart script.
#Once this has executed, you can run the program anywhere by typing "robotStart"
ROBOT_FILE="/robotStart.sh"
sudo ln -s $(dirname $(readlink -f $0))$ROBOT_FILE /usr/bin/robotStart
chmod +x robotStart.sh
echo "you can now run the program by typing: robotStart"

