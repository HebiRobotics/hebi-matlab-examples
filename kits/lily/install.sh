#!/bin/sh
#Run this script to create a dynamic link to the daisyStart script.
#Once this has executed, you can run the program anywhere by typing "daisyStart"
DAISY_FILE="/daisyStart.sh"
sudo ln -s $(dirname $(readlink -f $0))$DAISY_FILE /usr/bin/daisyStart
chmod +x daisyStart.sh
echo "you can now run the program by typing: daisyStart"

