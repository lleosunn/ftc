#!/bin/bash

echo -n "What is your username (in your home folder)? "
read uname
cd /Users/$uname/Downloads
if [ -f /Users/$uname/Desktop/ftc ]; then
  mv teamcode /Users/$uname/Desktop/ftc/SCORPIO-Teamcode
else
  mkdir /Users/$uname/Desktop/ftc
  mv teamcode /Users/$uname/Desktop/ftc/SCORPIO-Teamcode
fi
echo -n "Are the FTC skystone modules installed? Make sure you install them. Press enter when you are done... "
read a
echo -n "Install android studio... Use homebrew or the custom installer..."
read b
echo "Move your teamcode folder to the extraced skystone module."
echo "You are all done!"

# Copyright FTC Team Scorpio
