#!/bin/sh

gnome-terminal -e "bash -c 'ypspur-coordinator -p ~/researches/programs/platform/yp-robot-params/robot-params/M1.param -d /dev/ttyACM0'"
sleep 2
gnome-terminal -e "bash -c 'roslaunch ymbc kadai1.launch'"

exit 0
