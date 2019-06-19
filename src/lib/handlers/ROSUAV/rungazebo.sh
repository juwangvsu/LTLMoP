#!/bin/bash
#send out approperate mavros cmd
echo $1 $2 $#
gnome-terminal -x $SHELL -ic "echo 'p1 $1 p2 $2 p3 $3'; $1"
