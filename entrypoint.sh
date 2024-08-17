#!/bin/bash
source /opt/ros/humble/setup.bash

set -e

# Setup the Space ROS environment
source "${SPACEROS_DIR}/install/setup.bash"
export IKOS_SCAN_NOTIFIER_FILES="" # make ikos create .ikosbin files for compiled packages


echo "                                         _.oo.   "
echo "                 _.u[[/;:,.         .odMMMMMM'   "
echo "              .o888UU[[[/;:-.  .o@P^    MMM^     "
echo "             oN88888UU[[[/;::-.        dP^       "
echo "            dNMMNN888UU[[[/;:--.   .o@P^         "
echo "           ,MMMMMMN888UU[[/;::-. o@^             "
echo "           NNMMMNN888UU[[[/~.o@P^                "
echo "           888888888UU[[[/o@^-..                 "
echo "          oI8888UU[[[/o@P^:--..                  "
echo "       .@^  YUU[[[/o@^;::---..                   "
echo "     oMP     ^/o@P^;:::---..                     "
echo "  .dMMM    .o@^ ^;::---...                       "
echo " dMMMMMMM@^`       `^^^^                         "
echo "YMMMUP^                                          "
echo " ^^                                              "
echo "                   Welcome!                      "


exec "$@"
