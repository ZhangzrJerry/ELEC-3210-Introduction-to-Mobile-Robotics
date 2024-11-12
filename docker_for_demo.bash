#!/bin/bash
SCRIPT_DIR=$(cd $(dirname $0); pwd)

docker run -it --name elec3210demo --label elec3210 -p 3211:3211 -v $SCRIPT_DIR:/home/ubuntu/elec3210 xxxxxl/reconstruction:v0.1 /bin/bash