#!/bin/bash

set -e

./tools/build.sh
avrdude -v -c wiring -p m2560 -P /dev/ttyACM0 -b 115200 -D -F -U flash:w:build/Light.ino.hex
