#!/bin/bash
# upload via mqtt and pygateway

set -e

./tools/build.sh
mosquitto_pub -h DEPTH -t mega/flash -m "$(cat ./build/asol-firmware.ino.hex)"

echo "firmware sent to mega/flash"
